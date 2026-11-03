%% LIGHTING_DASHBOARD  Feedback / communication layer: monitoring + energy analytics
%
% Runs automatically at the end of run_lighting_simulation.m and when the
% Simulink model ClassroomLighting.slx finishes (model StopFcn).
% It needs the workspace variable sim_log and the parameters from lighting_params.m.
%
% sim_log columns:
%   1 t | 2 vo | 3 iL | 4 q (buck gate) | 5 d (buck duty) | 6-8 zone currents
%   9-11 zone gates | 12-14 zone duty commands | 15 E_in | 16-18 E_zone | 19 E_esp32
%   20 thermal derating flag | 21 humidity alert flag

if ~exist('sim_log', 'var') && exist('out', 'var')   % Simulink "single simulation output" mode
    try, sim_log = out.get('sim_log'); catch, end
end
if ~exist('sim_log', 'var') || isempty(sim_log)
    error('lighting_dashboard:noData', ...
          'No simulation data (sim_log). Run run_lighting_simulation or the Simulink model first.');
end
if ~exist('Vo_ref', 'var'), lighting_params; end

%% ---------------- Unpack the log ----------------
db_t    = sim_log(:,1);   db_vo  = sim_log(:,2);   db_iL = sim_log(:,3);
db_q    = sim_log(:,4);   db_d   = sim_log(:,5);
db_iz   = sim_log(:,6:8); db_g   = sim_log(:,9:11); db_duty = sim_log(:,12:14);
db_Ein  = sim_log(:,15);  db_Ez  = sim_log(:,16:18); db_Eesp = sim_log(:,19);
db_hr   = scen_hour(1) + db_t/T_slot;                    % time axis in "hour of the day"
db_iload = sum(db_iz, 2) + I_esp32;

% Hours that were fully simulated (a Simulink run may have been stopped early)
db_nS = min(N_slots, floor(db_t(end)/T_slot + 0.02));
if db_nS < 1
    % The run was stopped before the first simulated hour finished: show what
    % there is and explain, instead of failing.
    fprintf(['\nSimulation stopped early at t = %.1f ms of %.0f ms (%.0f %% done).\n' ...
             'The hourly report needs at least one full hour (%.0f ms).\n' ...
             'Press Run again and let it finish: the time at the bottom of the Simulink\n' ...
             'window must reach %.3f s. This takes about 0.5-2 minutes; do not press Stop.\n\n'], ...
            1e3*db_t(end), 1e3*T_stop, 100*db_t(end)/T_stop, 1e3*T_slot, T_stop);
    figure('Name', 'Partial run: buck converter start-up', 'NumberTitle', 'off', 'Color', 'w');
    subplot(2,1,1); plot(db_t*1e3, db_vo); grid on; ylabel('v_o [V]');
    title(sprintf('Partial run (stopped at %.1f ms) - output voltage', 1e3*db_t(end)));
    subplot(2,1,2); plot(db_t*1e3, db_iL); grid on; ylabel('i_L [A]'); xlabel('t [ms]');
    return
end

%% ---------------- Per-hour metrics (steady-state window of each slot) ----------------
db_win = 0.4;   % ignore the first 40 % of each slot (transient after a change)
m = struct();
m.duty = zeros(db_nS,3); m.lux_led = zeros(db_nS,3); m.lux_tot = zeros(db_nS,3);
m.vo_avg = zeros(db_nS,1); m.vo_pp = zeros(db_nS,1); m.iL_avg = zeros(db_nS,1); m.iL_pp = zeros(db_nS,1);
m.P_in = zeros(db_nS,1); m.P_z = zeros(db_nS,3); m.P_esp = zeros(db_nS,1); m.eta = zeros(db_nS,1);
m.flags = zeros(db_nS,2);
for s = 1:db_nS
    t0  = (s-1)*T_slot;
    idx = find(db_t >= t0 + db_win*T_slot & db_t < t0 + T_slot);
    i0 = idx(1); i1 = idx(end); dT = db_t(i1) - db_t(i0);
    m.duty(s,:)    = db_duty(i1,:);
    m.lux_led(s,:) = lux_led_max * mean(db_iz(idx,:), 1) / I_led_nom;
    m.lux_tot(s,:) = scenario(s,2:4) + m.lux_led(s,:);
    m.vo_avg(s) = mean(db_vo(idx));   m.vo_pp(s) = max(db_vo(idx)) - min(db_vo(idx));
    m.iL_avg(s) = mean(db_iL(idx));   m.iL_pp(s) = max(db_iL(idx)) - min(db_iL(idx));
    m.P_in(s)   = (db_Ein(i1) - db_Ein(i0)) / dT;
    m.P_z(s,:)  = (db_Ez(i1,:) - db_Ez(i0,:)) / dT;
    m.P_esp(s)  = (db_Eesp(i1) - db_Eesp(i0)) / dT;
    m.eta(s)    = (sum(m.P_z(s,:)) + m.P_esp(s)) / m.P_in(s);
    m.flags(s,:) = sim_log(i1, 20:21);
end
m.P_loss = m.P_in - sum(m.P_z,2) - m.P_esp;

% Energy over the day (each slot = hours_per_slot real hours)
E_led_smart   = sum(m.P_z(:)) * hours_per_slot;                 % [Wh] LED energy, smart system
E_in_smart    = sum(m.P_in) * hours_per_slot;                   % [Wh] from 12 V adapter (incl. ESP32 + losses)
E_led_base    = db_nS * N_zones * P_zone_nom * hours_per_slot;  % [Wh] conventional: all zones 100 % all day
save_led_pct  = 100*(1 - E_led_smart/E_led_base);
save_tot_pct  = 100*(1 - E_in_smart/E_led_base);

%% ---------------- Command Window report ----------------
fprintf('=================== CLASSROOM LIGHTING - HOURLY REPORT ===================\n');
fprintf(' Hour | PIR  | Daylight lux    | Duty %%         | Desk lux (target %d) | Vo [V] ripple | P_in  eta  | Flags\n', ctrl_cfg.lux_target);
fprintf('      | Z123 |  Z1   Z2   Z3   |  Z1  Z2  Z3    |  Z1   Z2   Z3        |  avg   [mV]   |  [W]  [%%]  |\n');
for s = 1:db_nS
    fl = '';
    if m.flags(s,1), fl = [fl 'HOT-derate ']; end
    if m.flags(s,2), fl = [fl 'HUMID-alert']; end
    fprintf(' %02d:00| %d%d%d  | %4.0f %4.0f %4.0f  | %3.0f %3.0f %3.0f    | %4.0f %4.0f %4.0f       | %5.3f %5.1f  | %5.2f %4.1f | %s\n', ...
        scen_hour(s), scenario(s,5:7), scenario(s,2:4), 100*m.duty(s,:), m.lux_tot(s,:), ...
        m.vo_avg(s), 1e3*m.vo_pp(s), m.P_in(s), 100*m.eta(s), fl);
end
fprintf('---------------------------------------------------------------------------\n');
fprintf(' Buck design: Vin = %g V, Vo = %g V, fsw = %g kHz, L = %g uH, C = %g uF\n', ...
        Vin, Vo_ref, fsw/1e3, L_buck*1e6, C_buck*1e6);
fprintf('   ideal duty D = Vo/Vin = %.3f, with diode drop D = %.3f, measured avg duty (last hour) = %.3f\n', ...
        Vo_ref/Vin, (Vo_ref+Vd)/(Vin+Vd), mean(db_d(db_t > db_t(end) - 0.5*T_slot)));
fprintf('   inductor ripple (theory, CCM) = %.2f A p-p, LC corner frequency = %.0f Hz\n', ...
        Vo_ref*(1-Vo_ref/Vin)/(fsw*L_buck), 1/(2*pi*sqrt(L_buck*C_buck)));
fprintf(' Energy for the day (%d h):\n', db_nS*hours_per_slot);
fprintf('   Conventional (all zones 100 %%, all day) : %6.1f Wh (LED)\n', E_led_base);
fprintf('   Smart system, LED energy                : %6.1f Wh  -> %4.1f %% saved\n', E_led_smart, save_led_pct);
fprintf('   Smart system, total from 12 V adapter   : %6.1f Wh  -> %4.1f %% saved (incl. ESP32 + converter losses)\n', ...
        E_in_smart, save_tot_pct);
fprintf('===========================================================================\n');

%% ---------------- Figures ----------------
db_hx = [scen_hour(1:db_nS); scen_hour(db_nS)+1];              % x for stairs plots
db_st = @(y) [y(1:db_nS,:); y(db_nS,:)];
db_col = [0.00 0.45 0.74; 0.85 0.33 0.10; 0.47 0.67 0.19];
db_xl  = [db_hx(1) db_hx(end)];

% ---- Figure 1: input layer + ESP32 decisions ----
figure('Name', '1. Sensors and ESP32 decisions', 'NumberTitle', 'off', 'Color', 'w');
subplot(2,2,1);
h = stairs(db_hx, db_st(scenario(:,2:4)), 'LineWidth', 1.5); hold on;
for k = 1:3, set(h(k), 'Color', db_col(k,:)); end
plot(db_hx([1 end]), ctrl_cfg.lux_target*[1 1], 'k--');
title('LDR: daylight on the desks'); xlabel('Hour'); ylabel('lux'); grid on; xlim(db_xl);
legend([zone_names, {'Target'}], 'Location', 'northeast');
subplot(2,2,2);
imagesc(scen_hour(1:db_nS), 1:3, scenario(1:db_nS,5:7)'); colormap(gca, [0.92 0.92 0.92; 0.30 0.70 0.30]);
set(gca, 'YTick', 1:3, 'YTickLabel', zone_names, 'CLim', [0 1]);
title('PIR: occupancy (green = occupied)'); xlabel('Hour');
subplot(2,2,3);
[db_ax, h1, h2] = plotyy(scen_hour(1:db_nS), scenario(1:db_nS,8), scen_hour(1:db_nS), scenario(1:db_nS,9));
set(h1, 'Marker', 'o', 'LineWidth', 1.5); set(h2, 'Marker', 's', 'LineWidth', 1.5);
ylabel(db_ax(1), 'Temperature [C]'); ylabel(db_ax(2), 'Humidity [%]');
title(sprintf('DHT11 (derate above %g C, alert above %g %%)', ctrl_cfg.T_hot, ctrl_cfg.H_high));
xlabel('Hour'); grid on;
subplot(2,2,4);
h = stairs(db_hx, 100*db_st(m.duty), 'LineWidth', 1.5);
for k = 1:3, set(h(k), 'Color', db_col(k,:)); end
title('ESP32 decision: PWM duty of each zone'); xlabel('Hour'); ylabel('Duty [%]');
ylim([0 130]); xlim(db_xl); grid on; legend({'Front', 'Middle', 'Back'}, 'Location', 'north', 'Orientation', 'horizontal');

% ---- Figure 2: illuminance result ----
figure('Name', '2. Desk illuminance per zone', 'NumberTitle', 'off', 'Color', 'w');
for k = 1:3
    subplot(1,3,k);
    hb = bar(scen_hour(1:db_nS), [scenario(1:db_nS,1+k), m.lux_led(:,k)], 'stacked');
    set(hb(1), 'FaceColor', [1.00 0.80 0.20]); set(hb(2), 'FaceColor', db_col(k,:));
    hold on; plot([scen_hour(1)-0.5, scen_hour(db_nS)+0.5], ctrl_cfg.lux_target*[1 1], 'k--', 'LineWidth', 1.2);
    title(zone_names{k}); xlabel('Hour'); ylabel('Desk illuminance [lux]'); grid on;
    set(gca, 'XTick', scen_hour(1):2:scen_hour(db_nS)); ylim([0 1.25*max(max(m.lux_tot(:)), ctrl_cfg.lux_target)]);
    if k == 1, legend('Daylight', 'LED', 'Target', 'Location', 'northwest'); end
end

% ---- Figure 3: buck converter overview ----
figure('Name', '3. Buck converter 12 V -> 5 V', 'NumberTitle', 'off', 'Color', 'w');
subplot(2,2,1); plot(db_hr, db_vo); grid on; xlim(db_xl);
title('Output voltage (whole day)'); xlabel('Hour'); ylabel('v_o [V]'); ylim([0 6]);
subplot(2,2,2); plot(db_hr, db_iL, 'Color', [0.85 0.33 0.10]); hold on;
stairs(db_hx, db_st(m.iL_avg), 'k', 'LineWidth', 1.5); grid on; xlim(db_xl);
title('Inductor current (with hourly average)'); xlabel('Hour'); ylabel('[A]'); legend('i_L', 'average i_L');
db_k = db_t <= 15e-3;
subplot(2,2,3); plot(db_t(db_k)*1e3, db_vo(db_k), db_t(db_k)*1e3, db_iL(db_k)); grid on;
title(sprintf('Soft-start (%g ms ramp)', t_ss*1e3)); xlabel('t [ms]'); legend('v_o [V]', 'i_L [A]');
[~, db_sb] = max(sum(m.duty, 2));                            % busiest hour
db_tz = (db_sb-1)*T_slot + 0.8*T_slot;
db_k = db_t >= db_tz & db_t <= db_tz + 4/fsw;
subplot(2,2,4); plot((db_t(db_k)-db_tz)*1e6, db_iL(db_k), 'LineWidth', 1.5); hold on;
plot((db_t(db_k)-db_tz)*1e6, 0.5*db_q(db_k), 'k');
grid on; xlabel('t [\mus]'); ylabel('[A]');
title(sprintf('Switching detail at %02d:00 (4 periods)', scen_hour(db_sb)));
legend('i_L', 'MOSFET gate (scaled)', 'Location', 'east');

% ---- Figure 4: switching waveforms ----
figure('Name', '4. MOSFET switching waveforms', 'NumberTitle', 'off', 'Color', 'w');
db_k = db_t >= db_tz & db_t <= db_tz + 5/fsw;  db_x = (db_t(db_k)-db_tz)*1e6;
subplot(3,1,1); plot(db_x, db_q(db_k), 'k', 'LineWidth', 1.2); ylim([-0.2 1.2]); grid on;
ylabel('Gate'); title(sprintf('Buck MOSFET at %02d:00, f_{sw} = %g kHz', scen_hour(db_sb), fsw/1e3));
subplot(3,1,2); plot(db_x, db_iL(db_k), 'LineWidth', 1.5); grid on; ylabel('i_L [A]');
subplot(3,1,3); plot(db_x, db_vo(db_k), 'LineWidth', 1.5); grid on; ylabel('v_o [V]'); xlabel('t [\mus]');

% ---- Figure 5: PWM dimming of the zones ----
figure('Name', '5. Zone MOSFET PWM dimming', 'NumberTitle', 'off', 'Color', 'w');
db_k = db_t >= db_tz & db_t <= db_tz + 3/f_led;  db_x = (db_t(db_k)-db_tz)*1e6;
for k = 1:3
    subplot(4,1,k); plot(db_x, db_iz(db_k,k), 'Color', db_col(k,:), 'LineWidth', 1.5); grid on;
    ylabel('[A]'); ylim([-0.1 1.3]);
    title(sprintf('%s: LED current, duty %.0f %%', zone_names{k}, 100*m.duty(db_sb,k)));
end
subplot(4,1,4); plot(db_x, db_iload(db_k), 'k', 'LineWidth', 1.2); grid on;
ylabel('[A]'); xlabel('t [\mus]');
title('Total load on the 5 V rail (interleaved zone PWM)');

% ---- Figure 6: energy analytics ----
figure('Name', '6. Energy dashboard', 'NumberTitle', 'off', 'Color', 'w');
subplot(2,2,1);
hb = bar(scen_hour(1:db_nS), [m.P_z, m.P_esp, m.P_loss], 'stacked');
for k = 1:3, set(hb(k), 'FaceColor', db_col(k,:)); end
set(hb(4), 'FaceColor', [0.5 0.5 0.5]); set(hb(5), 'FaceColor', [0.9 0.2 0.2]);
title('Power from the 12 V adapter'); xlabel('Hour'); ylabel('[W]'); grid on;
ylim([0 1.8*max(m.P_in)]); set(gca, 'XTick', scen_hour(1):2:scen_hour(db_nS));
legend([zone_names, {'ESP32 + sensors', 'Converter losses'}], 'Location', 'north', 'FontSize', 7);
subplot(2,2,2);
plot(scen_hour(1:db_nS), 100*m.eta, 'o-', 'LineWidth', 1.5); grid on; ylim([0 100]);
title('Buck converter efficiency'); xlabel('Hour'); ylabel('\eta [%]');
subplot(2,2,3);
plot(db_hx, [0; cumsum(N_zones*P_zone_nom*ones(db_nS,1))*hours_per_slot], 'k--', 'LineWidth', 1.5); hold on;
plot(db_hx, [0; cumsum(sum(m.P_z,2))*hours_per_slot], 'LineWidth', 1.5);
plot(db_hx, [0; cumsum(m.P_in)*hours_per_slot], 'LineWidth', 1.5); grid on;
title('Cumulative energy'); xlabel('Hour'); ylabel('[Wh]'); xlim(db_xl);
legend('Conventional (always 100 %)', 'Smart - LED', 'Smart - total 12 V input', 'Location', 'northwest');
subplot(2,2,4);
bar(1:3, [E_led_base, E_led_smart, E_in_smart], 'FaceColor', [0.30 0.60 0.80]); grid on;
set(gca, 'XTick', 1:3, 'XTickLabel', {'Conventional', 'Smart LED', 'Smart total'});
ylabel('Energy per day [Wh]');
title(sprintf('Energy saving: %.0f %% (LED), %.0f %% (total)', save_led_pct, save_tot_pct));
