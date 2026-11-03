%% RUN_LIGHTING_SIMULATION  Intelligent Multi-Zone Classroom Lighting System (plain MATLAB)
%
% Switching-level simulation of the whole system. Needs base MATLAB only,
% no Simulink and no toolboxes:
%
%   LDR / PIR / DHT11  ->  ESP32 decision engine  ->  zone PWM duty (5 kHz)
%   12 V --[MOSFET]--+--L--+-- 5 V rail --[MOSFET]-- LED strip zone 1..3
%                    |     |
%                  diode   C        (asynchronous buck, digital PID voltage control, 50 kHz)
%
% Every MOSFET switching event is simulated. The circuit is solved with a
% fixed time step of 0.2 us, so you see real inductor ripple, DCM at light
% load, PWM dimming currents, losses and efficiency.
%
% Run:  >> run_lighting_simulation
% Results: figures from lighting_dashboard.m, plus a summary table in the
% Command Window.

clear; clc; close all;
clear esp32_controller            % reset the controller's memory (persistent state)
here = fileparts(mfilename('fullpath'));
if ~isempty(here), addpath(here); end
lighting_params;

fprintf('Intelligent Multi-Zone Classroom Lighting System\n');
fprintf('Simulating %d hours (%.0f ms of circuit time, %d steps)...\n', ...
        N_slots, T_stop*1e3, round(T_stop/dt_sim));

%% ---------------- Simulation set-up ----------------
N          = round(T_stop/dt_sim);
ctrl_every = round(Ts_ctrl/dt_sim);     % lighting controller period (1 ms)
v_every    = round(Ts_v/dt_sim);        % voltage-loop period (one PWM period)
n_log      = floor((N-1)/log_decim) + 1;
sim_log    = zeros(n_log, 21);          % same column layout as the Simulink model
zone_phase = zone_phase(:);
lux_per_A  = lux_led_max/I_led_nom;     % LED illuminance per ampere of zone current

% States
iL = 0;               % inductor current            [A]
vc = 0;               % output capacitor voltage    [V] (without ESR drop)
vo = 0;               % output terminal voltage     [V]
xi = 0;               % PID integral term           [-]
dterm = 0;            % PID filtered derivative term
vs_prev = 0;          % previous ADC sample of vo
d = 0; d_next = 0;    % applied duty / duty computed for the next period
ldr_led = zeros(3,1); % LED light seen by each LDR, averaged over the last control period [lux]
ldr_acc = zeros(3,1); % running sum for the ADC averaging
E_in = 0; E_z = zeros(3,1); E_esp = 0;   % energies [J]
duty_zone = zeros(3,1); status = zeros(2,1);

tic;
row = 0;
for n = 0:N-1
    t = n*dt_sim;

    % ---------- ESP32: sensor reading + decision (every Ts_ctrl) ----------
    if mod(n, ctrl_every) == 0
        s = min(floor(t/T_slot + 1e-9) + 1, N_slots);
        if n > 0, ldr_led = ldr_acc/ctrl_every; end  % average of the last period (oversampling)
        ldr_acc = zeros(3,1);
        ldr_meas = scenario(s,2:4)' + ldr_led;      % LDR = daylight + own LEDs
        [duty_zone, status] = esp32_controller(ldr_meas, scenario(s,5:7)', ...
                                               scenario(s,8), scenario(s,9));
    end

    % ---------- ESP32: digital buck voltage controller (PID), once per PWM period ----------
    if mod(n, v_every) == 0
        d     = d_next;                             % duty computed last period is applied now
        vs    = vo;                                 % ADC sample
        vrs   = Vo_ref*min(t/t_ss, 1);              % soft-start reference
        e     = vrs - vs;
        dterm = (tau_d*dterm - Kd_v*(vs - vs_prev))/(tau_d + Ts_v);   % filtered derivative
        vs_prev = vs;
        d_next = min(max(vrs/Vin + Kp_v*e + xi + dterm, 0), d_max);
        xi    = min(max(xi + Ki_v*Ts_v*e, xi_min), xi_max);
    end
    q = double((t*fsw - floor(t*fsw)) < d);         % buck MOSFET gate (50 kHz PWM)

    % ---------- ESP32: zone PWM (LEDC, 5 kHz, phase shifted) ----------
    ph = t*f_led + zone_phase;
    g  = double((ph - floor(ph)) < duty_zone);      % zone MOSFET gates

    % ---------- Zone MOSFET + LED strip ----------
    iz    = g .* max(vc - Vf_led, 0) / R_led;       % zone currents [A]
    iload = sum(iz) + I_esp32;
    iC    = iL - iload;
    vo    = vc + ESR_C*iC;                          % terminal voltage incl. ESR

    % ---------- Log ----------
    if mod(n, log_decim) == 0
        row = row + 1;
        sim_log(row,:) = [t, vo, iL, q, d, iz', g', duty_zone', E_in, E_z', E_esp, status'];
    end

    % ---------- Buck power stage (MOSFET + diode + L + C) ----------
    % MOSFET on : vL = Vin - iL*(Rds_on + rL) - vo
    % MOSFET off: vL = -Vd - iL*rL - vo     (diode freewheeling)
    % iL >= 0   : diode blocks reverse current -> DCM at light load
    vL  = q*Vin - (1-q)*Vd - iL*(rL + q*Rds_on) - vo;
    E_in  = E_in  + Vin*q*iL*dt_sim;
    E_z   = E_z   + vo*iz*dt_sim;
    E_esp = E_esp + vo*I_esp32*dt_sim;
    ldr_acc = ldr_acc + lux_per_A*iz;
    iL  = max(iL + vL/L_buck*dt_sim, 0);
    vc  = vc + iC/C_buck*dt_sim;
end
sim_log = sim_log(1:row,:);
fprintf('Done in %.1f s.\n\n', toc);

%% ---------------- Results ----------------
lighting_dashboard;
