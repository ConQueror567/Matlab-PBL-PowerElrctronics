%% RUN_LIGHTING_SIMULATION  Intelligent Multi-Zone Classroom Lighting System (plain MATLAB)
%
% Switching-level simulation of the whole system. Needs base MATLAB only,
% no Simulink and no toolboxes:
%
%   LDR / PIR / DHT11  ->  ESP32 decision engine  ->  zone PWM duty (5 kHz)
%   12 V --[MOSFET]--+--L--+-- 5 V rail --[MOSFET]-- LED strip zone 1..3
%                    |     |
%                  diode   C        (asynchronous buck, PID voltage control, 50 kHz)
%
% Every MOSFET switching event is simulated. The circuit is solved with a
% fixed time step of 0.2 us, so you see real inductor ripple, DCM at light
% load, PWM dimming currents, losses and efficiency.
%
% Run:  >> run_lighting_simulation
% Results: figures from lighting_dashboard.m, plus a summary table in the
% Command Window.

clear; clc; close all;
here = fileparts(mfilename('fullpath'));
if ~isempty(here), addpath(here); end
lighting_params;

fprintf('Intelligent Multi-Zone Classroom Lighting System\n');
fprintf('Simulating %d hours (%.0f ms of circuit time, %d steps)...\n', ...
        N_slots, T_stop*1e3, round(T_stop/dt_sim));

%% ---------------- Simulation set-up ----------------
N          = round(T_stop/dt_sim);
ctrl_every = round(Ts_ctrl/dt_sim);
n_log      = floor((N-1)/log_decim) + 1;
sim_log    = zeros(n_log, 21);          % same column layout as the Simulink model
zone_phase = zone_phase(:);

% States
iL = 0;               % inductor current      [A]
vo = 0;               % output capacitor volt [V]
xi = 0;               % PI integral term      [-]
zf = 0;               % derivative filter state [V]
E_in = 0; E_z = zeros(3,1); E_esp = 0;   % energies [J]
duty_zone = zeros(3,1); status = zeros(2,1);

tic;
row = 0;
for n = 0:N-1
    t = n*dt_sim;

    % ---------- ESP32: sensor reading + decision (every Ts_ctrl) ----------
    if mod(n, ctrl_every) == 0
        s = min(floor(t/T_slot + 1e-9) + 1, N_slots);
        [duty_zone, status] = esp32_controller(scenario(s,2:4)', scenario(s,5:7)', ...
                                               scenario(s,8), scenario(s,9));
    end

    % ---------- ESP32: buck voltage controller (PID) + 50 kHz PWM ----------
    vref = Vo_ref*min(t/t_ss, 1);                  % soft-start reference
    e    = vref - vo;
    dterm = -Kd_v*(vo - zf)/tau_d;                 % filtered derivative of vo
    d    = min(max(vref/Vin + Kp_v*e + xi + dterm, 0), d_max);
    q    = double((t*fsw - floor(t*fsw)) < d);     % buck MOSFET gate

    % ---------- ESP32: zone PWM (LEDC, 5 kHz, phase shifted) ----------
    ph = t*f_led + zone_phase;
    g  = double((ph - floor(ph)) < duty_zone);     % zone MOSFET gates

    % ---------- Zone MOSFET + LED strip ----------
    iz    = g .* max(vo - Vf_led, 0) / R_led;      % zone currents [A]
    iload = sum(iz) + I_esp32;

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
    iC  = iL - iload;
    E_in  = E_in  + Vin*q*iL*dt_sim;
    E_z   = E_z   + vo*iz*dt_sim;
    E_esp = E_esp + vo*I_esp32*dt_sim;
    xi  = min(max(xi + Ki_v*e*dt_sim, xi_min), xi_max);
    zf  = zf + (vo - zf)/tau_d*dt_sim;
    iL  = max(iL + vL/L_buck*dt_sim, 0);
    vo  = vo + iC/C_buck*dt_sim;
end
sim_log = sim_log(1:row,:);
fprintf('Done in %.1f s.\n\n', toc);

%% ---------------- Results ----------------
lighting_dashboard;
