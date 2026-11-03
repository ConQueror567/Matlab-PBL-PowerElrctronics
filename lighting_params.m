%% LIGHTING_PARAMS  Parameters for the Intelligent Multi-Zone Classroom Lighting System
%
% This script puts every parameter in the workspace. It runs automatically from
%   * run_lighting_simulation.m       (plain MATLAB version)
%   * ClassroomLighting.slx           (Simulink version, through the model's InitFcn)
% If you change a value here, re-run the simulation. Do not rename the variables,
% because the Simulink blocks use these names.
%
% System chain (matches the block diagram):
%   Sensors (LDR, PIR, DHT11) -> ESP32 (decision + PWM) -> 12V->5V MOSFET buck
%   -> 3 x MOSFET low-side switch -> 3 x LED strip zones -> monitoring / energy

%% ---------------- 1. Custom buck converter (12 V -> 5 V) ----------------
Vin     = 12;        % [V]   DC adapter input
Vo_ref  = 5;         % [V]   regulated output rail (LED strips + ESP32)
fsw     = 50e3;      % [Hz]  MOSFET switching frequency
L_buck  = 68e-6;     % [H]   buck inductor
C_buck  = 1000e-6;   % [F]   output capacitor (2 x 470 uF / 16 V low-ESR in parallel)
ESR_C   = 0.03;      % [ohm] total capacitor ESR (each 470 uF ~ 60 mohm)
rL      = 0.05;      % [ohm] inductor winding resistance (DCR)
Rds_on  = 0.03;      % [ohm] high-side MOSFET on-resistance (P-ch e.g. AOD4185, or N-ch + bootstrap driver)
Vd      = 0.45;      % [V]   freewheeling Schottky diode drop (e.g. SS54 / SS34)

% Digital voltage-mode controller running on the ESP32:
%   the ADC samples vo once per PWM period (Ts_v); the new duty is applied one
%   period later (computation delay), exactly as firmware in a timer ISR would.
%   duty = Vref/Vin (feed-forward) + Kp*e + Ki*sum(e)*Ts - filtered D-term on vo
% Design (PID = "Type-III" compensator): the two PID zeros are placed on the
% LC double pole w0, with damping zeta_z, which cancels the lightly damped
% resonance. The loop then behaves like an integrator Vin*Kd*w0^2/s that
% crosses 0 dB at f_c.
% Why 1.5 kHz: with the sampling + one-period delay, a 3 kHz crossover leaves
% only 1.5-6 dB gain margin (depending on ESR). 1.5 kHz keeps ~8-12 dB gain
% margin and 35-60 deg phase margin for ESR between 0 and 60 mohm.
Ts_v    = 1/fsw;                     % [s] voltage-loop sample time (one ADC sample per PWM period)
w0      = 1/sqrt(L_buck*C_buck);     % [rad/s] LC resonance (~610 Hz)
f_c     = 1.5e3;                     % [Hz] target loop crossover
zeta_z  = 0.7;                       % damping of the compensator zeros
Kd_v    = 2*pi*f_c/(Vin*w0^2);       % derivative gain   [s/V]
Kp_v    = 2*zeta_z*w0*Kd_v;          % proportional gain [1/V]
Ki_v    = w0^2*Kd_v;                 % integral gain     [1/(V*s)]
tau_d   = 1/(2*pi*15e3);             % [s] derivative filter (pole at 15 kHz)
xi_max  = 0.20;      % anti-windup limits of the integral term
xi_min  = -0.20;
d_max   = 0.90;      % maximum duty cycle
t_ss    = 5e-3;      % [s] soft-start ramp time of the reference

%% ---------------- 2. LED zones (MOSFET switch + 5 V LED strip) ----------------
N_zones    = 3;
zone_names = {'Zone 1 (Front)', 'Zone 2 (Middle)', 'Zone 3 (Back)'};
f_led      = 5e3;            % [Hz] ESP32 LEDC PWM frequency used for dimming
zone_phase = [0 1/3 2/3];    % PWM phase shift of each zone (interleaving reduces rail ripple)
Vf_led     = 2.9;            % [V]   LED strip knee voltage (white LEDs)
R_led      = 2.1;            % [ohm] equivalent series resistance of one zone strip
I_led_nom  = (Vo_ref - Vf_led)/R_led;   % [A] zone current at 100 % duty (= 1 A)
P_zone_nom = Vo_ref*I_led_nom;          % [W] zone power at 100 % duty (= 5 W)
I_esp32    = 0.25;           % [A] ESP32 + sensors, drawn from the 5 V rail

%% ---------------- 3. ESP32 controller ----------------
Ts_ctrl     = 1e-3;                  % [s] lighting controller update period (sensor sampling)
% The ESP32 reads each LDR many times per control period and averages the
% readings (ADC oversampling). This removes the 5 kHz LED flicker from the
% measurement. The model uses the exact average over each 1 ms period.
ctrl_cfg    = controller_config();   % lux target, thresholds, ... (shared with the controller)
lux_led_max = ctrl_cfg.lux_led_max;  % [lux] desk illuminance of one zone at 100 % duty

%% ---------------- 4. Classroom day scenario (sensor readings) ----------------
% One row = one hour of the school day.
%   hour | daylight on the desks [lux] Z1 Z2 Z3 | PIR occupancy Z1 Z2 Z3 | Temp [C] | Hum [%]
% Note: each zone's LDR measures daylight PLUS the light of its own LED strip
% (the plant adds the LED part), so the controller works in closed loop.
scenario = [ ...
     8   120   80   40    1 1 1   26  62     % class starts, low morning light
     9   300  190   90    1 1 1   27  60
    10   480  300  150    1 1 0   29  58     % back row empty
    11   650  420  210    1 1 1   31  55     % bright sun near the windows
    12   720  460  230    0 0 0   32  52     % lunch break -> classroom empty
    13   600  380  190    1 1 1   34  50     % hot afternoon -> thermal derating
    14   380  240  120    1 0 1   33  54     % middle zone empty
    15   150  100   50    1 1 1   30  70     % cloudy
    16    60   40   20    1 1 1   28  85 ];  % evening class, humid -> alert
N_slots = size(scenario, 1);
scen_hour = scenario(:,1);   % hour of the day of each row

% Time compression: one hour of the day is simulated as T_slot seconds of
% real circuit time. This is long enough for the converter to reach steady
% state, and short enough that the switching-level simulation stays fast.
T_slot        = 25e-3;   % [s] simulated time per hour
hours_per_slot = 1;      % real hours represented by one slot (used for Wh)
scen_t        = (0:N_slots-1)'*T_slot;

% Sensor signals as [time, values] matrices (used by the From Workspace blocks)
sens_ldr  = [scen_t, scenario(:,2:4)];
sens_pir  = [scen_t, scenario(:,5:7)];
sens_temp = [scen_t, scenario(:,8)];
sens_hum  = [scen_t, scenario(:,9)];

%% ---------------- 5. Simulation settings ----------------
dt_sim    = 1/fsw/100;          % [s] fixed time step (100 points per switching period)
T_stop    = N_slots*T_slot;     % [s] total simulated time
log_decim = 5;                  % keep every 5th sample in the log (1 us resolution)
