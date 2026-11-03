function build_lighting_model(varargin)
%BUILD_LIGHTING_MODEL  Creates the Simulink model ClassroomLighting.slx and opens it.
%
%   build_lighting_model                 build, save and open the model
%   build_lighting_model('run')          ... and run the simulation straight away
%   build_lighting_model('interpreted')  use an Interpreted MATLAB Function block for
%                                        the ESP32 decision engine (no C compiler needed)
%   Options can be combined: build_lighting_model('interpreted', 'run')
%
% The model is built from core Simulink blocks only (no Simscape needed), and its
% top level follows the project block diagram:
%
%   Input Layer (Sensors) -> Processing Layer (ESP32) -> Power & Lighting Layer
%        ^  LED light on LDRs        ^  vo feedback  <-------------'   |
%        '---------------------------+-------------------------------'   |
%                                    '-> Feedback Layer (Monitoring + Energy) <-'
%
% After the build, press Run in Simulink. When the simulation ends, the
% dashboard figures open automatically (lighting_dashboard.m, the
% communication layer). Parameters are in lighting_params.m.

opts       = lower(varargin);
run_now    = any(strcmp(opts, 'run'));
use_interp = any(strcmp(opts, 'interpreted'));

mdl  = 'ClassroomLighting';
here = fileparts(mfilename('fullpath'));
addpath(here);
evalin('base', 'lighting_params;');          % parameters must exist while the diagram is built
zone_phase = evalin('base', 'zone_phase');

if bdIsLoaded(mdl), close_system(mdl, 0); end
slx = fullfile(here, [mdl '.slx']);
if exist(slx, 'file'), delete(slx); end

fprintf('Building %s ...\n', mdl);
new_system(mdl);

%% ---------------- Model configuration ----------------
set_param(mdl, 'SolverType', 'Fixed-step');
set_param(mdl, 'Solver', 'ode3', 'FixedStep', 'dt_sim', 'StopTime', 'T_stop');
if ~trySet(mdl, 'EnableMultiTasking', 'off')
    trySet(mdl, 'SolverMode', 'SingleTasking');
end
trySet(mdl, 'ReturnWorkspaceOutputs', 'off');   % To Workspace data goes straight to the base workspace
trySet(mdl, 'SaveTime', 'off');
trySet(mdl, 'SaveOutput', 'off');
trySet(mdl, 'SaveState', 'off');
set_param(mdl, 'PreLoadFcn', 'try, lighting_params; catch, end');
set_param(mdl, 'InitFcn', ['try, addpath(fileparts(get_param(''' mdl ''', ''FileName''))); catch, end' ...
                           char(10) 'lighting_params;' char(10) 'clear esp32_controller;']);
set_param(mdl, 'StopFcn', 'lighting_dashboard;');

%% ================= 1. INPUT LAYER (SENSORS) =================
I = 'Input Layer (Sensors)';
s = [mdl '/' I];
add_block('built-in/SubSystem', s, 'Position', box(40, 120, 150, 200));   % input 1: zone currents
sens = {'Daylight on desks (lux)', 'sens_ldr',  'LDR'; ...
        'PIR presence',            'sens_pir',  'PIR'; ...
        'DHT11 temperature (C)',   'sens_temp', 'Temp'; ...
        'DHT11 humidity (pct)',    'sens_hum',  'Hum'};
for i = 1:4
    y = 40 + (i-1)*90;
    b = [s '/' sens{i,1}];
    add_block('built-in/FromWorkspace', b, 'Position', box(40, y, 150, 50), ...
              'VariableName', sens{i,2}, 'SampleTime', 'Ts_ctrl', 'Interpolate', 'off');
    trySet(b, 'OutputAfterFinalValue', 'Holding final value');
    add_block('built-in/Outport', [s '/' sens{i,3}], 'Position', box(760, y+17, 30, 16), 'Port', num2str(i));
    if i > 1, connect(s, [sens{i,1} '/1'], [sens{i,3} '/1']); end
end
% Each LDR also sees the light of its own LED zone. The ESP32 averages its ADC
% readings over each control period (oversampling), which removes the 5 kHz flicker:
%   LDR = daylight + (1/Ts_ctrl) * integral of LED light over the last period
add_block('built-in/Inport', [s '/i_zones'], 'Position', box(40, 420, 30, 16), 'Port', '1');
add_block('built-in/Gain', [s '/LED lux per ampere'], 'Position', box(110, 413, 110, 30), ...
          'Gain', 'lux_led_max/I_led_nom');
add_block('built-in/Integrator', [s '/LED light integral'], 'Position', box(250, 413, 40, 30));
add_block('built-in/ZeroOrderHold', [s '/ADC sample'], 'Position', box(320, 413, 40, 30), 'SampleTime', 'Ts_ctrl');
add_block('built-in/DiscreteTransferFcn', [s '/Average over Ts_ctrl'], 'Position', box(390, 405, 120, 46), ...
          'Numerator', '[1 -1]', 'Denominator', '[Ts_ctrl 0]', 'SampleTime', 'Ts_ctrl');
add_block('built-in/Sum', [s '/LDR reading'], 'Position', box(620, 45, 30, 40), 'Inputs', '++');
connect(s, 'i_zones/1', 'LED lux per ampere/1');
connect(s, 'LED lux per ampere/1', 'LED light integral/1');
connect(s, 'LED light integral/1', 'ADC sample/1');
connect(s, 'ADC sample/1', 'Average over Ts_ctrl/1');
connect(s, [sens{1,1} '/1'], 'LDR reading/1');
connect(s, 'Average over Ts_ctrl/1', 'LDR reading/2');
connect(s, 'LDR reading/1', 'LDR/1');

%% ================= 2. PROCESSING LAYER (ESP32) =================
Pr = 'Processing Layer (ESP32)';
s = [mdl '/' Pr];
add_block('built-in/SubSystem', s, 'Position', box(280, 100, 170, 260));
inN = {'LDR', 'PIR', 'Temp', 'Hum', 'Vo feedback'};
for i = 1:5
    add_block('built-in/Inport', [s '/' inN{i}], 'Position', box(30, 40 + (i-1)*70, 30, 16), 'Port', num2str(i));
end

% --- 2a. AI decision engine (daylight harvesting + occupancy + thermal derating) ---
dec = 'AI Decision Engine';
if ~use_interp
    add_block('simulink/User-Defined Functions/MATLAB Function', [s '/' dec], ...
              'Position', box(200, 40, 180, 230));
    setMatlabFunctionCode([s '/' dec], sprintf([ ...
        'function [duty, status] = ai_decision(ldr, pir, temp, hum)\n' ...
        '%% ESP32 lighting decision engine: daylight harvesting + occupancy + thermal derating.\n' ...
        '%% The rules are in esp32_controller.m. Replace them with your trained ML model if you like.\n' ...
        '[duty, status] = esp32_controller(ldr, pir, temp, hum);\n']));
    for i = 1:4, connect(s, [inN{i} '/1'], sprintf('%s/%d', dec, i)); end
    srcDuty = [dec '/1'];  srcStatus = [dec '/2'];
else
    add_block('built-in/Mux', [s '/Sensor mux'], 'Position', box(130, 40, 8, 230), 'Inputs', '4');
    try
        add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', [s '/' dec], ...
                  'Position', box(200, 135, 180, 40));
    catch
        add_block('built-in/MATLABFcn', [s '/' dec], 'Position', box(200, 135, 180, 40));
    end
    set_param([s '/' dec], 'MATLABFcn', 'esp32_controller_vec(u)', 'OutputDimensions', '5');
    add_block('built-in/Demux', [s '/Decision demux'], 'Position', box(420, 120, 8, 70), 'Outputs', '[3 2]');
    for i = 1:4, connect(s, [inN{i} '/1'], sprintf('Sensor mux/%d', i)); end
    connect(s, 'Sensor mux/1', [dec '/1']);
    connect(s, [dec '/1'], 'Decision demux/1');
    srcDuty = 'Decision demux/1';  srcStatus = 'Decision demux/2';
end

% --- 2b. Digital buck voltage controller: ADC sample per PWM period, PID, 50 kHz PWM ---
add_block('built-in/Clock', [s '/Clock'], 'Position', box(30, 420, 30, 30));
add_block('built-in/Fcn', [s '/Soft-start Vref'], 'Position', box(90, 420, 150, 30), ...
          'Expr', 'Vo_ref*((u(1)<t_ss)*u(1)/t_ss + (u(1)>=t_ss))');
add_block('built-in/ZeroOrderHold', [s '/Vref sample'], 'Position', box(260, 420, 40, 30), 'SampleTime', 'Ts_v');
add_block('built-in/ZeroOrderHold', [s '/ADC sample vo'], 'Position', box(130, 320, 40, 30), 'SampleTime', 'Ts_v');
add_block('built-in/Sum', [s '/Error e'], 'Position', box(320, 450, 30, 30), 'Inputs', '+-');
add_block('built-in/Gain', [s '/Feed-forward 1 over Vin'], 'Position', box(380, 380, 60, 30), 'Gain', '1/Vin');
add_block('built-in/Gain', [s '/Kp'], 'Position', box(380, 440, 60, 30), 'Gain', 'Kp_v');
add_block('built-in/Gain', [s '/Ki'], 'Position', box(380, 500, 60, 30), 'Gain', 'Ki_v');
add_block('built-in/DiscreteIntegrator', [s '/Integral (anti-windup)'], 'Position', box(470, 500, 50, 30), ...
          'gainval', '1', 'SampleTime', 'Ts_v', 'LimitOutput', 'on', ...
          'UpperSaturationLimit', 'xi_max', 'LowerSaturationLimit', 'xi_min');
add_block('built-in/DiscreteTransferFcn', [s '/Filtered derivative'], 'Position', box(330, 560, 170, 46), ...
          'Numerator', '[-Kd_v Kd_v]', 'Denominator', '[tau_d+Ts_v -tau_d]', 'SampleTime', 'Ts_v');
add_block('built-in/Sum', [s '/Duty sum'], 'Position', box(560, 400, 30, 200), 'Inputs', '++++');
add_block('built-in/Saturate', [s '/Duty limit'], 'Position', box(620, 485, 40, 30), ...
          'UpperLimit', 'd_max', 'LowerLimit', '0');
add_block('built-in/UnitDelay', [s '/Computation delay'], 'Position', box(680, 485, 40, 30), 'SampleTime', 'Ts_v');
add_block('built-in/Mux', [s '/Mux buck PWM'], 'Position', box(750, 440, 8, 60), 'Inputs', '2');
add_block('built-in/Fcn', [s '/Buck PWM 50 kHz'], 'Position', box(790, 455, 200, 30), ...
          'Expr', '(u(1)*fsw - floor(u(1)*fsw)) < u(2)');
connect(s, 'Clock/1', 'Soft-start Vref/1');
connect(s, 'Soft-start Vref/1', 'Vref sample/1');
connect(s, 'Vo feedback/1', 'ADC sample vo/1');
connect(s, 'Vref sample/1', 'Error e/1');
connect(s, 'ADC sample vo/1', 'Error e/2');
connect(s, 'Vref sample/1', 'Feed-forward 1 over Vin/1');
connect(s, 'Error e/1', 'Kp/1');
connect(s, 'Error e/1', 'Ki/1');
connect(s, 'Ki/1', 'Integral (anti-windup)/1');
connect(s, 'ADC sample vo/1', 'Filtered derivative/1');
connect(s, 'Feed-forward 1 over Vin/1', 'Duty sum/1');
connect(s, 'Kp/1', 'Duty sum/2');
connect(s, 'Integral (anti-windup)/1', 'Duty sum/3');
connect(s, 'Filtered derivative/1', 'Duty sum/4');
connect(s, 'Duty sum/1', 'Duty limit/1');
connect(s, 'Duty limit/1', 'Computation delay/1');
connect(s, 'Clock/1', 'Mux buck PWM/1');
connect(s, 'Computation delay/1', 'Mux buck PWM/2');
connect(s, 'Mux buck PWM/1', 'Buck PWM 50 kHz/1');

% --- 2c. Zone PWM (ESP32 LEDC, 5 kHz, phase shifted by 120 degrees) ---
add_block('built-in/Demux', [s '/Duty demux'], 'Position', box(560, 60, 8, 150), 'Outputs', '3');
connect(s, srcDuty, 'Duty demux/1');
add_block('built-in/Mux', [s '/Gate mux'], 'Position', box(1010, 60, 8, 150), 'Inputs', '3');
for k = 1:3
    y  = 40 + (k-1)*60;
    mx = sprintf('Mux Z%d', k);
    fc = sprintf('Zone %d PWM 5 kHz', k);
    ph = sprintf('%.6f', zone_phase(k));
    add_block('built-in/Mux', [s '/' mx], 'Position', box(640, y, 8, 40), 'Inputs', '2');
    add_block('built-in/Fcn', [s '/' fc], 'Position', box(700, y+5, 270, 30), ...
              'Expr', ['(u(1)*f_led + ' ph ' - floor(u(1)*f_led + ' ph ')) < u(2)']);
    connect(s, 'Clock/1', [mx '/1']);
    connect(s, sprintf('Duty demux/%d', k), [mx '/2']);
    connect(s, [mx '/1'], [fc '/1']);
    connect(s, [fc '/1'], sprintf('Gate mux/%d', k));
end

outN = {'q_buck', 'g_zones', 'duty_zones', 'd_buck', 'status'};
outY = [470, 125, 250, 300, 350];
for i = 1:5
    add_block('built-in/Outport', [s '/' outN{i}], 'Position', box(1090, outY(i), 30, 16), 'Port', num2str(i));
end
connect(s, 'Buck PWM 50 kHz/1', 'q_buck/1');
connect(s, 'Gate mux/1', 'g_zones/1');
connect(s, srcDuty, 'duty_zones/1');
connect(s, 'Computation delay/1', 'd_buck/1');
connect(s, srcStatus, 'status/1');

%% ================= 3. POWER & LIGHTING LAYER =================
Pw = 'Power & Lighting Layer';
s = [mdl '/' Pw];
add_block('built-in/SubSystem', s, 'Position', box(560, 100, 170, 160));
add_block('built-in/Inport', [s '/q_buck'],  'Position', box(30, 60, 30, 16), 'Port', '1');
add_block('built-in/Inport', [s '/g_zones'], 'Position', box(30, 300, 30, 16), 'Port', '2');

% --- 3a. Custom buck converter: MOSFET + Schottky diode + L + C ---
B  = 'Buck Converter 12V-5V';
bs = [s '/' B];
add_block('built-in/SubSystem', bs, 'Position', box(150, 40, 170, 80));
add_block('built-in/Inport', [bs '/q'],      'Position', box(30, 50, 30, 16),  'Port', '1');
add_block('built-in/Inport', [bs '/i_load'], 'Position', box(30, 230, 30, 16), 'Port', '2');
add_block('built-in/Mux', [bs '/Mux'], 'Position', box(120, 30, 8, 110), 'Inputs', '3');
add_block('built-in/Fcn', [bs '/Inductor voltage vL'], 'Position', box(170, 70, 330, 30), ...
          'Expr', 'u(1)*Vin - (1-u(1))*Vd - u(2)*(rL + u(1)*Rds_on) - u(3)');
add_block('built-in/Gain', [bs '/1 over L'], 'Position', box(540, 70, 60, 30), 'Gain', '1/L_buck');
add_block('built-in/Integrator', [bs '/Inductor current'], 'Position', box(630, 70, 40, 30), ...
          'InitialCondition', '0', 'LimitOutput', 'on', 'LowerSaturationLimit', '0', 'UpperSaturationLimit', 'inf');
add_block('built-in/Sum', [bs '/Capacitor current'], 'Position', box(540, 210, 30, 40), 'Inputs', '+-');
add_block('built-in/Gain', [bs '/1 over C'], 'Position', box(600, 215, 60, 30), 'Gain', '1/C_buck');
add_block('built-in/Integrator', [bs '/Capacitor voltage'], 'Position', box(690, 215, 40, 30), 'InitialCondition', '0');
add_block('built-in/Gain', [bs '/ESR'], 'Position', box(600, 290, 60, 30), 'Gain', 'ESR_C');
add_block('built-in/Sum', [bs '/Terminal voltage vo'], 'Position', box(760, 215, 30, 60), 'Inputs', '++');
add_block('built-in/Outport', [bs '/vo'], 'Position', box(850, 237, 30, 16), 'Port', '1');
add_block('built-in/Outport', [bs '/iL'], 'Position', box(850, 77, 30, 16),  'Port', '2');
add_block('built-in/Outport', [bs '/vc'], 'Position', box(850, 330, 30, 16), 'Port', '3');
connect(bs, 'q/1', 'Mux/1');
connect(bs, 'Inductor current/1', 'Mux/2');
connect(bs, 'Terminal voltage vo/1', 'Mux/3');
connect(bs, 'Mux/1', 'Inductor voltage vL/1');
connect(bs, 'Inductor voltage vL/1', '1 over L/1');
connect(bs, '1 over L/1', 'Inductor current/1');
connect(bs, 'Inductor current/1', 'Capacitor current/1');
connect(bs, 'i_load/1', 'Capacitor current/2');
connect(bs, 'Capacitor current/1', '1 over C/1');
connect(bs, '1 over C/1', 'Capacitor voltage/1');
connect(bs, 'Capacitor current/1', 'ESR/1');
connect(bs, 'Capacitor voltage/1', 'Terminal voltage vo/1');
connect(bs, 'ESR/1', 'Terminal voltage vo/2');
connect(bs, 'Terminal voltage vo/1', 'vo/1');
connect(bs, 'Capacitor voltage/1', 'vc/1');
connect(bs, 'Inductor current/1', 'iL/1');

% --- 3b. Three zones: MOSFET low-side switch + 5 V LED strip ---
add_block('built-in/Demux', [s '/Gate demux'], 'Position', box(120, 260, 8, 120), 'Outputs', '3');
add_block('built-in/Mux', [s '/Zone current mux'], 'Position', box(560, 260, 8, 120), 'Inputs', '3');
add_block('built-in/Constant', [s '/ESP32 + sensors'], 'Position', box(330, 420, 80, 30), 'Value', 'I_esp32');
add_block('built-in/Sum', [s '/Total load current'], 'Position', box(560, 410, 30, 80), 'Inputs', '++++');
connect(s, 'q_buck/1', [B '/1']);
connect(s, 'g_zones/1', 'Gate demux/1');
for k = 1:3
    y  = 250 + (k-1)*50;
    mx = sprintf('Mux Z%d', k);
    fz = sprintf('Zone %d MOSFET + LED strip', k);
    add_block('built-in/Mux', [s '/' mx], 'Position', box(200, y, 8, 40), 'Inputs', '2');
    add_block('built-in/Fcn', [s '/' fz], 'Position', box(250, y+5, 260, 30), ...
              'Expr', 'u(1)*(u(2)>Vf_led)*(u(2)-Vf_led)/R_led');
    connect(s, sprintf('Gate demux/%d', k), [mx '/1']);
    connect(s, [B '/3'], [mx '/2']);     % capacitor voltage (avoids an algebraic loop via the ESR)
    connect(s, [mx '/1'], [fz '/1']);
    connect(s, [fz '/1'], sprintf('Zone current mux/%d', k));
    connect(s, [fz '/1'], sprintf('Total load current/%d', k));
end
connect(s, 'ESP32 + sensors/1', 'Total load current/4');
connect(s, 'Total load current/1', [B '/2']);
outN = {'vo', 'iL', 'i_zones', 'i_load'};
outY = [57, 97, 312, 442];
for i = 1:4
    add_block('built-in/Outport', [s '/' outN{i}], 'Position', box(660, outY(i), 30, 16), 'Port', num2str(i));
end
connect(s, [B '/1'], 'vo/1');
connect(s, [B '/2'], 'iL/1');
connect(s, 'Zone current mux/1', 'i_zones/1');
connect(s, 'Total load current/1', 'i_load/1');

%% ================= 4. FEEDBACK LAYER (MONITORING + ENERGY) =================
F = 'Feedback Layer (Monitoring + Energy)';
s = [mdl '/' F];
add_block('built-in/SubSystem', s, 'Position', box(840, 80, 190, 300));
inN = {'vo', 'iL', 'q_buck', 'd_buck', 'i_zones', 'g_zones', 'duty_zones', 'status'};
for i = 1:8
    add_block('built-in/Inport', [s '/' inN{i}], 'Position', box(30, 40 + (i-1)*60, 30, 16), 'Port', num2str(i));
end
add_block('built-in/Clock', [s '/Clock'], 'Position', box(30, 540, 30, 30));
% energy meters
add_block('built-in/Mux', [s '/Mux Pin'], 'Position', box(150, 600, 8, 50), 'Inputs', '2');
add_block('built-in/Fcn', [s '/Input power'], 'Position', box(190, 610, 120, 30), 'Expr', 'Vin*u(1)*u(2)');
add_block('built-in/Integrator', [s '/E_in'], 'Position', box(340, 610, 40, 30));
add_block('built-in/Product', [s '/Zone power'], 'Position', box(190, 680, 40, 40), 'Inputs', '2');
add_block('built-in/Integrator', [s '/E_zones'], 'Position', box(340, 685, 40, 30));
add_block('built-in/Gain', [s '/ESP32 power'], 'Position', box(190, 750, 60, 30), 'Gain', 'I_esp32');
add_block('built-in/Integrator', [s '/E_esp32'], 'Position', box(340, 750, 40, 30));
connect(s, 'q_buck/1', 'Mux Pin/1');
connect(s, 'iL/1', 'Mux Pin/2');
connect(s, 'Mux Pin/1', 'Input power/1');
connect(s, 'Input power/1', 'E_in/1');
connect(s, 'vo/1', 'Zone power/1');
connect(s, 'i_zones/1', 'Zone power/2');
connect(s, 'Zone power/1', 'E_zones/1');
connect(s, 'vo/1', 'ESP32 power/1');
connect(s, 'ESP32 power/1', 'E_esp32/1');
% data log for the dashboard (same columns as run_lighting_simulation.m)
add_block('built-in/Mux', [s '/Log mux'], 'Position', box(520, 40, 10, 760), 'Inputs', '12');
add_block('built-in/ToWorkspace', [s '/To Workspace sim_log'], 'Position', box(580, 400, 120, 40), ...
          'VariableName', 'sim_log', 'SaveFormat', 'Array', 'MaxDataPoints', 'inf', ...
          'Decimation', 'log_decim', 'SampleTime', '-1');
logSrc = {'Clock/1', 'vo/1', 'iL/1', 'q_buck/1', 'd_buck/1', 'i_zones/1', 'g_zones/1', ...
          'duty_zones/1', 'E_in/1', 'E_zones/1', 'E_esp32/1', 'status/1'};
for i = 1:numel(logSrc), connect(s, logSrc{i}, sprintf('Log mux/%d', i)); end
connect(s, 'Log mux/1', 'To Workspace sim_log/1');
% scopes
add_block('built-in/Scope', [s '/Scope Buck Vo and iL'], 'Position', box(250, 40, 40, 50));
set_param([s '/Scope Buck Vo and iL'], 'NumInputPorts', '2');
add_block('built-in/Scope', [s '/Scope Zone LED currents'], 'Position', box(250, 270, 40, 40));
add_block('built-in/Scope', [s '/Scope Zone duty from ESP32'], 'Position', box(250, 400, 40, 40));
connect(s, 'vo/1', 'Scope Buck Vo and iL/1');
connect(s, 'iL/1', 'Scope Buck Vo and iL/2');
connect(s, 'i_zones/1', 'Scope Zone LED currents/1');
connect(s, 'duty_zones/1', 'Scope Zone duty from ESP32/1');

%% ================= Top-level wiring =================
for i = 1:4, connect(mdl, sprintf('%s/%d', I, i), sprintf('%s/%d', Pr, i)); end
connect(mdl, [Pw '/1'], [Pr '/5']);       % output voltage feedback to the ESP32
connect(mdl, [Pw '/3'], [I '/1']);        % LED light reaches the LDRs (closed lux loop)
connect(mdl, [Pr '/1'], [Pw '/1']);       % buck MOSFET gate
connect(mdl, [Pr '/2'], [Pw '/2']);       % zone MOSFET gates
connect(mdl, [Pw '/1'], [F '/1']);
connect(mdl, [Pw '/2'], [F '/2']);
connect(mdl, [Pr '/1'], [F '/3']);
connect(mdl, [Pr '/4'], [F '/4']);
connect(mdl, [Pw '/3'], [F '/5']);
connect(mdl, [Pr '/2'], [F '/6']);
connect(mdl, [Pr '/3'], [F '/7']);
connect(mdl, [Pr '/5'], [F '/8']);
add_block('built-in/Terminator', [mdl '/Terminator'], 'Position', box(790, 225, 20, 20));
connect(mdl, [Pw '/4'], 'Terminator/1');

try
    note = Simulink.Annotation([mdl '/Intelligent Multi-Zone Classroom Lighting System - press Run. ' ...
        'Parameters: lighting_params.m | decision rules: esp32_controller.m | ' ...
        'Communication layer: dashboard figures open when the simulation ends (lighting_dashboard.m)']);
    note.Position = [40 20];
catch
end

save_system(mdl, slx);
open_system(mdl);
fprintf('Saved %s\nPress Run in Simulink (takes about 0.5-2 min), or type: sim(''%s'')\n', slx, mdl);

if run_now
    sim(mdl);
end
end

%% ---------------- helpers ----------------
function p = box(x, y, w, h)
p = [x, y, x + w, y + h];
end

function connect(sys, src, dst)
try
    add_line(sys, src, dst, 'autorouting', 'on');
catch
    add_line(sys, src, dst);
end
end

function ok = trySet(obj, name, value)
try
    set_param(obj, name, value);
    ok = true;
catch
    ok = false;
end
end

function setMatlabFunctionCode(blk, code)
% Works in new releases (MATLABFunctionConfiguration) and older ones (Stateflow API).
try
    cfg = get_param(blk, 'MATLABFunctionConfiguration');
    cfg.FunctionScript = code;
    return
catch
end
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', blk);
ch.Script = code;
end
