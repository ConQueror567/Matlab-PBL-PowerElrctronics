function cfg = controller_config()
%CONTROLLER_CONFIG  Settings of the ESP32 lighting decision engine.
%   Shared by esp32_controller.m (the "firmware") and lighting_params.m (the plant),
%   so the controller and the simulated room use the same calibration.
%#codegen

cfg.lux_target  = 500;  % [lux] target desk illuminance (EN 12464-1: 500 lux for classrooms)
cfg.lux_led_max = 600;  % [lux] desk illuminance one zone gives at 100 % duty (calibration)
cfg.duty_min    = 0.05; % below this, an occupied zone is switched fully off (daylight is enough)
cfg.T_hot       = 32;   % [C] above this room temperature, thermal derating is active
cfg.duty_max_hot = 0.80;% max duty while derating (less LED heat and driver stress)
cfg.H_high      = 80;   % [%] humidity alert threshold (sent to the dashboard)
end
