function [duty, status] = esp32_controller(ldr, pir, temp, hum)
%ESP32_CONTROLLER  Lighting decision engine of the ESP32 (one call per control period).
%
%   [duty, status] = esp32_controller(ldr, pir, temp, hum)
%
%   Inputs
%     ldr  : 3x1 daylight illuminance measured by each zone's LDR      [lux]
%     pir  : 3x1 presence detected by each zone's PIR sensor           [0/1]
%     temp : room temperature from the DHT11                           [C]
%     hum  : relative humidity from the DHT11                          [%]
%   Outputs
%     duty   : 3x1 PWM duty cycle sent to each zone's MOSFET           [0..1]
%     status : [thermal_derating_active; humidity_alert]               [0/1]
%
%   Decision rules (daylight harvesting + occupancy + thermal protection):
%     1. Zone empty (PIR = 0)        -> LEDs off.
%     2. Zone occupied               -> add only the light that daylight is missing:
%                                       duty = (lux_target - ldr) / lux_led_max
%     3. Very small duty (< duty_min) -> off, because daylight is already enough.
%     4. Room hotter than T_hot      -> duty limited to duty_max_hot.
%
%   This rule set stands in for the "AI Decision (ML model)" block of the project.
%   To use a trained model instead, keep the same inputs and outputs and replace
%   the body of the loop. The function is code-generation compatible, so the
%   Simulink MATLAB Function block can call it directly.
%#codegen

cfg = controller_config();
n = numel(ldr);
duty = zeros(n, 1);

hot = temp >= cfg.T_hot;

for k = 1:n
    if pir(k) > 0.5
        d = (cfg.lux_target - ldr(k)) / cfg.lux_led_max;
        d = min(max(d, 0), 1);
        if d < cfg.duty_min
            d = 0;
        end
        if hot
            d = min(d, cfg.duty_max_hot);
        end
        duty(k) = d;
    end
end

status = [double(hot); double(hum >= cfg.H_high)];
end
