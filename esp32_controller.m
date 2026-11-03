function [duty, status] = esp32_controller(ldr, pir, temp, hum)
%ESP32_CONTROLLER  Lighting decision engine of the ESP32 (one call per control period).
%
%   [duty, status] = esp32_controller(ldr, pir, temp, hum)
%
%   Inputs
%     ldr  : 3x1 illuminance measured by each zone's LDR (daylight + own LEDs) [lux]
%     pir  : 3x1 presence detected by each zone's PIR sensor                   [0/1]
%     temp : room temperature from the DHT11                                    [C]
%     hum  : relative humidity from the DHT11                                   [%]
%   Outputs
%     duty   : 3x1 PWM duty cycle sent to each zone's MOSFET                    [0..1]
%     status : [thermal_derating_active; humidity_alert]                        [0/1]
%
%   Decision rules (closed-loop daylight harvesting + occupancy + thermal protection):
%     1. Zone empty (PIR = 0)  -> LEDs off.
%     2. Zone occupied         -> integral control of the measured desk illuminance:
%                                 duty += k_lux * (lux_target - ldr) / lux_led_max
%        The LDR sees the zone's own LEDs too, so a simple open-loop rule
%        ("duty = missing daylight") would make the lights flicker. The integral
%        loop settles where daylight + LED = target, even if the LED calibration
%        (lux_led_max) is only roughly known.
%     3. Room hotter than T_hot -> duty limited to duty_max_hot.
%
%   This rule set stands in for the "AI Decision (ML model)" block of the project.
%   A trained model can replace it if it keeps the same inputs and outputs, for
%   example by predicting the lux target or the occupancy. The function is
%   code-generation compatible, so the Simulink MATLAB Function block can call it.
%   Run "clear esp32_controller" to reset its memory (the plain script does this).
%#codegen

persistent u_int                 % integrator state of each zone (the duty "memory")
cfg = controller_config();
n = numel(ldr);
if isempty(u_int)
    u_int = zeros(n, 1);
end

hot = temp >= cfg.T_hot;
d_cap = 1;
if hot
    d_cap = cfg.duty_max_hot;
end

duty = zeros(n, 1);
for k = 1:n
    if pir(k) > 0.5
        u_int(k) = u_int(k) + cfg.k_lux*(cfg.lux_target - ldr(k))/cfg.lux_led_max;
        u_int(k) = min(max(u_int(k), 0), d_cap);     % clamp = anti-windup
    else
        u_int(k) = 0;
    end
    duty(k) = u_int(k);
end

status = [double(hot); double(hum >= cfg.H_high)];
end
