function y = esp32_controller_vec(u)
%ESP32_CONTROLLER_VEC  Vector wrapper of esp32_controller for the Interpreted MATLAB Function block.
%   u = [ldr(3); pir(3); temp; hum]   ->   y = [duty(3); status(2)]
%   Only used when the model is built with build_lighting_model('interpreted').
[duty, status] = esp32_controller(u(1:3), u(4:6), u(7), u(8));
y = [duty(:); status(:)];
end
