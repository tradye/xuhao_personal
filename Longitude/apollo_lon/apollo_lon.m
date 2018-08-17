%% apollo longitude controller: P/PI controller 20180814

%%  Apollo longitude parameters    %%
%   ts: 0.01
%   brake_deadzone: 15
%   throttle_deadzone: 30
%   speed_controller_input_limit: 2.0
%   station_error_limit: 2.0
%   preview_window: 100.0
%   standstill_acceleration: -1.7
%   station_pid_conf {
%     integrator_enable: false
%     integrator_saturation_level: 0.3
%     kp: 0.2
%     ki: 0.0
%     kd: 0.0
%   }
%   low_speed_pid_conf {
%     integrator_enable: true
%     integrator_saturation_level: 0.3
%     kp: 4.0
%     ki: 0.4
%     kd: 0.0
%   }
%   high_speed_pid_conf {
%     integrator_enable: true
%     integrator_saturation_level: 0.3
%     kp: 1.0
%     ki: 0.3
%     kd: 0.0

%%
station_error_limit = 2.0; %unit: m
speed_controller_input_limit = 2.0; %unit: m/s
station_integral_enable = false;
station_integrator_saturation_level = 0.3;
station_Kp = 0.2;

switch_speed = 2.0; % m/s

low_speed_Kp = 4.0;
low_speed_Ki = 0.4;
low_speed_integrator_enable = true;
low_speed_integrator_saturation_level = 0.3;

high_speed_Kp = 1.0;
high_speed_Ki = 0.3;
high_speed_integrator_enable = true;
high_speed_integrator_saturation_level = 0.3;

ref_speed = 10;%m/s
ref_station = 10; % m

sim('apollo_longitude',200);























