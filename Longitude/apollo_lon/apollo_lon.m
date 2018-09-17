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
s=tf('s');% s in frequency domain,for transfer function
station_error_limit = 5.0; %unit: m 2.0
speed_controller_input_limit = 2.0; %unit: m/s 2.0
station_integral_enable = false;
station_integrator_saturation_level = 0.3;
station_Kp = 0.2;
station_Ki = 0.00;%0.01

switch_speed = 2.0; % m/s

low_speed_Kp = 4.0;
low_speed_Ki = 0.4;
low_speed_integrator_enable = true;
low_speed_integrator_saturation_level = 0.3;

high_speed_Kp = 1.0;
high_speed_Ki = 0.3;%0.3
high_speed_integrator_enable = true;
high_speed_integrator_saturation_level = 0.3;

ref_speed = 10;%m/s
ref_station = 10; % m

T_btno = 1;
T_btde = 0.5;
T_delay = 0.5;
%% transfer function of longitude controller
G_speedpid = high_speed_Kp * (1 + high_speed_Ki / s);%speed pid tf
G_throttlebranke = tf(T_btno,[T_btde 1]) * exp(-T_delay * s);%throttle/brake module tf
G_speed_open = G_speedpid * G_throttlebranke / s;
G_speed_close = G_speed_open / (1 + G_speed_open);
G_longitude_open = G_speed_close * station_Kp * (1 + station_Ki / s) / s;
G_longitude_close = G_longitude_open / (1 + G_longitude_open);
% bode(G_longitude_open);
% hold on,grid on
margin(minreal(G_longitude_open));hold on,grid on







% sim('apollo_longitude',200);
%% read plan points
 readPlanPoints();





















