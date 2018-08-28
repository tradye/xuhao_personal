%% weight identification of truck 20180821

%% read calibration table 
fid = fopen('/home/cidi-xh/xuhao/Codes/matlab/CIDI_control/Longitude/apollo_lon/final_resultwenbao.csv');%query time0.8，change the way of computing errors
cell_title = textscan(fid, '%s %s %s %s %s %s %s',1 , 'delimiter', ',');
cell_data  = textscan(fid, '%.9f %.9f %.9f %.9f %.9f %.9f %.9f','delimiter', ',');
fclose(fid);

data = cell2mat(cell_data);
control_u = data(:,1);
truck_v = data(:,2);
truck_a_1 = data(:,3);
truck_a_2 = data(:,4);
truck_a_3 = data(:,5);
truck_a_4 = data(:,6);
truck_a_5 = data(:,7);



%% longitude controller parameters
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

%% identification logic

%1. a mass 
% localization = interp3(-60,0,-1.7,length(control_u),control_u,truck_v,truck_a_1);

fid = fopen('/home/cidi-xh/xuhao/Codes/matlab/CIDI_control/Longitude/apollo_lon/two_datatest.csv');
cell_data1  = textscan(fid, '%.9f %.9f %.9f %.9f ','delimiter', ',');
fclose(fid);

data1 = cell2mat(cell_data1);
u = data1(:,1);
v = data1(:,2);
a = data1(:,3);%acceleration 
m = data1(:,4);% mass
V = [u,v,a,m];
[uu,vv,aa] = meshgrid(-60:.5:-50,0:.5:1,-3:.5:-1);
localization = interp3([-60 -50],[0.0 1],[-3.0385 -1.3662],V,u,v,a);
% localization = interp3(uu,vv,aa,V,u,v,a);
% vq = interp3(u,v,a,V,-60,0.8,-2.0);
% vq1 = interp3(u1,v1,a1,V,uu,vv,aa);
% sim('apollo_longitude',200);
