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

%% identification logic

%1. a mass 








