clear
global kDefaultSStep ;
global kDefaultSMax ;
global kDefaultTStep ;
global kDefaultTMax ;
global kSmoothCount ;
global kFallbackSpeedPointNum;
global kTsGraphSStep ;
global kTsGraphTStep ;
global kTsGraphTMax ;
global soft_centric_accel_limit;
global hard_centric_accel_limit;
global kappa_preview;
% global kKappaSmooth;
global kKappaPreviewBackwardRatio;
global default_cruise_speed;
global max_accel;
global max_decel;
global max_jerk;
global max_djerk;
global preferred_accel;
global preferred_decel;
global preferred_jerk;
global preferred_djerk;


kDefaultSStep = 1.0;
kDefaultSMax = 2.0;
kDefaultTStep = 0.1;
kDefaultTMax = 15.0;
kSmoothCount = 100;
kFallbackSpeedPointNum = 4;
kTsGraphSStep = 0.1;
kTsGraphTStep = 0.1;
kTsGraphTMax = 10.0;
% kKappaSmooth = 
kKappaPreviewBackwardRatio = 0.25;

soft_centric_accel_limit = 0.35;%configurable para
hard_centric_accel_limit = 1.0;% configurable para
kappa_preview = 15.0; %configurable para
default_cruise_speed = 8.28;
preferred_accel = 0.4;
preferred_decel = 0.7;
preferred_jerk = 0.5;
preferred_djerk = 3.0;
max_accel = 1.0;
max_decel = 3.5;
max_jerk =  3.0;
max_djerk = 100.0;

% navi_speed_decider_config {
%         preferred_accel: 0.4
%         preferred_decel: 0.7
%         preferred_jerk: 0.5
%         preferred_djerk: 3.0
%         max_accel: 1.0
%         max_decel: 3.5
%         max_jerk: 3.0
%         max_djerk: 100.0
%         enable_compress_t: true
%         cap_saved_ratio: 0.0
%         obstacle_buffer: 2.5
%         safe_distance_base: 1.5
%         safe_distance_ratio: 5.0
%         estop_range: 0.0
%         estop_speed_threshold: 0.0
%         soft_centric_accel_limit: 0.35
%         hard_centric_accel_limit: 1.0
%         enable_safe_path: false
%         enable_planning_start_point: true
%         kappa_preview: 15.0
%     }
start_v =5;
start_a = 0;
start_da = 0;

ts_graph_ = NaviSpeedTsGraph(0.1,0,0);

filepath = 'D:\CIDI\D_file\CIDI改装车辆\中车电动公交\trucksim_simulink\scripts\control_path_0904.OK.txt.smoothed';

path_points = readPathPoints(filepath);
% speed_data = MakeSpeedDecision(1, start_v , start_a , start_da , path_points , 1, 1, 1);

% message PathPoint {
%   // coordinates
%   optional double x = 1;
%   optional double y = 2;
%   optional double z = 3;
% 
%   // direction on the x-y plane
%   optional double theta = 4;
%   // curvature on the x-y planning
%   optional double kappa = 5;
%   // accumulated distance from beginning of the path
%   optional double s = 6;
% 
%   // derivative of kappa w.r.t s.
%   optional double dkappa = 7;
%   // derivative of derivative of kappa w.r.t s.
%   optional double ddkappa = 8;
%   // The lane ID where the path point is on
%   optional string lane_id = 9;
% }

start_s =  path_points(1).s;
end_s = path_points(end).s ;
planning_length = end_s - start_s ;
compress_t = false;
cap_saved_ratio = 0.0;

fprintf('start to make speed decision ,start_v : %f \n', start_v);
fprintf('start_a : %f \n', start_a);
fprintf('start_da : %f \n', start_da);
fprintf('start_s : %f \n', start_s);
fprintf('planning_length : %f \n', planning_length);

start_v = max(0 , start_v);

if planning_length > kTsGraphSStep
    s_step = kTsGraphSStep;
else
     s_step = planning_length / kFallbackSpeedPointNum;
end

ts_graph_.Reset(s_step, 150, kTsGraphTStep, kTsGraphTMax, compress_t, cap_saved_ratio,start_v,start_a, start_da);

%     ret = AddPerceptionRangeConstraints();
ret = AddCentricAccelerationConstraints(path_points,ts_graph_);

if ret ~= true
    fprintf('add t-s constraints base on centric acceleration failed');
    return;
end

ret = AddConfiguredConstraints(1,start_s,end_s ,ts_graph_);
if ret ~= true
    fprintf('add t-s constraints base on configs  failed');
    return;
end

if  ts_graph_.Solve() ~= true
    fprintf('Solve speed data failed');
    return;
end
speed_data = ts_graph_.output_;

% end MakeSpeedDecision function

function status = AddCentricAccelerationConstraints(path_points,ts_graph_)
    global soft_centric_accel_limit;
    global hard_centric_accel_limit;
    global kappa_preview;
%     global kTsGraphSStep;
%     global kKappaSmooth;
    global kKappaPreviewBackwardRatio;
    if length(path_points) < 2
        fprintf('too few path points');
        status = false;
        return;
    end
    
    max_kappa = 0.0;
    max_kappa_v = 10000;
    preffered_kappa_v = 10000;
    max_kappa_s = 0.0;
    bs = path_points(1).s ;
    
    s_table = zeros(length(path_points) , 1);
    v_max_table = zeros(length(path_points) , 1);
    v_preffered_table = zeros(length(path_points) , 1);
    
    for i =2:1:length(path_points)
        prev = path_points(1);
        cur = path_points(i);
        start_s = prev.s - bs;
        start_s = max(0 , start_s);
        end_s = cur.s - bs;
        end_s = max(0 , end_s);
        start_k =  prev.kappa;
        end_k = cur.kappa;
        kappa = abs((start_k + end_k ) / 2);
        
        v_preffered = sqrt(soft_centric_accel_limit / kappa);%preffered speed compute every points
        v_max = sqrt(hard_centric_accel_limit / kappa); %max speed compute every points
%         fprintf('v_preffered: %f \n' , v_preffered);
%         fprintf('v_max: %f \n' , v_max);
        
        start_s = max( (start_s - kappa_preview * ( 1 - kKappaPreviewBackwardRatio)) , 0);
        end_s = end_s + kappa_preview *  kKappaPreviewBackwardRatio ;
%         fprintf('start_s: %f \n' , start_s);
%         fprintf('end_s: %f \n' , end_s);
        
        constraints.v_max = v_max;
        constraints.v_preffered = v_preffered;
        constraints.a_max = 0;
        constraints.a_preffered = 0;
        constraints.b_max = 0;
        constraints.b_preffered = 0;
        constraints.da_max = 0;
        constraints.da_preffered = 0;
        constraints.dda_max = 0;
        constraints.dda_preffered = 0;

        ts_graph_.UpdateRangeConstraints(start_s , end_s , constraints);
%         s_table(i - 1) = end_s;
%         v_max_table(i - 1) = v_max;
%         v_preffered_table(i - 1) = v_preffered; 
%         if kappa > max_kappa
%             max_kappa = kappa;
%             max_kappa_v = v_max;
%             preffered_kappa_v = v_preffered;
%             max_kappa_s = start_s;
%         end
        
    end
        %kappa preview
%     for i = 1:1:length(s_table)
%         len = kappa_preview / kTsGraphSStep;
%         for j = i:1:length(s_table)
%             v_preffered_table(i) = min(  v_preffered_table(j) / ( 1.0 - sqrt( (j - i) / len  )  ) , v_preffered_table(i)    );
%         end
%     end

%     start_s = 0.0;
%     for i = 1:1:length(s_table)
%         end_s_tmp = s_table(i);
%         v_max_tmp = v_max_table(i);
%         v_preffered_tmp = v_preffered_table(i);
%         %NaviSpeedTsConstraints constraints;
%         constraints.v_max = v_max_tmp;
%         constraints.v_preffered = v_preffered_tmp;
%         constraints.a_max = 0;
%         constraints.a_preffered = 0;
%         constraints.b_max = 0;
%         constraints.b_preffered = 0;
%         constraints.da_max = 0;
%         constraints.da_preffered = 0;
%         constraints.dda_max = 0;
%         constraints.dda_preffered = 0;
%         fprintf('constraints.v_preffered: %f \n' , constraints.v_preffered);
%         fprintf('constraints.v_max: %f \n' , constraints.v_max);
%         ts_graph_.UpdateRangeConstraints(start_s , end_s + kKappaSmooth, constraints);
%         start_s = end_s_tmp;
%         
%     end
    fprintf('Add speed limit for centric acceleration with kappa: %f \n' , max_kappa);
    status = true;

end% end AddCentricAccelerationConstraints function

function status = AddConfiguredConstraints(reference_line_info , start_s , end_s,ts_graph)
    global max_accel;
    global preferred_accel;
    global max_decel;
    global preferred_decel;
    global preferred_jerk;
    global max_jerk;
    global kTsGraphSStep;
    global default_cruise_speed;
    
    constraints.v_max = default_cruise_speed;
    constraints.v_preffered = constraints.v_max;
    constraints.a_max = max_accel;
    constraints.a_preffered = preferred_accel;
    constraints.b_max = max_decel;
    constraints.b_preffered = preferred_decel;
    constraints.da_preffered = preferred_jerk;
    constraints.da_max = max_jerk;
    constraints.dda_max = 2.1475e+09;%max number in C++ file
    constraints.dda_preffered = 2.1475e+09;%max number in C++ file
    
    for s = start_s : kTsGraphSStep : end_s
        speed_saved = constraints.v_preffered;
        v_limit = default_cruise_speed;%to be using the comupting result v_limit based on reference_line_info here
        constraints.v_preffered = min(constraints.v_preffered, v_limit);
        ts_graph.UpdateRangeConstraints(s-start_s , (s - start_s + kTsGraphSStep) , constraints);
        constraints.v_preffered = speed_saved;
    end
    status = true;

end% end AddConfiguredConstraints function

function path_point_struct = MakePathPointsStructArray(size)
    num = size;
    field1 = 'x'; value1 = zeros(num,1);
    field2 = 'y'; value2 = zeros(num,1);
    field3 = 'z'; value3 = zeros(num,1);
    field4 = 'theta'; value4 = zeros(num,1);
    field5 = 'kappa'; value5 = zeros(num,1);
    field6 = 's'; value6 = zeros(num,1);
    field7 = 'dkappa'; value7 = zeros(num,1);
    field8 = 'ddkappa'; value8 = zeros(num,1);
    field9 = 'lane_id'; value9 = zeros(num,1);

     for i = 1:1:num
        path_point_struct(i).x= value1(i);
        path_point_struct(i).y= value2(i);
        path_point_struct(i).z= value3(i);
        path_point_struct(i).theta= value4(i);
        path_point_struct(i).kappa= value5(i);
        path_point_struct(i).s= value6(i);
        path_point_struct(i).dkappa= value7(i);
        path_point_struct(i).ddkappa= value8(i);
        path_point_struct(i).lane_id= value9(i);
    end
end% end MakeNaviSpeedTsPointsStructArray function

function path_points = readPathPoints(filepath)
%     filename = 'D:\CIDI\D_file\CIDI改装车辆\中车电动公交\trucksim_simulink\scripts\control_path_0904.OK.txt.smoothed';
    data1 = fopen(filepath);
    A = textscan(data1,'%s');
    fclose(data1);
    num = length(A{1,1});
    delta_num = 9;
    Kappa = [];
    S = [];
    Theta = [];
    X = [];
    Y = [];
    Dkappa = [];
    %use sscanf should be quicker
%     s = A{1,1}{4:delta_num:num,1};
    for i = 1:1:(num/delta_num)
        for j = 1:1:delta_num
            a1 = A{1,1}{(i-1)*delta_num + 1,1};
            a2 = A{1,1}{(i-1)*delta_num + 2 ,1};
            a3 = A{1,1}{(i-1)*delta_num + 3,1};
            a4 = A{1,1}{(i-1)*delta_num + 4,1};
            a5 = A{1,1}{(i-1)*delta_num + 5,1};
            a6 = A{1,1}{(i-1)*delta_num + 6,1};
            a7 = A{1,1}{(i-1)*delta_num + 7,1};
            a8 = A{1,1}{(i-1)*delta_num + 8,1};
            a9 = A{1,1}{(i-1)*delta_num + 9,1};
            a7_1 = a7(1,5:end);
            a8_1 = a8(1,5:end);
            a9_1 = a9(1,10:end-1);
    %         a = [a;a1];
    %         kappa = str2double(a2);
    %         kappa_table = [kappa_table;kappa];
        end
        kappa = str2double(a2);
        s = str2double(a4);
        theta = str2double(a6);
        x = str2double(a7_1);
        y = str2double(a8_1);
        dkappa = str2double(a9_1);

        Kappa = [Kappa;kappa];
        S = [S;s];
        Theta = [Theta;theta];
        X = [X;x];
        Y = [Y;y];
        Dkappa = [Dkappa;dkappa];

    end
    size_path_points = 200;
    path_points = MakePathPointsStructArray(size_path_points);
    for i = 1:1:length(path_points)
        path_points(i).x = X(i);
        path_points(i).y = Y(i);
        path_points(i).z = 0;
        path_points(i).theta = Theta(i);
        path_points(i).kappa = Kappa(i);
        path_points(i).s = S(i);
        path_points(i).dkappa = Dkappa(i);
        path_points(i).ddkappa = 0;
        path_points(i).lane_id = 0;
    end
end







