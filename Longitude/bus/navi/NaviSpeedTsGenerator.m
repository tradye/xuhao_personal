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
global kKappaSmooth;
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

soft_centric_accel_limit = 1.4;%configurable para
hard_centric_accel_limit = 1.4;% configurable para
kappa_preview = 0.12; %configurable para
 
 
s_step = 1;
s_max  = 100;
t_step = 0.1;
t_max = 8;
compress_t = true;
cap_saved_ratio = 1;
start_v = 0;
start_a = 0;
start_da = 0;
% s_step = planning_length > kTsGraphSStep
%                     ? kTsGraphSStep
%                     : planning_length / kFallbackSpeedPointNum;
end_s = 200;
start_s = 0;
vehicle_v = 0;
start_v = max(0,start_v);
planning_length = end_s - start_s; 
if(planning_length > kTsGraphSStep)
    s_step = kTsGraphSStep;
else
    s_step = planning_length / kFallbackSpeedPointNum;
end

ts_graph_ = NaviSpeedTsGraph(0.1,0,0);
% ts_graph_.Reset(s_step, planning_length, kTsGraphTStep, kTsGraphTMax, compress_t, cap_saved_ratio,start_v,start_a, start_da);
% ts_graph_.Solve();
% length(ts_graph.t_)
% ts_graph.t_
% ts_graph_.Solve();
size_path_points = 200;
path_points = MakePathPointsStructArray(size_path_points);
s_test = linspace(0,200,201);
for i = 1:1:size_path_points
    path_points(i).s = s_test(i);
end
speed_data = MakeSpeedDecision(1, start_v , start_a , start_da , path_points , 1, 1, 1,ts_graph_);

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

function speed_data = MakeSpeedDecision(reference_line_info , start_v , start_a , start_da , path_points , obstacles, find_obstacle, speed_data,ts_graph_)

%     global kDefaultSStep ;
%     global kDefaultSMax ;
%     global kDefaultTStep ;
%     global kDefaultTMax ;
%     global kSmoothCount ;
    global kFallbackSpeedPointNum;
    global kTsGraphSStep ;
    global kTsGraphTStep ;
    global kTsGraphTMax ;
    start_s =  path_points(1).s;
    end_s = path_points(end).s ;
    planning_length = end_s - start_s ;
    compress_t = true;
    cap_saved_ratio = 1.0;
    
    fprintf('start to make speed decision ,start_v : %f \n', start_v);
    fprintf(' start_a : %f \n', start_a);
    fprintf('start_da : %f \n', start_da);
    fprintf('start_s : %f \n', start_s);
    fprintf('planning_length : %f \n', planning_length);
    
    start_v = max(0 , start_v);
    s_step = 0;
    if planning_length > kTsGraphSStep
        s_step = kTsGraphSStep;
    else
         s_step = planning_length / kFallbackSpeedPointNum;
    end
    
    ts_graph_.Reset(s_step, planning_length, kTsGraphTStep, kTsGraphTMax, compress_t, cap_saved_ratio,start_v,start_a, start_da);

%     ret = AddPerceptionRangeConstraints();
    ret = AddCentricAccelerationConstraints(path_points,ts_graph_);
    
    if ret ~= true
        fprintf('add t-s constraints base on centruc acceleration failed');
        return;
    end
    
    if  ts_graph_.Solve() ~= true
        fprintf('Solve speed data failed');
        return;
    end
    speed_data = ts_graph_.output_;

end

function status = AddCentricAccelerationConstraints(path_points,ts_graph_)
    global soft_centric_accel_limit;
    global hard_centric_accel_limit;
    global kappa_preview;
    global kTsGraphSStep;
    global kKappaSmooth;
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
        
        s_table(i - 1) = end_s;
        v_max_table(i - 1) = v_max;
        v_preffered_table(i - 1) = v_preffered; 
        if kappa > max_kappa
            max_kappa = kappa;
            max_kappa_v = v_max;
            preffered_kappa_v = v_preffered;
            max_kappa_s = start_s;
        end
        
    end
        %kappa preview
    for i = 1:1:length(s_table)
        len = kappa_preview / kTsGraphSStep;
        for j = i:1:length(s_table)
            v_preffered_table(i) = min(  v_preffered_table(j) / ( 1.0 - sqrt( (j - i) / len  )  ) , v_preffered_table(i)    );
        end
    end

    start_s = 0.0;
    for i = 1:1:length(s_table)
        end_s_tmp = s_table(i);
        v_max_tmp = v_max_table(i);
        v_preffered_tmp = v_preffered_table(i);
        %NaviSpeedTsConstraints constraints;
        constraints.v_max = v_max_tmp;
        constraints.v_preffered = v_preffered_tmp;
        constraints.a_max = 0;
        constraints.a_preffered = 0;
        constraints.b_max = 0;
        constraints.b_preffered = 0;
        constraints.da_max = 0;
        constraints.da_preffered = 0;
        constraints.dda_max = 0;
        constraints.dda_preffered = 0;
        ts_graph_.UpdateRangeConstraints(start_s , end_s + kKappaSmooth, constraints);
        start_s = end_s_tmp;
        
    end
    fprintf('Add speed limit for centric acceleration with kappa: %f \n' , max_kappa);
    status = true;

end

% function status = AddConfiguredConstraints(reference_line_info , start_s , end_s)
%     constraints.v_max = FLAGS_planning_upper_speed_limit;
%     constraints.v_preffered = constraints.v_max;
%     constraints.a_max = max_accel_;
%     constraints.a_preffered = preferred_accel_;
%     constraints.b_max = max_decel_;
%     constraints.b_preffered = preferred_decel_;
%     constraints.da_preffered = preferred_jerk_;
%     constraints.da_max = max_jerk_;
%     
% 
% 
% 
% end

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







