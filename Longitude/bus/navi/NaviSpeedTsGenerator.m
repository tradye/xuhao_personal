
 kDefaultSStep = 1.0;
 kDefaultSMax = 2.0;
 kDefaultTStep = 0.1;
 kDefaultTMax = 15.0;
 kSmoothCount = 100;
 kFallbackSpeedPointNum = 4;
 kTsGraphSStep = 0.1;
 kTsGraphTStep = 0.1;
 kTsGraphTMax = 10.0;
 
 
s_step = 1;
s_max  = 100;
t_step = 0.1;
t_max = 8;
compress_t = true;
cap_saved_ratio = 1;
start_v = 1;
start_a = 0;
start_da = 2;
% s_step = planning_length > kTsGraphSStep
%                     ? kTsGraphSStep
%                     : planning_length / kFallbackSpeedPointNum;
end_s = 200;
start_s = 0;
vehicle_v = 1;
start_v = max(0,start_v);
planning_length = end_s - start_s; 
if(planning_length > kTsGraphSStep)
    s_step = kTsGraphSStep;
else
    s_step = planning_length / kFallbackSpeedPointNum;
end

ts_graph = NaviSpeedTsGraph(0.1,0,0);
ts_graph.Reset(s_step, planning_length, kTsGraphTStep, kTsGraphTMax, compress_t, cap_saved_ratio,start_v,start_a, start_da);
% length(ts_graph.t_)
% ts_graph.t_
ts_graph.Solve();









