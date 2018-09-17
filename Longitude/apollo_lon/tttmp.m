%% read calibration results

data = xlsread('D:\CIDI\Apollo\cidi_truck_calibration\cidi_calibration\data\MKZ002\result.csv');
control_cmd = data(: , 1) ;
spd = data(: , 2) ;
acc = data(: , 3) ;
cmd_table = [90 80 70 60 50 40 30 -30 -35 -40  -45 -50 -60];
cmd_index = [];
for i = 1:1: length(cmd_table)
    cmd_index= find(control_cmd == cmd_table(i));
end


cmd_minus_60 = find(control_cmd == -60);% findout the colume of cmd -60
spd_minus_60 = [];
acc_minus_60 = [];
speed_table = [0:0.2:25];





for i = 1:1:length(cmd_minus_60)
    spd_minus_60(i) = spd(cmd_minus_60(i));
    acc_minus_60(i) = acc(cmd_minus_60(i));
end

% 
% a = find(abs(spd_minus_60 - speed_table(3)) < 0.00001);
% c = mean(spd_minus_60(a)); 

for j = 1:1:length(speed_table)
        a = find(abs(spd_minus_60 - speed_table(j)) < 0.001);
        speed_minus_60(j) = mean(spd_minus_60(a));
        acceleration_minus_60(j) = mean(acc_minus_60(a));
    
        
end


for k = 1:1:length(speed_table)
        
        if (k>1 && k<length(speed_table))
            a=acceleration_minus_60(k) - acceleration_minus_60(k-1);
            b=acceleration_minus_60(k+1) - acceleration_minus_60(k);
            if(abs(a) > 0.8 && abs(b) >0.8)
                acceleration_minus_60(k) = acceleration_minus_60(k - 1);
            end
        end
        
end

LineWidth = 2;
plot(speed_minus_60,acceleration_minus_60,'bo','LineWidth', LineWidth);grid on





































