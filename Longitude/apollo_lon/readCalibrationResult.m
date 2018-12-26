%% read calibration results and plot

data = xlsread('D:\CIDI\D_file\CIDI改装车辆\中车电动公交\仿真及实车标定测试\calibration_available_20181127\calibration\data\MKZ002\result.csv');
control_cmd = data(: , 1) ;
spd = data(: , 2) ;
acc = data(: , 3) ;
cmd_table = [90 80 70 60 50 40 30 -30 -35 -40  -45 -50 -60];
cmd_index = [];
speed_table = [0:0.2:25];
LineWidth = 0.2;
cc=colormap(hsv(13));
ddd=[];

for i = 1:1: length(cmd_table)
    cmd_index= find(control_cmd == cmd_table(i));
    spd_table = [];
    acc_table = [];
    for m = 1:1:length(cmd_index)
        spd_table(m) = spd(cmd_index(m));
        acc_table(m) = acc(cmd_index(m));
    end
    
    for j = 1:1:length(speed_table)
        a = find(abs(spd_table - speed_table(j)) < 0.001);
        speed_array(j) = mean(spd_table(a));
        acc_array(j) = mean(acc_table(a));           
    end
    
    for k = 1:1:length(speed_table)
        
        if (k>1 && k<length(speed_table))
            a=acc_array(k) - acc_array(k-1);
            b=acc_array(k+1) - acc_array(k);
            
            if(abs(a) > 0.8 && abs(b) >0.8)
                acc_array(k) = acc_array(k - 1);
            end
        end        
    end
    
    plot(speed_array,acc_array, ':s' ,'LineWidth', LineWidth,'color',cc(i,:,:)); hold on
    grid on
       
end

title('calibration table results');
xlabel('speed m/s'); 
ylabel('acc m/s^2');
legend('cmd = 90','cmd = 80' ,'cmd = 70' ,'cmd = 60' ,'cmd = 50' ,'cmd = 40' , ...
              'cmd = 30' ,'cmd = -30' ,'cmd = -35' ,'cmd = -40' ,'cmd = -45' ,'cmd = -50' ,'cmd = -60');






































