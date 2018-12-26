%% read calibration results and plot
close all
clear all
data = xlsread('D:\CIDI\xuhao_GitHub\xuhao_personal\Longitude\apollo_lon\result_with_trailer.csv');
control_cmd = data(: , 1) ;
spd = data(: , 2) ;
acc = data(: , 3) ;
cmd_table = [95 90 80 70 60 50 40 30 25 -20 -25 -30  -40  -50 -60 -70];
len = length(cmd_table);
cmd_index = [];
speed_table = [0:0.2:31];
LineWidth = 0.2;
cc=colormap(hsv(len));
ddd=[];
acc_new_table = [];
vel_new_table = [];
cmd_new_table = [];

for i = 1:1:len
    cmd_index= find(control_cmd == cmd_table(i));
    spd_table = [];
    acc_table = [];
    acc_array = [];
    for m = 1:1:length(cmd_index)
        spd_table(m) = spd(cmd_index(m));
        acc_table(m) = acc(cmd_index(m));
    end
 
%     for j = 1:1:length(speed_table)
%         a = find(abs(spd_table - speed_table(j)) < 3);
%         speed_array(j) = mean(spd_table(a));
%         acc_array(j) = mean(acc_table(a));           
%     end
    
    v_max = 0:0.2:max(spd_table);
    for j = 1:1:length(v_max)
        a = find(abs(spd_table - speed_table(j)) < 1);
%         speed_array(j) = mean(spd_table(a));
        acc_array(j) = mean(acc_table(a));           
    end
    
  
%     if i > 1        
%         for k = 1:1:length(v_max)
% %             if cmd_table(i) > 0
% %                 if( acc_array(k) > last_acc_array(k) )
% %                     acc_array(k) = acc_array(k) - abs(acc_array(k) -  last_acc_array(k));
% %                 end
% %             else
% %                 if( acc_array(k) > last_acc_array(k) )
% %                     acc_array(k) = acc_array(k) - abs(acc_array(k) -  last_acc_array(k));
% %                 end
%              if( acc_array(k) > last_acc_array(k) )
% %                     acc_array(k) = acc_array(k) - abs(acc_array(k) -  last_acc_array(k));
%                 if ( acc_array(k) - last_acc_array(k)  > 0.1) && ( acc_array(k) - last_acc_array(k)  < 0.2)
%                     acc_array(k) = acc_array(k) - 0.3;
%                 elseif ( acc_array(k) - last_acc_array(k)  > 0.2) && ( acc_array(k) - last_acc_array(k)  < 0.3)
%                     acc_array(k) = acc_array(k) - 0.4;
%                 else
%                     acc_array(k) = acc_array(k) - 0.1;
%                 end
%              end       
%         end
%     end
    
    
%     for k = 1:1:length(speed_table)
%         
%         if (k>1 && k<length(speed_table))
%             a=acc_array(k) - acc_array(k-1);
%             b=acc_array(k+1) - acc_array(k);
%             
%             if(abs(a) > 0.8 && abs(b) >0.8)
%                 acc_array(k) = acc_array(k - 1);
%             end
%         end        
%     end


    acc_new_table = [acc_new_table ;acc_array'];
    vel_new_table = [vel_new_table ;v_max'];
    cmd_new_table = [cmd_new_table; linspace(cmd_table(i),cmd_table(i),length(v_max))'];
    plot(v_max,acc_array, ':s' ,'LineWidth', LineWidth,'color',cc(i,:,:)); hold on
    grid on
    legend('cmd = 95','cmd = 90','cmd = 80' ,'cmd = 70' ,'cmd = 60' ,'cmd = 50' ,'cmd = 40' , ...
              'cmd = 30' ,'cmd = 25','cmd = -20' ,'cmd = -25' ,'cmd = -35' ,'cmd = -40' ,...
              'cmd = -50' ,'cmd = -60' ,'cmd = -70');

    last_acc_array = acc_array;   
end

title('calibration table results 0917');
xlabel('speed m/s'); 
ylabel('acc m/s^2');



% n=find(y(:,1)==50);
% u=y(n,1);
% v=y(n,2);
% a=y(n,3);
% 
% v00=0:0.2:max(v);
% for i=1:length(v00)
%     n1=find(abs(v-v00(i))<2);
%     a1(i)=mean(a(n1));
% end
%     
%     
% figure(1),
% plot(v,a,'.b');grid on,hold on
% plot(v00,a1,'*r');


































