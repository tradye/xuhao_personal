%% 基于道路边沿采样的车道保持算法
alfa = 0; % 30 -30 45 -45 表示车辆航向角和道路的夹角
%初始化
if alfa == 0
    for i=1:100
    x(i)= 0.1*i;
    fl(i)= 10;
    fr(i)=-10;
    end
elseif (alfa == 30)
    for i=1:100
    x(i)= 0.1*i;
    fl(i)= 10;
    fr(i)=-10;
    end
elseif (alfa == -30)
    for i=1:100
    x(i)= 0.1*i;
    fl(i)= 10;
    fr(i)=-10;
    end
elseif (alfa == 45)
    for i=1:100
    x(i)= 0.1*i;
    fl(i)= 10;
    fr(i)=-10;
    end    
elseif (alfa == -45)
    for i=1:100
    x(i)= 0.1*i;
    fl(i)= 10;
    fr(i)=-10;
    end
else
    
end
    
% for i=1:20
%     x(i)=0.1*i;
%     fl(i)=-3/16*x(i)^2+3/2*x(i)+1;
%     fr(i)=-3/16*x(i)^2+3/2*x(i)-1;
% end
 
plot(x,fl);
hold on;
plot(x,fr);
 
 
kmax=1.5;klmin=kmax;krmax=-kmax;
x0=0;y0=0;s_thresh=1.2;
pi = 3.1415926;
K_expect = 0.0;
w1 = 0.5;
w2 = 0.5;
C_cost_min = 0.0;

delta_k = 0.05;
C_p = 0.000001;
C_s = 0.0;
%求解一次圆弧轨迹
for i=1:100
    kl(i)=2*fl(i)/(x(i)^2+fl(i)^2);%计算车辆当前位置到道路左边沿采样点圆弧的曲率
    kr(i)=2*fr(i)/(x(i)^2+fr(i)^2);%计算车辆当前位置到道路右边沿采样点圆弧的曲率
    
    sl(i)=2*asin(fl(i)/sqrt(x(i)^2+fl(i)^2))/kl(i);%计算车辆当前位置到道路左边沿采样点圆弧的弧长
    sr(i)=2*asin(fr(i)/sqrt(x(i)^2+fr(i)^2))/kr(i);%计算车辆当前位置到道路右边沿采样点圆弧的弧长
    
%     sl1(i)=(pi/2 + asin((fl(i)^2 - x(i)^2)/(x(i)^2+fl(i)^2)))/kl(i);%计算车辆当前位置到道路左边沿采样点圆弧的弧长
%     sr1(i)=(pi/2 + asin((fr(i)^2 - x(i)^2)/(x(i)^2+fr(i)^2)))/kr(i);%计算车辆当前位置到道路左边沿采样点圆弧的弧长
    
    s(i)=max(sl(i),sr(i));
    
    if (s(i)<s_thresh)
        continue;
    else
        if (kl(i)<klmin)
            klmin=kl(i);
        end
        if (kr(i)>-kmax)
            krmax=kr(i);
        end
        x_max=x(i);%x_max对应l_max，沿x方向的局部最大感知范围
        kr(i)=krmax;
        kl(i)=klmin;
    end
    %% 应满足任意时刻右侧曲率不大于左侧曲率，不然终止     
    if (kl(i)<kr(i))
        x_max=x(i-1);
        break;
    else
        
        p = floor((klmin - krmax) / delta_k);
        for j = 1:1:(p)
           kp(j) = krmax + (kl(i) - kr(i)) / p * (j - 1); 
           xp_ = linspace(0,x_max,p);
           sp_ = asin(kp(j)*(xp_-x0))/kp(j);
           yp_ = -1/kp(j)*cos(kp(j)*sp_)+1/kp(j)+y0;
           for n = 1:1:j
              C_p = C_p + min(abs(yp_(n) - fl(i)),abs(yp_(n) - fr(i)));
%               fprintf('C_p = %f \n',C_p);
%               fprintf('n = %f \n',n);
              C_proximity(n) = 1 / C_p;
%               fprintf('C_proximity = %f \n',C_proximity(n));
              C_s = C_s + (kp(n) - K_expect)^2;
              C_smooth(n) = C_s;
%               fprintf('C_smooth = %f \n',C_smooth(n));
           end
%            C_proximity = 1 / C_p;
%            fprintf('C_proximity = %f \n',C_proximity);

           C_cost = w1* C_proximity + w2 * C_smooth;
            if(j == 1)
               C_cost_min = C_cost(j);
               K_expect = kp(j);
               index=j;
            else
               if(C_cost(j) <= C_cost_min)
                   C_cost_min = C_cost(j);
                   K_expect = kp(j);
                   index = j;
               end
            end
            fprintf('C_cost_min = %f \n',C_cost_min);
           
        end
%         for n = 1:1:p
%             C_p = C_p + min(abs(yp_(n) - fl(i)),abs(yp_(n) - fr(i)));
%             %               fprintf('C_p = %f \n',C_p);
%             %               fprintf('n = %f \n',n);
%             C_proximity(n) = 1 / C_p;
%             %               fprintf('C_proximity = %f \n',C_proximity(n));
%             C_s = C_s + (kp(n) - K_expect)^2;
%             C_smooth(n) = C_s;
%             %               fprintf('C_smooth = %f \n',C_smooth(n));
%         end
%         C_cost = w1* C_proximity + w2 * C_smooth;
%         if(j == 1)
%            C_cost_min = C_cost(j);
%            K_expect = kp(j);
%            index=j;
%         else
%            if(C_cost(j) <= C_cost_min)
%                C_cost_min = C_cost(j);
%                K_expect = kp(j);
%                index = j;
%            end
%         end
        fprintf('K_expect = %f \n',K_expect);      
%         kp = [];
%         C_proximity = [];
%         C_smooth = [];
%         C_cost = [];
%         if (i==1)
%             CPY=cpy(i);
%             KY=ky(i);%本控制周期的最佳曲率KY 
%            index_i=i;
%         else
%             if(cpy(i)<CPY)
%                 CPY=cpy(i); 
%                 KY=ky(i);%本控制周期的最佳曲率KY
%                 index_i=i;
%             end
%         end
        
%         for 
%         C_p = C_p + min(fabs(yp_ - fl(i)),fabs(yp_ - fr(i)));
%         [cpy(i),ky(i)]=zyql1(x,x_max,kr(i),kl(i),x0,y0,fl,fr);%控制命令离散化及选择最优控制指令
%         if (i==1)
%             CPY=cpy(i);
%             KY=ky(i);%KY 最佳曲率
%            index_i=i;
%         else
%             if(cpy(i)<CPY)
%                 CPY=cpy(i); 
%                 KY=ky(i);
%                 index_i=i;
%             end
%         end
    end
    %% 控制空间内道路右侧所有的采样圆弧
    xr0=linspace(0,x_max,100);
    sr0=asin(kr(i)*(xr0-x0))/kr(i);
    yr0=-1/kr(i)*cos(kr(i)*sr0)+1/kr(i)+y0;
    plot(xr0,yr0,'go');%道路右侧所有的采样圆弧
     %% 控制空间内道路左侧所有的采样圆弧   
    xl0=linspace(0,x_max,100);%x坐标
    sl0=asin(kl(i)*(xl0-x0))/kl(i);%弧长
    yl0=-1/kl(i)*cos(kl(i)*sl0)+1/kl(i)+y0;%根据曲率和弧长，求y方向坐标
    plot(xl0,yl0,'ko');%道路左侧所有的采样圆弧
end

%% 将最优路用红色星型曲线表示出来
% xy=linspace(0,x(index_i),100);
% sy=asin(KY*(xy-x0))/KY;
%  fy=-1/KY*cos(KY*sy)+1/KY+y0;
% plot(xy,fy,'r*')

xy=linspace(0,x(index),100);
sy=asin(K_expect*(xy-x0))/K_expect;
fy=-1/K_expect*cos(K_expect*sy)+1/K_expect+y0;
plot(xy,fy,'r*')