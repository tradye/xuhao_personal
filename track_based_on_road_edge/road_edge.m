%% 基于道路边沿采样的车道保持算�?
alfa = -30; % 30 -30 45 -45 表示车辆航向角和道路的夹�?
%初始�?
if alfa == 0
    for i=1:100
        x(i)= 0.1*i;
        fl(i)= 10;
        fr(i)=-10;
    end
elseif (alfa == 30)
    for i=1:100
        x(i)= 0.1*i;
        fl(i)= sqrt(3)/3 * x(i) + 10;
        fr(i)= sqrt(3)/3 * x(i) - 10;
    end
elseif (alfa == -30)
    for i=1:100
        x(i)= 0.1*i;
        fl(i)= -sqrt(3)/3 * x(i) + 10;
        fr(i)= -sqrt(3)/3 * x(i) - 10;
    end
elseif (alfa == 45)
    for i=1:100
        x(i)= 0.1*i;
        fl(i)=x(i) + 10;
        fr(i)=x(i) - 10;
    end    
elseif (alfa == -45)
    for i=1:100
        x(i)= 0.1*i;
        fl(i)= -x(i) + 10;
        fr(i)= -x(i) - 10;
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
K_expect = 0.00;
w1 = 0.2;
w2 = 0.1;
C_cost_min = 0.0;

delta_k = 0.05;
C_p = 0.000001;
C_s = 0.0;
%求解�?次圆弧轨�?
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
        x_max=x(i);%x_max对应l_max，沿x方向的局部最大感知范�?
        kr(i)=krmax;
        kl(i)=klmin;
    end
    %% 应满足任意时刻右侧曲率不大于左侧曲率，不然终�?     
    if (kl(i)<kr(i))
        x_max=x(i-1);
        break;
    else
        
%         p = floor((klmin - krmax) / delta_k);
%         p = 100;
%         C_p = 0.000001;
%         C_s = 0.0;
%         for j = 1:1:(p )
% %            kp(j) = krmax + delta_k * (j - 1); 
%             kp = linspace(krmax,klmin,p);
%             xp_ = linspace(0,x_max,p);
%             sp_ = asin(kp(j)*(xp_-x0))/kp(j);
%             yp_ = -1/kp(j)*cos(kp(j)*sp_)+1/kp(j)+y0;
%            
% %             tmp = min(abs(yp_(i) - fl(i)),abs(yp_(i) - fr(i)));
% %             tmp = min(abs(yp_(j) - fl(i)),abs(yp_(j) - fr(i)));
%             tmp = min(abs(yp_(j) - fl(i)),abs(yp_(j) - fr(i)));
% 
%             C_p = C_p + tmp;
% 
%             C_proximity(j) = 1 / C_p;
%             C_s = C_s + (kp(j) - K_expect)^2;
%             C_smooth(j) = C_s;
%            
% %            plot(xp_,yp_,'y*');%离散化后的控制空�?
% 
%         end
%            
%         C_cost = w1* C_proximity + w2 * C_smooth;
% 
%         C_cost_min = C_cost(1);
%         for m = 1:1:length(C_cost)
%             if C_cost(m) <= C_cost_min
%                 C_cost_min = C_cost(m);
%                 K_expect = kp(m);
%                 index = m;
%             end 
%         end
% 
%         fprintf('K_expect = %f \n',K_expect);
%         fprintf('index = %f \n',index);

    end
    %% 控制空间内道路右侧所有的采样圆弧
    xr0=linspace(0,x_max,100);
    sr0=asin(kr(i)*(xr0-x0))/kr(i);
    yr0=-1/kr(i)*cos(kr(i)*sr0)+1/kr(i)+y0;
    plot(xr0,yr0,'go');%道路右侧�?有的采样圆弧
     %% 控制空间内道路左侧所有的采样圆弧   
    xl0=linspace(0,x_max,100);%x坐标
    sl0=asin(kl(i)*(xl0-x0))/kl(i);%弧长
    yl0=-1/kl(i)*cos(kl(i)*sl0)+1/kl(i)+y0;%根据曲率和弧长，求y方向坐标
    plot(xl0,yl0,'go');%道路左侧�?有的采样圆弧
    

end

%% 将最优路用红色星型曲线表示出�?
% xy=linspace(0,x(index_i),100);
% sy=asin(KY*(xy-x0))/KY;
%  fy=-1/KY*cos(KY*sy)+1/KY+y0;
% plot(xy,fy,'r*')
q = floor((klmin - krmax) / delta_k);
p = 100;

kp = linspace(krmax,klmin,p);

for j = 1:1:(p )
    xp_ = linspace(0,x_max,p);
    sp_ = asin(kp(j)*(xp_-x0))/kp(j);
    yp_ = -1/kp(j)*cos(kp(j)*sp_)+1/kp(j)+y0;

    for n = 1:1:length(x)
        tmp = min(abs(yp_( n ) - fl(n)),abs(yp_(n ) - fr(n)));
        C_p = C_p + tmp;
        fprintf('C_p = %f \n',C_p);
    end

    C_proximity(j) = 1 / C_p;
    C_smooth(j) = (kp(j) - K_expect)^2;
    C_p = 0.000001;           
%             plot(xp_,yp_,'y*');%离散化后的控制空�?

end

C_cost = w1* C_proximity + w2 * C_smooth;
C_cost_min = C_cost(1);
for m = 1:1:length(C_cost)
    if C_cost(m) <= C_cost_min
        C_cost_min = C_cost(m);
        K_expect = kp(m);
        index = m;
    end 
end

fprintf('K_expect = %f \n',K_expect);
fprintf('index = %f \n',index);  


 xy=linspace(0,x_max,100);
 sy=asin(K_expect*(xy-x0))/K_expect;
 fy=-1/K_expect*cos(K_expect*sy)+1/K_expect+y0;
 plot(xy,fy,'r*')