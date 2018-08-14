[cpy,ky]=select_best_path(x,x_max,kr_max,kl_max,x0,y0,fl,fr);%对控制空间内的每条路径进行离散化
%输入：左右两侧道路的曲率、坐标，初始位置，
%
delta_k = 0.01;% delta_k是一个重要的参数,离散的步长
w1 = 0.5;
w2 = 0.5;
p = floor((kl_max - kr_max) / delta_k);
%% 路径离散化
for j = 1:1:(p + 1)
   kp(j) = kr_max + (kl_max - kr_max) / delta_k * (j - 1); 
   sj(j) = asin(kr);
   fj(j) = 
   C_proximity = min(fabs(),fabs());
end
%xr0=linspace(0,x_max,100);
%sr0=asin(kr(i)*(xr0-x0))/kr(i);
%yr0=-1/kr(i)*cos(kr(i)*sr0)+1/kr(i)+y0;
% void linspace(float x0,float xf, float lx[], int sample_num)
% {
%     int i;
%     lx[0] = x0;
%     lx[sample_num - 1] = xf;
% 
%     for (i = 1; i < sample_num - 1; i++)
%     {
%         lx[i] = lx[i - 1] + ((xf - x0) / (sample_num - 1));
%     }
%     return;
% }

