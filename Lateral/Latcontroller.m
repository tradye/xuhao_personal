 %% �������������
%% 
clc
clear all
close all
warning off all

% %% �������ݷ���
% x1=xlsread('C:\Users\CIDI\Desktop\����\2018-07-08\lqr_data_0.5.csv');
% x2=xlsread('C:\Users\CIDI\Desktop\����\2018-07-08\lqr_data_50.csv');
% x3=xlsread('C:\Users\CIDI\Desktop\����\2018-07-08\lqr_data_50_0.5.csv');
% x4=xlsread('C:\Users\CIDI\Desktop\����\2018-07-08\lqr_data_100.csv');
% 
% y1=load('C:\Users\CIDI\Desktop\����\2018-07-08\bag\path_2018-07-06-17-38-32_0.bag.txt');
% 
% 
% 
% 
% cc=colormap(hsv(4));close 
% 
% 
% 
% figure(20)
% subplot(2,2,1),
% plot(x1(:,2),'.-','color',cc(1,:,:));hold on,grid on
% plot(x2(:,2),'.-','color',cc(2,:,:));
% plot(x3(:,2),'.-','color',cc(3,:,:));
% plot(x4(:,2),'.-','color',cc(4,:,:));
% % legend('�����С0.5','Ԥ��0.5s','�����С0.5+Ԥ��0.5s','Ԥ��1.0s');
% title('steer-angle���㣩');
% set(gca,'Fontsize',15);
% 
% 
% % figure(21)
% subplot(2,2,2),
% plot(x1(:,5),'.-','color',cc(1,:,:));hold on,grid on
% plot(x2(:,5),'.-','color',cc(2,:,:));
% plot(x3(:,5),'.-','color',cc(3,:,:));
% plot(x4(:,5),'.-','color',cc(4,:,:));
% % legend('�����С0.5','Ԥ��0.5s','�����С0.5+Ԥ��0.5s','Ԥ��1.0s');
% title('lateral-error(��) ');
% set(gca,'Fontsize',15);
% 
% % figure(22)
% subplot(2,2,3),
% plot(x1(:,7),'.-','color',cc(1,:,:));hold on,grid on
% plot(x2(:,7),'.-','color',cc(2,:,:));
% plot(x3(:,7),'.-','color',cc(3,:,:));
% plot(x4(:,7),'.-','color',cc(4,:,:));
% % legend('�����С0.5','Ԥ��0.5s','�����С0.5+Ԥ��0.5s','Ԥ��1.0s');
% title('heading-error(m)');
% set(gca,'Fontsize',15);
% 
% % figure(23)
% subplot(2,2,4),
% plot(x1(:,9),'.-','color',cc(1,:,:));hold on,grid on
% plot(x2(:,9),'.-','color',cc(2,:,:));
% plot(x3(:,9),'.-','color',cc(3,:,:));
% plot(x4(:,9),'.-','color',cc(4,:,:));
% legend('�����С0.5','Ԥ��0.5s','�����С0.5+Ԥ��0.5s','Ԥ��1.0s');
% title('v(m/s)');
% set(gca,'Fontsize',15);
% 
% figure(30)
% subplot(2,1,1),
% plot(x1(:,2),'.-','color',cc(1,:,:));hold on,grid on
% plot(x2(:,2),'.-','color',cc(2,:,:));
% plot(x3(:,2),'.-','color',cc(3,:,:));
% plot(x4(:,2),'.-','color',cc(4,:,:));
% % legend('�����С0.5','Ԥ��0.5s','�����С0.5+Ԥ��0.5s','Ԥ��1.0s');
% title('steer-angle���㣩');
% set(gca,'Fontsize',15);
% 
% subplot(2,1,2),
% plot(x1(:,10),'.-','color',cc(1,:,:));hold on,grid on
% plot(x2(:,10),'.-','color',cc(2,:,:));hold on,grid on
% plot(x3(:,10),'.-','color',cc(3,:,:));hold on,grid on
% plot(x4(:,10),'.-','color',cc(4,:,:));hold on,grid on
% 
% 
% 
% figure(33)
% subplot(2,2,[2 4]),
% plot(y1(:,1)-y1(1,1),y1(:,2)-y1(1,2),'.-');grid on
% xlabel('x(m)');
% ylabel('y(m)');
% 
% figure(33)
% subplot(2,2,1),
% plot([1:1:size(y1(:,2))]/100,y1(:,1)-y1(1,1),'.-');grid on
% subplot(2,2,3),
% plot([1:1:size(y1(:,2))]/100,y1(:,2)-y1(1,2),'.-');grid on
%%
s=tf('s');%������˹�任
flag=3;%2Ϊ����ģ�ͣ�3Ϊ����ģ��
%% ��������������������
control_period= 0.01;
trajectory_period= 0.1;
chassis_period= 0.01;
localization_period= 0.01;
max_status_interval_sec= 0.1;
max_planning_interval_sec= 0.2;
max_planning_delay_threshold= 4.0;
% action= STOP;
soft_estop_brake= 50.0;
% active_controllers= LAT_CONTROLLER;
% active_controllers= LON_CONTROLLER;
max_steering_percentage_allowed= 100;
minimum_speed_resolution= 0.2;

  ts= 0.01;%��������
  ts_=ts;
  preview_window= 0;
  cf_= 155494.663;
  cr_= 155494.663*2;
  mass_fl= 3000;
  mass_fr= 3000;
  mass_rl= 2000;
  mass_rr= 2000;
  eps= 0.01;
  
  cutoff_freq= 10;%���ָ���˲���ֹƵ��
  mean_filter_window_size= 10;
  max_iteration= 150;
  max_lateral_acceleration= 5.0;
  basic_state_size_=4;
  
  front_edge_to_center= 1.95;
  back_edge_to_center= 1.043;
  left_edge_to_center= 1.055;
  right_edge_to_center= 1.055;

  length= 4.933;
  width= 2.11;
  height= 1.48;
  min_turn_radius= 5.05386147161;
  max_acceleration= 2.0;
  max_deceleration= -6.0;
  max_steer_angle= 8.20304748437;
  max_steer_angle_rate= 6.98131700798;
  steer_ratio= 16;
  wheel_base= 2.8448;
  wheel_rolling_radius= 0.335;
  
  %%
  mass_front = mass_fl + mass_fr;
  mass_rear = mass_rl + mass_rr;
  mass_ = mass_front + mass_rear;  

  wheelbase_=4.75;%δ֪����
  lf_00 = wheelbase_ * (1.0 - mass_front / mass_);
  lr_00 = wheelbase_ * (1.0 - mass_rear / mass_);%���ڼ���ת������

  lf_ =1.95;
  lr_ =1.95;%л�������Ĳ���
  lr_2 =3.3;  

%   lf_ =1.135;
%   lr_ =3.625;%��Ƹ��Ĳ���

  iz_ = lf_00 * lf_00 * mass_front + lr_00 * lr_00 * mass_rear;


  matrix_size = basic_state_size_ + preview_window;
  matrix_a_ = zeros(basic_state_size_, basic_state_size_);
  matrix_ad_ = zeros(basic_state_size_, basic_state_size_);
  matrix_adc_ = zeros(matrix_size, matrix_size);
      
  if flag==2
      matrix_a_(1, 2) = 1.0;
      matrix_a_(2, 3) = (cf_ + cr_) / mass_;
      matrix_a_(3, 4) = 1.0;
      matrix_a_(4, 3) = (lf_ * cf_ - lr_ * cr_) / iz_;

      matrix_a_coeff_ = zeros(matrix_size, matrix_size);
      matrix_a_coeff_(2, 2) = -(cf_ + cr_) / mass_;
      matrix_a_coeff_(2, 4) = (lr_ * cr_ - lf_ * cf_) / mass_;
      matrix_a_coeff_(3, 4) = 1.0;
      matrix_a_coeff_(4, 2) = (lr_ * cr_ - lf_ * cf_) / iz_;
      matrix_a_coeff_(4, 4) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;
  
  elseif flag==3
      matrix_a_(1, 2) = 1.0;
      matrix_a_(2, 3) = (cf_ + 2*cr_) / mass_;
      matrix_a_(3, 4) = 1.0;
      matrix_a_(4, 3) = (lf_ * cf_ - (lr_ +lr_2)* cr_) / iz_;

      matrix_a_coeff_ = zeros(matrix_size, matrix_size);
      matrix_a_coeff_(2, 2) = -(cf_ + 2*cr_) / mass_;
      matrix_a_coeff_(2, 4) = ((lr_ +lr_2)* cr_ - lf_ * cf_) / mass_;
      matrix_a_coeff_(3, 4) = 1.0;
      matrix_a_coeff_(4, 2) = ((lr_ +lr_2)* cr_ - lf_ * cf_) / iz_;
      matrix_a_coeff_(4, 4) = -1.0 * (lf_ * lf_ * cf_ + (lr_ * lr_ +lr_2 * lr_2 )* cr_) / iz_;
  
  end
  %% 
  
  
  v=10;%���٣�m/s
  matrix_a_(2, 2) = matrix_a_coeff_(2, 2) / v;
  matrix_a_(2, 4) = matrix_a_coeff_(2, 4) / v;
  matrix_a_(4, 2) = matrix_a_coeff_(4, 2) / v;
  matrix_a_(4, 4) = matrix_a_coeff_(4, 4) / v;
  
  matrix_i=eye(4);
  matrix_ad_ = (matrix_i + ts_ * 0.5 * matrix_a_) *inv(matrix_i - ts_ * 0.5 * matrix_a_);%��ɢ��
               
  basic_state_size_=4;
  matrix_b_ = zeros(basic_state_size_, 1);
  matrix_bd_ = zeros(basic_state_size_, 1);
  matrix_bdc_ = zeros(matrix_size, 1);
  matrix_b_(2, 1) = cf_ / mass_;
  matrix_b_(4, 1) = lf_ * cf_ / iz_;
  matrix_bd_ = matrix_b_ * ts_;%��ɢ������

  matrix_state_ = zeros(matrix_size, 1);
  matrix_k_ = zeros(1, matrix_size);
%   matrix_r_ = Matrix::Identity(1, 1);

%% LQR������������ȡ
  matrix_q_1= 0.01;
  matrix_q_2= 0.00001;
  matrix_q_3= 0.1;
  matrix_q_4= 0.0001;
  matrix_q_ =diag([matrix_q_1; matrix_q_2; matrix_q_3; matrix_q_4]);  
  matrix_r_ =2;
  
%   Kcp=diag([1 1.2 0.7 1])*1.5;%У��ϵ��
  Kcp=diag([1 1 1 1])*1;%У��ϵ��
  
  Kc=lqr(matrix_a_,matrix_b_,matrix_q_,matrix_r_)*Kcp
%   Kc=[0.047 0.022 0.309 0.0577];%����ϵͳ���
%     0.0965    0.0515    0.7563    0.1393    266
  Kcd=dlqr(matrix_ad_,matrix_bd_,matrix_q_,matrix_r_)  ;%��ɢϵͳ���
% Kcd2=lqrsolve(matrix_ad_,matrix_bd_,matrix_q_,matrix_r_);

%% ʱ�����Է���
TFK=ts;%���ݱ�������
steerdelay=0.4*1;%�������ӳ�ʱ��
Tsteer=0.2*1;%������һ�׹��Ե�Чʱ�䳣��
Ksteer=1;%����������ϵ��
Gsteer=tf(Ksteer,[Tsteer 1])*exp(-steerdelay*s);%�����̴��ݺ���
steer0=-1/57.3;%��������λ
for backlash=[0 0.2]/57.3%�����̼�϶����
%% �����˲���
   M_PI=pi;

       wa = 2.0 * M_PI * cutoff_freq;  %// Analog frequency in rad/s
       alpha = wa * ts / 2.0;          %// tan(Wd/2), Wd is discrete frequency
       alpha_sqr = alpha * alpha;
       tmp_term = sqrt(2.0) * alpha + alpha_sqr;
       gain = alpha_sqr / (1.0 + tmp_term);

      den1=1.0;
      den2= 2.0 * (alpha_sqr - 1.0) / (1.0 + tmp_term);
      den3= (1.0 - sqrt(2.0) * alpha + alpha_sqr) /(1.0 + tmp_term);

      num1=gain;
      num2=2.0 * gain;
      num3=gain;

num=[num1 num2 num3];
den=[den1 den2 den3];
Gz=tf( num, den, ts );
Gfilter= d2c( Gz, 'tustin' ); %%���������ָ���˲���,����˫���Ա任
[num,den] = tfdata( Gfilter,'v' );%���s�����ķ��Ӻͷ�ĸ

%% ��λһ�׵�Ч���Ի���ʱ�䳣��
Tnavigation=0.005;
Gnavigation=tf(1,[Tnavigation 1]);

%% У���������
a1=1*1;
a2=0.001*1;
Tc=0.1;

Gc=tf([a1*Tc 1],[a2*Tc 1]);
Ki=0.2;
%%
Zc=0/57.3;
initial=[1 0 0/57.3 0];%��������ֵ
tend=20;
sim('LatcontrollerSim',tend);

%% Ƶ�����Է���
x=inv(eye(4)*s-matrix_a_)*matrix_b_;%��ȡ״̬����Ĵ��ݺ���

a22=matrix_a_(2,2);
a23=matrix_a_(2,3);
a24=matrix_a_(2,4);
a42=matrix_a_(4,2);
a43=matrix_a_(4,3);
a44=matrix_a_(4,4);

b2=matrix_b_(2);
b4=matrix_b_(4);

%%�����̵�����λ������
Gx2_steer=(b2*s^2 + (a24*b4 - a44*b2)*s + a23*b4 - a43*b2)/(s^3 + (- a22 - a44)*s^2 + (a22*a44 - a43 - a24*a42)*s + a22*a43 - a23*a42);
%�����̵�������
Gx4_steer=(b4*s^2 + (a42*b2 - a22*b4)*s)/(s^3 + (- a22 - a44)*s^2 + (a22*a44 - a43 - a24*a42)*s + a22*a43 - a23*a42);
%��������λ������
Gx2_x4=(- b2*s^2 + (a44*b2 - a24*b4)*s - a23*b4 + a43*b2)/(- b4*s^2 + (a22*b4 - a42*b2)*s);

Gz=Kc*x*Gsteer*Gfilter*Gnavigation;%�������ݺ���

Gzc=Kc*x*Gsteer*Gfilter*Gnavigation*Gc;%�������ݺ�������У����
%% ���ݿ��ӻ�����
LWH=2;%���ݵĴ�ϸ

figure(1);
plot(Latout.time,57.3*Latout.data(:,1),'r','LineWidth',LWH);hold on,grid on
plot(Latout.time,57.3*Latout.data(:,2),'b','LineWidth',LWH);
xlabel('t(s)');
ylabel('��');
legend('steer-angleָ��','steer-angleʵ��');

figure(2);
plot(Latout.time,Latout.data(:,3),'LineWidth',LWH);hold on,grid on
xlabel('t(s)');
ylabel('m');
legend('Lat-error');


figure(3);
plot(Latout.time,Latout.data(:,4),'LineWidth',LWH);hold on,grid on
xlabel('t(s)');
ylabel('m/s');
legend('Lat-error-rate');

figure(5);
plot(Latout.time,57.3*Latout.data(:,5),'LineWidth',LWH);hold on,grid on
xlabel('t(s)');
ylabel('��');
legend('heading-error');

figure(6);
plot(Latout.time,57.3*Latout.data(:,6),'LineWidth',LWH);hold on,grid on
xlabel('t(s)');
ylabel('��/s');
legend('heading-error-rate');


figure(7);
bode(Gc);hold on,grid on
margin(Gz);hold on,grid on
margin(Gzc);
legend('У������','У��ǰ','У����');
  
%     A=[0 1 0 0;
%           0 a22 a23 a24;
%           0 0 0 1;
%           0 a42 a43 a44];
%     B=[0 ;b2; 0; b4];
    


%     x=inv(eye(4)*s-matrix_a_)*matrix_b_;
  
  kv =lr_ * mass_ / 2 / cf_ / wheelbase_ - lf_ * mass_ / 2 / cr_ / wheelbase_;
  
    %% ����迨�᷽��
%     A=[-0.3 0.1 -0.05;
%             1 0.1 0;
%             -1.5 -8.9 -0.05];
%     B=[2 0 4]';
%     C=[1 2 3];
%     D=0;
%     
%     Q=eye(3);
%     R=1;
%     Kc=lqr(A,B,Q,R)
    
    
    
%   steer_angle_feedback = -(matrix_k_ * matrix_state_)(1, 1) * 180 /M_PI * steer_transmission_ratio_ /steer_single_direction_max_degree_ * 100;
%                                       
% filepath=fullfile('F:','Leon Files','MH_01_easy.bag');
% bag=rosbag(filepath);


% filePath = fullfile(fileparts(which('ROSWorkingWithRosbagsExample')), 'data', 'ex_multiple_topics.bag');
% %%
% % Retrieve information from the rosbag
% bagselect = rosbag(filePath);

% bag=rosbag('D:\����\��������\2018-07-06-17-38-31\2018-07-06-17-38-32_0.bag');
% control_message=select(bag,'MessageType','pb_msgs/ControlCommand');
% % control_message=select(bag,'MessageType','pb_msgs/PadMessage');
% control_data=readMessages(control_message);
% 
% [ts,~] = timeseries(select(bag,'Time',[control_message.StartTime control_message.StartTime + 1]));
% time=ts.Time;
% 
% bagselect2 = select(bagselect, 'Time',[bagselect.StartTime bagselect.StartTime + 1], 'Topic', '/odom');

  end
  
  