[X,Y] = meshgrid(-3:3);
V = peaks(7);
% figure (1)
% surf(X,Y,V)
% title('Original Sampling');
[Xq,Yq] = meshgrid(-3:0.25:3);
Vq = interp2(X,Y,V,Xq,Yq,'cubic');
% figure (2)
% surf(Xq,Yq,Vq);
% title('Cubic Interpolation Over Finer Grid');
% 
% reshape(repmat([4 5 6;16 19 20;10 18 23],1,2),[3,3,2]);s
% repmat([4 5 6;16 19 20;10 18 23],1,2)

fid = fopen('/home/cidi-xh/xuhao/Codes/matlab/CIDI_control/Longitude/apollo_lon/two_datatest.csv');%query time0.8，change the way of computing errors
cell_title = textscan(fid, '%s %s %s %s',1 , 'delimiter', ',');
cell_data  = textscan(fid, '%.9f %.9f %.9f %.9f','delimiter', ',');
fclose(fid);

data = cell2mat(cell_data);
u = data(:,1);
v = data(:,2);
a_10t = data(:,3);
a_20t = data(:,4);
a = [a_10t ;a_20t];
a2 = [a_10t a_20t];
% table_v(:) = [a v];
v(1:6,1)
V1 = [v a_10t];
V2 = [v a_20t];
V_table(:,:,1) = V1;
V_table(:,:,2) = V2;
V_table
size(V_table)
table_2D10 = [a_10t(1:6,1) a_10t(7:12,1)];%一个质量对应一个这样的表
table_2D20 = [a_20t(1:6,1) a_20t(7:12,1)];
table_3D(:,:,1) = table_2D10;
table_3D(:,:,2) = table_2D20;

% table_2D_minus60 = size(a_10t);%size(v(1:6,1),1)
% for i = 1:1:size(a_10t)
%     for j = 1:1:size(v(1:6,1),1)
%         table_2D_minus60(i,j) = -60;
%         table_2D_minus50(i,j) = -50;
%     end
% end
% 
% 
% % table_2D_minus50 = size(a_20t);%
% 
% table_3Dfor_mass(:,:,1) = table_2D_minus60; 
% table_3Dfor_mass(:,:,2) = table_2D_minus50;

a_space = (linspace(-3.3,-1.0,12))';
% u_0 = simout8(1,:)';
% u_02 = simout9(1,:)';
% u_04 = simout10(1,:)';
% u_06 = simout11(1,:)';
% u_08 = simout12(1,:)';
% u_10 = simout13(1,:)';
% 
% table_2D_minus60 = [u_0 u_02 u_04 u_06 u_08 u_10];
% table_2D_minus50 = [u_0 u_02 u_04 u_06 u_08 u_10]*0.6;
% 
% table_3Dfor_mass(:,:,1) = table_2D_minus60; 
% table_3Dfor_mass(:,:,2) = table_2D_minus50;
v_space = v(1:6,1);
u_space = [-60 ; -50];


table_2D_minus60 = size(a_10t);%size(v(1:6,1),1)
for i = 1:1:size(a_10t)
    for j = 1:1:size(v(1:6,1),1)
        table_2D_minus60(i,j) = 10;
        table_2D_minus50(i,j) = 20;
    end
end


% table_2D_minus50 = size(a_20t);%



table_2D_minus60=10+(20-10)*rand(12,6);  %rand(n,m)生成n行，m列的0-1之间的随机数
table_2D_minus50=table_2D_minus60 * 0.88;

table_3Dfor_mass(:,:,1) = table_2D_minus60; 
table_3Dfor_mass(:,:,2) = table_2D_minus50;


























x = [1;2;3];
y = [1;2;3];
table = [4 5 6;16 19 20;10 18 23];
a1(:,:,1) = [2 3 1 8 ;4 5 3 10];
a1(:,:,2)=[2 3 1 6 ;4 5 3 5]/2;
a1
size(a1)

% 
% [X,Y,Z,V] = flow(10);
% figure (1)
% slice(X,Y,Z,V,[6 9],2,0);
% shading flat
% %Create a query grid with spacing of 0.25.
% [Xq,Yq,Zq] = meshgrid(.1:.25:10,-3:.25:3,-3:.25:3);
% Vq = interp3(X,Y,Z,V,Xq,Yq,Zq);
% figure (2)
% slice(Xq,Yq,Zq,Vq,[6 9],2,0);
% shading flat


