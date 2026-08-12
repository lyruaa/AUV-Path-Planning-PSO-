%% 三维环境搭建
global x_limit;
global y_limit;
global z_limit;

x_limit=[0,200];
y_limit=[0,200];
z_limit=[-5,10];

plot3(x_limit(1),x_limit(2),y_limit(1),y_limit(2),z_limit(1),z_limit(2));
grid on
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')
hold on

%加载复杂的地形信息
global x_sea;
global y_sea;
global z_sea;

A=load('XYZmesh.mat');
x_sea=A.X .*2;
y_sea=A.Y .*2;
%将高度信息归一化，防止越过一定的高度
z_sea=A.Z;
h_max=max(max(z_sea)); %提取高度的最大值
h_min=min(min(z_sea)); %提取高度的最小值

L_attitude=length(x_sea);
for i=1:L_attitude
   for j=1:L_attitude
      z_sea(i,j)=(z_sea(i,j)-h_min)/(h_max-h_min);    
   end
end
z_sea=z_sea.*10-5;  %扩展其高度
mesh(x_sea,y_sea,z_sea');
axis([x_limit(1) x_limit(2) y_limit(1) y_limit(2) z_limit(1) z_limit(2)]);
colormap jet;
grid off;
hold on

%突起的最高处达到了5m.最低处为0m，在论文中这个点要体现出来.

%定义起始点
global start_point; %起始点
global goal_point;
start_point=[5,5,5];
goal_point=[160,180,0];
plot3(start_point(1),start_point(2),start_point(3), 'mh','MarkerSize',10,'MarkerFaceColor','m');
hold on;
plot3(goal_point(1),goal_point(2),goal_point(3),  'kh','MarkerSize',10,'MarkerFaceColor','g');
hold on;

%构造静态的球形与长方体障碍物
%global obs_cir;
%global obs_34;
global center_circle; %球体中心
global radius_circle; %球体半径
global size1;      
global origin;

%球形障碍物
color='k';
center_circle=[35,40,4;50,40,4;100,70,5;120,110,6;110,130,6;150,150,6];
radius_circle=[4,3,4,4,4,4];
n_circles=size(center_circle,1);
for i=1:n_circles
[x,y,z]=sphere();
obs(i)=patch(radius_circle(i)*x+center_circle(i,1),radius_circle(i)*y+center_circle(i,2),radius_circle(i)*z+center_circle(i,3),color);
axis equal
end
%obs_cir=[obs(1),obs(2),obs(3),obs(4),obs(5),obs(6)];
%长方体障碍物

%size一行表示一个长方体的长宽高
size1=[6,4,4;7,6,4;5,7,4;6,3,5];
%orgin一行表示长方体体心位置
origin=[90,80,3;80,110,4;80,120,3;135,150,6];

for i=1:4
x=([0 1 1 0 0 0;1 1 0 0 1 1;1 1 0 0 1 1;0 1 1 0 0 0]-0.5)*size1(i,1)+origin(i,1);
y=([0 0 1 1 0 0;0 1 1 0 0 0;0 1 1 0 1 1;0 0 1 1 1 1]-0.5)*size1(i,2)+origin(i,2);
z=([0 0 0 0 0 1;0 0 0 0 0 1;1 1 1 1 0 1;1 1 1 1 0 1]-0.5)*size1(i,3)+origin(i,3);

for I=1:6
    %M=[x(:,i),y(:,i),z(:,i)]
    h(i,I)=patch(x(:,I),y(:,I),z(:,I),[0 0 0]);
   % set(h,'edgecolor','k','facealpha',1)
end
axis equal
view(37.5,30)
end

%% 画一个三棱柱
global tria;
%tria=[45 60 0;60 60 0;55 75 0;45 60 10;55 75 10;60 60 10];
tria=[35 60 0;50 60 0; 45 75 0; 35 60 8; 45 75 8; 50 60 8];
f= [1 2 3 1;1 2 6 4;2 3 5 6;1 3 5 4;4 5 6 4];
patch('Faces',f,'Vertices',tria,'FaceColor','k');
view(37.5,30)
hold on

%% 画一个圆柱体
global ZHU
ZHU=[95,100,6,0,10];
[a,z]=ndgrid((0:.05:1)*2*pi,0:.05:10);
x=6*cos(a)+95;
y=6*sin(a)+100;
surf(x,y,z,x*0,'linestyle','none','Facealpha',1,'FaceColor','k')
hold on
[a,r]=ndgrid((0:.05:1)*2*pi,[0 1]);
x=6*cos(a).*r+95;
y=6*sin(a).*r+100;
surf(x,y,x*0,x*0,'linestyle','none','Facealpha',1,'FaceColor','k')
surf(x,y,x*0+10,x*0,'linestyle','none','Facealpha',1,'FaceColor','k')
%hold off
grid on
%view(70,60)
view(37.5,30)
%将长方体信息存储
obs_34=h;
