%绘制对比路径
PATH1=load('path1.mat','GlobalBest');
PATH2=load('path11.mat','GlobalBest');
path1=PATH1.GlobalBest;
path2=PATH2.GlobalBest;

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
global obs_cir;
global obs_34;
global center_circle;
global radius_circle;
global size1;
global origin;

%球形障碍物
color='k';
center_circle=[30,40,4;50,40,4;100,70,5;120,110,6;80,140,6;150,150,6];
radius_circle=[4,3,4,4,4,4];
n_circles=size(center_circle,1);
for i=1:n_circles
[x,y,z]=sphere();
obs(i)=patch(radius_circle(i)*x+center_circle(i,1),radius_circle(i)*y+center_circle(i,2),radius_circle(i)*z+center_circle(i,3),color);
axis equal
end
obs_cir=[obs(1),obs(2),obs(3),obs(4),obs(5),obs(6)];
%长方体障碍物

%size一行表示一个长方体的长宽高
size1=[6,4,4;7,6,4;5,7,4;6,3,4];
%orgin一行表示长方体体心位置
origin=[90,80,3;80,110,4;80,120,3;130,150,4];

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
ZHU=[90,100,6,0,10];
[a,z]=ndgrid((0:.05:1)*2*pi,0:.05:10);
x=6*cos(a)+90;
y=6*sin(a)+100;
surf(x,y,z,x*0,'linestyle','none','Facealpha',1,'FaceColor','k')
hold on
[a,r]=ndgrid((0:.05:1)*2*pi,[0 1]);
x=6*cos(a).*r+90;
y=6*sin(a).*r+100;
surf(x,y,x*0,x*0,'linestyle','none','Facealpha',1,'FaceColor','k')
surf(x,y,x*0+10,x*0,'linestyle','none','Facealpha',1,'FaceColor','k')
%hold off
grid on
%view(70,60)
view(37.5,30)
%将长方体信息存储
obs_34=h;

%% 绘制三维涡流场 （Lamb-Oseen vortex）

R_O=[50,50; 115,115; 100,160; 60,70; 30,100; 140,160;  100,100; 160,50; 40,70;150,40;90,170;110,60]; %涡流中心

RADIUS=10; %涡流半径
S=8;  %涡流力量

%利用公式计算涡流
r=0:4:200;
H=-5:5:10; %这里由于海底凸起最高为0.所以我们的洋流最低从0开始

[x_o,y_o]=meshgrid(r);
L_o=size(x_o,1);
H_O=size(H,2);
q_x_o=zeros(L_o,L_o,H_O);
q_y_o=zeros(L_o,L_o,H_O);
q_z_o=zeros(L_o,L_o,H_O);

X_O=linspace(x_limit(1),x_limit(2),L_o);
Y_O=linspace(y_limit(1),y_limit(2),L_o);

%对每一层的洋流分别计算
for h=1:H_O
   for i=1:L_o
       q_x_o(i,:,h)=X_O;
   end
end

for h=1:H_O
   for j=1:L_o
    q_y_o(:,j,h)=Y_O;
   end
end

for h=1:H_O
   q_z_o(:,:,h)=5*(h-2); 
end

%计算每一层的洋流值 （Lamb-Oseen vortex）

%初始化 预定的洋流大小  
XX22=zeros(L_o,L_o,H_O);
YY22=zeros(L_o,L_o,H_O);
ZZ22=zeros(L_o,L_o,H_O);

for h=1:H_O %在不同的高度上分别计算
         XX1=zeros(L_o,L_o);
         YY1=zeros(L_o,L_o);
         %% 这里分别计算三个方向上的洋流值
          
         %由于计算涡流，分别根据涡流中心计算不同位置处的涡流
         for k=1:3 %每一层都有三个涡流中心
            R_O1=R_O(1:3,:);
            %循环生成涡流
            W=zeros(L_o,L_o);
            W1=zeros(L_o,L_o);
            W(:,:)=R_O(k,1);
            W1(:,:)=R_O(k,2);
           xx=x_o-W;
           yy=y_o-W1;
           rr1=xx.^2+yy.^2;
           %Lamb-Oseen vortex
           %XX=-yy./rr1.*(1-exp(-rr1));  %生成涡流场的核心公式
           XX=-RADIUS*yy./(2*pi*rr1).*(1-exp(-rr1./(S*S)));
           %YY=xx./rr1.*(1-exp(-rr1));
           YY=RADIUS*xx./(2*pi*rr1).*(1-exp(-rr1./(S*S)));
           XX1=XX+XX1;
           YY1=YY+YY1;   
         end
          XX22(:,:,h)=(XX1+0.05)*15; %随着高度上升，洋流逐渐变大
          YY22(:,:,h)=(YY1+0.05)*15;
          
end

 for i=1:H_O
    [m,n]=size(q_x_o(:,:,1));
    W=zeros(m,n);
    ZZ22(:,:,h)=W;
 end
 quiver3(q_x_o,q_y_o,q_z_o,XX22,YY22,ZZ22,'b');
 view(37.5,30)
 %% 环境创建结束
 
 %提取第一个路径的控制点
 XS=path1.XS;
 YS=path1.YS;
 ZS=path1.ZS;
 %提取第一条路径的微元点
 xx=path1.xx;
 yy=path1.yy;
 zz=path1.zz;
 
 %提取第二条路径的控制点
 XS1=path2.XS;
 YS1=path2.YS;
 ZS1=path2.ZS;
 xx1=path2.xx;
 yy1=path2.yy;
 zz1=path2.zz;
 
   plot3(XS,YS,ZS,'bo','LineWidth',4,...
      'MarkerEdgeColor','y',...
      'MarkerFaceColor','y',...
      'MarkerSize',4 );
  plot3(xx,yy,zz, '--rs', 'LineWidth',2, 'MarkerEdgeColor','k',...
      'MarkerFaceColor','g',...
       'MarkerSize', 2);
 
   plot3(XS1,YS1,ZS1,'bo','LineWidth',4,...
      'MarkerEdgeColor','y',...
      'MarkerFaceColor','y',...
      'MarkerSize',4 );
  plot3(xx1,yy1,zz1, '--ks', 'LineWidth',2, 'MarkerEdgeColor','k',...
      'MarkerFaceColor','g',...
       'MarkerSize', 2);
 
 
 