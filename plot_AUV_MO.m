%% 本代码绘制车辆的运动信息，计算的数据来源于每一次迭代中最优的粒子,,

pos_x=load('AUV_DATA1.mat');
POS_X=pos_x.AUV_UNDER_POS1;
pos_y=load('AUV_DATA2.mat');
POS_Y=pos_y.AUV_UNDER_POS2;
pos_z=load('AUV_DATA3.mat');
POS_Z=pos_z.AUV_UNDER_POS3;

pos_p=load('AUV_DATA11.mat');
POS_P=pos_p.AUV_UNDER_POS11;
pos_q=load('AUV_DATA22.mat');
POS_Q=pos_q.AUV_UNDER_POS22;
pos_r=load('AUV_DATA33.mat');
POS_R=pos_r.AUV_UNDER_POS33;
T=88.3106;  %算法运行时间
T1=89.3869;


%计算路径点的长度
L_en=length(POS_X);
T=linspace(1,T,L_en/3-1);
theta=[];
FAI=[];
for j=1:3
    pos_x=POS_X(100*(j-1)+1: 100*j);
    pos_y=POS_Y(100*(j-1)+1: 100*j);
    pos_z=POS_Z(100*(j-1)+1: 100*j);
for i=1:L_en/3-1
   %抓取前后两个点
   pos_x1=pos_x(i);
   pos_x2=pos_x(i+1);
   
   pos_y1=pos_y(i);
   pos_y2=pos_y(i+1);
   
   pos_z1=pos_z(i);
   pos_z2=pos_z(i+1);
   
   %利用公式计算角度变化
   FAI1=atan(abs(pos_y2-pos_y1)/abs(pos_x2-pos_x1));
   FAI(j,i)=FAI1;
   
   dist=sqrt((pos_x2-pos_x1)^2+(pos_y2-pos_y1)^2);
   theta1=atan(-abs(pos_z2-pos_z1)/dist);
   theta(j,i)=theta1;
   
end
end

%计算路径点的长度
L_en1=length(POS_P);
T1=linspace(1,T1,L_en1/3-1);
theta_c=[];
FAI_c=[];
for j=1:3
    pos_x=POS_P(100*(j-1)+1: 100*j);
    pos_y=POS_Q(100*(j-1)+1: 100*j);
    pos_z=POS_R(100*(j-1)+1: 100*j);
for i=1:L_en/3-1
   %抓取前后两个点
   pos_x1=pos_x(i);
   pos_x2=pos_x(i+1);
   
   pos_y1=pos_y(i);
   pos_y2=pos_y(i+1);
   
   pos_z1=pos_z(i);
   pos_z2=pos_z(i+1);
   
   %利用公式计算角度变化
   FAI1=atan(abs(pos_y2-pos_y1)/abs(pos_x2-pos_x1));
   FAI_c(j,i)=FAI1;
   
   dist=sqrt((pos_x2-pos_x1)^2+(pos_y2-pos_y1)^2);
   theta1=atan(-abs(pos_z2-pos_z1)/dist);
   theta_c(j,i)=theta1;
   
end
end


%%   进行速度的计算。只计算最后一次。。
% 计算车辆本身速度
FA=FAI(3,:);
TH=theta(3,:);
u=[];
v=[];
w=[];   %AUV的本身速度..
v_max=6;
for i=1:length(FA)
   u(i)=v_max*cos(TH(i))*cos(FA(i));
   v(i)=v_max*cos(TH(i))*sin(FA(i));
   w(i)=v_max*sin(TH(i));
end

%% 计算在洋流下其速度值
L=length(model.current_x);
for j=3
    pos_x=POS_X(100*(j-1)+1: 100*j);
    pos_y=POS_Y(100*(j-1)+1: 100*j);
    pos_z=POS_Z(100*(j-1)+1: 100*j);
for i=1:L_en/3-1
   %抓取前后两个点
   pos_x1=pos_x(i);
   pos_x2=pos_x(i+1);
   
   pos_y1=pos_y(i);
   pos_y2=pos_y(i+1);
   
   pos_z1=pos_z(i);
   pos_z2=pos_z(i+1);
   
    x_index_0=int8(L/200*pos_x1)+1;
    y_index_0=int8(L/200*pos_y1)+1;
    x_index_1=int8(L/200*pos_x2)+1;
    y_index_1=int8(L/200*pos_y2)+1;
    
    %洋流边界限制
    if x_index_0>L
        x_index_0=L;
    end
    if y_index_0>L
        y_index_0=L;
    end
    if x_index_0<=1
        x_index_0=1;
    end
    if y_index_0<=1
        y_index_0=1;
    end
   %--
   if x_index_1>L
        x_index_1=L;
    end
    if y_index_1>L
        y_index_1=L;
    end
    if x_index_1<=1
        x_index_1=1;
    end
    if y_index_1<=1
        y_index_1=1;
    end
   %%%%%%%%%%%%%%%%%%%%%
    %针对前一个位置 current_position_z1
    if pos_z1>=-5 &&pos_z1<0
       v_sea_x1=model.current_x(x_index_0,y_index_0,1);
       v_sea_y1=model.current_y(x_index_0,y_index_0,1); 
       v_sea_x2=model.current_x(x_index_0,y_index_0,2);
       v_sea_y2=model.current_y(x_index_0,y_index_0,2);
       v_sea_x=(v_sea_x1+v_sea_x2)/2;
       v_sea_y=(v_sea_y1+v_sea_y2)/2;
       v_sea_z=0;
    elseif pos_z1>=0 && pos_z1<5
        v_sea_x1=model.current_x(x_index_0,y_index_0,2);
        v_sea_y1=model.current_y(x_index_0,y_index_0,2);
        v_sea_x2=model.current_x(x_index_0,y_index_0,3);
        v_sea_y2=model.current_y(x_index_0,y_index_0,3);
        v_sea_x=(v_sea_x1+v_sea_x2)/2;
        v_sea_y=(v_sea_y1+v_sea_y2)/2;
        v_sea_z=0;
    else 
        v_sea_x1=model.current_x(x_index_0,y_index_0,3);
        v_sea_y1=model.current_y(x_index_0,y_index_0,3);
        v_sea_x2=model.current_x(x_index_0,y_index_0,4);
        v_sea_y2=model.current_y(x_index_0,y_index_0,4);
        v_sea_x=(v_sea_x1+v_sea_x2)/2;
        v_sea_y=(v_sea_y1+v_sea_y2)/2;
        v_sea_z=0;
    end
    
    %针对前一个位置 current_position_z2
    if pos_z2>=-5 &&pos_z2<0
       v_sea_x1=model.current_x(x_index_1,y_index_1,1);
       v_sea_y1=model.current_y(x_index_1,y_index_1,1); 
       v_sea_x2=model.current_x(x_index_1,y_index_1,2);
       v_sea_y2=model.current_y(x_index_1,y_index_1,2);
       v_sea_x11=(v_sea_x1+v_sea_x2)/2;
       v_sea_y11=(v_sea_y1+v_sea_y2)/2;
       v_sea_z11=0;
    elseif pos_z2>=0 && pos_z2<5
        v_sea_x1=model.current_x(x_index_1,y_index_1,2);
        v_sea_y1=model.current_y(x_index_1,y_index_1,2);
        v_sea_x2=model.current_x(x_index_1,y_index_1,3);
        v_sea_y2=model.current_y(x_index_1,y_index_1,3);
        v_sea_x11=(v_sea_x1+v_sea_x2)/2;
        v_sea_y11=(v_sea_y1+v_sea_y2)/2;
        v_sea_z11=0;
    else 
        v_sea_x1=model.current_x(x_index_1,y_index_1,3);
        v_sea_y1=model.current_y(x_index_1,y_index_1,3);
        v_sea_x2=model.current_x(x_index_1,y_index_1,4);
        v_sea_y2=model.current_y(x_index_1,y_index_1,4);
        v_sea_x11=(v_sea_x1+v_sea_x2)/2;
        v_sea_y11=(v_sea_y1+v_sea_y2)/2;
        v_sea_z11=0;
    end
    
    %% 这里在洋流的计算上，取前后两个值的平均值作为这条路洋流的大小
    v_sea_x=(v_sea_x+v_sea_x11)/2;
    v_sea_y=(v_sea_y+v_sea_y11)/2;
    v_sea_z=(v_sea_z+v_sea_z11)/2;
    u1(i)=u(i)+v_sea_x;
    v1(i)=v(i)+v_sea_y;
    w1(i)=w(i)+v_sea_z;
   %%%%%%%%%%%%%%%%%%
   
   
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   进行速度的计算。只计算最后一次。。
% 计算车辆本身速度
FA=FAI_c(3,:);
TH=theta_c(3,:);
u_C=[];
v_C=[];
w_C=[];
v_max=6;
for i=1:length(FA)
   u_c(i)=v_max*cos(TH(i))*cos(FA(i));
   v_c(i)=v_max*cos(TH(i))*sin(FA(i));
   w_c(i)=v_max*sin(TH(i));
    
end

%% 计算在洋流下其速度值
L=length(model.current_x);
for j=3
    pos_x=POS_P(100*(j-1)+1: 100*j);
    pos_y=POS_Q(100*(j-1)+1: 100*j);
    pos_z=POS_R(100*(j-1)+1: 100*j);
for i=1:L_en/3-1
   %抓取前后两个点
   pos_x1=pos_x(i);
   pos_x2=pos_x(i+1);
   
   pos_y1=pos_y(i);
   pos_y2=pos_y(i+1);
   
   pos_z1=pos_z(i);
   pos_z2=pos_z(i+1);
   
    x_index_0=int8(L/200*pos_x1)+1;
    y_index_0=int8(L/200*pos_y1)+1;
    x_index_1=int8(L/200*pos_x2)+1;
    y_index_1=int8(L/200*pos_y2)+1;
    
    %洋流边界限制
    if x_index_0>L
        x_index_0=L;
    end
    if y_index_0>L
        y_index_0=L;
    end
    if x_index_0<=1
        x_index_0=1;
    end
    if y_index_0<=1
        y_index_0=1;
    end
   %--
   if x_index_1>L
        x_index_1=L;
    end
    if y_index_1>L
        y_index_1=L;
    end
    if x_index_1<=1
        x_index_1=1;
    end
    if y_index_1<=1
        y_index_1=1;
    end
   %%%%%%%%%%%%%%%%%%%%%
    %针对前一个位置 current_position_z1
    if pos_z1>=-5 &&pos_z1<0
       v_sea_x1=model.current_x(x_index_0,y_index_0,1);
       v_sea_y1=model.current_y(x_index_0,y_index_0,1); 
       v_sea_x2=model.current_x(x_index_0,y_index_0,2);
       v_sea_y2=model.current_y(x_index_0,y_index_0,2);
       v_sea_x=(v_sea_x1+v_sea_x2)/2;
       v_sea_y=(v_sea_y1+v_sea_y2)/2;
       v_sea_z=0;
    elseif pos_z1>=0 && pos_z1<5
        v_sea_x1=model.current_x(x_index_0,y_index_0,2);
        v_sea_y1=model.current_y(x_index_0,y_index_0,2);
        v_sea_x2=model.current_x(x_index_0,y_index_0,3);
        v_sea_y2=model.current_y(x_index_0,y_index_0,3);
        v_sea_x=(v_sea_x1+v_sea_x2)/2;
        v_sea_y=(v_sea_y1+v_sea_y2)/2;
        v_sea_z=0;
    else 
        v_sea_x1=model.current_x(x_index_0,y_index_0,3);
        v_sea_y1=model.current_y(x_index_0,y_index_0,3);
        v_sea_x2=model.current_x(x_index_0,y_index_0,4);
        v_sea_y2=model.current_y(x_index_0,y_index_0,4);
        v_sea_x=(v_sea_x1+v_sea_x2)/2;
        v_sea_y=(v_sea_y1+v_sea_y2)/2;
        v_sea_z=0;
    end
    
    %针对前一个位置 current_position_z2
    if pos_z2>=-5 &&pos_z2<0
       v_sea_x1=model.current_x(x_index_1,y_index_1,1);
       v_sea_y1=model.current_y(x_index_1,y_index_1,1); 
       v_sea_x2=model.current_x(x_index_1,y_index_1,2);
       v_sea_y2=model.current_y(x_index_1,y_index_1,2);
       v_sea_x11=(v_sea_x1+v_sea_x2)/2;
       v_sea_y11=(v_sea_y1+v_sea_y2)/2;
       v_sea_z11=0;
    elseif pos_z2>=0 && pos_z2<5
        v_sea_x1=model.current_x(x_index_1,y_index_1,2);
        v_sea_y1=model.current_y(x_index_1,y_index_1,2);
        v_sea_x2=model.current_x(x_index_1,y_index_1,3);
        v_sea_y2=model.current_y(x_index_1,y_index_1,3);
        v_sea_x11=(v_sea_x1+v_sea_x2)/2;
        v_sea_y11=(v_sea_y1+v_sea_y2)/2;
        v_sea_z11=0;
    else 
        v_sea_x1=model.current_x(x_index_1,y_index_1,3);
        v_sea_y1=model.current_y(x_index_1,y_index_1,3);
        v_sea_x2=model.current_x(x_index_1,y_index_1,4);
        v_sea_y2=model.current_y(x_index_1,y_index_1,4);
        v_sea_x11=(v_sea_x1+v_sea_x2)/2;
        v_sea_y11=(v_sea_y1+v_sea_y2)/2;
        v_sea_z11=0;
    end
    
    %% 这里在洋流的计算上，取前后两个值的平均值作为这条路洋流的大小
    v_sea_x=(v_sea_x+v_sea_x11)/2;
    v_sea_y=(v_sea_y+v_sea_y11)/2;
    v_sea_z=(v_sea_z+v_sea_z11)/2;
    u_C(i)=u_c(i)+v_sea_x;
    v_C(i)=v_c(i)+v_sea_y;
    w_C(i)=w_c(i)+v_sea_z;
   %%%%%%%%%%%%%%%%%%
   
   
end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%进行速度的绘制
figure(13)
plot(T,u,T,v,T,w);
hold on;
plot(T,u1,T,v1,T,w1);
%plot(T1,u_C,T1,v_C,T,w_C);
hold on;
legend('u1','u_C');



% 进行角度的绘制

%FAI
figure(10)
for j=1:3

plot(T,FAI(j,:),'b',T,theta(j,:),'r');
plot(T1,FAI_c(j,:),'y',T1,theta(j,:),'g');
hold on
end
legend('FAI','THETA','FAI_C','THETA_C')
