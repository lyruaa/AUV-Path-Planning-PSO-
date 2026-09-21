%% 三维环境搭建
% 环境构建逻辑已抽取到 createEnvironment.m，便于复用与单独调试
createEnvironment();

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

%% 绘制三维涡流场 （Lamb-Oseen vortex）

R_O=[50,50; 115,115; 100,160;150,40;90,170;110,60; 60,70; 30,100; 140,160;  100,100; 160,50; 40,70;]; %涡流中心

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
 
 %将生成的洋流数据保存
 save('sea_x.mat','XX22');
 save('sea_y.mat','YY22');
 save('sea_z.mat','ZZ22');
 
 
 %% 主算法开始
 
 tic; %时间计算
  model=CreateModel1(XX22,YY22,ZZ22);  %创造模型
  
  model.n=10; %潜在控制点的数目
  nVar=10;
  VarSize=[1 nVar];
  %控制边界 （距离范围）
  VarMin.x=model.xmin;
  VarMax.x=model.xmax;
  VarMin.y=model.ymin;
  VarMax.y=model.ymax;
  VarMin.z=model.zmin;
  VarMax.z=model.zmax;
  
  %% PSO 粒子算法参数
  MaxIt=200;
  nPop=40;
  energy_to=zeros(1,nPop); %定义每次循环时能量矩阵
  energy_opt=zeros(MaxIt+1,1); %定义最优能量矩阵
  
  %初始化碰撞次数
  Violation=zeros(MaxIt+1,1);
  Violation_fen=zeros(1,nPop);
  
  Symbol=zeros(MaxIt,1); %碰撞次数计算
  
%velocity(预定每个方向上的速度的最值)
alpha=0.1;
max_Vel=[alpha*(x_limit(2)-x_limit(1)) alpha*(y_limit(2)-y_limit(1)) alpha*(z_limit(2)-z_limit(1))];
min_Vel=[-alpha*(x_limit(2)-x_limit(1)) -alpha*(y_limit(2)-y_limit(1)) -alpha*(z_limit(2)-z_limit(1))];  
VelMax.x=max_Vel(1);
VelMin.x=min_Vel(1);
VelMax.y=max_Vel(2);
VelMin.y=min_Vel(2);
VelMax.z=max_Vel(3);
VelMin.z=min_Vel(3);

%Create Empty Particle Structure
empty_particle.Posistion=[]; %粒子群位置
empty_particle.Velocity=[]; %速度
empty_particle.Cost=[];
empty_particle.Sol=[];

empty_particle.Best.Position=[]; %最优粒子的位置
empty_particle.Best.Cost=[]; 
empty_particle.Best.Sol=[];

%初始化全局最优粒子代价
GlobalBest.Cost=inf;

%create particle martix
particle=repmat(empty_particle,nPop,1);

for i=1:nPop
    %随机点进行初始化（在采取区间限制的方式进行位置的初始化）
    %边界限制
    xx_i=linspace(start_point(1),goal_point(1),model.n+1);
    %提取边界
    xx_low=xx_i(1,1:end-1);
    xx_upper=xx_i(1,2:end);
    
    yy_i=linspace(start_point(2),goal_point(2),model.n+1);
    yy_low=yy_i(1,1:end-1);
    yy_upper=yy_i(1,2:end);
    
    zz_i=linspace(start_point(3),goal_point(3), model.n+1);
    zz_low=zz_i(1,1:end-1);
    zz_upper=zz_i(1,2:end);

    %%进行初始的位置赋值 (对潜在的控制点在给定的区间上赋初始值)
    L_xx_low=length(xx_low);
    for  II=1:L_xx_low
        xx11(II)=xx_low(II)+rand*(xx_upper(II)-xx_low(II))+rand(1)*30;
        yy11(II)=yy_low(II)+rand*(yy_upper(II)-yy_low(II))+rand(1)*20;
        zz11(II)=zz_low(II)+rand*(zz_upper(II)-zz_low(II))+rand(1)*10;
    end
    
    %初始化位置赋值
    particle(i).Posistion.x=xx11;
    particle(i).Posistion.y=yy11;
    particle(i).Posistion.z=zz11;
    
    %初始化粒子的速度
    Velocity_x=zeros(1,model.n);
    Velocity_y=zeros(1,model.n);
    Velocity_z=zeros(1,model.n);
    %将粒子的速度存储在结构体上
    particle(i).Velocity.x=Velocity_x;
    particle(i).Velocity.y=Velocity_y;
    particle(i).Velocity.z=Velocity_z;
    
    
    %% 这里粒子速度是抽象的，不应该利用洋流进行计算
   [particle(i).Cost, particle(i).Sol, energy,violation]=CostFunction(particle(i),model);
    
   Violation_fen(1,i)=violation;
   
   energy_to(1,i)=energy;
   
   particle(i).Best.Position=particle(i).Posistion;
   particle(i).Best.Cost=particle(i).Cost;
   particle(i).Best.Sol=particle(i).Sol;
   %update Global Best
   if particle(i).Best.Cost<GlobalBest.Cost
      GlobalBest=particle(i).Best;
      %% 找到最好粒子的索引值
      best_particle_index=i;
       
   end
   
    
end
 energy_opt(1,1)=energy_to(1,best_particle_index);
    %计算在三维环境下粒子的碰撞次数
    Violation(1,1)=sum(Violation_fen);
    BestCost=zeros(MaxIt,1);
    
    
%% 为了计算车辆在运行过程中相关的信息，这里我们将每一次最优的粒子的位置信息进行初始化操作。方便后面的计算..
AUV_UNDER_POS1=[]; %行的形式进行存储,,
AUV_UNDER_POS2=[];
AUV_UNDER_POS3=[];

AUV_CONTROL_POS1=[];
AUV_CONTROL_POS2=[];
AUV_CONTROL_POS3=[];

%%   main loop
    for it=1:MaxIt
        symbol=0;
        w_max=0.9;
        w_min=0.4;
        w=((w_max-w_min)/2)*cos(pi*(it/MaxIt))+((w_max+w_min)/2);
        c_a=1;
        c_b=1.5;
        c_af=1;
        c_beta=1.5;
        
        c1=c_a*sin(pi/2*((MaxIt/2-it)/(MaxIt/2)))+c_b;
        
        c2=c_af*sin(pi/2*((it-(MaxIt/2))/(MaxIt/2)))+c_beta;
        energy_to=zeros(1,nPop);
        
        %% 算法改进，计算各个粒子之间的距离
        DISTANCE=zeros(1,nPop);
        for i=1:nPop
           dis=cal_distance(particle,nPop,i);
           DISTANCE(1,i)=dis;
        end
        % 对DISTANCE 矩阵进行处理，寻找其最值与全局最小值索引值
        d_max=max(DISTANCE);
        d_min=min(DISTANCE);
        %计算全局最优粒子的值
        d_g=DISTANCE(1,best_particle_index);
        
        %距离进化因子的计算
        E_f=(d_g-d_min)/(d_max-d_min);
        
        %对粒子群的所有粒子进行迭代操作
        for i=1:nPop
            if (E_f>=0 && E_f<0.25)
                particle(i).Velocity.x=w*particle(i).Velocity.x...
                    +c1*rand(VarSize).*(particle(i).Best.Position.x-particle(i).Posistion.x)...
                    +c2*rand(VarSize).*(GlobalBest.Position.x-particle(i).Posistion.x);
                particle(i).Velocity.y=w*particle(i).Velocity.y...
                    +c1*rand(VarSize).*(particle(i).Best.Position.y-particle(i).Posistion.y)...
                    +c2*rand(VarSize).*(GlobalBest.Position.y-particle(i).Posistion.y);
                particle(i).Velocity.z=w*particle(i).Velocity.z...
                    +c1*rand(VarSize).*(particle(i).Best.Position.z-particle(i).Posistion.z)...
                    +c2*rand(VarSize).*(GlobalBest.Position.z-particle(i).Posistion.z);
                %更新速度边界x,y,z;
                particle(i).Velocity.x=max(particle(i).Velocity.x, VelMin.x);
                particle(i).Velocity.x=min(particle(i).Velocity.x, VelMax.x);
                
                particle(i).Velocity.y=max(particle(i).Velocity.y, VelMin.y);
                particle(i).Velocity.y=min(particle(i).Velocity.y, VelMax.y);
                
                particle(i).Velocity.z=max(particle(i).Velocity.z,VelMin.z);
                particle(i).Velocity.z=min(particle(i).Velocity.z, VelMax.z);
                
                %位置更新
                particle(i).Posistion.x=particle(i).Posistion.x+particle(i).Velocity.x;
                particle(i).Posistion.y=particle(i).Posistion.y+particle(i).Velocity.y;
                particle(i).Posistion.z=particle(i).Posistion.z+particle(i).Velocity.z;
                
                %velocity mirroring
                OutOfTheRange=(particle(i).Posistion.x<VarMin.x | particle(i).Posistion.x> VarMax.x);
                particle(i).Velocity.x(OutOfTheRange)=-particle(i).Velocity.x(OutOfTheRange);
                
                 OutOfTheRange=(particle(i).Posistion.y<VarMin.y | particle(i).Posistion.y> VarMax.y);
                particle(i).Velocity.y(OutOfTheRange)=-particle(i).Velocity.y(OutOfTheRange);
                
                 OutOfTheRange=(particle(i).Posistion.z<VarMin.z | particle(i).Posistion.z> VarMax.z);
                particle(i).Velocity.z(OutOfTheRange)=-particle(i).Velocity.z(OutOfTheRange);
                
                %update position boundary
                particle(i).Posistion.x=max(particle(i).Posistion.x, model.xmin);
                particle(i).Posistion.x=min(particle(i).Posistion.x,model.xmax);
                
                particle(i).Posistion.y=max(particle(i).Posistion.y, model.ymin);
                particle(i).Posistion.y=min(particle(i).Posistion.y, model.ymax);
                
                particle(i).Posistion.z=max(particle(i).Posistion.z, model.zmin);
                particle(i).Posistion.z=min(particle(i).Posistion.z, model.zmax);
                
            elseif E_f>=0.25 && E_f<=1
                symbol=symbol+1;
               %% 设计跳出机制
               jump_to(particle(i),best_particle_index,i,model);
                particle(i).Velocity.x=w*particle(i).Velocity.x...
                    +c1*rand(VarSize).*(particle(i).Best.Position.x-particle(i).Posistion.x)...
                    +c2*rand(VarSize).*(GlobalBest.Position.x-particle(i).Posistion.x);
                particle(i).Velocity.y=w*particle(i).Velocity.y...
                    +c1*rand(VarSize).*(particle(i).Best.Position.y-particle(i).Posistion.y)...
                    +c2*rand(VarSize).*(GlobalBest.Position.y-particle(i).Posistion.y);
                particle(i).Velocity.z=w*particle(i).Velocity.z...
                    +c1*rand(VarSize).*(particle(i).Best.Position.z-particle(i).Posistion.z)...
                    +c2*rand(VarSize).*(GlobalBest.Position.z-particle(i).Posistion.z);
                
                %更新速度边界x,y,z;
                particle(i).Velocity.x=max(particle(i).Velocity.x, VelMin.x);
                particle(i).Velocity.x=min(particle(i).Velocity.x, VelMax.x);
                
                particle(i).Velocity.y=max(particle(i).Velocity.y, VelMin.y);
                particle(i).Velocity.y=min(particle(i).Velocity.y, VelMax.y);
                
                particle(i).Velocity.z=max(particle(i).Velocity.z,VelMin.z);
                particle(i).Velocity.z=min(particle(i).Velocity.z, VelMax.z);
                
                   %位置更新
                particle(i).Posistion.x=particle(i).Posistion.x+particle(i).Velocity.x;
                particle(i).Posistion.y=particle(i).Posistion.y+particle(i).Velocity.y;
                particle(i).Posistion.z=particle(i).Posistion.z+particle(i).Velocity.z;
                
                %velocity mirroring
                OutOfTheRange=(particle(i).Posistion.x<VarMin.x | particle(i).Posistion.x> VarMax.x);
                particle(i).Velocity.x(OutOfTheRange)=-particle(i).Velocity.x(OutOfTheRange);
                
                 OutOfTheRange=(particle(i).Posistion.y<VarMin.y | particle(i).Posistion.y> VarMax.y);
                particle(i).Velocity.y(OutOfTheRange)=-particle(i).Velocity.y(OutOfTheRange);
                
                 OutOfTheRange=(particle(i).Posistion.z<VarMin.z | particle(i).Posistion.z> VarMax.z);
                particle(i).Velocity.z(OutOfTheRange)=-particle(i).Velocity.z(OutOfTheRange);
                
                %update position boundary
                particle(i).Posistion.x=max(particle(i).Posistion.x, model.xmin);
                particle(i).Posistion.x=min(particle(i).Posistion.x,model.xmax);
                
                particle(i).Posistion.y=max(particle(i).Posistion.y, model.ymin);
                particle(i).Posistion.y=min(particle(i).Posistion.y, model.ymax);
                
                particle(i).Posistion.z=max(particle(i).Posistion.z, model.zmin);
                particle(i).Posistion.z=min(particle(i).Posistion.z, model.zmax);
                
                
            end %end for if(E_f
            
            %计算代价值
            [particle(i).Cost,particle(i).Sol, energy, violation]=CostFunction(particle(i), model);
            %进行碰撞次数的统计
            Violation_fen(1,i)=violation;
            energy_to(1,i)=energy;
            
            %update personal best
            if particle(i).Cost<particle(i).Best.Cost
                %如果成立，说明当前更新后的粒子位置更优，应该更新个体最优粒子
                particle(i).Best.Position=particle(i).Posistion;
                particle(i).Best.Cost=particle(i).Cost;
                particle(i).Best.Sol=particle(i).Sol;
                
                %% Update Global best
                if particle(i).Best.Cost<GlobalBest.Cost
                    GlobalBest=particle(i).Best;
                    best_particle_index=i; 
                end
               
            end
            
        end %end for i=1:nPop
        
        %为了计算AUV信息，将当前进化中最优的粒子的信息存储在定义的数组上
        if (it>=MaxIt-2 && it<=MaxIt)
        AUV_UNDER_POS1=[AUV_UNDER_POS1 GlobalBest.Sol.xx];
        AUV_UNDER_POS2=[AUV_UNDER_POS2 GlobalBest.Sol.yy];
        AUV_UNDER_POS3=[AUV_UNDER_POS3 GlobalBest.Sol.zz];
        else
            
        end
        
        %存储当前最优的位置点
        if (it>495 && it<=MaxIt)
        AUV_CONTROL_POS1 =[AUV_CONTROL_POS1 GlobalBest.Sol.XS];
        AUV_CONTROL_POS2= [AUV_CONTROL_POS2 GlobalBest.Sol.YS];
        AUV_CONTROL_POS3= [AUV_CONTROL_POS3 GlobalBest.Sol.ZS];
        else
            
        end
        %在所有粒子更新完一轮后，总结本轮粒子的效果
        energy_opt(it+1,1)=energy_to(best_particle_index);
        Violation(it+1,1)=sum(Violation_fen); 
        %update best cost
        BestCost(it)=GlobalBest.Cost;
        
         % Show Iteration Information
       if GlobalBest.Sol.IsFeasible
          Flag=' @';
       else
          Flag=[', ' num2str(GlobalBest.Sol.Violation)];
       end
       % 进行路径平滑
      % sol = path_smooth(it);
      
       disp(['Iteration ' num2str(it)]);
        %Plot solution
       % PlotSolution(GlobalBest.Sol,model);
       %% 在迭代过程中进行本次迭代最优路径的绘制
       sol=GlobalBest.Sol;
      
       view(-40,30)
       %绘制离散点
       H1=plot3(sol.XS, sol.YS, sol.ZS,'bo','LineWidth',4,...
      'MarkerEdgeColor','y',...
      'MarkerFaceColor','y',...
      'MarkerSize',4 );
       H2=plot3(sol.xx , sol.yy, sol.zz, '--ks', 'LineWidth',2, 'MarkerEdgeColor','k',...
      'MarkerFaceColor','g',...
       'MarkerSize', 2);
       pause(0.2);
       if it~=MaxIt
       delete(H1);
       delete(H2);
       else
           %如果等于最大次数，不删除，保留
       end
       
        Symbol(it,1)=symbol;
   
    end % end for it=1:MaxIt

   
 
 
 
 
 
