function [z,sol, E1, Violation_num]=CostFunction(particle,model)
  %cost function 
  %罚函数思想。 参数代表各自的重要性
  beta1=2;
  beta2=2;  %洋流的权重因子..
  beta3=0.8;  %障碍物的碰撞权重因子
  
  c_d=3; %车辆的阻尼系数
  
  sol1=particle.Posistion;
  sol=ParseSolution(sol1,model);
  
  %碰撞次数计算
  Violation_num=sol.Violation_num;
  
  beta=300;
  z=beta3*(sol.L)*(1+beta*sol.Violation);
  
  %% 能量的计算
  x_start=model.xs;
  y_start=model.ys;
  z_start=model.zs;
  x_goal=model.xt;
  y_goal=model.yt;
  z_goal=model.zt;
  distance=sqrt((x_goal-x_start)^2+(y_goal-y_start)^2+...
      (z_goal-z_start)^2);
  % 计算AUV速度分量(不考虑洋流下)
  [v_x,v_y,v_z]=cal_speed(x_start,y_start,z_start,x_goal,y_goal,z_goal,model);
  
  %得到洋流的最大值
  v_currentx_max=model.current_x_max;
  v_currenty_max=model.current_y_max;
  v_currentz_max=model.current_z_max;
  
  %% 计算AUV在洋流作用下的和速度（载体坐标系下）
  v_x=v_x+v_currentx_max; %u
  v_y=v_y+v_currenty_max; %v 
  v_z=v_z+v_currentz_max; %w
  
  %计算在惯性坐标系下AUV速度
  FAI=atan(abs(y_goal-y_start)/abs(x_goal-x_start));
 
  dist=sqrt((x_goal-x_start)^2+(y_goal-y_start)^2);
  theta=atan(-abs(z_goal-z_start)/dist);
  %计算速度 (最大的极限速度)
  vx_dot_max=v_x*cos(FAI)*cos(theta)-v_y*sin(FAI)+v_z*cos(FAI)*sin(theta);
  vy_dot_max=v_x*sin(FAI)*cos(theta)+v_y*cos(FAI)+v_z*sin(FAI)*sin(theta);
  vz_dot_max=-v_x*sin(theta)+v_z*cos(theta);
  
  %计算速度的标量值
  Velocity_max=sqrt(vx_dot_max^2+vy_dot_max^2+vz_dot_max^2);
  %计算时间（min）
  T_shortest=distance/Velocity_max;
  
  %计算最小速度
   %% 计算AUV在洋流作用下的和速度（载体坐标系下）
  v_x_min=v_x-v_currentx_max; %u
  v_y_min=v_y-v_currenty_max; %v 
  v_z_min=v_z-v_currentz_max; %w
  %计算速度
  vx_dot_min=v_x_min*cos(FAI)*cos(theta)-v_y_min*sin(FAI)+v_z_min*cos(FAI)*sin(theta);
  vy_dot_min=v_x_min*sin(FAI)*cos(theta)+v_y_min*cos(FAI)+v_z_min*sin(FAI)*sin(theta);
  vz_dot_min=-v_x_min*sin(theta)+v_z_min*cos(theta);
  v_total_min=sqrt(vx_dot_min^2+vy_dot_min^2+vz_dot_min^2);
  
   E_min=c_d*v_total_min^3*T_shortest;  %计算的最小能量值
   
   %计算每一次的路径能量
   E1=cost_energy_cal(particle,model);
   
   %构造函数
   goal_function=(E1-E_min)^2/E_min^2;
   z=z+10*goal_function;

end