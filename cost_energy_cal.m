function energy=cost_energy_cal(particle,model)
  L=length(model.current_x);
  %初始化能量总和
  cost_all=0;
  c_d=3;
  position=particle.Posistion;
  x=position.x;
  y=position.y;
  z=position.z;
  x_all=[model.xs x model.xt];
  y_all=[model.ys y model.yt];
  z_all=[model.zs z model.zt];
  
  for i=1:model.n+1
     current_position_x1=x_all(i);
     current_position_y1=y_all(i);
     current_position_z1=z_all(i);
     
     current_position_x2=x_all(i+1);
     current_position_y2=y_all(i+1);
     current_position_z2=z_all(i+1);
     
     
    x_index_0=int8(L/model.ymax*current_position_x1)+1;
    y_index_0=int8(L/model.xmax*current_position_y1)+1;
    x_index_1=int8(L/model.ymax*current_position_x2)+1;
    y_index_1=int8(L/model.xmax*current_position_y2)+1;
    
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
    %% 计算速度
    %计算角度值
    FAI=atan(abs(current_position_y2-current_position_y1)/abs(current_position_x2-current_position_x1));
 
    dist=sqrt((current_position_x2-current_position_x1)^2+(current_position_y2-current_position_y1)^2);
    theta=atan(-abs(current_position_z2-current_position_z1)/dist);
    %计算没有洋流下AUV本身的速度
    v_x=model.v_max*cos(theta)*cos(FAI);
    v_y=model.v_max*cos(theta)*sin(FAI);
    v_z=model.v_max*sin(theta);
    
   
    
    %计算洋流的速度（通过索引获取不同位置的洋流值）
    %由于是三维环境，需要考虑在不同位置处的洋流值
    % 这里我们做近似计算，取对应上下两个层的平均值
    
    %根据高度值选择两个不同的洋流的层
    
    %针对前一个位置 current_position_z1
    if current_position_z1>=-5 &&current_position_z1<0
       v_sea_x1=model.current_x(x_index_0,y_index_0,1);
       v_sea_y1=model.current_y(x_index_0,y_index_0,1); 
       v_sea_x2=model.current_x(x_index_0,y_index_0,2);
       v_sea_y2=model.current_y(x_index_0,y_index_0,2);
       v_sea_x=(v_sea_x1+v_sea_x2)/2;
       v_sea_y=(v_sea_y1+v_sea_y2)/2;
       v_sea_z=0;
    elseif current_position_z1>=0 && current_position_z1<5
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
    if current_position_z2>=-5 &&current_position_z2<0
       v_sea_x1=model.current_x(x_index_1,y_index_1,1);
       v_sea_y1=model.current_y(x_index_1,y_index_1,1); 
       v_sea_x2=model.current_x(x_index_1,y_index_1,2);
       v_sea_y2=model.current_y(x_index_1,y_index_1,2);
       v_sea_x11=(v_sea_x1+v_sea_x2)/2;
       v_sea_y11=(v_sea_y1+v_sea_y2)/2;
       v_sea_z11=0;
    elseif current_position_z2>=0 && current_position_z2<5
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
    
    %将AUV本身的速度与洋流速度结合
    v_x=v_x+v_sea_x;
    v_y=v_y+v_sea_y;
    v_z=v_z+v_sea_z;
    
    %计算惯性坐标系下的速度
    v_dotx=v_x*cos(FAI)*cos(theta)-v_y*sin(FAI)+v_z*cos(FAI)*sin(theta);
    v_doty=v_x*sin(FAI)*cos(theta)+v_y*cos(FAI)+v_z*sin(FAI)*sin(theta);
    v_dotz=-v_x*sin(theta)+v_z*cos(theta);
   
    Velocity=sqrt(v_dotx^2+v_doty^2+v_dotz^2);
    %计算当前两个点之间的距离
    distance_bet_dot=sqrt((current_position_x2-current_position_x1)^2+...
        (current_position_y2-current_position_y1)^2+(current_position_z2-current_position_z1)^2);
    %计算当前的时间
    T=distance_bet_dot/Velocity;
    %计算当前小段两个点之间的能量值大小
    energy_middle=c_d*Velocity^3*T;
    cost_all=cost_all+energy_middle;
    
  end

energy=cost_all; %这就是计算后的全部路线的能量值大小,,




end