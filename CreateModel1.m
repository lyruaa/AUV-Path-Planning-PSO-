function model=CreateModel(XX22,YY22,ZZ22) %传入的是洋流数据
 global start_point;
 global goal_point;
 global x_limit
 global y_limit
 global z_limit
%起始点
  xs=start_point(1);
  ys=start_point(2);
  zs=start_point(3);
  
  xt=goal_point(1);
  yt=goal_point(2);
  zt=goal_point(3);
  
  xmin=x_limit(1);
  xmax=x_limit(2);
  
  ymin=y_limit(1);
  ymax=y_limit(2);
  
  zmin=z_limit(1);
  zmax=z_limit(2);
  
  v_max=6;  %AUV的最大速度
  n=5;  %PSO中控制点的数目

  %三维洋流赋值
  u=XX22;
  v=YY22;
  w=ZZ22;
  
  current_x_max=max(max(max(abs(u))));
  current_y_max=max(max(max(abs(v))));
  current_z_max=max(max(max(abs(w))));
  
  model.xs=xs;
  model.ys=ys;
  model.zs=zs; %起始位置
  model.xt=xt;
  model.yt=yt;
  model.zt=zt; %目标位置
  model.xmin=xmin;
  model.xmax=xmax;
  model.ymin=ymin;
  model.ymax=ymax;
  model.zmin=zmin;
  model.zmax=zmax;
  model.n=n;
  model.current_x=u;
  model.current_y=v;
  model.current_z=w;
  model.v_max=v_max;
  model.current_x_max=current_x_max;
  model.current_y_max=current_y_max;
  model.current_z_max=current_z_max;

end