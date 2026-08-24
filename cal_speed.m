function [v_x,v_y,v_z]=cal_speed(x_start,y_start,z_start,x_goal,y_goal,z_goal,model)
 %假设从起点沿着直线走到目标点
 FAI=atan(abs(y_goal-y_start)/abs(x_goal-x_start));
 
 dist=sqrt((x_goal-x_start)^2+(y_goal-y_start)^2);
 theta=atan(-abs(z_goal-z_start)/dist);
 
 %计算AUV速度分量(不考虑洋流)
 v_x=model.v_max*cos(theta)*cos(FAI);
 v_y=model.v_max*cos(theta)*sin(FAI);
 v_z=model.v_max*sin(theta);


end