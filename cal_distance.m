%% 计算粒子之间的距离
function sum_total=cal_distance(particle,nPop,i)
  particle_boss=particle(i).Posistion;
  x_boss=particle_boss.x;%提取要计算的粒子的位置
  y_boss=particle_boss.y;
  z_boss=particle_boss.z;
  sum_total=0;
  L=size(x_boss,2);
  %初始化要计算的粒子点与其他的粒子点的距离
  
  for j=1:nPop
      dist=0;
      if j ~= i
         %粒子不是当前本身的粒子
         Value=particle(j).Posistion;
         x=Value.x;
         y=Value.y;
         z=Value.z;
         for ii=1:L
            %计算要计算的粒子与粒子群其他粒子的距离
            value=sqrt((x_boss(ii)-x(ii))^2+(y_boss(ii)-y(ii))^2+(z_boss(ii)-z(ii))^2); 
            dist=value+dist;
            
         end
          
      else
          dist=dist+0;
          
      end
      
      sum_total=sum_total+dist;
  end %end for j=1:nPop

  sum_total=sum_total/(nPop-1);
 
end