function sol2=ParseSolution(sol1,model)
%sol1表示控制点位置...
global center_circle;
global radius_circle; %球体信息
global size1;
global origin; %全局化球体与长方体信息
global tria; %三棱柱信息
global ZHU;  %圆柱信息
global z_sea;

  x=sol1.x;
  y=sol1.y;
  z=sol1.z;  %划分的离散随机点
  
  xs=model.xs;
  ys=model.ys;
  zs=model.zs;
  xt=model.xt;
  yt=model.yt;
  zt=model.zt;
  
  XS=[xs x xt];
  YS=[ys y yt];
  ZS=[zs z zt];
  
  k=numel(XS);
  
  TS=linspace(0,1,k);
  tt=linspace(0,1,100);
  xx=spline(TS,XS,tt);
  yy=spline(TS,YS,tt);
  zz=spline(TS,ZS,tt);
  
  dx=diff(xx);
  dy=diff(yy);
  dz=diff(zz); %差分计算
  
  %计算路径的长度
  L=sum(sqrt(dx.^2+dy.^2+dz.^2));
  
  %计算球体障碍物数目
  L_obs_cir=size(center_circle,1);
  %长方体个数
  L_obs_34=size(origin,1);
  L_dot=length(xx);
  
  Violation=0; %障碍物碰撞代价值
  Violation_num=0; %障碍物上微元点碰撞的个数
  
  %% 针对球形障碍物避障判断
  for i=1:L_obs_cir
      d=sqrt((xx-center_circle(i,1)).^2+(yy-center_circle(i,2)).^2+(zz-center_circle(i,3)).^2);
      m=max(1-d/radius_circle(i),0);
      Violation=Violation+mean(m);
      %计算路径点在这个球体障碍物处个数
      for K=1:100
         if m(K)~=0
            Violation_num=Violation_num+1; 
         end
      end
  end
  %% 针对长方体避障判断
  for k=1:L_obs_34
      for i=1:L_dot
        %判断当前点是否在障碍物内
          if ((xx(i)>=origin(k,1)-size1(k,1)/2) && ((xx(i)<=origin(k,1)+size1(k,1)/2)))...
                  &&((yy(i)>=origin(k,2)-size1(k,2)/2) && ((yy(i)<=origin(k,2)+size1(k,2)/2)))...
                  &&((zz(i)>=origin(k,3)-size1(k,3)/2) && ((zz(i)<=origin(k,3)+size1(k,3)/2)))
              Violation=Violation+0.1; %如果在区间内，增加代价值
              %如果在区间内，说明当前路径点经过长方体内部，增加其碰撞数目
              Violation_num=Violation_num+1;
          end  
          
      end
      
  end
  
  
  %%  针对海底平面进行代价值计算
   for i=1:L_dot
      x_cor=xx(i);  %取水平面的x-axis坐标（规划生成的路线）
      y_cor=yy(i);  %取y-axis坐标  
      %向上取整
      x_cor=ceil(x_cor);
      y_cor=ceil(y_cor);
      %这里由于受到海底地图坐标数据都是基于偶数点生成的，所以这里只能近似计算，当我们取整后其x_cor与y_cor本身为偶数，
      %则不用变化，否则要将其转化为偶数。这里近似是合理的，
      %一方面，地形是连续变化的，一般取相近的点不会有太大的变化
      %另外这种变化相对地图范围来说可以忽略不计
      
      %判断x_cor与y_dor的奇偶性
      if (mod(x_cor,2))==0
          
      else
          x_cor=x_cor-1; %向后移动位置
      end
      
      if (mod(y_cor,2))==0
          
      else
          y_cor=y_cor-1;
      end
      
      %边界限定
      if x_cor<2
          x_cor=2;
      end
      if x_cor>202
          x_cor=202;
      end
      if y_cor<2
          y_cor=2;
      end
      if y_cor>202
          y_cor=202;
      end
      
      %运行到此处保证了其x_cor与y_dor都是偶数
      
      %判断对应位置的地形的高度是否超过当前规划的路线的高度，如果是，加大代价值
  
      
      if z_sea(x_cor/2,y_cor/2)>=zz(i)
          Violation=Violation+0.1; %碰撞到海底的点的代价值增加
          %针对碰到海底的微元点数目也增加
          Violation_num=Violation_num+1;
      else
          Violation=Violation+0;
          Violation_num=Violation_num+0;
      end
       
   end
   
   %% 计算三棱柱的避障
   %判断三棱柱中点关系
for i=1:L_dot
   
    px=xx(i);
    py=yy(i); %将微元点插入

p0=[tria(1,1),tria(1,2)];%p0
p1=[tria(2,1),tria(2,2)]; %p1
p2=[tria(3,1),tria(3,2)]; %p2
p0x=p0(1);
p0y=p0(2);
p1x=p1(1);
p1y=p1(2);
p2x=p2(1);
p2y=p2(2);
Area = 0.5 *(-p1y*p2x + p0y*(-p1x + p2x) + p0x*(p1y - p2y) + p1x*p2y);

u = 1/(2*Area)*(p0y*p2x - p0x*p2y + (p2y - p0y)*px + (p0x - p2x)*py);
v = 1/(2*Area)*(p0x*p1y - p0y*p1x + (p0y - p1y)*px + (p1x - p0x)*py);

if (u>0 && v>0 && u+v<1 ) && (zz(i)>=0 && zz(i)<=8)
    %% 说明微元点在三棱锥的内部
    %增加带价值
    Violation=Violation+0.1;
    Violation_num=Violation_num+1;
else
    %说明在其外部
    Violation=Violation+0;
    Violation_num=Violation_num+0;
end
end

%% 判断圆柱障碍物代价值
zhu_pos=[ZHU(1),ZHU(2)]; %圆柱中心位置
zhu_radius=ZHU(3); %圆柱半径
zhu_height=[ZHU(4),ZHU(5)]; %圆柱高度
for i=1:L_dot
    %计算微元点到圆柱的距离
    x=xx(i);
    y=yy(i);
    z=zz(i);
    %判断当前微元点是否在圆柱内
    dist=sqrt((x-zhu_pos(1))^2+(y-zhu_pos(2))^2);
    if (dist<=zhu_radius)&&((z>=zhu_height(1) && z<=zhu_height(2)))
       %此时说明在圆柱内部
       Violation=Violation+0.1; %增加带价值
       Violation_num=Violation_num+1; %增加微元点数目
    else
        Violation=Violation+0;
        Violation_num=Violation_num+0;
    end
    
    
    
end

 sol2.TS=TS;
 sol2.XS=XS;
 sol2.YS=YS;
 sol2.ZS=ZS;
 sol2.tt=tt;
 sol2.xx=xx;
 sol2.yy=yy;
 sol2.zz=zz;
 sol2.dx=dx;
 sol2.dy=dy;
 sol2.dz=dz;
 sol2.L=L;
 sol2.Violation=Violation;
 sol2.Violation_num=Violation_num;
 sol2.IsFeasible=(Violation==0);
 

end