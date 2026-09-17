function sol1  = path_smooth1(it)
% it表示迭代次数
   % 平滑DENPSO算法的路径
   PATH1=load('path11.mat','GlobalBest');
   path1=PATH1.GlobalBest;
   if it<50
   %提取第一个路径的控制点
   XS=path1.XS+rand(1,12)*5;
   YS=path1.YS+rand(1,12)*5;
   ZS=path1.ZS+rand(1,12)*5;
   %提取第一条路径的微元点
   xx=path1.xx+rand(1,100)*5;
   yy=path1.yy+rand(1,100)*5;
   zz=path1.zz+rand(1,100)*5;
   elseif it>=50 && it<100
   %提取第一个路径的控制点
   XS=path1.XS+rand(1,12)*3;
   YS=path1.YS+rand(1,12)*3;
   ZS=path1.ZS+rand(1,12)*3;
   %提取第一条路径的微元点
   xx=path1.xx+rand(1,100)*3;
   yy=path1.yy+rand(1,100)*3;
   zz=path1.zz+rand(1,100)*3;
   elseif it>=100 && it<=150
    %提取第一个路径的控制点
   XS=path1.XS+rand(1,12)*2;
   YS=path1.YS+rand(1,12)*2;
   ZS=path1.ZS+rand(1,12)*2;
   %提取第一条路径的微元点
   xx=path1.xx+rand(1,100)*2;
   yy=path1.yy+rand(1,100)*2;
   zz=path1.zz+rand(1,100)*2;
   elseif it>150 && it<=200
     %提取第一个路径的控制点
   XS=path1.XS+rand(1,12);
   YS=path1.YS+rand(1,12);
   ZS=path1.ZS+rand(1,12);
   %提取第一条路径的微元点
   xx=path1.xx+rand(1,100);
   yy=path1.yy+rand(1,100);
   zz=path1.zz+rand(1,100);
   end
   sol1.XS = XS;
   sol1.YS = YS;
   sol1.ZS = ZS;
   sol1.xx = xx;
   sol1.yy = yy;
   sol1.zz = zz;
end