function  jump_to(particle,best_index,II,model)
%跳出机制的详细实现，对第II个粒子进行跳出机制
%注意在跳出机制时，应该保留精英的粒子，保持其优良的特性

Position_x=zeros(1,model.n);
Position_y=zeros(1,model.n);
Position_z=zeros(1,model.n);
%对于每一个这样的粒子进行同样的操作
if II~=best_index
    %提取要干扰的坐标
    Particle=particle.Posistion; % control points
    x=Particle.x;
    y=Particle.y; 
    z=Particle.z; %原始坐标
    L=size(x,2);
    step=2;
    %假设可视化半径
    visual=4;
    iter=6;  %计算干扰后的粒子的寻优的次数
    
    %计算粒子的适应度  根据次数计算，三次探索局部的最优值，，进行随机的计算...
    for it=1:iter
        
         X=zeros(1,L);
         Y=zeros(1,L);
         Z=zeros(1,L);
        
       for j=1:L   %针对每一个点
        
         %计算扰动后的位置
         X(1,j)=x(1,j)+rand()*visual;
         Y(1,j)=y(1,j)+rand()*visual;
         Z(1,j)=z(1,j)+rand()*visual*0.5; %计算扰动后的位置
  
       end% end for j=1:L
      % particle.Position.x=X;
      % particle.Position.y=Y;
       particle.Posistion.x=X;
       particle.Posistion.y=Y;
       particle.Posistion.z=Z;
       [Cost, part,energy,VIO]=CostFunction(particle,model);
        %根据其适应度函数进行比较
        if Cost<particle.Cost 
            %将当前路径赋值到particle中
            for KK=1:model.n
              Position_x(1,KK)=x(1,KK)+step*rand()*(X(1,KK)-x(1,KK))/abs(X(1,KK)-x(1,KK));
              Position_y(1,KK)=y(1,KK)+step*rand()*(Y(1,KK)-y(1,KK))/abs(Y(1,KK)-y(1,KK));
              Position_z(1,KK)=z(1,KK)+step*rand()*(Z(1,KK)-z(1,KK))/abs(Z(1,KK)-z(1,KK));
            end
            particle.Posistion.x=Position_x;
            particle.Posistion.y=Position_y; 
            particle.Posistion.z=Position_z;
            break;  %条件满足就跳出循环，不做下一步探索...
            
        else
            %啥也不做,寻路失败
    
        end
    
    end
  else
    %这里保留精英群
end