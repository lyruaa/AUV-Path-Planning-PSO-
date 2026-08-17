[a,z]=ndgrid((0:.05:1)*2*pi,0:.05:20);
x=7.5*cos(a)+60;
y=7.5*sin(a)+70;
surf(x,y,z,x*0,'linestyle','none','Facealpha',0.7,'FaceColor','k')
hold on
[a,r]=ndgrid((0:.05:1)*2*pi,[0 1]);
x=7.5*cos(a).*r+60;
y=7.5*sin(a).*r+70;
surf(x,y,x*0,x*0,'linestyle','none','Facealpha',0.7,'FaceColor','k')
surf(x,y,x*0+20,x*0,'linestyle','none','Facealpha',0.7,'FaceColor','k')
hold off
grid on
view(70,60)