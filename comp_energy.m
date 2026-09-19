%能量消耗比较
Energy1=load('energy1.mat','energy_opt');
Energy_comp_1=Energy1.energy_opt;

Energy2=load('energy11.mat','energy_opt');
Energy_comp_2=Energy2.energy_opt;

plot(Energy_comp_1,'-r','LineWidth',1);
hold on;
plot(Energy_comp_2,'-k','LineWidth',1);
hold on;
legend('NPSO','LPSO')
xlabel('Iteration');
ylabel('Energy consumption (J)');

box off
ax2 = axes('Position',get(gca,'Position'),...
           'XAxisLocation','top',...
           'YAxisLocation','right',...
           'Color','none',...
           'XColor','k','YColor','k');
set(ax2,'YTick', []);
set(ax2,'XTick', []);
box on
axis equal
