%best_cost±???
Violation1=load('violation1.mat','Violation');
Violation_comp_1=Violation1.Violation;

Violation2=load('violation11.mat','Violation');
Violation_comp_2=Violation2.Violation;

plot(Violation_comp_1,'-r','LineWidth',1);
hold on;
plot(Violation_comp_2,'-k','LineWidth',1);
hold on;
xlabel('Iteration');
ylabel('Micro\_factor');
hold on
box off
ax2 = axes('Position',get(gca,'Position'),...
           'XAxisLocation','top',...
           'YAxisLocation','right',...
           'Color','none',...
           'XColor','k','YColor','k');
set(ax2,'YTick', []);
set(ax2,'XTick', []);
box on