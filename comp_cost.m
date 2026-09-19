%best_cost±???
Cost1=load('best_cost1.mat','BestCost');
Cost_comp_1=Cost1.BestCost;

Cost2=load('best_cost11.mat','BestCost');
Cost_comp_2=Cost2.BestCost;


plot(Cost_comp_1,'-k','LineWidth',2);
hold on;
plot(Cost_comp_2,'-r','LineWidth',1);
hold on;
legend('DENPSO','LPSO')
xlabel('Iteration');
ylabel('Cost value');
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