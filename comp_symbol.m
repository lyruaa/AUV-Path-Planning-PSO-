Symbol=load('Symbol.mat','Symbol');
Sym=Symbol.Symbol;
for i=1:100
    if Sym(i,1)==15
        Sym(i,1)=14;
    end    
end
yy=linspace(1,200,200);

scatter(yy,Sym,'+');
xlabel('Iteration');
ylabel('Random perturbation');
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