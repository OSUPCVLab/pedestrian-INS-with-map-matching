close all;
load('dataQuantative.mat')

for i=1:length(quantative)
   a=abs(quantative(i,1) - quantative(i,3));
   b=abs(quantative(i,2) - quantative(i,3));
   
   if(a<b)
       dist(i,1) = a;
   else
       dist(i,1) = b;
   end
end
scale = 9.7536/200; % meter / pixel
dist = scale*dist; % converted from pixels to meters 

figure;histogram(dist);
figure; set(gcf,'position',[186 439 1328 359]); clf;
bar(dist);
xlabel('Stride','interpreter','latex','fontsize',24);
ylabel('Error (meter)','interpreter','latex','fontsize',24);
axis([0 i 0 0.5]);
set(gca,'position',[0.0723    0.1560    0.9225    0.7994]);
set(gca,'xtick',[0:30:i],'xticklabel',{'','30','60','90','','',...
    '180','210','240','270'},'TickLabelInterpreter', 'latex','fontsize',24);
set(gca,'ytick',[0:0.1:0.5],'fontsize',24,'TickLabelInterpreter', 'latex');
grid on;
set(gca,'GridLineStyle','--');
figure;histfit(dist);
figure;boxplot(dist);