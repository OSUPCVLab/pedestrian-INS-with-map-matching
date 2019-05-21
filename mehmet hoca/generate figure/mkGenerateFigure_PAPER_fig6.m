clc, clear, close all;
%%%bolz 30 pixel initial hypotheses
load('img.mat')
%load('mk60QuantBolz20px.mat')
load('mk60QuantBolz30px.mat')
%load('mk60QuantBolz40px.mat')
%load('mk60QuantBolz60px.mat')
figure(1); set(figure(1),'position',[1 59 1114 671],'papertype','a3');
set(gcf,'position',[564 61 927 670]);
imshow(img); hold on;
% plot(data(1,:), abs(data(2,:)), '.')
plot(data(1,1),data(2,1),'ro','markersize',10,'linewidth',2);
n = length(data);
plot(data(1,2),abs(data(2,2)),'gx','markersize',15,'linewidth',2);
r=10;
for i=1:1:length(data)
    viscircles([data(1,i),abs(data(2,i))],r)
end
plot(data(1,2),abs(data(2,2)),'gx','markersize',15,'linewidth',2);
legend('all hypotheses','true hypothesis');
set(legend,'fontsize',22,'interpreter','latex','location','northeast');
hold off;
print('bolz30pixelinitialHypotheses','-dpdf','-opengl');