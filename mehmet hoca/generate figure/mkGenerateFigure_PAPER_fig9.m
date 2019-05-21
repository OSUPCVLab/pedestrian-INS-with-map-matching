clear all; close all; clc;
load('1_58_30pixel_probabilities.mat')

figure(1);
semilogy(quantize(1).log);
hold on
grid on

for i=2:length(quantize)
    semilogy(quantize(i).log);
end

for i=1:length(quantize)
    semilogy(quantize2(i).log);
end

for i=1:length(quantize)
    semilogy(quantize3(i).log);
end

for i=1:length(quantize)
    semilogy(quantize4(i).log);
end
hold off;

%%
figure(2); clf;
set(gcf,'position',[396 378 817 420]);
plot(quantize(1).log,'linewidth',1.5);
hold on;
grid on;
set(gca,'GridLineStyle','--',...
    'position',[0.0647    0.1333    0.9133    0.8452]);

for i=2:length(quantize)
    plot(quantize(i).log,'linewidth',1.5);
end

for i=1:length(quantize)
    plot(quantize2(i).log,'linewidth',1.5);
end

for i=1:length(quantize)
    plot(quantize3(i).log,'linewidth',1.5);
end

for i=1:length(quantize)
    plot(quantize4(i).log,'linewidth',1.5);
end
x = [0.59 0.46];
y = 0.03+[0.7 0.53];
an = annotation('textarrow',x,y,'String','true hypothesis','linewidth',1.8);
set(an,'interpreter','latex','fontsize',20);
hold off;
set(gca,'fontsize',14);
xlabel('Stride','interpreter','latex','fontsize',20);
ylabel('Probability','interpreter','latex','fontsize',20);
xticks(0:10:140);
xticklabels({'','','20','','40','','60','','80','','100','','120',...
    '','140'});
yticklabels({'0','','','','','','','','','','1'});