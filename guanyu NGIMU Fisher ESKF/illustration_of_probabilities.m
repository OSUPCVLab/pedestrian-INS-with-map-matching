function lastFigNumber = illustration_of_probabilities(subsequentStrides,...
                            probStrides,figNumber,color,hypNumber)
                        
for i=1:hypNumber
    fprintf('Hypothesis %i probability of subsequent strides ',i);
    fprintf('(total probability is %.3f)\n',sum(probStrides(i,:)));
    for j=1:subsequentStrides
        if (j == subsequentStrides)
            fprintf('%.3f\n', probStrides(i,j));
        else
            fprintf('%.3f - ', probStrides(i,j));
        end
    end
end

fprintf('Hypothesis scores for ');
nOf8groups = ceil(hypNumber/8);
hypCount = 0;
firstFigNumber = figNumber+1;
for i=1:nOf8groups
figNumber = figNumber + 1;
figure(figNumber); clf; set(gcf,'position',[213 187 1120 420]);
if (mod(hypCount,8) == 0)
hypCount = hypCount+1;
if (hypCount > hypNumber)
    break;
end
subplot(2,4,1);
stem(1:subsequentStrides, probStrides(hypCount,:),'g.','linewidth',1.7,...
    'color',color(1,:));
grid on; xlabel('\textbf{stride}','interpreter','latex');
ylabel('\textbf{probability}','interpreter','latex');
title(['$P(H_{', num2str(hypCount),'}) = \sum_{i=1}^{10} p_i \;=\;$', ...
    num2str(sum(probStrides(hypCount,:)))],'interpreter','latex','fontsize',10);
set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
set(gca,'ytick',[0:0.2:1],'yticklabel',{'','.2','.4','.6','.8','1'},'fontsize',10,'TickLabelInterpreter', 'latex');
set(gca,'GridLineStyle','--','position',[0.0342 0.6001 0.2345 0.3216]);
axis([0 subsequentStrides 0 1]);
fprintf('h%i ',hypCount);
end

if (mod(hypCount,8) == 1)
hypCount = hypCount+1;
if (hypCount > hypNumber)
    break;
end
subplot(2,4,2);
stem(1:subsequentStrides, probStrides(hypCount,:),'y.','linewidth',1.7,...
    'color',color(2,:));
grid on; xlabel('\textbf{stride}','interpreter','latex');
title(['$P(H_{',num2str(hypCount),'}) = \sum_{i=1}^{10} p_i \;=\;$', ...
    num2str(sum(probStrides(hypCount,:)))],'interpreter','latex','fontsize',10);
set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
set(gca,'GridLineStyle','--','position',[0.0342+0.2345+0.005 0.6001 0.2345 0.3216]);
axis([0 subsequentStrides 0 1]);
fprintf('h%i ',hypCount);
end

if (mod(hypCount,8) == 2)
hypCount = hypCount+1;
if (hypCount > hypNumber)
    break;
end
subplot(2,4,3);
stem(1:subsequentStrides, probStrides(hypCount,:),'y.','linewidth',1.7,...
    'color',color(3,:));
grid on; xlabel('\textbf{stride}','interpreter','latex');
title(['$P(H_{',num2str(hypCount),'}) = \sum_{i=1}^{10} p_i \;=\;$', ...
    num2str(sum(probStrides(hypCount,:)))],'interpreter','latex','fontsize',10);
set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
set(gca,'GridLineStyle','--','position',[0.0342+2*0.2345+2*0.005 0.6001 0.2345 0.3216]);
axis([0 subsequentStrides 0 1]);
fprintf('h%i ',hypCount);
end

if (mod(hypCount,8) == 3)
hypCount = hypCount+1;
if (hypCount > hypNumber)
    break;
end
subplot(2,4,4);
stem(1:subsequentStrides, probStrides(hypCount,:),'y.','linewidth',1.7,...
    'color',color(4,:));
grid on; xlabel('\textbf{stride}','interpreter','latex');
title(['$P(H_{',num2str(hypCount),'}) = \sum_{i=1}^{10} p_i \;=\;$', ...
    num2str(sum(probStrides(hypCount,:)))],'interpreter','latex','fontsize',10);
set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
set(gca,'GridLineStyle','--','position',[0.0342+3*0.2345+3*0.005 0.6001 0.2345 0.3216]);
axis([0 subsequentStrides 0 1]);
fprintf('h%i ',hypCount);
end

if (mod(hypCount,8) == 4)
hypCount = hypCount+1;
if (hypCount > hypNumber)
    break;
end
subplot(2,4,5);
stem(1:subsequentStrides, probStrides(hypCount,:),'g.','linewidth',1.7,...
    'color',color(5,:));
grid on; xlabel('\textbf{stride}','interpreter','latex');
ylabel('\textbf{probability}','interpreter','latex');
title(['$P(H_{',num2str(hypCount),'}) = \sum_{i=1}^{10} p_i \;=\;$', ...
    num2str(sum(probStrides(hypCount,:)))],'interpreter','latex','fontsize',10);
set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
set(gca,'ytick',[0:0.2:1],'yticklabel',{'','.2','.4','.6','.8','1'},'fontsize',10,'TickLabelInterpreter', 'latex');
set(gca,'GridLineStyle','--','position',[0.0342 0.1221 0.2345 0.3216]); % [0.1442 0.1221 0.2345 0.3216]
axis([0 subsequentStrides 0 1]);
fprintf('h%i ',hypCount);
end

if (mod(hypCount,8) == 5)
hypCount = hypCount+1;
if (hypCount > hypNumber)
    break;
end
subplot(2,4,6);
stem(1:subsequentStrides, probStrides(hypCount,:),'g.','linewidth',1.7,...
    'color',color(6,:));
grid on; xlabel('\textbf{stride}','interpreter','latex');
title(['$P(H_{',num2str(hypCount),'}) = \sum_{i=1}^{10} p_i \;=\;$', ...
    num2str(sum(probStrides(hypCount,:)))],'interpreter','latex','fontsize',10);
set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
set(gca,'GridLineStyle','--','position',[0.0342+0.2345+0.005 0.1221 0.2345 0.3216]);
axis([0 subsequentStrides 0 1]);
fprintf('h%i ',hypCount);
end

if (mod(hypCount,8) == 6)
hypCount = hypCount+1;
if (hypCount > hypNumber)
    break;
end
subplot(2,4,7);
stem(1:subsequentStrides, probStrides(hypCount,:),'g.','linewidth',1.7,...
    'color',color(7,:));
grid on; xlabel('\textbf{stride}','interpreter','latex');
title(['$P(H_{',num2str(hypCount),'}) = \sum_{i=1}^{10} p_i \;=\;$', ...
    num2str(sum(probStrides(hypCount,:)))],'interpreter','latex','fontsize',10);
set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
set(gca,'GridLineStyle','--','position',[0.0342+2*0.2345+2*0.005 0.1221 0.2345 0.3216]);
axis([0 subsequentStrides 0 1]);
fprintf('h%i ',hypCount);
end

if (mod(hypCount,8) == 7)
hypCount = hypCount+1;
if (hypCount > hypNumber)
    break;
end
subplot(2,4,8);
stem(1:subsequentStrides, probStrides(hypCount,:),'g.','linewidth',1.7,...
    'color',color(8,:));
grid on; xlabel('\textbf{stride}','interpreter','latex');
title(['$P(H_{',num2str(hypCount),'}) = \sum_{i=1}^{10} p_i \;=\;$', ...
    num2str(sum(probStrides(hypCount,:)))],'interpreter','latex','fontsize',10);
set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
set(gca,'GridLineStyle','--','position',[0.0342+3*0.2345+3*0.005 0.1221 0.2345 0.3216]);
axis([0 subsequentStrides 0 1]);
fprintf('h%i ',hypCount);
end
end

fprintf('are illustrated.\n',hypCount);
if (hypCount == hypNumber)
    fprintf('Total of %i hypotheses with 10 strides scores are illustrated in \nFigures ',hypCount);
    for i=firstFigNumber:figNumber-1
        fprintf('%i, ',i);
    end
    fprintf('%i\n',figNumber);
end
lastFigNumber = figNumber;

% figure(lastFigNumber+2); clf; set(gcf,'position',[213 187 1120 420]);
% subplot(2,4,1);
% stem(1:subsequentStrides, probStrides(9,:),'g.','linewidth',1.7,...
%     'color',color(1,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% ylabel('\textbf{probability}','interpreter','latex');
% title(['$P(H_9) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(9,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','.2','.4','.6','.8','1'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342 0.6001 0.2345 0.3216]);
% subplot(2,4,2);
% stem(1:subsequentStrides, probStrides(10,:),'y.','linewidth',1.7,...
%     'color',color(2,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{10}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(10,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+0.2345+0.005 0.6001 0.2345 0.3216]);
% subplot(2,4,3);
% stem(1:subsequentStrides, probStrides(11,:),'y.','linewidth',1.7,...
%     'color',color(3,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{11}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(11,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+2*0.2345+2*0.005 0.6001 0.2345 0.3216]);
% subplot(2,4,4);
% stem(1:subsequentStrides, probStrides(12,:),'y.','linewidth',1.7,...
%     'color',color(4,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{12}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(12,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+3*0.2345+3*0.005 0.6001 0.2345 0.3216]);
% subplot(2,4,8);
% stem(1:subsequentStrides, probStrides(16,:),'g.','linewidth',1.7,...
%     'color',color(8,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{16}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(16,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+3*0.2345+3*0.005 0.1221 0.2345 0.3216]);
% subplot(2,4,7);
% stem(1:subsequentStrides, probStrides(15,:),'g.','linewidth',1.7,...
%     'color',color(7,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{15}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(15,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+2*0.2345+2*0.005 0.1221 0.2345 0.3216]);
% subplot(2,4,5);
% stem(1:subsequentStrides, probStrides(13,:),'g.','linewidth',1.7,...
%     'color',color(5,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% ylabel('\textbf{probability}','interpreter','latex');
% title(['$P(H_{13}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(13,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','.2','.4','.6','.8','1'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342 0.1221 0.2345 0.3216]); % [0.1442 0.1221 0.2345 0.3216]
% subplot(2,4,6);
% stem(1:subsequentStrides, probStrides(14,:),'g.','linewidth',1.7,...
%     'color',color(6,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{14}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(14,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+0.2345+0.005 0.1221 0.2345 0.3216]);
% 
% figure(lastFigNumber+3); clf; set(gcf,'position',[213 187 1120 420]);
% subplot(2,4,1);
% stem(1:subsequentStrides, probStrides(17,:),'g.','linewidth',1.7,...
%     'color',color(1,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% ylabel('\textbf{probability}','interpreter','latex');
% title(['$P(H_{17}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(17,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','.2','.4','.6','.8','1'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342 0.6001 0.2345 0.3216]);
% subplot(2,4,2);
% stem(1:subsequentStrides, probStrides(18,:),'y.','linewidth',1.7,...
%     'color',color(2,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{18}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(18,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+0.2345+0.005 0.6001 0.2345 0.3216]);
% subplot(2,4,3);
% stem(1:subsequentStrides, probStrides(19,:),'y.','linewidth',1.7,...
%     'color',color(3,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{19}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(19,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+2*0.2345+2*0.005 0.6001 0.2345 0.3216]);
% subplot(2,4,4);
% stem(1:subsequentStrides, probStrides(20,:),'y.','linewidth',1.7,...
%     'color',color(4,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{20}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(20,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+3*0.2345+3*0.005 0.6001 0.2345 0.3216]);
% subplot(2,4,8);
% stem(1:subsequentStrides, probStrides(24,:),'g.','linewidth',1.7,...
%     'color',color(8,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{24}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(24,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+3*0.2345+3*0.005 0.1221 0.2345 0.3216]);
% subplot(2,4,7);
% stem(1:subsequentStrides, probStrides(23,:),'g.','linewidth',1.7,...
%     'color',color(7,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{23}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(23,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+2*0.2345+2*0.005 0.1221 0.2345 0.3216]);
% subplot(2,4,5);
% stem(1:subsequentStrides, probStrides(21,:),'g.','linewidth',1.7,...
%     'color',color(5,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% ylabel('\textbf{probability}','interpreter','latex');
% title(['$P(H_{21}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(21,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','.2','.4','.6','.8','1'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342 0.1221 0.2345 0.3216]); % [0.1442 0.1221 0.2345 0.3216]
% subplot(2,4,6);
% stem(1:subsequentStrides, probStrides(22,:),'g.','linewidth',1.7,...
%     'color',color(6,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{22}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(22,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+0.2345+0.005 0.1221 0.2345 0.3216]);
% 
% figure(lastFigNumber+4); clf; set(gcf,'position',[213 187 1120 420]);
% subplot(2,4,1);
% stem(1:subsequentStrides, probStrides(25,:),'g.','linewidth',1.7,...
%     'color',color(1,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% ylabel('\textbf{probability}','interpreter','latex');
% title(['$P(H_{25}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(25,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','.2','.4','.6','.8','1'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342 0.6001 0.2345 0.3216]);
% subplot(2,4,2);
% stem(1:subsequentStrides, probStrides(26,:),'y.','linewidth',1.7,...
%     'color',color(2,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{26}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(26,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+0.2345+0.005 0.6001 0.2345 0.3216]);
% subplot(2,4,3);
% stem(1:subsequentStrides, probStrides(27,:),'y.','linewidth',1.7,...
%     'color',color(3,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{27}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(27,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+2*0.2345+2*0.005 0.6001 0.2345 0.3216]);
% subplot(2,4,4);
% stem(1:subsequentStrides, probStrides(28,:),'y.','linewidth',1.7,...
%     'color',color(4,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{28}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(28,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+3*0.2345+3*0.005 0.6001 0.2345 0.3216]);
% subplot(2,4,8);
% stem(1:subsequentStrides, probStrides(32,:),'g.','linewidth',1.7,...
%     'color',color(8,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{32}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(32,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+3*0.2345+3*0.005 0.1221 0.2345 0.3216]);
% subplot(2,4,7);
% stem(1:subsequentStrides, probStrides(31,:),'g.','linewidth',1.7,...
%     'color',color(7,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{31}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(31,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+2*0.2345+2*0.005 0.1221 0.2345 0.3216]);
% subplot(2,4,5);
% stem(1:subsequentStrides, probStrides(29,:),'g.','linewidth',1.7,...
%     'color',color(5,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% ylabel('\textbf{probability}','interpreter','latex');
% title(['$P(H_{29}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(29,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','.2','.4','.6','.8','1'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342 0.1221 0.2345 0.3216]); % [0.1442 0.1221 0.2345 0.3216]
% subplot(2,4,6);
% stem(1:subsequentStrides, probStrides(30,:),'g.','linewidth',1.7,...
%     'color',color(6,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{30}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(30,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+0.2345+0.005 0.1221 0.2345 0.3216]);
% 
% figure(lastFigNumber+5); clf; set(gcf,'position',[213 187 1120 420]);
% subplot(2,4,1);
% stem(1:subsequentStrides, probStrides(33,:),'g.','linewidth',1.7,...
%     'color',color(1,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% ylabel('\textbf{probability}','interpreter','latex');
% title(['$P(H_{33}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(33,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','.2','.4','.6','.8','1'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342 0.6001 0.2345 0.3216]);
% subplot(2,4,2);
% stem(1:subsequentStrides, probStrides(34,:),'y.','linewidth',1.7,...
%     'color',color(2,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{34}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(34,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+0.2345+0.005 0.6001 0.2345 0.3216]);
% subplot(2,4,3);
% stem(1:subsequentStrides, probStrides(35,:),'y.','linewidth',1.7,...
%     'color',color(3,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{35}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(35,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+2*0.2345+2*0.005 0.6001 0.2345 0.3216]);
% subplot(2,4,4);
% stem(1:subsequentStrides, probStrides(36,:),'y.','linewidth',1.7,...
%     'color',color(4,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{36}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(36,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+3*0.2345+3*0.005 0.6001 0.2345 0.3216]);
% subplot(2,4,8);
% stem(1:subsequentStrides, probStrides(40,:),'g.','linewidth',1.7,...
%     'color',color(8,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{40}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(40,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+3*0.2345+3*0.005 0.1221 0.2345 0.3216]);
% subplot(2,4,7);
% stem(1:subsequentStrides, probStrides(39,:),'g.','linewidth',1.7,...
%     'color',color(7,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{39}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(39,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+2*0.2345+2*0.005 0.1221 0.2345 0.3216]);
% subplot(2,4,5);
% stem(1:subsequentStrides, probStrides(37,:),'g.','linewidth',1.7,...
%     'color',color(5,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% ylabel('\textbf{probability}','interpreter','latex');
% title(['$P(H_{37}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(37,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','.2','.4','.6','.8','1'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342 0.1221 0.2345 0.3216]); % [0.1442 0.1221 0.2345 0.3216]
% subplot(2,4,6);
% stem(1:subsequentStrides, probStrides(38,:),'g.','linewidth',1.7,...
%     'color',color(6,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{38}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(38,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+0.2345+0.005 0.1221 0.2345 0.3216]);
% 
% figure(lastFigNumber+6); clf; set(gcf,'position',[213 187 1120 420]);
% subplot(2,4,1);
% stem(1:subsequentStrides, probStrides(41,:),'g.','linewidth',1.7,...
%     'color',color(1,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% ylabel('\textbf{probability}','interpreter','latex');
% title(['$P(H_{41}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(41,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','.2','.4','.6','.8','1'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342 0.6001 0.2345 0.3216]);
% subplot(2,4,2);
% stem(1:subsequentStrides, probStrides(42,:),'y.','linewidth',1.7,...
%     'color',color(2,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{42}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(42,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+0.2345+0.005 0.6001 0.2345 0.3216]);
% subplot(2,4,3);
% stem(1:subsequentStrides, probStrides(43,:),'y.','linewidth',1.7,...
%     'color',color(3,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{43}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(43,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+2*0.2345+2*0.005 0.6001 0.2345 0.3216]);
% subplot(2,4,4);
% stem(1:subsequentStrides, probStrides(44,:),'y.','linewidth',1.7,...
%     'color',color(4,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{44}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(44,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+3*0.2345+3*0.005 0.6001 0.2345 0.3216]);
% subplot(2,4,8);
% stem(1:subsequentStrides, probStrides(48,:),'g.','linewidth',1.7,...
%     'color',color(8,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{48}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(48,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+3*0.2345+3*0.005 0.1221 0.2345 0.3216]);
% subplot(2,4,7);
% stem(1:subsequentStrides, probStrides(47,:),'g.','linewidth',1.7,...
%     'color',color(7,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{47}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(47,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+2*0.2345+2*0.005 0.1221 0.2345 0.3216]);
% subplot(2,4,5);
% stem(1:subsequentStrides, probStrides(45,:),'g.','linewidth',1.7,...
%     'color',color(5,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% ylabel('\textbf{probability}','interpreter','latex');
% title(['$P(H_{45}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(45,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','.2','.4','.6','.8','1'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342 0.1221 0.2345 0.3216]); % [0.1442 0.1221 0.2345 0.3216]
% subplot(2,4,6);
% stem(1:subsequentStrides, probStrides(46,:),'g.','linewidth',1.7,...
%     'color',color(6,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{46}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(46,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+0.2345+0.005 0.1221 0.2345 0.3216]);
% 
% %%
% figure(lastFigNumber+7); clf; set(gcf,'position',[213 187 1120 420]);
% subplot(2,4,1);
% stem(1:subsequentStrides, probStrides(49,:),'g.','linewidth',1.7,...
%     'color',color(1,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% ylabel('\textbf{probability}','interpreter','latex');
% title(['$P(H_{49}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(49,:)))],'interpreter','latex','fontsize',10);
% axis([0 subsequentStrides 0 1]);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','.2','.4','.6','.8','1'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342 0.6001 0.2345 0.3216]);
% subplot(2,4,2);
% stem(1:subsequentStrides, probStrides(50,:),'y.','linewidth',1.7,...
%     'color',color(2,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{50}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(50,:)))],'interpreter','latex','fontsize',10);
% axis([0 subsequentStrides 0 1]);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+0.2345+0.005 0.6001 0.2345 0.3216]);
% subplot(2,4,3);
% stem(1:subsequentStrides, probStrides(51,:),'y.','linewidth',1.7,...
%     'color',color(3,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{51}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(51,:)))],'interpreter','latex','fontsize',10);
% axis([0 subsequentStrides 0 1]);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+2*0.2345+2*0.005 0.6001 0.2345 0.3216]);
% subplot(2,4,4);
% stem(1:subsequentStrides, probStrides(52,:),'y.','linewidth',1.7,...
%     'color',color(4,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{52}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(52,:)))],'interpreter','latex','fontsize',10);
% axis([0 subsequentStrides 0 1]);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+3*0.2345+3*0.005 0.6001 0.2345 0.3216]);
% subplot(2,4,8);
% stem(1:subsequentStrides, probStrides(56,:),'g.','linewidth',1.7,...
%     'color',color(8,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{56}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(56,:)))],'interpreter','latex','fontsize',10);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+3*0.2345+3*0.005 0.1221 0.2345 0.3216]);
% subplot(2,4,7);
% stem(1:subsequentStrides, probStrides(55,:),'g.','linewidth',1.7,...
%     'color',color(7,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{55}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(55,:)))],'interpreter','latex','fontsize',10);
% axis([0 subsequentStrides 0 1]);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+2*0.2345+2*0.005 0.1221 0.2345 0.3216]);
% subplot(2,4,5);
% stem(1:subsequentStrides, probStrides(53,:),'g.','linewidth',1.7,...
%     'color',color(5,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% ylabel('\textbf{probability}','interpreter','latex');
% title(['$P(H_{53}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(53,:)))],'interpreter','latex','fontsize',10);
% axis([0 subsequentStrides 0 1]);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','.2','.4','.6','.8','1'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342 0.1221 0.2345 0.3216]); % [0.1442 0.1221 0.2345 0.3216]
% subplot(2,4,6);
% stem(1:subsequentStrides, probStrides(54,:),'g.','linewidth',1.7,...
%     'color',color(6,:));
% grid on; xlabel('\textbf{stride}','interpreter','latex');
% title(['$P(H_{54}) = \sum_{i=1}^{10} p_i \;=\;$', ...
%     num2str(sum(probStrides(54,:)))],'interpreter','latex','fontsize',10);
% axis([0 subsequentStrides 0 1]);
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'GridLineStyle','--','position',[0.0342+0.2345+0.005 0.1221 0.2345 0.3216]);