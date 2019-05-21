load four_times_spin.mat
figure(1); set(figure(1),'position',[1 59 1114 671]);
set(gcf,'position',[564 61 927 670]);
imshow(img); hold on;
if (choice == 1 || choice == 2)
plot(mu(1,1:end),mu(2,1:end),'b-x','linewidth',1.5); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'r-x','linewidth',1.5);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',10);
plot(finalEstimate(1,end),finalEstimate(2,end),'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',10);
else (choice == 3)
plot(mu(1,1:end-30),mu(2,1:end-30), 'b-.','linewidth',2); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,:),finalEstimate(2,:), 'r-','linewidth',2);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',10);
plot(finalEstimate(1,end-30),finalEstimate(2,end-30), 'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',10);
end
hold off;
legend('error state KF with ZUPT','non-recursive Bayesian map-matching','start','stop');
set(legend,'fontsize',18,'interpreter','latex','location','northeast');