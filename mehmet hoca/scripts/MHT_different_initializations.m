clear all; close all; clc;
load bolz_MHT_initialization_58.mat

figure(1); % set(figure(5),'position',[481 14 1114 799]);
set(gcf,'position',[2 88 3000 670],'papertype','a2');
subplot(1,3,1);
imshow(img); hold on;
% upsamplingRate = 1; muUS = zeros(size(mu,1),upsamplingRate*size(mu,2));
% muUS(1,:) = interp(mu(1,:),upsamplingRate);
% muUS(2,:) = interp(mu(2,:),upsamplingRate);
if (choice == 1 || choice == 2)
plot(mu(1,1:end),mu(2,1:end),'bx-','linewidth',1); % this is IMU (EKF+DR+ZUPT) solution
%plot(muUS(1,1:end-18*upsamplingRate),muUS(2,1:end-18*upsamplingRate),'b-.','linewidth',3); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'rx-','linewidth',1);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',1,'markerfacecolor','r','markeredgecolor','k','markersize',10);
plot(finalEstimate(1,end),finalEstimate(2,end),'gs','linewidth',1,'markerfacecolor','g','markeredgecolor','k','markersize',10);
else (choice == 3)
plot(mu(1,1:end-30),mu(2,1:end-30), 'b-.','linewidth',2); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,:),finalEstimate(2,:), 'r-','linewidth',2);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',10);
plot(finalEstimate(1,end-30),finalEstimate(2,end-30), 'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',10);
end
hold off;
legend('error-state KF with ZUPT aid','non-recursive Bayesian map-matching with MHT','start','stop');
set(legend,'fontsize',10,'interpreter','latex','location','northeast');
set(gca,'position',[0.001    0.01    0.33    0.8150]);

load bolz_MHT_initialization_51.mat
subplot(1,3,2);
imshow(img); hold on;
% upsamplingRate = 1; muUS = zeros(size(mu,1),upsamplingRate*size(mu,2));
% muUS(1,:) = interp(mu(1,:),upsamplingRate);
% muUS(2,:) = interp(mu(2,:),upsamplingRate);
if (choice == 1 || choice == 2)
plot(mu(1,1:end),mu(2,1:end),'bx-','linewidth',1); % this is IMU (EKF+DR+ZUPT) solution
%plot(muUS(1,1:end-18*upsamplingRate),muUS(2,1:end-18*upsamplingRate),'b-.','linewidth',3); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'rx-','linewidth',1);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',1,'markerfacecolor','r','markeredgecolor','k','markersize',10);
plot(finalEstimate(1,end),finalEstimate(2,end),'gs','linewidth',1,'markerfacecolor','g','markeredgecolor','k','markersize',10);
else (choice == 3)
plot(mu(1,1:end-30),mu(2,1:end-30), 'b-.','linewidth',2); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,:),finalEstimate(2,:), 'r-','linewidth',2);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',10);
plot(finalEstimate(1,end-30),finalEstimate(2,end-30), 'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',10);
end
hold off;
legend('error-state KF with ZUPT aid','non-recursive Bayesian map-matching with MHT','start','stop');
set(legend,'fontsize',10,'interpreter','latex','location','northeast');
set(gca,'position',[0.001+0.33    0.01    0.33    0.8150]);

load bolz_MHT_initialization_52.mat
subplot(1,3,3);
imshow(img); hold on;
% upsamplingRate = 1; muUS = zeros(size(mu,1),upsamplingRate*size(mu,2));
% muUS(1,:) = interp(mu(1,:),upsamplingRate);
% muUS(2,:) = interp(mu(2,:),upsamplingRate);
if (choice == 1 || choice == 2)
plot(mu(1,1:end),mu(2,1:end),'bx-','linewidth',1); % this is IMU (EKF+DR+ZUPT) solution
%plot(muUS(1,1:end-18*upsamplingRate),muUS(2,1:end-18*upsamplingRate),'b-.','linewidth',3); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'rx-','linewidth',1);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',1,'markerfacecolor','r','markeredgecolor','k','markersize',10);
plot(finalEstimate(1,end),finalEstimate(2,end),'gs','linewidth',1,'markerfacecolor','g','markeredgecolor','k','markersize',10);
else (choice == 3)
plot(mu(1,1:end-30),mu(2,1:end-30), 'b-.','linewidth',2); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,:),finalEstimate(2,:), 'r-','linewidth',2);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',10);
plot(finalEstimate(1,end-30),finalEstimate(2,end-30), 'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',10);
end
hold off;
legend('error-state KF with ZUPT aid','non-recursive Bayesian map-matching with MHT','start','stop');
set(legend,'fontsize',10,'interpreter','latex','location','northeast');
set(gca,'position',[0.001+2*0.33    0.01    0.33    0.8150]);
% print('figure','-dpdf','-opengl');