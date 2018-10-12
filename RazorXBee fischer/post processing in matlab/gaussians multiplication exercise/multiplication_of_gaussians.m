clear all;
close all;
clc;

% a simulation to verify that the result of multiplication of gaussians are
% another gaussian as happens in Kalman Filter.
T = 1e-3;
x = -4:T:14;
sigma0 = 1; mu0 = 1;
A = 4; sigma2_w = 0.01; %w = sqrt(sigma2_w)*randn();
sigma2 = 0.75; mu2 = 8;
sigma3 = 0.6; mu3 = 5;
% previous time posterior --> px_kmin1givenZ_kmin1 stands for p(x_{k-1}|Z_{k-1})
% current time prior --> px_kgivenZ_min1 stands for p(x_k|Z_{k-1)
% current time likelihood --> pz_kgivenx_k stands for p(z_k|x_k)
% current time posterior --> px_kgivenz_k stands for p(x_k|z_k)

px_kmin1givenZ_kmin1 = ( 1 / sqrt(2*pi*sigma0^2) ) * exp( -0.5 * ( (x-mu0).^2 / (sigma0^2) ) );
% apply motion model, i.e., propagate the state to obtain prior at time k
%muPrior = A * mu0 + w;
muPrior = A * mu0;
sigmaPrior = 1.4*sigma0;
% prior
px_kgivenZ_kmin1 = ( 1 / sqrt(2*pi*sigmaPrior^2) ) * exp( -0.5 * ( (x-muPrior).^2 / (sigmaPrior^2) ) );
% sensor 1 likelihood
p1z_kgivenx_k = ( 1 / sqrt(2*pi*sigma2^2) ) * exp( -0.5 * ( (x-mu2).^2 / (sigma2^2) ) );
% sensor 2 likelihood
p2z_kgivenx_k = ( 1 / sqrt(2*pi*sigma3^2) ) * exp( -0.5 * ( (x-mu3).^2 / (sigma3^2) ) );
% posterior when only sensor 1 likelihood is used
p1x_kgivenz_k = px_kgivenZ_kmin1.*p1z_kgivenx_k; % posterior is proportional to prior x likelihood
[muTheory1,sigmaTheory1] = theoretical_multiplication_of_two_gaussians(muPrior,sigmaPrior,mu2,sigma2);
p1x_kgivenz_k_theoretical = ( 1 / sqrt(2*pi*sigmaTheory1^2) ) * exp( -0.5 * ( (x-muTheory1).^2 / (sigmaTheory1^2) ) );
% posterior when only sensor 1 and 2 likelihoods are used - numerical solution
p12x_kgivenz_k = px_kgivenZ_kmin1.*p1z_kgivenx_k.*p2z_kgivenx_k;
% normalize posterior pdfs
N1 = find_integral(p1x_kgivenz_k, T);
N12 = find_integral(p12x_kgivenz_k, T);
p1x_kgivenz_k = p1x_kgivenz_k / N1;
p12x_kgivenz_k = p12x_kgivenz_k / N12;
% posterior when only sensor 1 and 2 likelihoods are used - closed-form solution
[muTemp,sigmaTemp] = theoretical_multiplication_of_two_gaussians(muPrior,sigmaPrior,mu2,sigma2);
[muPost2,sigmaPost2] = theoretical_multiplication_of_two_gaussians(muTemp,sigmaTemp,mu3,sigma3);
p12x_kgivenz_k_theoretical = ( 1 / sqrt(2*pi*sigmaPost2^2) ) * exp( -0.5 * ( (x-muPost2).^2 / (sigmaPost2^2) ) );
%%
figure(1); set(gcf,'position',[43 383 795 350]); lw = 1.5;
%subplot(2,1,1);
plot(x, px_kmin1givenZ_kmin1, 'k-', 'linewidth', lw); % posterior at time k-1
grid on; hold on;
plot(x, px_kgivenZ_kmin1,'b-','linewidth',lw); % prior at time k
plot(x, p1z_kgivenx_k, 'g-', 'linewidth', lw); % likelihood 1
plot(x, p1x_kgivenz_k,'r-','linewidth',lw); % posterior at time k
plot(x, p1x_kgivenz_k_theoretical, 'y--','linewidth',lw); % posterior at time k computed with closed-form solution
h = legend('$p(\mathbf{x_{k-1}}|Z_{k-1})$ - posterior at time k-1',...
    '$p(\mathbf{x_k}|Z_{k-1})$ - prior at time k',...
    '$p(\mathbf{z_k}|\mathbf{x_k})$ - likelihood at time k',...
    '$p(\mathbf{x_k}|Z_k)$ - posterior at time k (numerical)',...
    '$p(\mathbf{x_k}|Z_k)$ - posterior at time k (closed-form)');
set(h, 'interpreter', 'latex', 'location', 'northwest','fontsize',11);
set(gca,'position',[0.0446 0.1100 0.9353 0.8150],'gridlinestyle','--');
hold off;
%%
figure(2); set(gcf,'position',[739 383 795 350]); lw = 1.5;
%subplot(2,1,1);
plot(x, px_kmin1givenZ_kmin1, 'k-', 'linewidth', lw); % posterior at time k-1
grid on; hold on;
plot(x, px_kgivenZ_kmin1,'b-','linewidth',lw); % prior at time k
plot(x, p1z_kgivenx_k, 'g-', 'linewidth', lw); % likelihood 1
plot(x, p2z_kgivenx_k, 'm-', 'linewidth', lw); % likelihood 1
plot(x, p12x_kgivenz_k,'r-','linewidth',lw); % posterior at time k - numerical solution
plot(x, p12x_kgivenz_k_theoretical, 'y--','linewidth',lw); % posterior at time k computed with closed-form solution
h = legend('$p(\mathbf{x_{k-1}}|Z_{k-1})$ - posterior at time k-1',...
    '$p(\mathbf{x_k}|Z_{k-1})$ - prior at time k',...
    '$p(\mathbf{z_k^1}|\mathbf{x_k})$ - likelihood 1 at time k',...
    '$p(\mathbf{z_k^2}|\mathbf{x_k})$ - likelihood 2 at time k',...
    '$p(\mathbf{x_k}|Z_k)$ - posterior at time k (numerical)',...
    '$p(\mathbf{x_k}|Z_k)$ - posterior at time k (closed-form)');
set(h, 'interpreter', 'latex', 'location', 'northwest','fontsize',11);
set(gca,'position',[0.0446 0.1100 0.9353 0.8150],'gridlinestyle','--');
hold off;