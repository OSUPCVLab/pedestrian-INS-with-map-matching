clear all;
close all;
clc;

T = 0.1;
x = -3:T:8;
y = -3:T:7;
[X,Y] = meshgrid(x,y);
mu1 = [1;1]; sigma1 = [1;2]; mu2 = [4;3]; sigma2 = [4;1];
f1 = exp(-0.5*(((X-mu1(1))./sigma1(1)).^2 + ((Y-mu1(2))./sigma1(2)).^2));
f2 = exp(-0.5*(((X-mu2(1))./sigma2(1)).^2 + ((Y-mu2(2))./sigma2(2)).^2));
postNumerical = f1.*f2;
postNumerical = postNumerical./find_integral_2d(postNumerical,T);
[muTh, sigmaTh] = theoretical_multiplication_of_two_2d_gaussians(mu1,sigma1,mu2,sigma2);
postTheory = exp(-0.5*(((X-muTh(1))./sigmaTh(1)).^2 + ((Y-muTh(2))./sigmaTh(2)).^2));
postTheory = postTheory./find_integral_2d(postTheory,T);

%%
figure(1); set(gcf,'position',[503   121   802   677]);
subplot(221);
surf(X,Y,postNumerical); grid on; colormap jet; shading flat;
xlabel('x'); ylabel('y');
subplot(222);
surf(X,Y,postTheory); grid on; colormap jet; shading flat;
xlabel('x'); ylabel('y');
subplot(223);
contour(X,Y,postNumerical,30); grid on; colormap jet;
xlabel('x'); ylabel('y'); title('Numerical');
subplot(224);
contour(X,Y,postTheory,30); grid on; colormap jet;
xlabel('x'); ylabel('y'); title('Closed form');