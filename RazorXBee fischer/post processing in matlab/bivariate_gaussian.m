mu = [0;0];
sigma = 5;
x = (mu(1) - 30) : 1 : (mu(1) + 30);
y = (mu(2) - 30) : 1 : (mu(2) + 30);
[X,Y] = meshgrid(x,y);

f = exp(-0.5*(((X-mu(1))./sigma).^2 + ((Y-mu(2))./sigma).^2));

figure(1);
surf(X,Y,f);
grid on; colormap jet;
xlabel('x'); ylabel('y');
set(gca,'Ydir','reverse');