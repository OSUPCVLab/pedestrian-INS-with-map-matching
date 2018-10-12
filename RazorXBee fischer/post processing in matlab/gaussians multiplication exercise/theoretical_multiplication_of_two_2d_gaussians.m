function [mu, sigma] = theoretical_multiplication_of_two_2d_gaussians(mu1,sigma1,mu2,sigma2)

mu(1) = (sigma1(1)^(-2)*mu1(1) + sigma2(1)^(-2)*mu2(1)) / (sigma1(1)^(-2)+sigma2(1)^(-2));
mu(2) = (sigma1(2)^(-2)*mu1(2) + sigma2(2)^(-2)*mu2(2)) / (sigma1(2)^(-2)+sigma2(2)^(-2));
sigma(1) = sqrt((sigma1(1)^2)*(sigma2(1)^2) / (sigma1(1)^2+sigma2(1)^2));
sigma(2) = sqrt((sigma1(2)^2)*(sigma2(2)^2) / (sigma1(2)^2+sigma2(2)^2));