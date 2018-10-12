function [mu, sigma] = theoretical_multiplication_of_two_gaussians(mu1,sigma1,mu2,sigma2)

mu = (sigma1^(-2)*mu1 + sigma2^(-2)*mu2) / (sigma1^(-2)+sigma2^(-2));
sigma = sqrt((sigma1^2)*(sigma2^2) / (sigma1^2+sigma2^2));