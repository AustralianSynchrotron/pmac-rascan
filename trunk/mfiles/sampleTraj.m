function [y] = sampleTraj(x,R,L,K)

%%

y = sqrt(R^2-L^2*x^2)/K;


y = floor(y);