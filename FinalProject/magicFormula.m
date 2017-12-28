function [magic] = magicFormula(alpha,k)
%fitting Coefficients
b = 0.214;
c = 1.60;
e = -0.20;

%alpha is side slip angle in degrees 
% k is longitunal slip percentage
magic = 5*(exp(-5*(k*pi/180).^2/(abs((alpha*pi/40)^2)+1)^0.5)+0.1)*sin(c* atan(b*(1-e)*alpha+e*atan(b*alpha)))/1.1; 
end