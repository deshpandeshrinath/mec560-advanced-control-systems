% clc; clear all;
% alpha = -100:2:100;
% k =(-100:2:100);
% [x,y] = meshgrid(alpha,k);
% Fx = zeros(max(size(alpha(1,:))));
% for i=1:1:max(size(alpha(1,:)))
%     for j=1:1:max(size(alpha(1,:)))
% Fx(i,j) =  magicFormula(alpha(i),k(j));
%     end
% end
% figure
% surface(x,y,Fx)
k = -40:0.1:40;
d = 5;
Fx = zeros(max(size(k(1,:))),1);
for i=1:1:max(size(k(1,:)))

Fx(i) =  magicFormula(k(i),d);

end
plot(k,Fx)
xlabel('Slip angle (degrees)')
ylabel('Lateral Force Coefficient')
% d =(-100:1:100);
% f = exp(-5*(d*pi/180).^2)+0.2;
% plot(d,f)
% axis([-100 100 0 1.5])