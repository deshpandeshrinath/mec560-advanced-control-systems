function [ dq ] = dynamicsDriftTrackingNoSlip(t,q)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
model_params;
x = q(1);
y = q(2);
th = q(3);

phi_d = q(7);
V = q(8);
Fu_d = q(9);

if(t < 2)
Fu = 1000;
else
Fu = 0;    
end
% 
if(t > 0 && t < 1.6)
phi2 = 0.2;
else
phi2 = 0;    
end
 dphi_d = 50*(phi2 - phi_d);
 dFu_d = 100*(Fu - Fu_d);
 dV = Fu_d;

dx = -V*sin(th)*cos(phi_d);
dy = V*cos(th)*cos(phi_d);
dth = V*sin(phi_d)/a;
ddx = V*sin(th)*sin(phi_d)*dphi_d - V*cos(th)*cos(phi_d)*dth - dV*sin(th)*cos(phi_d);
ddy = -V*cos(th)*sin(phi_d)*dphi_d - V*sin(th)*cos(phi_d)*dth + dV*cos(th)*cos(phi_d);
ddth = (V*cos(phi_d) + dV*sin(phi_d))/(a); 
 
dq = [dx;dy;dth;ddx;ddy;ddth;dphi_d;dV;dFu_d];
end

