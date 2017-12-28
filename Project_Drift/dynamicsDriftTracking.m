function [ dq ] = dynamicsDriftTracking(t,q)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
model_params;
x = q(1);
y = q(2);
th = q(3);
dx = q(4);
dy = q(5);
dth = q(6);
phi_d = q(7);
V = q(8);
Fu_d = q(9);

generate_desired_Polynomials;

x_d = ppval(pp_states.x,t);
y_d = ppval(pp_states.y,t);
th_d = ppval(pp_states.th,t);
dx_d = ppval(pp_states.dxd,t);
dy_d = ppval(pp_states.dyd,t);
dth_d = ppval(pp_states.dth,t);
V_d = ppval(pp_states.V,t);
Fu_act_d = ppval(pp_states.Fu,t);
phi_act_d = ppval(pp_states.phi,t);
Fu_c = ppval(pp_control.Fu_c,t);
phi_c = ppval(pp_control.phi_c,t);

q = [x;y;th;dx;dy;dth];
q_d = [x_d;y_d;th_d;dx_d;dy_d;dth_d];
% % 
% % u_f_d = [phi_c;Fu_c];
% % % % u_f_e = -100*[cos(th) sin(th) 0;...
% % % %     -sin(th) cos(th) 1]*(q(1:3)-q_d(1:3));
% % % % % u_f_e(2,1) = 100*(y_d - q(2))/;
% % % % % u_f_e(1,1) = pi*1.5/18*sign(th_d - q(2)-(x_d-q(1))+(y_d-q(2))*tan(th));
% % % % u_f_e(2) =  pi*1.5/18*sign(u_f_e(2));
% % % % u_f = u_f_e + u_f_d;
% % % % phi_c = u_f(1);
% % % % Fu_c = u_f(2);

Fu = Fu_c;
phi2 = phi_c;


% % % if(t < 5)
% % % Fu = 10;
% % % else
% % % Fu = 0;    
% % % end
% % % % 
% % % if(t > 0.8 && t < 1.6)
% % % phi2 = 0.2;
% % % else
% % % phi2 = 0;    
% % % end


 dphi_d = 50*(phi2 - phi_d);
 dFu_d = 100*(Fu - Fu_d);
 dV = Fu_d/m;
    if( sqrt(dx^2 + dy^2) > 50 )
alpha1 = getslipAnlge((dy - dth*c),(dx - dth*a),phi_d,th);
alpha2 = getslipAnlge((dy + dth*c),(dx - dth*a),phi_d,th);
alpha3 = getslipAnlge((dy + dth*c),(dx + dth*b),0,th);
alpha4 = getslipAnlge((dy - dth*c),(dx + dth*b),0,th);

if (Fu_d <= -100)
    k = -100;
else
    k = 0;
end

F1 = magicFormula(alpha1*180/pi,k)*u*m*g;
F2 = magicFormula(alpha2*180/pi,k)*u*m*g;
F3 = magicFormula(alpha3*180/pi,k)*u*m*g;
F4 = magicFormula(alpha4*180/pi,k)*u*m*g;

ax = (-F1*cos(phi_d)- F2*cos(phi_d) - F3 - F4)/m;
ay = (-F1*sin(phi_d)- F2*sin(phi_d) + Fu_d*u*m*g/100 + Fu_d*u*m*g/100)/m;

ddx = ax*cos(th)-ay*sin(th);
ddx  = ddx - u*dx;
ddy = ax*sin(th)+ay*cos(th);
ddth = ((F1*sin(phi_d)-F2*sin(phi_d))*c+(F1*cos(phi_d)+F2*cos(phi_d))*a-(F3+F4)*b)/I;
ddth  = ddth - u*dth;

% u = [Fu;Fu;phi1;phi2];
    else
        dx = -V*sin(th)*cos(phi_d);
        dy = V*cos(th)*cos(phi_d);
        dth = V*sin(phi_d)/(a+b);
        ddx = V*sin(th)*sin(phi_d)*dphi_d - V*cos(th)*cos(phi_d)*dth - dV*sin(th)*cos(phi_d);
        ddy = -V*cos(th)*sin(phi_d)*dphi_d - V*sin(th)*cos(phi_d)*dth + dV*cos(th)*cos(phi_d);
        ddth = (V*cos(phi_d) + dV*sin(phi_d))/(a+b); 
    end
dq = [dx;dy;dth;ddx;ddy;ddth;dphi_d;dV;dFu_d];
end

