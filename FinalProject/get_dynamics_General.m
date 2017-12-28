function dq = get_dynamics_General(t,q,ctrl)
uc1 = ctrl(1);
uc2 = ctrl(2);

model_params;
x = q(1);
y = q(2);
th = q(3);
dx = q(4);
dy = q(5);
dth = q(6);
u1 = q(7);
u2 = q(8);
phi = q(9);
V = q(10);

%%

Kx = 100;
Ky = 100;
Ku1 = 100;
Ku2 = 100;

du1 = Ku1*(uc1 - u1);
du2 = Ku2*(uc2 - u2);

phic = atan2(u2*l,u1);
if(phic ~= 0)
    Vc = u2*l/sin(phic);
else
    Vc = u1/cos(phic);
end

dV_dPhi = [cos(phic) l*sin(phic); -sin(phic)/Vc l*cos(phic)/Vc]*[u1;u2];
dV = dV_dPhi(1,1);
if(Vc == 0)
dphi = 0;
else
dphi = dV_dPhi(2,1);
end

dphi = Ku1*(phic - phi);
dV = Ku1*(Vc - V);
Fu_d = dV ;
% % generate_desired_Polynomials;
% % 
% % x_d = ppval(pp_states.x,t);
% % y_d = ppval(pp_states.y,t);
% % th_d = ppval(pp_states.th,t);
% % dx_d = ppval(pp_states.dxd,t);
% % dy_d = ppval(pp_states.dyd,t);
% % dth_d = ppval(pp_states.dth,t);
% % V_d = ppval(pp_states.V,t);
% % Fu_act_d = ppval(pp_states.Fu,t);
% % phi_act_d = ppval(pp_states.phi,t);
% % Fu_c = ppval(pp_control.Fu_c,t);
% % phi_c = ppval(pp_control.phi_c,t);

%%control law





    if( sqrt(dx^2 + dy^2) > 50 )
        
alpha1 = getslipAnlge((dy - dth*c),(dx - dth*a),phi,th);
alpha2 = getslipAnlge((dy + dth*c),(dx - dth*a),phi,th);
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

ax = (-F1*cos(phi)- F2*cos(phi) - F3 - F4)/m;
ay = (-F1*sin(phi)- F2*sin(phi) + Fu_d*u*m*g/100 + Fu_d*u*m*g/100)/m;

ddx = ax*cos(th)-ay*sin(th);
ddx  = ddx - u*dx;
ddy = ax*sin(th)+ay*cos(th);
ddth = ((F1*sin(phi)-F2*sin(phi))*c+(F1*cos(phi)+F2*cos(phi))*a-(F3+F4)*b)/I;
ddth  = ddth - u*dth;

% u = [Fu;Fu;phi1;phi2];
    else
        dx = -sin(th)*u1 - a*cos(th)*u2;
        dy = cos(th)*u1 - a*sin(th)*u2;
        dth = u2;
        ddx = V*sin(th)*sin(phi)*dphi - V*cos(th)*cos(phi)*dth - dV*sin(th)*cos(phi);
        ddy = -V*cos(th)*sin(phi)*dphi - V*sin(th)*cos(phi)*dth + dV*cos(th)*cos(phi);
        ddth = (V*cos(phi) + dV*sin(phi))/(a+b); 
    end
dq = [dx;dy;dth;ddx;ddy;ddth;du1;du2;dphi;dV];
end