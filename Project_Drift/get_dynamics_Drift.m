function dq_all = get_dynamics_Drift(x_all,y_all,th_all,dx_all,dy_all,dth_all,phi_d_all,Fu_all,phi1_all,phi2_all,Fu_d_all)
model_params; 

dq_all = zeros(length(th_all),8);

for i = 1:length(th_all);
    th = th_all(i);    
    x = x_all(i);
    y = y_all(i);
    dy = dy_all(i);
    dth = dth_all(i);    
    dx = dx_all(i);
    Fu = Fu_all(i);
    phi_d = phi_d_all(i);
    Fu_d = Fu_d_all(i);
    phi1 = phi1_all(i);
    phi2 = phi1;

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
dphi_d = 50*(phi2 - phi_d);
dFu_d = 100*(Fu - Fu_d);
% u = [Fu;Fu;phi1;phi2];

dq_all(i,:) = [dx;dy;dth;ddx;ddy;ddth;dphi_d;dFu_d];
end
end