function dQ = dynamics(t,Q)
model_params; 

x = Q(1);
y = Q(2);
th = Q(3);
dx = Q(4);
dy = Q(5);
dth = Q(6);

alpha1 = atan((dx - dth*a)/(dy - dth*c));
alpha2 = atan((dx - dth*a)/(dy + dth*c));
alpha3 = atan((dx + dth*b)/(dy + dth*c));
alpha4 = atan((dx + dth*b)/(dy - dth*c));

F1 = magicFormula(alpha1,k)*u*m*g;
F2 = magicFormula(alpha2,k)*u*m*g;
F3 = magicFormula(alpha3,k)*u*m*g;
F4 = magicFormula(alpha4,k)*u*m*g;

ax = (-F1*cos(phi1)- F2*cos(phi2) - F3 - F4)/m;
ay = (-F1*sin(phi1)- F2*sin(phi2) + Fu1 + Fu1)/m;

ddx = ax*cos(th)-ay*sin(th);
ddy = dx*sin(th)+ay*cos(th);
ddth = ((F1*sin(phi1)-F2*sin(phi2))*c+(F1*cos(phi1)+F2*cos(phi2))*a-(F3+F4)*b)/I;

u = [Fu1;Fu2;phi1;phi2];

dQ = [dx;dy;dth;ddx;ddy;ddth];
end