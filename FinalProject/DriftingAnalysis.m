% states = solution.phase.state;
% control = solution.phase.control;
% time = solution.phase.time;
function [states] = DriftingAnalysis(time,states)
model_params;

figure;
subplot(2,1,1)
plot(time,states(:,1),'r-',time,states(:,2),'b-',time,states(:,3),'g-')
xlabel('time')
title('position and orientation')
legend('x','y','\theta')
subplot(2,1,2)
plot(time,states(:,4),'r-',time,states(:,5),'b-',time,states(:,6),'g-')
xlabel('time')
title('velocities')
legend('V_x','V_y','\omega')
figure;
subplot(2,1,1)
plot(time,states(:,9),'b-')
xlabel('time')
ylabel('Propulsion')
legend('F_u Actual')
subplot(2,1,2)
plot(time,states(:,8),'b-',time,states(:,6),'r-')
xlabel('time')
ylabel('WheelVelocoty')
legend('V','\alpha')
figure;
plot(time,states(:,7)*180/pi,'b-')
xlabel('time')
ylabel('steeringAnlges')
legend('\phi Actual')

information = zeros(length(states(:,1)),11);

for i = 1:length(states(:,1));
    th = states(i,3);    
    x = states(i,1);
    y = states(i,2);
    dy = states(i,5);
    dth = states(i,6);    
    dx = states(i,4);
    Fu = states(i,8);
    phi_d = states(i,7);
% %     if(time(i) > 0.7 && time(i) < 1.6)
% %         phi1 = 1;
% %         phi_d = 1;
% %     else
% %         phi1 = 0;
% %         phi_d = 0;    
% %     end
% %     
% %     if(t < 2)
% %         thrust = 100*time(i)^2;
% %     else
% %         thrust = 0;    
% %     end

alpha1 = getslipAnlge((dy - dth*c),(dx - dth*a),phi_d,th);
alpha2 = getslipAnlge((dy + dth*c),(dx - dth*a),phi_d,th);
alpha3 = getslipAnlge((dy + dth*c),(dx + dth*b),0,th);
alpha4 = getslipAnlge((dy - dth*c),(dx + dth*b),0,th);

F1 = magicFormula(alpha1,0)*u*m*g;
F2 = magicFormula(alpha2,0)*u*m*g;
F3 = magicFormula(alpha3,0)*u*m*g;
F4 = magicFormula(alpha4,0)*u*m*g;
ax = (-F1*cos(phi_d)- F2*cos(phi_d) - F3 - F4)/m;
ay = (-F1*sin(phi_d)- F2*sin(phi_d) + Fu + Fu)/m;

ddx = ax*cos(th)-ay*sin(th);
ddy = dx*sin(th)+ay*cos(th);
ddth = ((F1*sin(phi_d)-F2*sin(phi_d))*c+(F1*cos(phi_d)+F2*cos(phi_d))*a-(F3+F4)*b)/I;

% u = [Fu;Fu;phi1;phi_d];

information(i,:) = [alpha1;alpha2;alpha3;alpha4;sqrt(dx^2+dy^2)*sin(th);ax;F1;F2;F3;F4;ddth];
end

% figure;
% subplot(2,1,1)
% plot(time,information(:,1)*180/pi,time,information(:,2)*180/pi,time,information(:,3)*180/pi,time,information(:,4)*180/pi)
% xlabel('time')
% ylabel('slipAngles')
% legend('\alpha_1','\alpha_2','\alpha_3','\alpha_4');
% subplot(2,1,2)
% plot(time,information(:,5),time,information(:,6),time,information(:,11))
% xlabel('time')
% ylabel('centrifugal terms')
% legend('speed','a_x','M_x');
% figure;
% plot(time,information(:,7),time,information(:,8),time,information(:,9),time,information(:,10))
% xlabel('time')
% ylabel('LatForces')
% legend('F_1','F_2','F_3','F_1');
end