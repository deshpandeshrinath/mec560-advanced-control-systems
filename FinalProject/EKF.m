function time = EKF()
generate_desired_Polynomials;
model_params
dt = 0.001;
v = .5;
head_ang = pi/4;
i = 1;
states = zeros(6,length(0:dt:max(time)));
for t = 0:dt:max(time);
states(i,1) = ppval(pp_states.x,t);
states(i,2) = ppval(pp_states.y,t);
states(i,3) = ppval(pp_states.th,t);
states(i,4) = ppval(pp_states.dxd,t);
states(i,5) = ppval(pp_states.dyd,t);
states(i,6) = ppval(pp_states.dth,t);
i = i+1;
end
time = 0:dt:max(time);

C = [1 0 0;
    0 1 0];
R = diag([.01 .01]);

Q = diag([.0001 .0001]);
P_pl = eye(3);

X_hat_0 = [0;0;0];
X_0 = [states(1,1);states(1,2);states(1,3)];
X_hat_0 = X_0;


X = [states(:,1)';states(:,2)';states(:,3)'];
dX = [states(:,4)';states(:,5)';states(:,6)'];
Y(:,1) = C*X(:,1);
t(1) = 0;
X_hat(:,1)= X_hat_0;
t = time;

for i = 1:length(time)-1
    dt = t(i+1)-t(i);
    Kx = 200;
    Ky = 200;
    Kth = 200;
    u = [-sin(X_hat(3,i)) cos(X_hat(3,i)); -cos(X_hat(3,i))/a -sin(X_hat(3,i))/a]*[-Kx*(X_hat(1,i)-X(1,i))+dX(1,i); -Ky*(X_hat(2,i)-X(2,i))+dX(2,i)];
    u1 = u(1);
%     u2 = u(2);
    u2 = - Kth*(X_hat(3,i)-X(3,i))+dX(3,i);
    % True measurement
    Y(:,i+1) = C*X(:,i+1) + sqrt(R)*randn(size(C,1),1);  %*randn(2,1);
    % Observer model
    dFdx = [0 0 -cos(X_hat(3,i))*u1+a*u2*sin(X_hat(3,i)); 0 0 -sin(X_hat(3,i))*u1-a*u2*cos(X_hat(3,i)); 0 0 0 ];
%     dFdx = [0 0 -v*sin(X_hat(3,i)); 0 0 v*cos(X_hat(3,i)); 0 0 0 ];
    dFdx_d = eye(3) + dt*dFdx;
    
%     F = [dt*cos(X_hat(3,i)) 0;
%         dt*sin(X_hat(3,i)) 0;
%         0 dt];
    F = [-sin(X_hat(3,i)) -a*cos(X_hat(3,i));...
    cos(X_hat(3,i)) -a*sin(X_hat(3,i));
    0 1]*dt;
    
    P_mi = dFdx_d*P_pl*dFdx_d' + F*Q*F';
    
    X_hat(:,i+1) = X_hat(:,i) + dt*[-sin(X_hat(3,i))*u1 - a*cos(X_hat(3,i))*u2;
        cos(X_hat(3,i))*u1 - a*sin(X_hat(3,i))*u2;
        u2];
    Y_hat(:,i+1) = C*X_hat(:,i+1);
    % Update based on measurement
    e_Y  = Y(:,i+1) - Y_hat(:,i+1);
    S = C*P_mi*C'+R;
    K = P_mi*C'*inv(S);
    P_pl = (eye(3) - K*C)*P_mi;
    X_hat(:,i+1)=X_hat(:,i+1) + K*e_Y;
end
X_hat(3,1) = X_hat(3,end);
P_pl = P_mi;

figure;
subplot(2,1,1)
plot(t,X(1,:),t,X_hat(1,:),'--')
legend('true','Tracked')
%axis([0 2.5 0 40])
xlabel('time')
ylabel('X')
% axis([0 4 0 2])
subplot(2,1,2)
plot(t,X(2,:),t,X_hat(2,:),'--')
legend('true','Tracked')
xlabel('time')
ylabel('Y')
% axis([0 4 0 2])
figure;
plot(t,X(3,:),t,X_hat(3,:),'--')
% axis([0 4 0 1])
legend('true','Tracked')
xlabel('time')
ylabel('Theta')
% NewStates(:,1) = X_hat(1,:)';
% NewStates(:,2) = X_hat(2,:)';
% NewStates(:,2) = X_hat(3,:)';
end


%axis([0 2.5 0 2])figure;