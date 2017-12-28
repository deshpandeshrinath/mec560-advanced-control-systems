clc
close all
clear all


dt = 0.001;
v = .5;
head_ang = pi/4;


C = [1 0 0;
    0 1 0];
R = diag([.001 .001]);

Q = diag([.1 .01]);
P_pl = eye(3);

X_hat_0 = [0;0;0];
X_0 = [.1;.1;head_ang];
X_hat_0 = X_0/2;


X(:,1)= X_0;
Y(:,1) = C*X(:,1);
t(1) = 0;
X_hat(:,1)= X_hat_0;

for i = 1:4000
    u = [.5;0];
        
    t(i+1) = t(i)+dt;
    % True process
    X(:,i+1)= X(:,i) + dt*[u(1)*cos(X(3,i));
        u(1)*sin(X(3,i));
        u(2);];
    Y(:,i+1) = C*X(:,i+1) + sqrt(R)*randn(size(C,1),1);  %*randn(2,1);
    % Observer model
        
    dFdx = [0 0 -v*sin(X_hat(3,i)); 0 0 v*cos(X_hat(3,i)); 0 0 0 ];
    dFdx_d = eye(3) + dt*dFdx;
    
    F = [dt*cos(X_hat(3,i)) 0;
    dt*sin(X_hat(3,i)) 0;
    0 dt];
    
    P_mi = dFdx_d*P_pl*dFdx_d' + F*Q*F';
       
    X_hat(:,i+1) = X_hat(:,i) + dt*[u(1)*cos(X_hat(3,i));
            u(1)*sin(X_hat(3,i));
          u(2);];
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
axis([0 4 0 2])
subplot(2,1,2)
plot(t,X(2,:),t,X_hat(2,:),'--')
legend('true','Tracked')
xlabel('time')
ylabel('Y')
axis([0 4 0 2])
figure;
plot(t,X(3,:),t,X_hat(3,:),'--')
axis([0 4 0 1])
xlabel('time')
ylabel('Theta')
%axis([0 2.5 0 2])figure;