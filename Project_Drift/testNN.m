clc
close all
clear all

dt = 0.001; 
P = 0.01; 

alp = 2;

X(1) = 1;
X_a(1) = 1;
t(1) = 0;
K = 50;

W(:,1) = 0*rand(16,1);

for i = 2:100000
    t(i) = t(i-1) + dt;
    
    D(:,i) = alp - (X(i-1));
    phi_x = [1;sin(X(i-1));sin(2*X(i-1));sin(3*X(i-1));
             sin(4*X(i-1));sin(5*X(i-1));sin(6*X(i-1));
             sin(7*X(i-1));sin(8*X(i-1));sin(9*X(i-1));
             sin(10*X(i-1));sin(11*X(i-1));sin(12*X(i-1));
             sin(13*X(i-1));sin(14*X(i-1));sin(15*X(i-1));];
    
    u = -W(:,i-1)'*phi_x  - .1*X(i-1);
    % True system
    X(i) = X(i-1) + dt*( D(:,i) + u);
    % Approximate system
    X_a(i) = X_a(i-1) + dt*( W(:,i-1)'*phi_x + u + K*(X(i-1)-X_a(i-1)));
    W(:,i) = W(:,i-1) + P*phi_x*(X(i)-X_a(i)) - .0001*abs(X(i)-X_a(i))*W(:,i-1);
    D_approx(:,i) = W(:,i-1)'*phi_x ; 
end

figure;
plot(t,X(1,:),t,X_a(1,:));
figure;
subplot(2,1,1)
plot(t,D,t,D_approx)
subplot(2,1,2)
plot(t,D-D_approx)

figure;
subplot(2,2,1)
plot(t,W(1,:))
ylabel('w_1')
xlabel('time')
subplot(2,2,2)
plot(t,W(2,:))
ylabel('w_2')
xlabel('time')
subplot(2,2,3)
plot(t,W(3,:))
ylabel('w_3')
xlabel('time')
subplot(2,2,4)
plot(t,W(4,:))
ylabel('w_4')
xlabel('time')