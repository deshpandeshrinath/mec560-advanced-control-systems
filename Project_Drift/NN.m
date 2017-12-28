clc
close all
clear all

dt = 0.001; 
P = 0.01; 

alp = -2;

X(:,1) = [1;3];
X_a(:,1) = [1;3];
t(1) = 0;
K = 50*eye(2);

W(:,:,1) = 0*rand(16,2);

for i = 2:100000
    t(i) = t(i-1) + dt;
    
    D(:,i) = alp+(sin(X(:,i-1)));
    for j=1:1:1
    phi_x(:,j) = [1;sin(X(j,i-1));sin(2*X(j,i-1));sin(3*X(j,i-1));
             sin(4*X(j,i-1));sin(5*X(j,i-1));sin(6*X(j,i-1));
             sin(7*X(j,i-1));sin(8*X(j,i-1));sin(9*X(j,i-1));
             sin(10*X(j,i-1));sin(11*X(j,i-1));sin(12*X(j,i-1));
             sin(13*X(j,i-1));sin(14*X(j,i-1));sin(15*X(j,i-1));];
    end
    u = -W(:,:,i-1)'*phi_x + [sin(.5*t(i-1));sin(.5*t(i-1))] - .1*X(:,i-1);
    % True system
    X(:,i) = X(:,i-1) + dt*( D(:,i) + u);
    % Approximate system
    X_a(:,i) = X_a(:,i-1) + dt*( W(:,:,i-1)'*phi_x + u + K*(X(:,i-1)-X_a(:,i-1)));
    for j=1:1:2
     W(:,j,i) = W(:,j,i-1) + P*phi_x*(X(j,i)-X_a(j,i)) - .0001*abs(X(j,i)-X_a(j,i))*W(:,j,i-1);
    end
    D_approx(:,i) = W(:,:,i-1)'*phi_x ; 
end

figure;
plot(t,X(1,:),t,X_a(1,:));
hold on
plot(t,X(2,:),t,X_a(2,:));
hold off
figure;
subplot(2,1,1)
plot(t,D(1,:),t,D_approx(1,:))
subplot(2,1,2)
plot(t,D(2,:),t,D_approx(2,:))