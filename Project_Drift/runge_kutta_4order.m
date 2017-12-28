function [t,y] = runge_kutta_4order(dy, tspan, y0, dt)

 
t0 = tspan(1); tfinal = tspan(end);
 
% set up the t values at which we will approximate the solution
t = [t0:dt:tfinal]';
 
% include tfinal even if h does not evenly divide tfinal-t0
if t(end)~=tfinal, t=[t tfinal]; end
 
m = length(t);
y = [y0 zeros(length(y0), length(t)-1)];
 
 for i=1:(m-1)
 k1 = feval(dy,t(i),y(:,i),i);
 k2 = feval(dy,t(i)+0.5*dt, y(:,i)+(dt.*k1)/2,i);
 k3 = feval(dy,t(i)+0.5*dt, y(:,i)+(dt.*k2)/2,i+1);
 k4 = feval(dy,t(i)+dt, y(:,i)+dt.*k3,i+1);
 
 y(:,i+1) = y(:,i)+(dt*(k1+2*k2+2*k3+k4))/6;
 end