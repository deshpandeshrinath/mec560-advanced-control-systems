clc
close all
clear all


generate_desired_Polynomials;
DriftingAnalysis(time,states);
t = 0;

x_0 = ppval(pp_states.x,t);
y_0 = ppval(pp_states.y,t);
th_0 = ppval(pp_states.th,t);
dx_0 = ppval(pp_states.dxd,t);
dy_0 = ppval(pp_states.dyd,t);
dth_0 = ppval(pp_states.dth,t);
v0 = ppval(pp_states.Fu,t);
Fu_act_0 = ppval(pp_states.Fu,t);
phi_act_0 = ppval(pp_states.phi,t);

q = [x_0;y_0;th_0;dx_0;dy_0;dth_0;phi_act_0;v0;Fu_act_0];

time_span = 0:0.01:max(time);

% % q = [0;0;0;0;0;0;0;0;0];
% % time_span = 0:0.01:1.8;
[time,states] = ode45(@dynamicsDriftTracking,time_span,q);

plotDifting;
DriftingAnalysis(time,states);