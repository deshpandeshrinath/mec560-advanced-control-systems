clc
close all
clear all

q = [0;0;0;0;0;0;0;0];
generate_desired_Polynomials;
% DriftingAnalysis(time,states);
t = 0;

dq = dynamicsDriftTracking(t,q);
time_span = 0:0.01:1;

Tspan = [0 13];
[time,states] = runge_kutta_4order(@(t,x,i)dynamicsDriftTracking(obj,t,x,i), Tspan, [1 0 1 0]', timeStep);

plotDifting;
DriftingAnalysis(time,states);