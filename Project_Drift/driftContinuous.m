function phaseout = driftContinuous(input)


x_all   = input.phase.state(:,1);
y_all   = input.phase.state(:,2);
th_all  = input.phase.state(:,3);
dx_all  = input.phase.state(:,4);
dy_all  = input.phase.state(:,5);
dth_all = input.phase.state(:,6);
phi_d_all = input.phase.state(:,7);
V_all = input.phase.state(:,8);
Fu_d_all = input.phase.state(:,9);
Fu_all   = input.phase.control(:,1);
phi1_all   = input.phase.control(:,2);
phi2_all   = input.phase.control(:,2);

dq_all = get_dynamics_General(x_all,y_all,th_all,dx_all,dy_all,dth_all,phi_d_all,V_all,Fu_all,phi1_all,phi2_all,Fu_d_all);


phaseout.dynamics  = dq_all;
% phaseout.integrand = zeros(size(Fu_all));
phaseout.integrand =  phi2_all.^2;
end