function phaseout = driftContinuous(input)

Q = input.phase.state;
Cntrl = input.phase.control;
% x_all   = input.phase.state(:,1);
% y_all   = input.phase.state(:,2);
th_all  = input.phase.state(:,3);
% dx_all  = input.phase.state(:,4);
% dy_all  = input.phase.state(:,5);
% dth_all = input.phase.state(:,6);
% phi_d_all = input.phase.state(:,7);
% V_all = input.phase.state(:,8);
% Fu_d_all = input.phase.state(:,9);
% Fu_all   = input.phase.control(:,1);
% phi1_all   = input.phase.control(:,2);
 phi2_all   = input.phase.control(:,2);

dq_all = zeros(length(th_all),length(input.phase.state(1,:)));

for i = 1:length(th_all)
    q = Q(i,:);
    cntrl = Cntrl(i,:);
dq_all(i,:)  = get_dynamics_General(0,q,cntrl);
end

phaseout.dynamics  = dq_all;
phaseout.integrand = zeros(size(phi2_all));
% phaseout.integrand =  phi2_all.^2;
end