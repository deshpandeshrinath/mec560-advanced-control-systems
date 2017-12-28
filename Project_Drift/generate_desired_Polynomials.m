load('kinematicTurn.mat');
time = solution.phase.time;
states = solution.phase.state;
control = solution.phase.control;

pp_x = spline(time,states(:,1));
pp_y = spline(time,states(:,2));
pp_th = spline(time,states(:,3));
pp_dx = spline(time,states(:,4));
pp_dy = spline(time,states(:,5));
pp_dth = spline(time,states(:,6));
pp_phi = spline(time,states(:,7));
pp_V = spline(time,states(:,8));
pp_Fu = spline(time,states(:,9));
pp_phi_c = spline(time,control(:,2));
pp_Fu_c = spline(time,control(:,1));

pp_states.x = pp_x;
pp_states.y = pp_y;
pp_states.th = pp_th;
pp_states.dxd = pp_dx;
pp_states.dyd = pp_dy;
pp_states.dth = pp_dth;
pp_states.phi = pp_phi;
pp_states.V = pp_Fu;
pp_states.Fu = pp_Fu;
pp_control.phi_c = pp_phi_c;
pp_control.Fu_c = pp_Fu_c;

