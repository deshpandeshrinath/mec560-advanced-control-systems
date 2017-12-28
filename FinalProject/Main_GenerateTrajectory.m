%-------------------------- Cart-pole Problem -----------------------%
%--------------------------------------------------------------------%
clear all
close all
clc

x0 = 0;
xf = 3;
y0 = 0;
yf = 3;
th0 = 0;
thf = 0;
phi_d0 = 0;
phi_df = 0;
dV0 = -10;
dVf = 10;
Fu_d0 = 0;
dx0 = 0;
dxf = 0;
dy0 = 0;
dyf = 0;
dth0 = 0;
dthf = 0;


% Auxillary data:
%-------------------------------------------------------------------%
%-------------------- Data Required by Problem ---------------------%
%-------------------------------------------------------------------%
% Some model related constants
gamma = .0001;
auxdata.gamma = gamma ;


% Parameters:
%-------------------------------------------------------------------%
%-------------------- Data Required by Problem ---------------------%
%-------------------------------------------------------------------%


%-------------------------------------------------------------------%
%----------------------------- Bounds ------------------------------%
%-------------------------------------------------------------------%

%-------------------------------------------------------------------%
t0  = 0;
tf  = 5;

xMin = [x0-2 y0-2 -6.28 -20 -20 -20 -50 -50  -pi/4 -50];       %minimum of coordinates
xMax = [xf+2 yf+2 +6.28 20 20 20 50 50 -pi/4 50];       %maximum of coordinates

uMin = [-50 -50]; %minimum of torques
uMax = [50 50]; %maximum of torques

% setting up bounds
bounds.phase.initialtime.lower  = 0;
bounds.phase.initialtime.upper  = 0;
bounds.phase.finaltime.lower    = 0.5; 
bounds.phase.finaltime.upper    = tf;
bounds.phase.initialstate.lower = [x0 y0 th0 dx0 dy0 dth0 -5 -5  -pi/5 -5];
bounds.phase.initialstate.upper = [x0 y0 th0 dx0 dy0 dthf 5 5  pi/5 5];
bounds.phase.state.lower        = xMin;
bounds.phase.state.upper        = xMax;
bounds.phase.finalstate.lower   = [xf yf thf dxf dyf dth0 -5 -5  -pi/5 -5];
bounds.phase.finalstate.upper   = [xf yf thf dxf dyf dthf 5 5  pi/5 5];
bounds.phase.control.lower      = uMin; 
bounds.phase.control.upper      = uMax; 
bounds.phase.integral.lower     = 0;
bounds.phase.integral.upper     = 100;



%-------------------------------------------------------------------%
%--------------------------- Initial Guess -------------------------%
%-------------------------------------------------------------------%
rng(0);

xGuess = [x0;xf]; 
yGuess = [y0;yf]; 
thGuess = [th0;thf]; 
dxGuess = [dx0;dxf];
dyGuess = [dy0;dyf]; 
dthGuess = [dth0;dthf];
phi_dGuess = [phi_d0;phi_df];
vd_Guess = [0;0]; 
Fu_dGuess = [Fu_d0;Fu_d0];
u1Guess = [0;0];
u2Guess = [0;0];
u3Guess = [0;0];
tGuess = [0;tf]; 

guess.phase.time  = tGuess;
guess.phase.state = [xGuess,yGuess,thGuess,dxGuess,dyGuess,dthGuess,u1Guess,u2Guess,phi_dGuess,vd_Guess];
guess.phase.control        = [u1Guess,u2Guess];
guess.phase.integral         = 0;

% 
% load solution.mat
% guess  = solution

%-------------------------------------------------------------------%
%--------------------------- Problem Setup -------------------------%
%-------------------------------------------------------------------%
setup.name                        = 'drft-Problem';
setup.functions.continuous        = @driftContinuous;
setup.functions.endpoint          = @drftEndpoint;
setup.bounds                      = bounds;
setup.auxdata                     = auxdata;
setup.functions.report            = @report;
setup.guess                       = guess;
setup.nlp.solver                  = 'ipopt';
setup.derivatives.supplier        = 'sparseCD';
setup.derivatives.derivativelevel = 'first';
setup.scales.method               = 'none';
setup.derivatives.dependencies    = 'full';
setup.mesh.method                 = 'hp-PattersonRao';
setup.mesh.tolerance              = 1e-1;
setup.method                      = 'RPM-Integration';

%-------------------------------------------------------------------%
%------------------- Solve Problem Using GPOPS2 --------------------%
%-------------------------------------------------------------------%
output = gpops2(setup);
output.result.nlptime
solution = output.result.solution;

%-------------------------------------------------------------------%
%--------------------------- Plot Solution -------------------------%
%-------------------------------------------------------------------%
states = solution.phase.state;
control = solution.phase.control;
time = solution.phase.time;

plotDifting;
DriftingAnalysis(time,states);













