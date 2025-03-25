function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters
% params.mass
% params.I  
% params.invI
% params.gravity 
% params.arm_length 
% 
% params.minF 
% params.maxF 

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Thrust
% F = 0;original

% Moment
% M = zeros(3); original

% *** my code ***

gravity = params.gravity;
mass = params.mass;
I = params.I;
invI = params.invI;

xdes_ddot = des_state.acc(1);
xdes_dot = des_state.vel(1);
xdes = des_state.pos(1);

ydes_ddot = des_state.acc(2);
ydes_dot = des_state.vel(2);
ydes = des_state.pos(2);

zdes_ddot = des_state.acc(3);
zdes_dot = des_state.vel(3);
zdes = des_state.pos(3);

yaw = des_state.yaw;
yawdot = des_state.yawdot;

% -- 1. Define state vector
x = state.pos(1);
y = state.pos(2);
z = state.pos(3);

x_dot = state.vel(1);
y_dot = state.vel(2);
z_dot = state.vel(3);


% rotation angles
phi = state.rot(1);
theta = state.rot(2);
psi = state.rot(3);


% angle acceleration omegas
p = state.omega(1);
q = state.omega(2);
r = state.omega(3);

% --- Track Trajectory: Define position and velocity errors
errpos = des_state.pos - state.pos;
errvel = des_state.vel - state.vel;

% -- desired states ---
xdes_ddot = des_state.acc(1);
ydes_ddot = des_state.acc(2);
zdes_ddot = des_state.acc(3);

% (12) Hover Control
psiT = des_state.yaw;
% TO DO: Check if input is degree or radian ??
spsiT = sin(psiT);
cpsiT = cos(psiT);

based = 5;
kd1 = based; 
kp1 = kd1 * 5; 

kd2 = based; 
kp2 = kd2 * 5; 

kd3 = based; 
kp3 = kd3 * 5; 

r1des_ddt = xdes_ddot + kd1 * errvel(1) + kp1 * errpos(1);
r2des_ddt = ydes_ddot + kd2 * errvel(2) + kp2 * errpos(2);
r3des_ddt = zdes_ddot + kd3 * errvel(3) + kp3 * errpos(3);

phi_des = (1/gravity) * (r1des_ddt * spsiT - r2des_ddt * cpsiT);
theta_des = (1/gravity) * (r1des_ddt * cpsiT + r2des_ddt * spsiT);


% --- u1 ---
kd3 = 5;  
kp3 = kd3 * 6;  
% (13) TO DO: Check instruction (16) against handout about u1
u1 = mass * (gravity + r3des_ddt); 


% Attitude Control: u2 formula (10)
pdes = 0; % (15)
qdes = 0;
psi_des = yaw;
rdes = yawdot; % (16)

base = 20;      
kpphi = base * 25;
kdphi = kpphi /7 ; 

kptheta = base * 25;
kdtheta = kptheta / 7; 

kppsi = base * 25;
kdpsi= kppsi / 7;

% Orientation (8) & (5): Newton Equation of Motion: position vector r's
% deriviatives

% (10) Atitude Control

u2 = [kpphi * (phi_des - phi) + kdphi * (pdes - p); ...
      kptheta * (theta_des - theta) + kdtheta * (qdes - q); ...
      kppsi * (psi_des - psi) + kdpsi * (rdes - r)];
 
    
% --- Final Final Results F and M ---
F = u1;
M = I * u2;

% 12/28 submit run results
% Evaluating...
% 
% Line trajectory:
% Cumulative position error: 0.024062
% 
% Helix trajectory:
% Cumulative position error: 0.25815
% 
% Test trajectory generator:
% Successfully passed through 5/5 waypoints in 17.69 sec with a trajectory length of 7.3695 meters
% =================== Your code ends here ===================

end
