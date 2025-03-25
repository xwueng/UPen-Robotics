function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

% Original code
u1 = 0;
u2 = 0;

% *** my code ***

gravity = params.gravity;
mass = params.mass;
Ixx = params.Ixx;

ydes_ddot = des_state.acc(1,1);
ydes_dot = des_state.vel(1,1);
ydes = des_state.pos(1,1);

zdes_ddot = des_state.acc(2,1);
zdes_dot = des_state.vel(2,1);
zdes = des_state.pos(2,1);


% -- 1. Define state vector
y = state.pos(1, 1);
z = state.pos(2, 1);
y_dot = state.vel(1, 1);
z_dot = state.vel(2, 1);
phi = state.rot;
phi_dot = state.omega;


% --- 3. Track Trajectory: Define position and velocity errors
% errp = des_state.pos - state.pos;
% errv = des_state.vel - state.vel;


% --- 5. Trajectory Controller Equations
% 5.1 phic
kvy = 6; % according to the guide, Y-position accuracy
kpy = 0;  % according to the guide, Y-position accuracy
phic = -(1/gravity) * (ydes_ddot + kvy * (ydes_dot - y_dot) + kpy * (ydes - y)); 


% 5.2 u1
kvz = 5;  % ?? tune Z-position
kpz = kvz * 6;  % ?? tune  Z-position
u1 = mass * (gravity + zdes_ddot + kvz * (zdes_dot - z_dot) + kpz * (zdes - z));

% 5.3 u2
% kvphi & kvphi: the rate of change of Phi. 
% Considerably affects the trajectory precision for SIN case in Y-direction.
kvphi = 20;      % 2: nose dive, 5: ended at -13, 10: phi ~ 0, CPE: 0.13939
kpphi = kvphi * 25;
phic_ddot = 0;  % accroding to the guide
phic_dot = 0;   % accroding to the guide

u2 = Ixx * (phic_ddot + kvphi * (phic_dot - phi_dot) + kpphi * (phic - phi));
    

% To test: trajhandle.m
%  Line Trajectory: error 0.08 - 0.15
%  Sine wave trajectory: 0.10 - 0.20

% FILL IN YOUR CODE HERE

end

