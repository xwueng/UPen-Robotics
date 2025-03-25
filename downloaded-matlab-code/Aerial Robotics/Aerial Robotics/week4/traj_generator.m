function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

persistent waypoints0 traj_time d0
if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
else
    if(t > traj_time(end))
        t = traj_time(end);
    end
    t_index = find(traj_time >= t,1);

    if(t_index > 1)
        t = t - traj_time(t_index-1);
    end
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
    else
        scale = t/d0(t_index-1);
        desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
    end
    desired_state.vel = zeros(3,1);
    desired_state.acc = zeros(3,1);
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end
%


%% Fill in your code here

% -- my code --
Si = traj_time;
Ti = d0; %or set to 15 sec

% -- temp --
t = 0.05;
S = [0, 3.4641, 6.9282, 10.3923, 13.8564];
T = [3.4641    3.4641    3.4641    3.4641];
% textbook: i = code i+1, 
% textbook: Si i: 1-4 = code S(i+1) i: 2 - 5, 
% S(1) = 0 and fro i = 2, ... n+1, S(i)=sum(Ti) for k= 2,.. n+1:
% pt(1) = a11 + a12*x ... + a18*x^7  S(1+1)=S(i+1) T(1=i)
% pt(2) = a21 + a22*x ... + a28*x^7  S(2+1)=S(i+1) T(2=i)
% pt(3) = a31 + a32*x.. + a38*x^7  S(3+1)=S(i+1) T(3=i)
% pt(4) = a41 + a42*x... + a48*x^7  S(4+1), T(4=i)
% -- temp end --

n = 4; % for n+1 = 5 waypoints
[nrow, ncol] = deal(n, 8);

% (18) minimum snap trajectory defined by 4/four of 7th order polynomial pi
% a(i,j): 4*8=32 coefficients to solve
syms a [nrow ncol];
A = zeros(32,1);
pt = zeros(nrow, 1);
for i = 1:nrow
    x = (t - S(i))/T(i);
    for j = 1:ncol
        pt(i) = pt(i) + a(i, j) * x^(j-1);
    end
end 
pt



%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot

% desired_state.pos = zeros(3,1);
% desired_state.vel = zeros(3,1);
% desired_state.acc = zeros(3,1);
% desired_state.yaw = 0;


% waypoints0 =
% 
%      0     1     2     3     4
%      0     1     0    -1     0
%      0     1     2     1     0
% 
% traj_time i.e. Si: 
% Si =
% 
%          0    3.4641    6.9282   10.3923   13.8564
% 
% d0 i.e. Ti:
% Ti =
% 
%     3.4641    3.4641    3.4641    3.4641
% 

% -- my code ends ---


end

