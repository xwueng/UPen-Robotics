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

% persistent waypoints0 traj_time d0
persistent waypoints0 traj_time d0 coefx coefy coefz
syms x
ncoef = 8;
nd = 6;

if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;

    % -- student's code ---
    [coefx, coefy, coefz] = get3DCoefs(x, ncoef, nd, waypoints0, d0);
else
    % populate d0, traj_time/S, and waypoints0 
    % if this section is called first
    if size(d0,1) == 0 
        d = waypoints(:,2:end) - waypoints(:,1:end-1);
        d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
        traj_time = [0, cumsum(d0)];
        waypoints0 = waypoints;
    end

    desired_state = getDesiredState(t, waypoints0, d0, traj_time, coefx, coefy, coefz);
end % else (i.e. nargn <= 2)
%


%% Fill in your code here

% -- my code --
desired_state = getDesiredState(t, waypoints0, d0, traj_time, coefx, coefy, coefz);
% +++ end

end %traj_generator end


