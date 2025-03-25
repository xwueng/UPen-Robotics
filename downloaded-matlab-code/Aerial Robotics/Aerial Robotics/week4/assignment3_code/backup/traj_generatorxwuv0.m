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

if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
    nd = 4;
    [coefx, coefy, coefz] = get3DCoefs(x, nd, waypoints0, d0)
else
    if size(d0,1) == 0 
        d = waypoints(:,2:end) - waypoints(:,1:end-1);
        d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
        traj_time = [0, cumsum(d0)];
        waypoints0 = waypoints;
    end
    if(t > traj_time(end))
        t = traj_time(end);
    end
    % 1. t_index points to the 1st Si (w0 to wi travel time) greater than t
    % e.g. t=7, t_index = 4 (for traj_time 10.3923)
    % 2. t then changes to (7 - 6.9282)=-0.0718
    % 3. scale = (0.0718 - 10.3923)/3.4641
    % 4. ++ index 
    t_index = find(traj_time >= t,1);

    if(t_index > 1)
        t = t - traj_time(t_index-1);
    end
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
    else

        % scale = t/d0(t_index-1);
        scale = (t-traj_time(t_index))/d0(t_index-1);
        % ++
        t0 = flip(polyT(8,0,scale))';
        t1 = flip(polyT(8,1,scale));
        t2 = flip(polyT(8,2,scale));
        velscale = (t1.*(1/d0(t_index-1)))';
        accscale = (t2.*(1/d0(t_index-1)^2))';

        % ++
        % index = [(t_index-1)*8+1:t_index*8];
        index = t_index-1;
        
        coefxsegt = coefx(:,index)';
        coefysegt = coefy(:,index)';
        coefzsegt = coefz(:,index)';
        
        % desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
        desired_state.pos = [coefxsegt*t0; coefysegt*t0; coefzsegt*t0];
        desired_state.vel = [coefxsegt*velscale; coefysegt*velscale; coefzsegt*velscale];
        desired_state.acc = [coefxsegt*accscale; coefysegt*accscale; coefzsegt*accscale];

    end % t== 0 else end


end % else (i.e. nargn <= 2)
%


%% Fill in your code here

% -- my code --
desired_state = getDesiredState(t, waypoints0, d0, traj_time, coefx, coefy, coefz);
% +++ end

end %traj_generator end

function desired_state = getDesiredState(t, waypoints, d0, traj_time, coefx, coefy, coefz)

    desired_state.pos = zeros(3,1);
    desired_state.vel = zeros(3,1);
    desired_state.acc = zeros(3,1);
    desired_state.yaw = 0;
    desired_state.yawdot = 0;

    if size(t,1) > 0

        % if size(d0,1) == 0 
        %         d = waypoints(:,2:end) - waypoints(:,1:end-1);
        %         d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
        %         traj_time = [0, cumsum(d0)];
        %         waypoints0 = waypoints;
        % end
        if(t > traj_time(end))
            t = traj_time(end);
        end
        % 1. t_index points to the 1st Si (w0 to wi travel time) greater than t
        % e.g. t=7, t_index = 4 (for traj_time 10.3923)
        % 2. t then changes to (7 - 6.9282)=-0.0718
        % 3. scale = (0.0718 - 10.3923)/3.4641
        % 4. ++ index 
        t_index = find(traj_time >= t,1);
    
        if(t_index > 1)
            t = t - traj_time(t_index-1);
        end
        if(t == 0)
            desired_state.pos = waypoints(:,1);
        else
    
            % scale = t/d0(t_index-1);
            scale = (t-traj_time(t_index))/d0(t_index-1);
            % ++
            t0 = flip(polyT(8,0,scale))';
            t1 = flip(polyT(8,1,scale));
            t2 = flip(polyT(8,2,scale));
            velscale = (t1.*(1/d0(t_index-1)))';
            accscale = (t2.*(1/d0(t_index-1)^2))';
    
            % ++
            % index = [(t_index-1)*8+1:t_index*8];
            index = t_index-1;
            
            coefxsegt = coefx(:,index)';
            coefysegt = coefy(:,index)';
            coefzsegt = coefz(:,index)';
            
            % desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
            desired_state.pos = [coefxsegt*t0; coefysegt*t0; coefzsegt*t0];
            desired_state.vel = [coefxsegt*velscale; coefysegt*velscale; coefzsegt*velscale];
            desired_state.acc = [coefxsegt*accscale; coefysegt*accscale; coefzsegt*accscale];
        end 
    end % end size(t,1) >0
        desired_state;
end % end of getDesiredStae function
