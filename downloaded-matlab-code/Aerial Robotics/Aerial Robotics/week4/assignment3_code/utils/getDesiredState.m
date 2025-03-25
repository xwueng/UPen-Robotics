function desired_state = getDesiredState(t, waypoints, d0, traj_time, coefx, coefy, coefz)

    desired_state.pos = zeros(3,1);
    desired_state.vel = zeros(3,1);
    desired_state.acc = zeros(3,1);
    desired_state.yaw = 0;
    desired_state.yawdot = 0;

    if size(waypoints,1) > 0

        if(t > traj_time(end))
            t = traj_time(end);
        end
        % 1. t_index points to the 1st Si (w0 to wi travel time) greater than t
        % e.g. t=7, t_index = 4 (for traj_time 10.3923)
        % 2. t then changes to (7 - 6.9282)= 0.0718
        % 3. scale = 0.0718/3.4641
    
        t_index = find(traj_time >= t,1);
    
        if(t_index > 1)
            t = t - traj_time(t_index-1);
        end
        if(t == 0)
            desired_state.pos = waypoints(:,1);
        else
    
            scale = t/d0(t_index-1);
            % scale = (t-traj_time(t_index))/d0(t_index-1);
           
            t0 = flip(polyT(8,0,scale))';
            t1 = flip(polyT(8,1,scale));
            t2 = flip(polyT(8,2,scale));
            velscale = (t1.*(1/d0(t_index-1)))';
            accscale = (t2.*(1/d0(t_index-1)^2))';
    
            % ++
            % index = [(t_index-1)*8+1:t_index*8];
            index = t_index-1;

            coefsegt=[coefx(:,index)'; coefy(:,index)'; coefz(:,index)'];
            
            % coefxsegt = coefx(:,index)';
            % coefysegt = coefy(:,index)';
            % coefzsegt = coefz(:,index)';
            % 
            desired_state.pos = coefsegt*t0;
            desired_state.vel = coefsegt*velscale;
            desired_state.acc = coefsegt*accscale;

            % desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
            % desired_state.pos = [coefxsegt*t0; coefysegt*t0; coefzsegt*t0];
            % desired_state.vel = [coefxsegt*velscale; coefysegt*velscale; coefzsegt*velscale];
            % desired_state.acc = [coefxsegt*accscale; coefysegt*accscale; coefzsegt*accscale];
        end 
    end % end size(t,1) >0
        desired_state;
end % end of getDesiredStae function