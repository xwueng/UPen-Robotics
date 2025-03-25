function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

%original code
%u = 0; 



% --- FILL IN YOUR CODE HERE ----
% Requirements:
% Hover: z = 0
% Step: Rise 1m in less than 1s, 0.9 meters in < 1s, overshoot < 5%

% 
kp = 320;
kv = 30;


delta = s_des - s;
err = delta(1);
err_dot = delta(2);

if s_des(1) == 0
    z_ddot = 0;
else
    z_ddot = 1;
end

u = params.mass*(z_ddot + kp * err + kv * err_dot + params.gravity);
   


% --- End of Your CODE  ----


end

