% % ---Q2 ------
% r30 = pi*30/180;
% r45 = pi*45/180;
% t = [sin(r30) * cos(r45); sin(r30) * sin(r45); cos(r30)]
% 
% 
% b3 = [0; 0; 1]
% za3psi = r45; % yaw, z, a3
% 
% syms phi theta
% 
% spsi = sin(za3psi)
% cpsi = cos(za3psi)
% 
% 
% % Rotation Matrix rrot
% rrot = [cpsi * cos(theta) - sin(phi) * spsi * sin(theta), -cos(phi) * spsi, cpsi * sin(theta) + cos(theta) * sin(phi) * spsi; ...
%         cos(theta) * spsi + cpsi * sin(phi) * sin(theta),  cos(phi) * cpsi, spsi * sin(theta) - cos(theta) * sin(phi) * cpsi; ...
%         -cos(phi) * sin(theta), sin(phi), cos(phi) * cos(theta)] 
% eqn = rrot * b3 == t/norm(t)
% S = solve(eqn)
% 
% 
% sphi = sin(S.phi(1,1))
% cphi = cos(S.phi(1,1))
% 
% stheta = sin(S.theta(1,1))
% ctheta = cos(S.theta(1,1))
% 
% rrot = [cpsi * cos(theta) - sin(phi) * spsi * sin(theta), -cos(phi) * spsi, cpsi * sin(theta) + cos(theta) * sin(phi) * spsi; ...
%         cos(theta) * spsi + cpsi * sin(phi) * sin(theta),  cos(phi) * cpsi, spsi * sin(theta) - cos(theta) * sin(phi) * cpsi; ...
%         -cos(phi) * sin(theta), sin(phi), cos(phi) * cos(theta)] 


% --- Q3: error/delta between R' and Rdesired ----
R = [0.7244 0.1294 0.6771;
    0.6424 -0.483 -0.595;
    0.25 0.866 -0.433]
Rdes = [0 0 1; 1 0 0; 0 1 0]

R' * Rdes

% ans =
% 
%     0.6424    0.2500    0.7244
%    -0.4830    0.8660    0.1294
%    -0.5950   -0.4330    0.6771