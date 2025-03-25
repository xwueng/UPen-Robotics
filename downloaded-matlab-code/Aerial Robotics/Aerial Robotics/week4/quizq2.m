% ---Q2 ------
r30 = pi*30/180;
r45 = pi*45/180;
t = [sin(r30) * cos(r45), sin(r30) * sin(r45), cos(r30)]'

b3 = [0; 0; 1];
za3psi = r45; % yaw, z, a3
spsi = sin(za3psi);
cpsi = cos(za3psi);

syms phi theta

% Rotation Matrix rrot
rrot = [cpsi * cos(theta) - sin(phi) * spsi * sin(theta), -cos(phi) * spsi, cpsi * sin(theta) + cos(theta) * sin(phi) * spsi; ...
        cos(theta) * spsi + cpsi * sin(phi) * sin(theta),  cos(phi) * cpsi, spsi * sin(theta) - cos(theta) * sin(phi) * cpsi; ...
        -cos(phi) * sin(theta), sin(phi), cos(phi) * cos(theta)] 
eqn = rrot * b3 == t/norm(t);
S = solve(eqn);
theta = S.theta(1,1)
phi = S.phi(1,1);
eval(rrot);
% desired rotation matrix
sphi = sin(phi);
cphi = cos(phi);

stheta = sin(theta);
ctheta = cos(theta);


rdes = [cpsi * ctheta - sphi * spsi * stheta, -cphi * spsi, cpsi * stheta + ctheta * sphi * spsi; ...
        ctheta * spsi + cpsi * sphi * stheta,  cphi * cpsi, spsi * stheta - ctheta * sphi * cpsi; ...
        -cphi * stheta, sphi, cphi * ctheta];
eval(rdes)

% ans =
% 
%     0.6124   -0.7071    0.3536
%     0.6124    0.7071    0.3536
%    -0.5000         0    0.8660
