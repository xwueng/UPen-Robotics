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
a_max = 2;
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


% -- 2. Linearized Differential Equations
y_ddot = -gravity * phi;
z_ddot = 1;  % ?? tune
phi_ddot = 1; % ?? tune


% --- 3. Track Trajectory: Define position and velocity errors
errp = des_state.pos - state.pos;
errv = des_state.vel - state.vel;

%       3.1 Convergence Equation
% kp = 320;  % ?? tune
% kv = 30;  % ?? tune
% rddotc = des_state.acc + kp * errp + kv * errv;

% if des_state.acc == [a_max; 0] | ...
%    des_state.acc == [0; 0]  | ...
%    des_state.acc == [-a_max; 0] 
%     % --- 4. Hover Controller Equations
%     %  desired roll = 0;
%     kvz = 30;  % ?? tune
%     kpz = 320;  % ?? tune
%     kvy = 30;  % ?? tune
%     kpy = 320;
%     y0 = 0;
%     z0 = 0;
%     phi0 = 0;
%     phides = phi0;
%     phic_ddot = 0;  % accroding to the guide
%     phic_dot = 0;   % accroding to the guide
% 
%     kvphi = 30;      % ?? tune
% 
%     phides = phi0;      % ?? tune
% 
%     u1 = mass * [gravity - kvz * z_dot + kpz * (z0 - z)];
% 
%     u2 = Ixx * (phic_ddot + kvphi(phic_dot - phi_dot) + kpphi * (phic - phi));
% 
%     phic = -(1/gravity) * (kvy * (- y_dot)) + kpy * (y0 - y);
% 
% else

    % --- 5. Trajectory Controller Equations
    % 5.1 phic
    kvy = 8; % according to the guide, Y-position accuracy
    kpy = 0;  % according to the guide, Y-position accuracy
    phic = -(1/gravity) * (ydes_ddot + kvy * (ydes_dot - y_dot) + kpy * (ydes - y)); 


    % 5.2 u1
    kvz = 5;  % ?? tune Z-position
    kpz = kvz * 11;  % ?? tune  Z-position
    u1 = mass * (gravity + zdes_ddot + kvz * (zdes_dot - z_dot) + kpz * (zdes - z));
    
    % 5.3 u2
    % kvphi & kvphi: the rate of change of Phi. 
    % Considerably affects the trajectory precision for SIN case in Y-direction.
    kvphi = 5;      % ?? tune
    kpphi = kvphi * 11;
    phic_ddot = 0;  % accroding to the guide
    phic_dot = 0;   % accroding to the guide
    
    u2 = Ixx * (phic_ddot + kvphi * (phic_dot - phi_dot) + kpphi * (phic - phi));
    
% end;




% assignment 1 code
% kp = 320;
% kv = 30;
% 
% 
% delta = s_des - s;
% err = delta(1);
% err_dot = delta(2);
% 
% if s_des(1) == 0
%     z_ddot = 0;
% else
%     z_ddot = 1;
% end

% u = params.mass*(z_ddot + kp * err + kv * err_dot + params.gravity);
% -- end assignment 1 code



% phi: angle around a2, affects u2 moment
% r =(y, z)': position vector of the planar quadrotor in A, a3 direction (b3 thrust
% force vs. g)

% u1 = F1 + F2;
% u2 = parameters.armlength(F1 - F2) = Ixx * phi_ddot

% -- system model: (y_ddot; z_ddt; phi_ddot) = (0; -parameters.gravity; 0] +
% (-sin(phi)/parameter.mass 0; cos(phi)/parameter.mass 0; 0 1/Ixx) * (u1;
% u2)
% 
% -- Differential Equations
%  y_ddot = u1 * sin(phi)/m
%  z_ddot = (u1*cos(phi))/m - g
%  phi_ddot = u2/Ixx or Ixx * phi_ddt=u2

% -- Linearization
% y0, z0, phi0 = 0; u(1, 0) = parameters.mass * parameters.gravity; u(2, 0
% ) = 0
% 
% -- Differential Equations being linearized near phi = 0, sin(phi) = 0,
% cos(phi) = 1
%  y_ddot = -parameters.gravity * phi = -gphi
%  z_ddot = (u1*cos(phi) - m*g)/m = 
%           -parameters.gravity + u1/arameters.mass = 
%           -g + u1/m
%  phi_ddot = u2/Ixx 

% -- state variables 
%   r: state variable (y, z, or phi)
%   rc: commanded acceleration of that state, 
%   rddotc: a PD controller
%   errorp: position error = r'(t)  - r
%   errorv: velocity error = r_dot'(t) - r_dlot

% -- hover condition (the desired equation)
%    (rddt'(t) - rddtc) + kp * ep + kv * ev = 0
%    rddtc = rddt'(t) + kp * ep + kv * ev

% -- Derive input u1 and u2
%    (6): u1 = m*g + m*zddtc = m*{g + zddt'(t) + kv,z*(zdot'-zdot) + kp,z *
%    (z'(t) - z)
%    (7): u2 = Ixx * phiddtc = Ixx*(phiddot'(t) + kv,phi(phidot' - phidot) +
%    kp,z(phi'(t) - phi))
%    (8): phic = yddotc/g = -1/g *(yddot'(t) + kv,y * (yddot'(t) - ydot) + kp,v
%    * (y'(t) - y))

% -- hover controller 
%  roll is zero: 
%               r'(t) = r0 = (y0; z0)'
%               phi'(t) = phi0
%               rdot'(t) = rddot'(t) = 0 : no velocity change or
%               acceleration
%   u1 = m * [g - kvz*(z0-z]
%   u2 = Ixx * (phiddot'(t) = kvphi *(phidot'(t) - phi) + kpz*(phi'(t) -
%   phi)
%   phic = -/g * (kvy(-ydot) + kpy(y0-y))

%  Trajectory Controller
%  given/known: r'(t), rdot'(t), rddot'(t)
%  to derive u1, u2 using (6)-(8) in Derive input u1 and u2 section

% To test: trajhandle.m
%  Line Trajectory: error 0.08 - 0.15
%  Sine wave trajectory: 0.10 - 0.20

% FILL IN YOUR CODE HERE

end

