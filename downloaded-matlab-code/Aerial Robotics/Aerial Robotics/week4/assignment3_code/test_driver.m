
syms x;
ncoef = 8
nseg = 4;
% nth derivative
nd = 4; 
waypoints = ...
    [ 0     1     2     3     4;
      0     1     0    -1     0;
      0     1     2     1     0];
wpdindex = 1; % for x
wpsindex = 1; % 1 for segment 1
xval = 1;
wpdindex = 1;
S = [0    3.4641    6.9282   10.3923   13.8564];
traj_time = S;
% T/d0 travel time from segment start to seg end
T = [3.4641    3.4641    3.4641    3.4641];
d0 = T

[master_eqtable, deriv_tables, master_coefs, cnstr_table] = ...
    getMasterTables(x, ncoef, nd, waypoints, wpdindex, traj_time, d0)

% eval(master_eqtable*master_coefs==cnstr_table(1));

% [coefs, deriv_table, deriv_values, constraints] = ...
%     getCoefs(x, ncoef, nd, waypoints0, wpdindex, wpsindex, xval)
% 
% coefx = zeros(ncoef, nseg);
% coefy = zeros(ncoef, nseg);
% coefz = zeros(ncoef, nseg);
% 
% for i = 1:3  % loop for x,y,z
%     for j = 1:4 
%         fprintf("***dimension %d   segment %d ***\n", i, j);
%             [coefs, deriv_table, deriv_values, constraints] = ...
%         getCoefs(x, ncoef, nd, waypoints0, i, j, xval)
%         switch i
%             case 1 
%                 coefx(:, j) = coefs;
%             case 2 
%                 coefy(:, j) = coefs;  
%             case 3 
%                 coefz(:, j) = coefs;
%         end
% 
%     end %loop for waypoint segements
% end
% 
%  fprintf("***coefx, y, z, constraints***\n");
% coefx
% coefy
% coefz
% constraints
%% 



% 
% ans = 
% 
%   struct with fields:
% 
%     c1: 0
%     c2: 0
%     c3: 0
%     c4: 0
%     c5: 35
%     c6: -84
%     c7: 70
%     c8: -20
