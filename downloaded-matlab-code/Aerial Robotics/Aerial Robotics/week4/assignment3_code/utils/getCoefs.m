
function [coefs, deriv_table, deriv_values, constraints] = getCoefs(x, ncoef, nd, waypoints, wpdindex, wpsindex, xval)
    % getCoefs implements Euler-Lagrange equation to find the minimum 
    % nth deriviative's trajectory, it solves and returns the coefficients 
    % for trajectory 
    % inputs:
    %    x: the equation variable
    %    ncoef: highest order of polynomials 
    %           if y=a+x^2, ncoef 2
    %    nd: nth derivivative required
    %    waypoints: for constraints
    %    wpdindex: waypoints dimension index: x:1, y:2, z:3 
    %    wpsindex: waypoints segment index, i.e. wp column
    %    xval: the value for the x
    % outputs:
    %    coefs [ncoef, 1]
    %    deriv_table
    %    deriv_values
    %    constraints

    ncol = ncoef;  
    nrow = 2*nd;   % *2 for each seg's start and end
    
    coefs = flip(sym('c', [nrow,1]));
    
    % for i = 1:ncol
    %     X(i) = x^(ncol-i);
    % end
    % 
    % pm: polynomial equation matrix
    % deriv_table = getPolyMatrix(X, nd);

   deriv_table = getPolyMatrix(x, ncoef, nd);

%     deriv_table =
% 
% [      0,       0,      0,      0,     0,   0, 0, 1]
% [    x^7,     x^6,    x^5,    x^4,   x^3, x^2, x, 1]
% [      0,       0,      0,      0,     0,   0, 1, 0]
% [  7*x^6,   6*x^5,  5*x^4,  4*x^3, 3*x^2, 2*x, 1, 0]
% [      0,       0,      0,      0,     0,   2, 0, 0]
% [ 42*x^5,  30*x^4, 20*x^3, 12*x^2,   6*x,   2, 0, 0]
% [      0,       0,      0,      0,     6,   0, 0, 0]
% [210*x^4, 120*x^3, 60*x^2,   24*x,     6,   0, 0, 0]
    
    x = xval;
    deriv_values = subs(deriv_table);
    
    constraints = zeros(ncol, 1);
    % constraints = [waypoints(1,1); waypoints(1,2); zeros(ncol-2,1)];
    constraints(wpsindex:wpsindex+1, 1) = waypoints(wpdindex, wpsindex:(wpsindex+1))';
    % matlab: X = A\B solves the symbolic system of linear 
    % equations in matrix form, A*X = B for X

    coefs = deriv_values\constraints;
    [coefs, deriv_table, deriv_values, constraints];
end % end of getCoefs

% coefs =
% 
% -20
%  70
% -84
%  35
%   0
%   0
%   0
%   0
% 
% 
% deriv_table =
% 
% [      0,       0,      0,      0,     0,   0, 0, 1]
% [    x^7,     x^6,    x^5,    x^4,   x^3, x^2, x, 1]
% [      0,       0,      0,      0,     0,   0, 1, 0]
% [  7*x^6,   6*x^5,  5*x^4,  4*x^3, 3*x^2, 2*x, 1, 0]
% [      0,       0,      0,      0,     0,   2, 0, 0]
% [ 42*x^5,  30*x^4, 20*x^3, 12*x^2,   6*x,   2, 0, 0]
% [      0,       0,      0,      0,     6,   0, 0, 0]
% [210*x^4, 120*x^3, 60*x^2,   24*x,     6,   0, 0, 0]
% 
% 
% deriv_values =
% 
% [  0,   0,  0,  0, 0, 0, 0, 1]
% [  1,   1,  1,  1, 1, 1, 1, 1]
% [  0,   0,  0,  0, 0, 0, 1, 0]
% [  7,   6,  5,  4, 3, 2, 1, 0]
% [  0,   0,  0,  0, 0, 2, 0, 0]
% [ 42,  30, 20, 12, 6, 2, 0, 0]
% [  0,   0,  0,  0, 6, 0, 0, 0]
% [210, 120, 60, 24, 6, 0, 0, 0]
% 
% 
% constraints =
% 
%      0
%      1
%      0
%      0
%      0
%      0
%      0
%      0