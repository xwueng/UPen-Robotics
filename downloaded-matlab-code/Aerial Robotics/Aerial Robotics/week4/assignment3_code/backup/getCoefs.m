
function [coefs] = getCoefs(x, nd, waypoints, xval)
    % getCoefs implements Euler-Lagrange equation to find the minimum 
    % nth deriviative's trajectory, it solves and returns the coefficients 
    % for trajectory 
    % inputs:
    %    x: the equation variable
    %    nd: nth derivivative required
    %    waypoints: for constraints
    %    xval: the value for the x
    % outputs:
    %    coefs [2*nd, 1]

    ncol = 2*nd;
    nrow = ncol;
    
    coefs = flip(sym('c', [nrow,1]));
    
    for i = 1:ncol
        X(i) = x^(ncol-i);
    end
  
    % pm: polynomial equation matrix
    pm = getPolyMatrix(X, nd);
    
    x = xval;
    pm = subs(pm);
    
    constraints = [waypoints(1,1); waypoints(1,2); zeros(ncol-2,1)];
    % matlab: X = A\B solves the symbolic system of linear 
    % equations in matrix form, A*X = B for X

    coefs = pm\constraints;
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



function [pm] = getPolyMatrix(X, nd)
% calculate 0 to nth deriviatives of X 
% constructs a matrix with X's derivatives and selectors
% inputs:
% X: the x's polynomial equation
% nd: the nth derivatives, 1: velocity, 2: acc, 3: jerk, 4: snap
    ncol = size(X,2);
    nrow = 2 * nd;

    syms pm [nrow, ncol];
    for i = 1:nd
       Xdn = diff(X, i-1);
      
       pm(2*i - 1, :)= 0;
       pm(2*i - 1, ncol-i+1) = Xdn(ncol-i+1);

       pm(2*i,1:end) = Xdn;
  
    end % end of derivative for loop

end

% pm =
% 
% [     0,      0,     0,   0, 0, 1]
% [   x^5,    x^4,   x^3, x^2, x, 1]
% [     0,      0,     0,   0, 1, 0]
% [ 5*x^4,  4*x^3, 3*x^2, 2*x, 1, 0]
% [     0,      0,     0,   2, 0, 0]
% [20*x^3, 12*x^2,   6*x,   2, 0, 0]
