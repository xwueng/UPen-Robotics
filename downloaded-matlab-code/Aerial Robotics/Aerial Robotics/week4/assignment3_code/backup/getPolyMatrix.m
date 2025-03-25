function [deriv_table] = getPolyMatrix(x, ncoef, nd)
    % calculate 0 to nth deriviatives of X 
    % constructs a matrix with X's derivatives and selectors
    % inputs:
    % x: the variable of polynomial equations
    % ncoef: number of coefs in first order eq
    % nd: the nth derivatives, 1: velocity, 2: acc, 3: jerk, 4: snap
    % output:
    % derivative table containing equations for 0-nth derivatives
    ncol = ncoef;
    nrow = 2 * (nd+1);

    % syms x;
    syms deriv_table [nrow, ncol];

    for i = 1:ncol
       X(i) = x^(ncol-i);
    end
    for i = 1:nd+1
       Xdn = diff(X, i-1);
      
       deriv_table(2*i - 1, :)= 0;
       deriv_table(2*i - 1, ncol-i+1) = Xdn(ncol-i+1);

       deriv_table(2*i,1:end) = Xdn;
  
    end % end of the for loop of derivative 

end

% deriv_table =
% 
% [     0,      0,     0,   0, 0, 1]
% [   x^5,    x^4,   x^3, x^2, x, 1]
% [     0,      0,     0,   0, 1, 0]
% [ 5*x^4,  4*x^3, 3*x^2, 2*x, 1, 0]
% [     0,      0,     0,   2, 0, 0]
% [20*x^3, 12*x^2,   6*x,   2, 0, 0]