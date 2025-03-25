function [coefx, coefy, coefz] = get3DCoefs(x, ncoef, nd, waypoints, T)
    % get3DCoefs implements Euler-Lagrange equation to find the minimum 
    % nth deriviative's trajectory, it solves and returns coefficients for 
    % x, y, z
    % 
    % inputs:
    %    x: the equation variable
    %    ncoef: the number of coeffs required
    %    nd: nth derivivative required
    %    waypoints: for constraints
    %    T: each segament's travel time
    % outputs:
    %    coefx [ncoef, 1]
    %    coefy [ncoef, 1]
    %    coefx [ncoef, 1]

    nrow = ncoef;
    ncol = size(waypoints,2) - 1;
    coefx = zeros(nrow, ncol);
    coefy = zeros(nrow, ncol);
    coefz = zeros(nrow, ncol);
    
    for i = 1:ncol
        
        coefx(:,i) = getCoefs(x, nd, waypoints(1, i:i+1), T(i));
        coefy(:,i) = getCoefs(x, nd, waypoints(2, i:i+1), T(i));
        coefz(:,i) = getCoefs(x, nd, waypoints(3, i:i+1), T(i));
    end % i loop
    coefx;
    coefy;
    coefz;
end