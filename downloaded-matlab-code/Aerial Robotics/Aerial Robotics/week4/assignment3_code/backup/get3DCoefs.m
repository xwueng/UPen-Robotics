function [coefx, coefy, coefz] = get3DCoefs(x, nd, waypoints, T)
    % get3DCoefs implements Euler-Lagrange equation to find the minimum 
    % nth deriviative's trajectory, it solves and returns coefficients for 
    % x, y, z
    % 
    % inputs:
    %    x: the equation variable
    %    nd: nth derivivative required
    %    waypoints: for constraints
    %    T: an array of traveling times for each segament
    % outputs:
    %    coefx [2*nd, 1]
    %    coefy [2*nd, 1]
    %    coefx [2*nd, 1]

    nrow = 2*nd;
    ncol = size(waypoints,2) - 1;
    coefx = zeros(nrow, ncol);
    coefy = zeros(nrow, ncol);
    coefz = zeros(nrow, ncol);
    

    % T = d0
    for i = 1:ncol
        
        coefx(:,i) = getCoefs(x, nd, waypoints(1, i:i+1), T(i));
        coefy(:,i) = getCoefs(x, nd, waypoints(2, i:i+1), T(i));
        coefz(:,i) = getCoefs(x, nd, waypoints(3, i:i+1), T(i));
    end % i loop
    coefx;
    coefy;
    coefz;
end