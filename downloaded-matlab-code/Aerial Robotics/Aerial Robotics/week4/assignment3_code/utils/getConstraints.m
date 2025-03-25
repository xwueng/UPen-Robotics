function [constraints] = getConstraints(nrow, waypoints, wpdindex )
    constraints = zeros(nrow, 1);
    ncol = size(waypoints, 2);
    for i = 1:ncol-1
        constraints(i:i+1) = waypoints(wpdindex, i:i+1)';
    end % loop i
end % function getConstraints