function myPose = particleLocalization(ranges, scanAngles, map, param)

    % Parameters for particle filter
    N = 50;  % Number of particles
    % sigma_motion = [0.01; 0.01; 0.01];  % Motion noise (x, y, theta)
    sigma_x = 0.2;  % For movement in x-direction
    sigma_y = 0.2;  % For movement in y-direction
    sigma_theta = 0.1;  % For rotational noise

    sigma_motion = diag([sigma_x^2, sigma_y^2, sigma_theta^2]);

    % Map parameters
    resol = param.resol;   % Grid resolution (cells per meter)
    origin = param.origin; % Map origin (in grid coordinates)

    % Initialize particle poses: [x; y; theta] for each particle
    K = size(ranges, 2);  % Number of time steps (columns in ranges)
    myPose = zeros(3, K);  % Result: 3-by-K for the estimated pose over time
    particles = repmat(param.init_pose, 1, N);  % Initialize all particles at the initial pose

    % Initial weights for the particles
    weights = ones(1, N) / N;

    % Set the initial pose (j = 1) to the given initial pose
    myPose(:, 1) = param.init_pose;  % Set the first pose to the initial pose provided
    % Skip LIDAR data for j = 1 (ignore ranges(:,1))

    particles = repmat(myPose(:,1), [1, N]);

    % Loop over each time step starting from j = 2
    for k = 2:K
        
        % Step 1: Propagate particles using motion model
        particles = propagateParticles(particles, sigma_motion);

        % Step 2: Measurement update - calculate weights based on LIDAR and map
        weights = measurementUpdate(particles, ranges(:, k), scanAngles, map, resol, origin);

        % Step 3: Resampling - resample particles based on weights
        particles = resampleParticles(particles, weights);

        % Step 4: Estimate the pose (mean of the particles)
        myPose(:, k) = mean(particles, 2);
        
        % % if mod(k, 100) == 0 
        %     close all;
        %     figure; 
        %     plot(myPose(1, 1:k), myPose(2, 1:k), 'b-', 'LineWidth', 2);  % Estimated pose in blue
        %     hold on;
        %     scatter(particles(1, :), particles(2, :), 'ro');  % Scatter plot of particles in red
        %     title("k=", k);
        %     hold off;
        % % end % plot
    end

  

end

%% Function to propagate particles using motion model
function particles = propagateParticles(particles, sigma_motion)
    N = size(particles, 2);
    
    % Generate random forward movement and rotational noise
    delta_d = randn(1, N) * 0.1;  % Forward motion noise
    delta_theta = randn(1, N) * 0.05;  % Rotation noise

    % Update particle positions based on the forward movement and orientation
    particles(1, :) = particles(1, :) + delta_d .* cos(particles(3, :));  % Update x based on theta
    particles(2, :) = particles(2, :) + delta_d .* -sin(particles(3, :));  % Update y based on theta
    particles(3, :) = particles(3, :) + delta_theta;  % Update theta (orientation)
end


%% Function to update weights based on LIDAR measurements
function weights = measurementUpdate(particles, ranges, scanAngles, map, resol, origin)
    N = size(particles, 2);
    weights = zeros(1, N);
    
    % Loop over each particle to compute the likelihood
    for i = 1:N
        % Get particle pose
        x_particle = particles(1, i);
        y_particle = particles(2, i);
        theta_particle = particles(3, i);

        % Transform LIDAR points from local frame to global frame
        [x_global, y_global] = lidarToGlobal(ranges, scanAngles, x_particle, y_particle, theta_particle, map, resol, origin);

        % Check occupancy in the map for the transformed points
        weights(i) = computeCorrelation(x_global, y_global, map);
    end
    
    % Normalize weights
    weights = weights / sum(weights);
end

%% Function to transform LIDAR points from local frame to global map frame
function [x_global, y_global] = lidarToGlobal(ranges, scanAngles, x_particle, y_particle, theta_particle, map, resol, origin)
    % Convert lidar measurements to global coordinates
    x_global = x_particle + ranges .* cos(scanAngles + theta_particle);
    y_global = y_particle + ranges .* -sin(scanAngles + theta_particle);
    
    % Convert global coordinates to map indices
    x_global = ceil(resol * x_global) + origin(1);
    y_global = ceil(resol * y_global) + origin(2);

    % Ensure indices are within the valid range of the map dimensions
    x_global = max(1, min(x_global, size(map, 2)));  % Clip x_global to be within [1, n], where n is map width
    y_global = max(1, min(y_global, size(map, 1)));  % Clip y_global to be within [1, m], where m is map height
end

%% Function to compute correlation between LIDAR points and the map
function score = computeCorrelation(x_global, y_global, map)
    % Check if LIDAR points hit an obstacle or free space in the map
  
    map_values = map(sub2ind(size(map), y_global, x_global));  % Get map values for the LIDAR points
    score = sum(map_values > 0.6);  % Example scoring: Count the hits for obstacles
end

%% Function to resample particles based on weights

function particles = resampleParticles(particles, weights)
    N = size(particles, 2);  % Number of particles

    % Step 1: Normalize weights
    weights = weights / sum(weights);

    % Step 2: Compute the cumulative sum of weights
    cumulative_weights = cumsum(weights);

    % Step 3: Generate a random number for the starting point
    r = rand() / N;

    % Step 4: Perform systematic resampling
    new_particles = zeros(size(particles));  % Placeholder for new particles
    j = 1;  % Index of current particle in cumulative_weights
    for i = 1:N
        u = r + (i-1) / N;  % Systematic selection
        while u > cumulative_weights(j)
            j = j + 1;
        end
        new_particles(:, i) = particles(:, j);  % Select particle j
    end

    % Output the resampled particles
    particles = new_particles;
end
