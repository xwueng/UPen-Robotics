
function u = controllerNoisyEnc(params, t, obs, th, dth)
   % This is the starter file for the week 6: 6.b
    % Provided params are
    % params.g: gravitational constant
    % params.mr: mass of the "rod"
    % params.ir: rotational inertia of the rod
    % params.d: distance of rod CoM from the wheel axis
    % params.r: wheel radius
    % params.traj(t): a desired trajectory in the “x” coordinate
    
    % Provided states are:
    % th: wheel angle (relative to body)
    % phi: body pitch
    % dth, dphi: time-derivatives of above

    % t: time
    
    % obs = [ay; az; gx] (same as last week)
    % obs[1:2]: ay, az: unit:g (m/(s^2) accel_sensor data
    % obs[3]: velocity: gx: gyro_sensor data

    % Now you only receive noisy measurements for theta, and must use your EKF from week 3 to filter the data and get an estimate of the state

    % New for 6b: you also have access to params.traj(t)
    
    % Input
  

    u=0;

 
    persistent pratio xratio xdesd phidesd kpp kdp kpx kdx ki ei tlastc;

    if isempty(tlastc)
        tlastc = 0;
        % suggestions Position: Kpx < 1;  Kdx ~= 0.1*Kpx
        % Balance: Kp < 50; Kd ~= 0.01*Kp; Ki < 0.1
        % position tuning params from 6.2a for 
        % PID balancing control of a MIP
        xdesd = 0;
        phidesd = 0; 
        xratio = 10;
        kpx = 0.5; % over
        % kdx = kpx / xratio;
        kdx = 0.02;
        pratio = 10;
        % kpp = 39.99; %overshoot
        % kpp = 30;  % overshoot
        kpp = 11.8;
        % kdp = kpp/pratio; % smooth but overshot
        % kdp = 0.005;
        kdp = 0.025;
        % ki = 0.06;
        % ei = 0;

    end

   % Define persistent variables at the beginning of your function
    persistent test_name printfreq plotfreq plotit timeHistory xdesHistory xHistory phidesHistory phiHistory phidesdHistory phidHistory;
    
    % Initialize the persistent variables on the first call
    if isempty(timeHistory)
        test_name = 'trajSin';
        printfreq = 200;
        plotfreq = printfreq * 2;
        plotit = true;
        timeHistory = [];
        xdesHistory = [];
        xHistory = [];
        phidesHistory = [];
        phiHistory = [];
        phidesdHistory = [];
        phidHistory = [];
    end

    persistent counter;
    if isempty(counter)
      counter = 1;
    else
      
     if mod(counter, printfreq) == 0
        
        fprintf('counter = %d  last: t = %d  x = %d   xdes = %d  phi = %d  phides = %d \n', ...
            counter, timeHistory(counter), xHistory(counter),  xdesHistory(counter), phiHistory(counter), phidesHistory(counter));
 
        if plotit && (mod(counter, plotfreq) == 0)
            close all;

            figure;  % Create a new figure window every 50,000 counts

            % Position Tracking
            subplot(3, 1, 1);
            plot(timeHistory, xdesHistory, 'r', timeHistory, xHistory, 'b');
            xlim auto;
            ylim auto;
            % xlim([0 0.1]);
            % plot(timeHistory, xdesHistory, 'r', 'DisplayName', 'x_{des}');
            % % plot(timeHistory, xdesHistory, 'r');
            % hold on;
            % % plot(timeHistory, xHistory, 'b');
            % plot(timeHistory, xHistory, 'b', 'DisplayName', 'x');
            % hold off;
            % legend();
            title([test_name ' \color{red}Position Ref \color{blue}Actual kpp = ' sprintf('%.3f', kpp)])
  
            subtitle(['kpp = ' sprintf('%.3f', kpp) ', kdp = ' sprintf('%.3f', kdp) ...
        ' kpx = ' sprintf('%.3f', kpx) ...
        ' kdx = ' sprintf('%.3f', kdx)]);


            xlabel('Time (s)');
            ylabel('x (m)');

            % Angle Tracking
            subplot(3, 1, 2);
            plot(timeHistory, phidesHistory, 'r', timeHistory, phiHistory, 'b');
            xlim auto;
            ylim auto;
            % plot(timeHistory, phidesHistory, 'r', 'DisplayName', '\phi_{des}');
            % hold on;
            % plot(timeHistory, phiHistory, 'b', 'DisplayName', '\phi');
            % hold off;
            % legend();
            title('Angle Tracking Red: Ref, Blue: Actual');
            xlabel('Time (s)');
            ylabel('\phi (rad)');

            % Angular Velocity Tracking
            subplot(3, 1, 3);
            plot(timeHistory, phidesdHistory, 'r', timeHistory, phidHistory, 'b');
            % xlim([0 0.005]);
            xlim auto;
            ylim auto;
            % plot(timeHistory, phidesdHistory, 'r', 'DisplayName', '\dot{\phi}_{des}');
            % hold on;
            % plot(timeHistory, phidHistory, 'b', 'DisplayName', '\dot{\phi}');
            % hold off;
            % legend();sa
            title('Angular Velocity Tracking Red: Ref, Blue: Actual');
            xlabel('Time (s)');
            ylabel('$\dot{\phi}$ (rad/s)', 'Interpreter','latex');

             % Force MATLAB to update the figure window
            drawnow;
            filename = ['6b' test_name '.png'];
            saveas(gcf, filename);
        end % plot
      end % counter mod
      counter = counter + 1;
    end

    r = params.r;
    dt = t - tlastc;
    xhat = EKFupdate(params, t, obs);
    phi = xhat(1);
    phid = xhat(2);

    % 1. eq
    x = r * (th + phi);
    xd = r * (dth + phid);
    xdes = params.traj(t);

    % 2. eq
    xerr = xdes - x;
    % 3. eq
    xderr = xdesd - xd;
    
    % 4. eq: outer loop 
    phides = kpx * xerr + kdx * xderr;
  
    % 5. eq
    phi_err = phides - phi;

     % 6. eq
    phid_err = phidesd - phid;
 
    
    % u: torque applied at the wheel
     % Purpose: The inner loop is responsible for 
    % balancing and quickly stabilizing the MIP’s angle.
    % smooth way to saturate upd​
    % when the angle error is large.
    % 7. eq inner loop

    % u = -(kpp * sin(phi_err) + kdp * phid_err);
    % u = -(kpp * phi_err + kdp * phid_err + ki * ei);
    u = (kpp * phi_err + kdp * phid_err);

    tlastc = t;
    % ei = ei + ki * ei;

    % --- % Collect data during each call --
    timeHistory = [timeHistory; t];
    xdesHistory = [xdesHistory; xdes];  % Desired position
    xHistory = [xHistory; x];  % Actual/estimated position
    phidesHistory = [phidesHistory; phides];  % Desired pitch angle
    phiHistory = [phiHistory; phi];  % Estimated pitch angle from EKFUpdate
    phidesdHistory = [phidesdHistory; phidesd];  % Desired angular velocity
    phidHistory = [phidHistory; phid];  % Estimated angular velocity from EKFUpdate
   
end

function xhats = EKFupdate(~, t, z)

    % Objective: to estimate the roll angle phi and its derivative phidot
    
    % xhats returns [phi; phidot] which can be considered as observed phi
    % because it's derived from observation z.
    
    % Reference: week 3's code but with a single predict-update stop

    % z = [ay; az; gx] with a* in units of g's, and gx in units of rad/s
    % You can borrow most of your week 3 solution, but this must only implement 
    % a single predict-update step of the EKF
    % Recall (from assignment 5b) that you can use persistent variables to 
    % create/update any additional state that you need.

    
    % -- modified module 4 code ---
    % you are provided a vector of timestamps (of length T), 
    % and a 3xT matrix of observations, z.
    % Input:
    % t: ?x1
    % z: 3*?
    % z[1:2,:]: ay, az: unit:g (m/(s^2) accel_sensor data
    % z[3,:]: velocity: gx: gyro_sensor data
    % Output:
    % xhats[2,T]: 2xT: 
    %     [phis, T];
    %      phidots, T]
    
    % define persistent variables/states
    persistent tlast Q rfactor R ay_init az_init phi_init phidot_init xhatk_1 Hk_1 hk_1 Pk_1 Ak_1
   

    if isempty(tlast) 
        % Q 2x2 (process noise convariance uncertainty in the system mode)
        Q = [500 0; 0 0.0001];
        
        % R (Measurement Noise Covariance): This matrix represents 
        % the uncertainty in the measurements. 
        % R 3x3
        rfactor = 0.009;
        R = diag([1, 1, 0.8])*rfactor;
        tlast = 0;
        Pk_1 = diag([250, 250]);
        Ak_1 = [1 t(1); 0 1];
        
        % Estimate initial phi 
        ay_init = 0.0114;
        az_init = 0.7966;
        phi_init = atan2(ay_init, az_init);
        
        % Estimate initial phidot
        phidot_init = 8.7725;
        
        %hatk_1 is a prediction : 2x1 column vector
        xhatk_1 = [phi_init; phidot_init];
        
        Hk_1 = [cos(phi_init), 0; ...
              -sin(phi_init), 0; ...
              0, 1];
        hk_1 = h(xhatk_1);
        invalid = any(isinf(Hk_1(:)) | isnan(Hk_1(:)));
        if invalid
            Hk_1s = mat2str(Hk_1);
            zs = mat2str(z);
            fprintf('ERROR Hk_1 = %s  t = %.3f,  z = %s', Hk_1s, t, zs);
            pause;
        end
    end


    k = 1;

    nobs = size(t, 1);
    % dt has time differences
    % dt = (t - tlast) * 1.8;
    dt = (t - tlast);
    
    
    % values for Pk_1
    % P: 2x2xT
    % Pk_1: 2x2
    % Pk_1 = diag([250, 250]);
    % P = zeros(2,2,nobs);
    
    % A: 2x2
    % Ak_1 = [1 t(1); 0 1];
    
    % xhat 2xT, xhat(i) = [phi; phidot]
    xhats = zeros(2,nobs);

    % xhat(:,i) must contain [phi, phidot] where phi is the roll angle in radian, 
    % and phidot is the angular velocity in rand/s.
    
    % Predict xhatk and Pk: xhatk 2x1 column vector  
    xhatk = Ak_1 * xhatk_1;

    % Pk 2x2;
    Pk = Ak_1 * Pk_1 * Ak_1' + Q;
    
    % Optimal Kalman Gain
    % Hk is 3x2 Jacobian matrix to linearize the measurment
    phi = xhatk(1);
    Hk =[cos(phi), 0; ...
        -sin(phi), 0; ...
        0, 1];

    % Kk 2x3
    Kk = Pk * Hk' / (Hk * Pk * Hk' + R);

    % Update xhat
    zk = z(:, k);
    % hk: 3x1
    hk = h(xhatk);
    % hk = hk_1 + Hk *(xhatk - xhatk_1);

    % xhatk 2x1
    xhatk = xhatk + Kk *(zk - hk);
    invalid = any(any(isinf(xhatk(:)) | isnan(xhatk(:))));
    if invalid
        xhatks = mat2str(xhatk);
        zs = mat2str(z);
        fprintf('ERROR xhatk = %s  t = %.3f,  z = %s \n', xhatks, t, zs);
        pause;
    end
    xhats(:,k) = xhatk;
    Pk = (eye(2) - Kk * Hk) * Pk;
    % P(:, :, k) = Pk;

    % Update variables for next loop
    xhatk_1 = xhatk;
    Pk_1 = Pk;
    hk_1 = hk;
    Hk_1 = Hk;
    tlast = t;
 
end % EKFupdate

  % Function to calculate the expected measurement
function y = h(x)
    % input x: [phi, phidot]
    % output y: 3x1 column vector
    y = [sin(x(1)); cos(x(1)); x(2)];

end
