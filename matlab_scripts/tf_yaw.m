
% This transfer function calculations work only for Yaw as assumptions only
% applicable for Yaw scenario

% Original dynamics equation: M*v_dot + C(v)v+ D(v)v + g(n) = tau
% where M is the intertia matrix
% C is the coriolis matrix
% D represents hydrodynamic drag
% g represents gravitational and buoyancy forces
% tau is the control forces and torques

% Define Variables:
m = 0.1200; % mass of robot in kg
R = 0.032; % radius of robot in m
rho_w = 997; % density of water in kg/m^3
C_d = 0.5; % approx val (Reynolds number for spherical body)
mu = 10^(-3); % fluid viscosity in Pa
a = 20; % in degrees
alpha = 0.024; % in m

m_a = 2/3 * pi * R^3 * rho_w; % added mass of fluid interaction
I_aa = 2/5 * m * R^2; % moment of intertia of robot

% Define Matrices
syms v v_dot % define symbolic v for velocity to find D(V) matrix
M = [(m+m_a).* eye(3) ,zeros(3); zeros(3), eye(3) .* I_aa];
D = [(-1/2 * rho_w * C_d * pi * R^2 * v) * eye(3), zeros(3);
    zeros(3), (-8*pi*R^3*mu)*eye(3)]*v;

D_linear = sym(zeros(size(D)));
% Loop through each element of the matrix to compute the Jacobian
for i = 1:size(D, 1)
    for j = 1:size(D, 2)
        D_linear(i, j) = diff(D(i, j), v);  % Take the derivative with respect to v
    end
end
% Define an equilibrium point for v
v_e = 0.01;  
% Evaluate the Jacobian at the equilibrium point
D0 = subs(D_linear, v, v_e);
% Convert from symbolic to numeric matrix
D_0 = double(D0);

% we can neglect the coriolis force as "Coriolis forces are 
% caused by the earth's rotation and the speed of the moving 
% object. The speed of the SUR-II robot is less than 0.3 m/s, 
% and most of the factors in the Coriolis matrix are related 
% to the robot's velocity, so Coriolis forces can be ignored 
% for these low-speed cases" (Hydrodynamic Analysis of the Spherical
% Underwater Robot SUR-II).

% In real situations, the restoring force exists. However, 
% it has no influence on the motion of surge, sway, heave and yaw
%(Hydrodynamic Analysis of the Spherical Underwater Robot SUR-II).
% so we choose to neglect g(n)

% this defines our equation as M*v_dot + D(v)*v = tau
% create transfer function
s = tf('s');
H = (s .* eye(6) + inv(M)*D_0)^(-1) * inv(M);

for i = 1:size(H, 1)  
    for j = 1:size(H, 2)  
        H_ij = H(i, j);  
        if isproper(H_ij)  
            % Extract numerator and denominator
            [num, den] = tfdata(H_ij, 'v');  % Get numerator and denominator as vectors
            
            % Convert to symbolic for better formatting
            syms s_sym;  % Define symbolic variable for display
            num_sym = poly2sym(num, s_sym);  % Convert numerator to symbolic
            if double(num_sym) == 0
                continue
            end
            den_sym = poly2sym(den, s_sym);  % Convert denominator to symbolic
            
            % Print the transfer function equation
            fprintf('H_{%d%d}(s) = %s / %s\n', i, j, char(num_sym), char(den_sym));
        else
            fprintf('H_{%d%d}(s) is not a proper transfer function.\n', i, j);
        end
    end
end

% we state the nu matrix to be defined as [x,y,z,phi,theta,psi] such that 
% phi,theta,psi are the roll, pitch, and yaw angles, respectively
% this means we are interested in the relation between the input of yaw
% and output of pitch, so we select the H(6,6) transfer function 
H_s = H(6,6);
[num, den] = tfdata(H_s, 'v');
sys = tf(num,den)

Kp = 1;
Kd = 1;
Ki = 1;

Ke2c = Kp + Kd * s + Ki / s;

G_s = feedback(1/(1+sys*Ke2c),1)

% Main Fuzzy PID Control Function
function [Kp, Ki, Kd] = fuzzyPID(error, error_dot)
    % Normalize error and error_dot to the range [-1, 1]
    error_norm = normalizeValue(error, -1, 1);
    error_dot_norm = normalizeValue(error_dot, -1, 1);
    
    % Calculate each gain based on fuzzy logic
    Kp = fuzzyPIDLogic('Kp', error_norm, error_dot_norm);
    Ki = fuzzyPIDLogic('Ki', error_norm, error_dot_norm);
    Kd = fuzzyPIDLogic('Kd', error_norm, error_dot_norm);
end

% Function to calculate the specific gain value based on fuzzy rules
function K = fuzzyPIDLogic(gainType, error, error_dot)
    % Set initial value and scaling factor for each gain
    scaling_factor = 0.1;  % Tuning parameter for adjustment
    
    % Define thresholds for error and error_dot (example values, adjust as needed)
    error_small = 0.1;
    error_medium = 0.5;
    error_large = 1.0;
    error_dot_small = 0.1;
    error_dot_medium = 0.5;
    error_dot_large = 1.0;
    
    % Initialize `m_k` (membership value) and `K_previous`
    m_k = 0;
    K_previous = 1;  % Assume a default previous gain
    
    % Define rules based on gain type
    switch gainType
        case 'Kp'
            if error > error_medium && error_dot > error_medium
                m_k = max(min(error / error_large, error_dot / error_large), 0);
                K_change = scaling_factor * m_k;  % Increase Kp
            elseif error <= error_medium && error_dot <= error_medium
                m_k = max(min(error / error_medium, error_dot / error_medium), 0);
                K_change = -scaling_factor * m_k;  % Decrease Kp
            else
                K_change = 0;  % Neutral change
            end
            
        case 'Ki'
            if error < error_small && error_dot < error_small
                m_k = max(min(error / error_small, error_dot / error_small), 0);
                K_change = scaling_factor * m_k;  % Increase Ki
            elseif error >= error_medium || error_dot >= error_medium
                m_k = max(min(error / error_medium, error_dot / error_medium), 0);
                K_change = -scaling_factor * m_k;  % Decrease Ki
            else
                K_change = 0;  % Neutral change
            end

        case 'Kd'
            if abs(error_dot) < error_dot_small
                m_k = max(min(error_dot / error_dot_small, 1), 0);
                K_change = -scaling_factor * m_k;  % Decrease Kd (less damping)
            elseif abs(error_dot) > error_dot_large
                m_k = max(min(error_dot / error_dot_large, 1), 0);
                K_change = scaling_factor * m_k;  % Increase Kd (more damping)
            else
                K_change = 0;  % Neutral change
            end
    end
    
    % Update the gain with the calculated change
    K = K_previous + K_change;
end

% Helper function to normalize a value to a target range
function normalized = normalizeValue(value, minRange, maxRange)
    % Normalize `value` based on an estimated range (tune this range as needed)
    minVal = -10;  % Example minimum value of error
    maxVal = 10;   % Example maximum value of error
    
    % Scale `value` to the new range
    normalized = ((value - minVal) / (maxVal - minVal)) * (maxRange - minRange) + minRange;
end


% Function to simulate system response using Euler's method
function [time, response] = euler_simulation(H, Kp, Ki, Kd, dt, total_time)
    % Initial conditions
    response = zeros(1, total_time / dt);
    time = 0:dt:total_time;

    % Initialize error and integrated error
    prev_error = 0;
    integral_error = 0;
    u = 0;  % initial control input

    for i = 1:length(time)
        % Calculate error (e.g., step response target of 1)
        error = 1 - response(i);

        % Update integral and derivative of error
        integral_error = integral_error + error * dt;
        error_dot = (error - prev_error) / dt;

        % Update PID gains using fuzzy logic
        [Kp, Ki, Kd] = fuzzy_pid_control(error, error_dot);

        % Compute control input with PID controller
        u = Kp * error + Ki * integral_error + Kd * error_dot;

        % System response update: Euler step for state variable
        response(i + 1) = response(i) + dt * (-H * response(i) + u);

        % Update error for next step
        prev_error = error;
    end
end
