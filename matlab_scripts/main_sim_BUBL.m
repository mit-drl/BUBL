%{
    Yaw Simulation for BUBL
    Author: Guillermo D. Mendoza

    Dynamics Equation:
        Mv_dot + C(v)v + D(v)v + g(eta) = tau

        Where:
            - v is the velocity of the robot
            - M is the inertia matrix
            - D is the hydrodynamic drag
            - g is the gravitational and buoyancy forces (restoring forces)
            - tau is the control forces and torques
            - eta is the state vector of the positions of the robot in the
            intertial frame

    Assumptions:
        - Coriolis Force can be neglected due to slow velocity of robot
          (~0.3m/s)
        - Restoring force has not influence on the motion of surge, sway, heave
          and yaw (therefore we can neglect g(eta)

    From our assumptions the equation becomes:
        Mv_dot + D(v)v = tau
%}

%% Define Variables and Transfer Function for Yaw

m = 0.1200; % mass of robot (kg)
R = 0.032; % radius of robot (m)
C_d = 0.5; % approximate value (Reynolds number for spherical body)

rho_w = 997; % density of water (kg/m^3)
mu = 10^(-3); % fluid viscosity (Pa)

a = 20 * pi/180; % degree selected to provide minimum sufficient dive thrust
alpha = 0.024; % moment arm for the fluid jets (m)

% define the actuation matrix
B = [cos(a), -cos(a), cos(a), -cos(a);
     zeros(1,4);
     sin(a), sin(a), sin(a), sin(a);
     -alpha * sin(a), -alpha * sin(a), alpha * sin(a), alpha * sin(a);
     zeros(1,4);
     -alpha * cos(a), alpha * cos(a), alpha * cos(a), -alpha * cos(a)];

m_a = 2/3 * pi * R^3 * rho_w; % added mass of fluid interaction
I_aa = 2/5 * m * R^2; % moment of intertia of robot

yaw_tf = yaw_transfer_func(m,m_a,rho_w,C_d,R, I_aa, mu, B)

%% Define Initial Kp, Ki, Kd values

% Established by Pascal previously
Kp0 = 12;
Kd0 = 9;

%% Simulate Step Response of PID and Fuzzy PID:

input_type = "sin"; % define as square or sin
step_response(yaw_tf, input_type, Kp0, Kd0)

%% Simulate Disturbance Based Step Response:

input_type = "line"; % can define input_type as line
dist_type = "instant"; % define as sin or instant
disturbance_response(yaw_tf, input_type, dist_type, Kp0, Kd0)