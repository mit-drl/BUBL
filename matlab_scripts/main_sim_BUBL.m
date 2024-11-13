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

%% Define Folder Location for File Saving
plotFolder = 'generated_plots';

if ~exist(plotFolder, 'dir')
    mkdir(plotFolder);
end

%% Define Variables and Transfer Function for Yaw:

m = 0.1200; % mass of robot (kg)
R = 0.032; % radius of robot (m)
C_d = 0.5; % approximate value (Reynolds number for spherical body)

rho_w = 997; % density of water (kg/m^3)
mu = 10^(-3); % fluid viscosity (Pa)

a = 20 * pi/180; % degree selected to provide minimum sufficient dive thrust
alpha = 0.024; % moment arm for the fluid jets (m)

m_a = 2/3 * pi * R^3 * rho_w; % added mass of fluid interaction
I_aa = 2/5 * m * R^2; % moment of intertia of robot

yaw_tf = yaw_transfer_func(m,m_a,rho_w,C_d,R, I_aa, mu, alpha, a)

%% Define Initial Kp, Ki, Kd values

% Determined via step response
Kp0 = 16;
Ki0 = 9;
Kd0 = 0.25;

%% Finding Initial Kp, Ki, Kd values
s = tf('s');
Ks = Kp0 + Ki0 / s + Kd0 * s;

figure(1);
step(feedback(yaw_tf * Ks, 1))

hold off
figure(2);
hold on
margin(yaw_tf)
margin(Ks)
margin(yaw_tf * Ks)
hold off

%% Euler Step Simulation:

% Define initial conditions/state
dt = 0.001;
time_final = 10;
desired_yaw = 180;

euler_plots = euler_simulation(yaw_tf,dt,time_final, Kp0, Ki0, Kd0, desired_yaw);

% saves plot output to selected folder (can comment out if not needed)
euler_plot_file = fullfile(plotFolder, 'euler_model_plot.png');
saveas(euler_plots, euler_plot_file);

%% Frequency Response Simulation (Step Response Included):

% Define initial conditions/state
dt = 0.01;

freq_plot = freq_response(yaw_tf, dt, Kp0, Ki0, Kd0);

% saves plot output to selected folder (can comment out if not needed)
freq_response_plot_file = fullfile(plotFolder, 'freq_response_plots.png');
saveas(freq_plot, freq_response_plot_file);