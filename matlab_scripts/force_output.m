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

dt = 0.01; 

% testing going towards negative yaw direction
yaw = [pi/256, pi/128, pi/64];

%% Yaw Dynamics Function Call
yaw_dyanmics(yaw, m,m_a,rho_w,C_d,R, I_aa, mu, B, dt)