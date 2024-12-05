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

a_max = 1 * 2 * pi; % for 1 rev/s^2 (6.28 rad/s^2)
v_max = 2 * 2 * pi; % for 2 rev/s (~12.57 rad/s)

d_dot_yaw = a_max;
dot_yaw = v_max;

% pos a_max and v_max --> pos yaw

M = [(m+m_a).* eye(3) ,zeros(3); zeros(3), eye(3) .* I_aa];
    
% defining 0 translational movement
v_x = 0;
v_y = 0;
v_z = 0;

D = [(-1/2 * rho_w * C_d * pi * R^2) * [v_x, 0, 0; 0, v_y, 0; 0,v_z, 0] * eye(3), zeros(3);
zeros(3), (-8*pi*R^3*mu)*eye(3)]
  
% define state vectors
a_vector = zeros(6,1);
a_vector(6) = d_dot_yaw;

a_vector

phi = 0;
theta = 0;
psi_var = 0;

x_dot = 0;
y_dot = 0;
z_dot = 0;
phi_dot = 0;
theta_dot = 0;
psi_dot = dot_yaw;

% rotation matrix from the inertial frame to the body frame:
R = [cos(psi_var) * cos(theta), sin(psi_var)*cos(theta), -sin(theta);
    -sin(psi_var) * cos(phi) + cos(psi_var) * sin(theta)*sin(phi), cos(psi_var) * cos(phi) + sin(psi_var) * sin(theta) * sin(phi), sin(phi) * cos(theta);
    sin(psi_var) * sin(phi) + cos(psi_var) * sin(theta) * cos(phi), -cos(psi_var) * sin(phi) + sin(psi_var) * sin(theta) * cos(phi), cos(phi) * cos(theta)];

% matrix to transform rotational velocities from the inertial frame to the body frame
J = [1, 0, -sin(theta); 
    0, cos(phi), cos(theta) * sin(phi);
    0, -sin(phi), cos(theta) * cos(phi)];

eta_dot = [x_dot; y_dot; z_dot; phi_dot; theta_dot; psi_dot];

% find velocity vector in terms of respective velocities 
v_vector = [R, zeros(3); zeros(3), J] * eta_dot

tau_vector = M * a_vector + D * v_vector

[L,U,P] = lu(B);

y = L\(P*tau_vector);
Forces = U\y

%%

a = 30 * pi/180; % degree selected to provide minimum sufficient dive thrust
alpha = 0.024; % moment arm for the fluid jets (m)

B = [cos(a), -cos(a), cos(a), -cos(a);
     sin(a), sin(a), sin(a), sin(a);
     -alpha * sin(a), -alpha * sin(a), alpha * sin(a), alpha * sin(a);
     -alpha * cos(a), alpha * cos(a), alpha * cos(a), -alpha * cos(a)];

[L,U,P] = lu(B);

desired_thrust = 400; % in mN

tau_vector = [desired_thrust/1000;0.001;0;0];
y = L\(P * tau_vector);
Forces = U\y

%% From Ardunino Code:

theta = 30 * pi/180;
alpha = 0;

B = [cos(theta+alpha), -cos(theta-alpha), cos(theta+alpha), -cos(theta-alpha);
     -cos(theta+alpha), cos(theta-alpha), cos(theta+alpha), -cos(theta-alpha);
     sin(theta + alpha), sin(theta-alpha), sin(theta+alpha), sin(theta-alpha);
     -sin(theta+alpha), -sin(theta-alpha), sin(theta+alpha), sin(theta-alpha)];

det(B);

desired_thrust = 400; % in mN

[L,U,P] = lu(B);

tau_vector = [desired_thrust/1000;-1;0;0];
y = L\(P * tau_vector);
Forces = U\y

%% 
Forces = pinv(B) * tau_vector;