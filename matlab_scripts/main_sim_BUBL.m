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

% Define Variables:

m = 0.1200; % mass of robot (kg)
R = 0.032; % radius of robot (m)
C_d = 0.5; % approximate value (Reynolds number for spherical body)

rho_w = 997; % density of water (kg/m^3)
mu = 10^(-3); % fluid viscosity (Pa)

a = 20; % degree selected to provide minimum sufficient dive thrust
alpha = 0.024; % moment arm for the fluid jets (m)

m_a = 2/3 * pi * R^3 * rho_w; % added mass of fluid interaction
I_aa = 2/5 * m * R^2; % moment of intertia of robot

v_e = 0.15; % Define an equilibrium point for v

syms v v_dot % define symbolic v for velocity to find D(V) matrix

M = [(m+m_a).* eye(3) ,zeros(3); zeros(3), eye(3) .* I_aa];
D = [(-1/2 * rho_w * C_d * pi * R^2 * v) * eye(3), zeros(3);
    zeros(3), (-8*pi*R^3*mu)*eye(3)]*v;

% Linearization of Matrix D

D_linear = sym(zeros(size(D)));

% Loop through each element of the matrix to compute the Jacobian
for i = 1:size(D, 1)
    for j = 1:size(D, 2)
        D_linear(i, j) = diff(D(i, j), v);  % Take derivative with respect to v
    end
end


D0 = subs(D_linear, v, v_e); % Evaluate the Jacobian at the equilibrium point
D_0 = double(D0); % Convert from symbolic to numeric matrix

% have to define torque vector (mapping input to output to define tf)
yaw_tf = yaw_transfer_func(M,D_0) % find yaw transfer function

s = tf('s');
Ks = 1 + 0.15 * s;

freq_response(yaw_tf, Ks)