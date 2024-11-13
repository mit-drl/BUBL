%{
    Yaw Transfer Function Derivation for BUBL
    Author: Guillermo D. Mendoza

    The derivation of the yaw transfer function undergoes the
    simplifications and assumptions explained in main. The only further
    simplifcation is the idea that yaw is isolated. Simulations will use
    the idea that yaw is tested in isolation to its pitch, roll, x,y, and z
    counterparts. This means that the pitch and roll will be assumed to be
    0 as well as their respective derivatives.

    This code uses matlab's symbolic tool to solve for the transfer
    function of yaw by applying linear algebra and previous knowns about
    how the positional inputs and output vectors coincide with 

    Inputs:
        - float: m  (mass of robot in kg)
        - float: R (radius of robot in m)
        - float: C_d (Reynolds number for a spherical body)
        - int: rho_w (density of water in kg/m^3)
        - float: mu (fluid viscosity in Pa)
        - int: a (degree of actuators positioning)
        - float: alpha (moment arm for the fluid jets in m)
        - float: m_a (added mass of fluid interaction)
        - float: I_aa (moment of intertia of robot)

    Output:
        - tf: yaw_tf (transfer function of yaw with respect to given
        parameters)

%}

function yaw_tf = yaw_transfer_func(m,m_a,rho_w,C_d,R, I_aa, mu, alpha, a)

    syms v_x v_y v_z % define symbolic v for velocity to find D(V) matrix

    % initialize Mass/Intertial and Damping Matrix
    M = [(m+m_a).* eye(3) ,zeros(3); zeros(3), eye(3) .* I_aa];
    D = [(-1/2 * rho_w * C_d * pi * R^2) * [v_x, 0, 0; 0, v_y, 0; 0,v_z, 0] * eye(3), zeros(3);
    zeros(3), (-8*pi*R^3*mu)*eye(3)];

    syms phi theta psi_var
    syms x_dot y_dot z_dot phi_dot theta_dot psi_dot

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
    v_vector = [R, zeros(3); zeros(3), J] * eta_dot;

    syms x_d_dot y_d_dot z_d_dot phi_d_dot theta_d_dot psi_d_dot

    v_dot_vector = [x_d_dot; y_d_dot; z_d_dot; phi_d_dot; theta_d_dot; psi_d_dot];

    eqn = M * v_dot_vector + D * v_vector;
    eqn = subs(eqn, {v_x, v_y, v_z}, {v_vector(1), v_vector(2), v_vector(3)});
    % substitute all velocities into expression to solve for yaw's velocity
    eqn = subs(eqn, {phi, theta, phi_dot, theta_dot, phi_d_dot, theta_d_dot}, {0,0,0,0,0,0});

    yaw_eqn = eqn(6);

    sympref('FloatingPointOutput',true);

    syms s
    
    % convert into frequency domain using laplace transform
    psi_d_dot_laplace = s^2;   
    psi_dot_laplace = s;    

    % replace psi (yaw) with respective laplace transformation
    yaw_eqn_laplace = subs(yaw_eqn, {psi_d_dot, psi_dot}, {psi_d_dot_laplace, psi_dot_laplace});

    % numerator from input forces
    num = alpha * cos(a);
    % denominator is the laplace transform of the yaw based expression
    denom = sym2poly(yaw_eqn_laplace);

    clear s;

    % create transfer function for yaw
    tf('s')
    yaw_tf = tf(num,denom);

    clear s;

end
