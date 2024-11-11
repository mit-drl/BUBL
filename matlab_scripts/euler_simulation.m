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

function yaw_out = euler_simulation(dt,tf)
    
    time_final = 10;
    num_step = floor(tf/dt);
    time_span = linspace(0,tf,num_step);

    yaw_0 = [-pi/4; pi/2; 0; 0];
    yaw_out = zeros(4,num_step);
    yaw_0(:,1) = yaw_0;

    for i=1:num_steps-1
        d_yaw = dynamics(time_span(i), yaw_out(:,i));
        yaw_out(3:4,i+1) = yaw_out(3:4,i) + d_yaw(3:4) * dt;
        yaw_out(1:2,i+1) = yaw_out(1:2,i) + yaw_out(3:4,i+1)*dt;

    end
end