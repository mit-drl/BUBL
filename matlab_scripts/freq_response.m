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

function freq_response(Hs,Ks)

    figure(1);
    hold on;
    margin(Ks);
    margin(Hs);
    margin(Ks*Hs);
    hold off;
    
    figure(2);
    step(feedback(Ks*Hs,1),1);

end