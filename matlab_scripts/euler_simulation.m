%{
    Euler Step Simulation for BUBL
    Author: Guillermo D. Mendoza

    
%}

function yaw_out = euler_simulation(tf, dt, time_final, Kp0, Ki0, Kd0, desired_yaw)

    num_steps = floor(time_final / dt);
    time_span = linspace(0, time_final, num_steps);

    yaw_0 = [-pi/4; pi/2; 0; 0];
    yaw_out = zeros(4, num_steps);
    yaw_out(:, 1) = yaw_0;

    integral = 0;
    previous_error = 0;

    yaw_tf_discrete = c2d(tf, dt, 'zoh');

    for i = 1:num_steps - 1
        error = desired_yaw - yaw_out(1, i);
        d_error = (error - previous_error) / dt;

        [Kp, Ki, Kd] = fuzzyRules(Kp0, Ki0, Kd0, error, d_error);

        integral = integral + error * dt;
        control_signal = Kp * error + Ki * integral + Kd * d_error;

        d_yaw = dynamics(yaw_out(:, i), control_signal, yaw_tf_discrete); 
        yaw_out(3:4, i + 1) = yaw_out(3:4, i) + d_yaw(3:4) * dt; 
        yaw_out(1:2, i + 1) = yaw_out(1:2, i) + yaw_out(3:4, i + 1) * dt; 

        previous_error = error;
    end
end
