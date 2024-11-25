
function step_response(yaw_tf, input_signal_type, Kp, Kd)
    
    s = tf('s');

    dt = 0.01; % time between imu measurements (estimated)
    t = 0:dt:10; % measured in seconds
    omega = log(10); % frequency
    prev_error = 0; % for calculating derivative of error

    % establish what type of input_signal
    switch input_signal_type
        case "square"
            u = square(omega * t);
        case "sin"
            u = sin(omega * t);
    end

    yaw = zeros(size(t));

    for k = 2:length(t)
        error = u(k) - yaw(k-1); % error between input signal and measured value
        Ks = Kp + Kd * s;
        y = lsim(feedback(yaw_tf*Ks,1),u,t);
        yaw(k) = y(k);
        prev_error = error;
    end

    hold on
    plot(t, u)
    plot(t, yaw)

    yaw = zeros(size(t));

    for k = 2:length(t)
        error = u(k) - yaw(k-1);
        d_error = (error - prev_error) / dt;
        [Kp, Kd] = fuzzyRules(Kp, Kd, error, d_error);
        Ks = Kp + Kd * s;
        y = lsim(feedback(yaw_tf*Ks,1), u, t);
        yaw(k) = y(k);
        prev_error = error;
    end

    plot(t, yaw)
    xlabel("t(s)")
    ylabel("yaw (rad)")
    legend("Input Signal", "PID", "Fuzzy PID")
    hold off

    % saveas(gcf, 'generated_plots/step_response_plots.png')
end