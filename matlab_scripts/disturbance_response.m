
function disturbance_response(yaw_tf, input_signal_type, disturbance_type, Kp, Kd)
    
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
        case "line"
            u = zeros(1,length(t));
    end

    disturbance = zeros(size(t));
    disturbance_idx = round(length(t)/2);

    % establish disturbance type
    switch disturbance_type
        case "sin"
            disturbance = sin(0.1 * t);
        case "instant"
            disturbance(disturbance_idx) = pi/2;
    end

    yaw = zeros(size(t));

    % simulate system with PID controller
    for k = 2:length(t)
        Ks = Kp + Kd * s;
        y = lsim(feedback(yaw_tf*Ks,1),u + disturbance,t);
        yaw(k) = y(k) + disturbance(k);
    end

    hold on
    plot(t, u + disturbance)
    plot(t, yaw)

    yaw = zeros(size(t));

    % simulate system with Fuzzy PID logic
    for k = 2:length(t)
        error = u(k) - yaw(k-1);
        d_error = (error - prev_error) / dt;
        [Kp, Kd] = fuzzyRules(Kp, Kd, error, d_error);
        Ks = Kp + Kd * s;
        y = lsim(feedback(yaw_tf*Ks,1),u + disturbance,t);
        yaw(k) = y(k) + disturbance(k);
        prev_error = error;
    end

    plot(t, yaw)
    xlabel("t(s)")
    ylabel("yaw (rad)")
    legend("Input w/ Disturbance", "PID", "Fuzzy PID")
    hold off

    % saveas(gcf, 'generated_plots/dist_response_plots.png')
end