%{
    Frequency Response Simulation
    Author: Guillermo D. Mendoza

    
    Inputs:
        - tf: yaw_tf (transfer function of yaw with respect to given
        parameters)
        - float: dt (change in time)
        - float: Kp0 (Proportional Gain)
        - float: Ki0 (Integral Gain)
        - float: Kd0 (Derivative Gain)

    Outputs:
        - plot: freq_plot (plot displaying the magnitude and phase of each
        response to a a certain inputted frequency
    
%}

function freq_plot = freq_response(yaw_tf, dt, Kp0, Ki0, Kd0)
    frequencies = logspace(-1,2,50);
    magnitude = zeros(size(frequencies));
    phase = zeros(size(frequencies));

    previous_error = 0;

    t = 0:dt:10;

    for i = 1:length(frequencies)
        omega = frequencies(i);
        
        u = sin(omega * t);

        yaw = zeros(size(t));

        for k = 2:length(t)
            error = u(k) - yaw(k-1);
            d_error = (error - previous_error) / dt;
            [Kp, Ki, Kd] = fuzzyRules(Kp0, Ki0, Kd0, error, d_error);
            clear s;
            s = tf('s');
            Ks = Kp + Ki / s + Kd * s;

            yaw(k) = lsim(feedback(yaw_tf*Ks,1), u, t);
            
            Kp0 = Kp;
            Ki0 = Ki;
            Kd0 = Kd;

            previous_error = error;
        end

        yaw_ss = yaw(end-100:end);
        magnitude(i) = max(yaw_ss) / max(u);
        phase(i) = angle(hilbert(yaw_ss));
    end

    freq_plot = figure; 
    subplot(2,1,1);
    semilogx(frequencies, 20 * log10(magnitude));
    xlabel('Frequency (rad/s)');
    ylabel('Magntidude (db)');
    title('Frequnecy Response with Fuzzy PID Controller');

    subplot(2,1,2);
    semilogx(frequencies, phase * 180/pi);
    xlabel('Frequency (rad/s)');
    ylabel('Phase (degrees)');

end