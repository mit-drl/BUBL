%{
    Fuzzy Rules for BUBL PID tuning
    Author: Guillermo D. Mendoza

    function fuzzyRules:
        Inputs:
            - float: e  (error in desired yaw angle vs actual yaw angle)
            - float: e_dot (change in error of desired yaw angle vs actual yaw angle)

        Output:
            - float: Kp (Proportional Gain)
            - float: Ki (Integral Gain)
            - float: Kd (Derivative Gain)

    function fuzzyKpFIS():
        Inputs:
            - float: E (normalized error)
            - float: E_c (normalized change in error)
        Output:
            - float: Kp (fuzzified proportional gain [0,1] range in value)

    function fuzzyKiFIS():
        Inputs:
            - float: E (normalized error)
            - float: E_c (normalized change in error)
        Output:
            - float: Ki (fuzzified integral gain [0,1] range in value)

    function fuzzyKd FIS():
        Inputs:
            - float: E (normalized error)
            - float: E_c (normalized change in error)
        Output:
            - float: Kd (fuzzified derivative gain [0,1] range in value)


%}

function [Kp,Kd] = fuzzyRules(Kp0, Kd0, e, e_dot)
    
    D = 0.5;

    E = double(mod(e,360));
    E = E / 360; % meant to normalize yaw error to between [-1,1]
    % established to be ~2 revolutions/s ~= 12.56 rad/s
    E_c = e_dot/720; % meant to normalize change in yaw error to between [-1,1]
    
    Kp_fis = fuzzyKpFIS();
    delta_Kp = evalfis(Kp_fis, [E,E_c]);

    Kd_fis = fuzzyKdFIS();
    delta_Kd = evalfis(Kd_fis, [E,E_c]);
    
    Kp = Kp0 + D * delta_Kp;
    Kd = Kd0 + D * delta_Kd;

end

function Kp_fis = fuzzyKpFIS()
    Kp_fis = mamfis('Name', 'KpController');
    
    Kp_fis = addInput(Kp_fis, [-1 1], 'Name', 'E');
    Kp_fis = addInput(Kp_fis, [-1 1], 'Name', 'Ec');
    
    Kp_fis = addOutput(Kp_fis, [0 1], 'Name', 'Kp');
    
    names = {'NB', 'NM', 'NS', 'ZO', 'PS', 'PM', 'PB'};
    ranges = [-1 -0.5 -0.2 0 0.2 0.5 1]; 
        
    for i = 1:length(names)
        if i == 1
            Kp_fis = addMF(Kp_fis, 'E', 'trapmf', [ranges(i) ranges(i) ranges(i+1) ranges(i+2)], 'Name', names{i});
            Kp_fis = addMF(Kp_fis, 'Ec', 'trapmf', [ranges(i) ranges(i) ranges(i+1) ranges(i+2)], 'Name', names{i});
        elseif i == length(names)
            Kp_fis = addMF(Kp_fis, 'E', 'trapmf', [ranges(i-1) ranges(i) ranges(i) ranges(i)], 'Name', names{i});
            Kp_fis = addMF(Kp_fis, 'Ec', 'trapmf', [ranges(i-1) ranges(i) ranges(i) ranges(i)], 'Name', names{i});
        else
            Kp_fis = addMF(Kp_fis, 'E', 'trimf', [ranges(i-1) ranges(i) ranges(i+1)], 'Name', names{i});
            Kp_fis = addMF(Kp_fis, 'Ec', 'trimf', [ranges(i-1) ranges(i) ranges(i+1)], 'Name', names{i});
        end
    end
    
    for i = 1:length(names)
        a = max(0, (i - 2) * 0.15);
        b = (i - 1) * 0.15;
        c = min(1, i) * 0.15;

        sorted_points = sort([a, b, c]);

        Kp_fis = addMF(Kp_fis, 'Kp', 'trimf', sorted_points, 'Name', names{i});
    end
    
    rules = [
        1 1 1 1 1; % If E is NB and Ec is NB, then Kp is NB
        1 2 1 1 1; % If E is NB and Ec is NM, then Kp is NB
        1 3 2 1 1; % If E is NB and Ec is NS, then Kp is NM
        1 4 4 1 1; % If E is NB and Ec is ZO, then Kp is ZO
        1 5 5 1 1; % If E is NB and Ec is PS, then Kp is PS
        1 6 6 1 1; % If E is NB and Ec is PM, then Kp is PM
        1 7 7 1 1; % If E is NB and Ec is PB, then Kp is PB

        2 1 1 1 1; % If E is NM and Ec is NB, then Kp is NB
        2 2 2 1 1; % If E is NM and Ec is NM, then Kp is NM
        2 3 2 1 1; % If E is NM and Ec is NS, then Kp is NM
        2 4 4 1 1; % If E is NM and Ec is ZO, then Kp is ZO
        2 5 4 1 1; % If E is NM and Ec is PS, then Kp is ZO
        2 6 4 1 1; % If E is NM and Ec is PM, then Kp is ZO
        2 7 5 1 1; % If E is NM and Ec is PB, then Kp is PS

        3 1 2 1 1; % If E is NS and Ec is NB, then Kp is NM
        3 2 2 1 1; % If E is NS and Ec is NM, then Kp is NM
        3 3 3 1 1; % If E is NS and Ec is NS, then Kp is NS
        3 4 5 1 1; % If E is NS and Ec is ZO, then Kp is PS
        3 5 5 1 1; % If E is NS and Ec is PS, then Kp is PS
        3 6 5 1 1; % If E is NS and Ec is PM, then Kp is PS
        3 7 6 1 1; % If E is NS and Ec is PB, then Kp is PM

        4 1 2 1 1; % If E is ZO and Ec is NB, then Kp is NM
        4 2 2 1 1; % If E is ZO and Ec is NM, then Kp is NM
        4 3 3 1 1; % If E is ZO and Ec is NS, then Kp is NS
        4 4 5 1 1; % If E is ZO and Ec is ZO, then Kp is PS
        4 5 5 1 1; % If E is ZO and Ec is PS, then Kp is PS
        4 6 6 1 1; % If E is ZO and Ec is PM, then Kp is PM
        4 7 6 1 1; % If E is ZO and Ec is PB, then Kp is PM

        5 1 2 1 1; % If E is PS and Ec is NB, then Kp is NM
        5 2 3 1 1; % If E is PS and Ec is NM, then Kp is NS
        5 3 3 1 1; % If E is PS and Ec is NS, then Kp is NS
        5 4 6 1 1; % If E is PS and Ec is ZO, then Kp is PM
        5 5 6 1 1; % If E is PS and Ec is PS, then Kp is PM
        5 6 6 1 1; % If E is PS and Ec is PM, then Kp is PM
        5 7 7 1 1; % If E is PS and Ec is PB, then Kp is PB

        6 1 3 1 1; % If E is PM and Ec is NB, then Kp is NS
        6 2 3 1 1; % If E is PM and Ec is NM, then Kp is NS
        6 3 4 1 1; % If E is PM and Ec is NS, then Kp is ZO
        6 4 6 1 1; % If E is PM and Ec is ZO, then Kp is PM
        6 5 6 1 1; % If E is PM and Ec is PS, then Kp is PM
        6 6 7 1 1; % If E is PM and Ec is PM, then Kp is PB
        6 7 7 1 1; % If E is PM and Ec is PB, then Kp is PB

        7 1 4 1 1; % If E is PB and Ec is NB, then Kp is ZO
        7 2 4 1 1; % If E is PB and Ec is NM, then Kp is ZO
        7 3 5 1 1; % If E is PB and Ec is NS, then Kp is PS
        7 4 6 1 1; % If E is PB and Ec is ZO, then Kp is PM
        7 5 7 1 1; % If E is PB and Ec is PS, then Kp is PB
        7 6 7 1 1; % If E is PB and Ec is PM, then Kp is PB
        7 7 7 1 1; % If E is PB and Ec is PB, then Kp is PB
    ];

    Kp_fis = addRule(Kp_fis, rules);
end

function Kd_fis = fuzzyKdFIS()
    Kd_fis = mamfis('Name', 'KpController');
    
    Kd_fis = addInput(Kd_fis, [-1 1], 'Name', 'E');
    Kd_fis = addInput(Kd_fis, [-1 1], 'Name', 'Ec');
    
    Kd_fis = addOutput(Kd_fis, [0 1], 'Name', 'Kd');
    
    names = {'NB', 'NM', 'PS', 'ZO', 'NS', 'PM', 'PB'};
    ranges = [-1 -0.1 -0.05 0 0.05 0.1 1];
    
    for i = 1:length(names)
        if i == 1
            Kd_fis = addMF(Kd_fis, 'E', 'trapmf', [ranges(i) ranges(i) ranges(i+1) ranges(i+2)], 'Name', names{i});
            Kd_fis = addMF(Kd_fis, 'Ec', 'trapmf', [ranges(i) ranges(i) ranges(i+1) ranges(i+2)], 'Name', names{i});
        elseif i == length(names)
            Kd_fis = addMF(Kd_fis, 'E', 'trapmf', [ranges(i-1) ranges(i) ranges(i) ranges(i)], 'Name', names{i});
            Kd_fis = addMF(Kd_fis, 'Ec', 'trapmf', [ranges(i-1) ranges(i) ranges(i) ranges(i)], 'Name', names{i});
        else
            Kd_fis = addMF(Kd_fis, 'E', 'trimf', [ranges(i-1) ranges(i) ranges(i+1)], 'Name', names{i});
            Kd_fis = addMF(Kd_fis, 'Ec', 'trimf', [ranges(i-1) ranges(i) ranges(i+1)], 'Name', names{i});
        end
    end
    
    for i = 1:length(names)
        a = max(0, i - 2) * 0.05;
        b = (i - 1) * 0.05;
        c = min(1,i) * 0.05;
        sorted_points = sort([a,b,c]);

        Kd_fis = addMF(Kd_fis, 'Kd', 'trimf', sorted_points, 'Name', names{i});
    end
    
    rules = [
        1, 1, 1, 1, 1; % If E is NB and Ec is NB, then Kd is NB
        1, 2, 1, 1, 1; % If E is NB and Ec is NM, then Kd is NB
        1, 3, 2, 1, 1; % If E is NB and Ec is NS, then Kd is NM
        1, 4, 2, 1, 1; % If E is NB and Ec is ZO, then Kd is NM
        1, 5, 3, 1, 1; % If E is NB and Ec is PS, then Kd is NS
        1, 6, 3, 1, 1; % If E is NB and Ec is PM, then Kd is NS
        1, 7, 4, 1, 1; % If E is NB and Ec is PB, then Kd is ZO

        2, 1, 1, 1, 1; % If E is NM and Ec is NB, then Kd is NB
        2, 2, 2, 1, 1; % If E is NM and Ec is NM, then Kd is NM
        2, 3, 2, 1, 1; % If E is NM and Ec is NS, then Kd is NM
        2, 4, 3, 1, 1; % If E is NM and Ec is ZO, then Kd is NS
        2, 5, 3, 1, 1; % If E is NM and Ec is PS, then Kd is NS
        2, 6, 4, 1, 1; % If E is NM and Ec is PM, then Kd is ZO
        2, 7, 4, 1, 1; % If E is NM and Ec is PB, then Kd is ZO
    
        3, 1, 2, 1, 1; % If E is NS and Ec is NB, then Kd is NM
        3, 2, 2, 1, 1; % If E is NS and Ec is NM, then Kd is NM
        3, 3, 3, 1, 1; % If E is NS and Ec is NS, then Kd is NS
        3, 4, 3, 1, 1; % If E is NS and Ec is ZO, then Kd is NS
        3, 5, 4, 1, 1; % If E is NS and Ec is PS, then Kd is ZO
        3, 6, 4, 1, 1; % If E is NS and Ec is PM, then Kd is ZO
        3, 7, 5, 1, 1; % If E is NS and Ec is PB, then Kd is PS
    
        4, 1, 2, 1, 1; % If E is ZO and Ec is NB, then Kd is NM
        4, 2, 3, 1, 1; % If E is ZO and Ec is NM, then Kd is NS
        4, 3, 3, 1, 1; % If E is ZO and Ec is NS, then Kd is NS
        4, 4, 4, 1, 1; % If E is ZO and Ec is ZO, then Kd is ZO
        4, 5, 4, 1, 1; % If E is ZO and Ec is PS, then Kd is ZO
        4, 6, 5, 1, 1; % If E is ZO and Ec is PM, then Kd is PS
        4, 7, 5, 1, 1; % If E is ZO and Ec is PB, then Kd is PS
    
        5, 1, 3, 1, 1; % If E is PS and Ec is NB, then Kd is NS
        5, 2, 3, 1, 1; % If E is PS and Ec is NM, then Kd is NS
        5, 3, 4, 1, 1; % If E is PS and Ec is NS, then Kd is ZO
        5, 4, 4, 1, 1; % If E is PS and Ec is ZO, then Kd is ZO
        5, 5, 5, 1, 1; % If E is PS and Ec is PS, then Kd is PS
        5, 6, 5, 1, 1; % If E is PS and Ec is PM, then Kd is PS
        5, 7, 6, 1, 1; % If E is PS and Ec is PB, then Kd is PM
    
        6, 1, 3, 1, 1; % If E is PM and Ec is NB, then Kd is NS
        6, 2, 4, 1, 1; % If E is PM and Ec is NM, then Kd is ZO
        6, 3, 4, 1, 1; % If E is PM and Ec is NS, then Kd is ZO
        6, 4, 5, 1, 1; % If E is PM and Ec is ZO, then Kd is PS
        6, 5, 5, 1, 1; % If E is PM and Ec is PS, then Kd is PS
        6, 6, 6, 1, 1; % If E is PM and Ec is PM, then Kd is PM
        6, 7, 6, 1, 1; % If E is PM and Ec is PB, then Kd is PM
    
        7, 1, 4, 1, 1; % If E is PB and Ec is NB, then Kd is ZO
        7, 2, 4, 1, 1; % If E is PB and Ec is NM, then Kd is ZO
        7, 3, 5, 1, 1; % If E is PB and Ec is NS, then Kd is PS
        7, 4, 5, 1, 1; % If E is PB and Ec is ZO, then Kd is PS
        7, 5, 6, 1, 1; % If E is PB and Ec is PS, then Kd is PM
        7, 6, 6, 1, 1; % If E is PB and Ec is PM, then Kd is PM
        7, 7, 7, 1, 1; % If E is PB and Ec is PB, then Kd is PB
    ];

    % Add rules to the FIS
    Kd_fis = addRule(Kd_fis, rules);
end
