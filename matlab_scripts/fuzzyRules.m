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

function [Kp,Ki,Kd] = fuzzyRules(Kp0, Ki0, Kd0, e, e_dot)
    
    D = 0.5;

    E = float(mod(e,360));
    E = E / 360.00; % meant to normalize yaw error to between [-1,1]
    E_c = e_dot/6.25; % meant to normalize change in yaw error to between [-1,1]
    
    Kp_fis = fuzzyKpFIS();
    delta_Kp = evalfis([E,E_c], Kp_fis);

    Ki_fis = fuzzyKiFIS();
    delta_Ki = evalfis([E,E_c], Ki_fis);

    Kd_fis = fuzzyKdFIS();
    delta_Kd = evalfis([E,E_c], Kd_fis);
    
    Kp = Kp0 + D * delta_Kp;
    Ki = Ki0 + D * delta_Ki;
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


function Ki_fis = fuzzyKiFIS()
    Ki_fis = mamfis('Name', 'KiController');
    
    Ki_fis = addInput(Ki_fis, [-1 1], 'Name', 'E');
    Ki_fis = addInput(Ki_fis, [-1 1], 'Name', 'Ec');
    
    Ki_fis = addOutput(Ki_fis, [0 1], 'Name', 'Ki');
    
    names = {'NB', 'NM', 'NS', 'ZO', 'PS', 'PM', 'PB'};
    ranges = [-1 -0.3 -0.2 0 0.2 0.3 1];
    
    for i = 1:length(names)
        if i == 1
            Ki_fis = addMF(Ki_fis, 'E', 'trapmf', [ranges(i) ranges(i) ranges(i+1) ranges(i+2)], 'Name', names{i});
            Ki_fis = addMF(Ki_fis, 'Ec', 'trapmf', [ranges(i) ranges(i) ranges(i+1) ranges(i+2)], 'Name', names{i});
        elseif i == length(names)
            Ki_fis = addMF(Ki_fis, 'E', 'trapmf', [ranges(i-1) ranges(i) ranges(i) ranges(i)], 'Name', names{i});
            Ki_fis = addMF(Ki_fis, 'Ec', 'trapmf', [ranges(i-1) ranges(i) ranges(i) ranges(i)], 'Name', names{i});
        else
            Ki_fis = addMF(Ki_fis, 'E', 'trimf', [ranges(i-1) ranges(i) ranges(i+1)], 'Name', names{i});
            Ki_fis = addMF(Ki_fis, 'Ec', 'trimf', [ranges(i-1) ranges(i) ranges(i+1)], 'Name', names{i});
        end
    end    
    
    for i = 1:length(names)
        a = max(0, i - 2) * 0.15;
        b = (i - 1) * 0.1;
        c = min(1,i) * 0.1;
        sorted_points = sort([a,b,c]);

        Ki_fis = addMF(Ki_fis, 'Ki', 'trimf', sorted_points, 'Name', names{i});
    end
    
    rules = [
        1 1 1 1 1;  % NB E, NB Ec -> NB Ki (strong negative response)
        1 2 1 1 1;  % NB E, NM Ec -> NB Ki (strong negative response)
        2 1 2 1 1;  % NM E, NB Ec -> NM Ki (slightly negative response)
        3 3 4 1 1;  % NS E, NS Ec -> ZO Ki (neutral response)
        4 4 4 1 1;  % ZO E, ZO Ec -> ZO Ki (neutral response)
        5 5 6 1 1;  % PS E, PS Ec -> PM Ki (positive response)
        6 6 7 1 1;  % PM E, PM Ec -> PB Ki (strong positive response)
        7 7 7 1 1;  % PB E, PB Ec -> PB Ki (strong positive response)
    ];

    Ki_fis = addRule(Ki_fis, rules);
end



function Kd_fis = fuzzyKdFIS()
    Kd_fis = mamfis('Name', 'KdController');
    
    Kd_fis = addInput(Kd_fis, [-1 1], 'Name', 'E');
    Kd_fis = addInput(Kd_fis, [-1 1], 'Name', 'Ec');
    
    Kd_fis = addOutput(Kd_fis, [0 1], 'Name', 'Kd');
    
    names = {'NB', 'NM', 'NS', 'ZO', 'PS', 'PM', 'PB'};
    ranges = [-1 -0.6 -0.3 0 0.3 0.6 1];
    
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
        1 1 3 1 1;  % NB E, NB Ec -> NS Kd (slightly negative differential action)
        1 2 2 1 1;  % NB E, NM Ec -> NM Kd (moderate low differential action)
        1 3 1 1 1;  % NB E, NS Ec -> NB Kd (strongly reduce differential action)
        3 3 4 1 1;  % NS E, NS Ec -> ZO Kd (neutral Kd)
        4 4 5 1 1;  % ZO E, ZO Ec -> PS Kd (positive differential action)
        5 5 6 1 1;  % PS E, PS Ec -> PM Kd (moderate positive differential action)
        6 6 7 1 1;  % PM E, PM Ec -> PB Kd (strong positive differential action)
        7 7 7 1 1;  % PB E, PB Ec -> PB Kd (strong positive differential action)
    ];

    Kd_fis = addRule(Kd_fis, rules);
end
