%{
    Yaw Simulation for BUBL
    Author: Guillermo D. Mendoza

    
%}

function yaw_tf = yaw_transfer_func(M,D_0)

    s = tf('s');
    H = (s .* eye(6) + M\D_0)^(-1) / M;

    [num, den] = tfdata(H(6,6), 'v');
    yaw_tf = tf(num,den);

end