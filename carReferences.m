%% Target Vehicle Reference
function carRef = carReferences()
    r = 2;          
    v_target = 0.1;
    omega = 2*pi/40;

    Ts = 0.005;
    total_time = 80;
    N = total_time / Ts;

    % Full Simulation Time Vector
    t = linspace(0, total_time, N+1);

    X_target = -5 + v_target * t;
    Y_target = zeros(size(t));

    % 1st Car Reference
    X1 = X_target + r * cos(omega * t);
    Y1 = Y_target + r * sin(omega * t);

    % 2nd Car Reference
    X2 = X_target + r * cos(omega * t + pi);
    Y2 = Y_target + r * sin(omega * t + pi);

    carRef = [X1; Y1; X2; Y2];
end