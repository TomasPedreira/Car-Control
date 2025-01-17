function [uout] = controllerTracking(xin)
    persistent utraj k

    if isempty(utraj)
        % Array Size
        k = 1;
        utraj = [0;0];
    end
    
    ref = [5*sin(k*0.005/5);5*cos(k*0.005/10)];

    error_norm = norm(ref - xin(1:2));  % Distance to Reference
    ref_aux = ref - xin(1:2); % Position Error
    error_orientation = wrapToPi(atan2(ref_aux(2), ref_aux(1)) - xin(3));  % Orientation Error
    
    v_km1 = 0.5*(utraj(1)+utraj(2));
    v = min(5, error_norm * 0.5);
    v = min(v, v_km1 + 0.005);

    alpha = atan2(2 * 0.2 * sin(error_orientation), 0.4);
    % alpha = min(max(alpha, deg2rad(-30)), deg2rad(30)); % Limit steering angle

    vr = v + alpha*0.2/2;
    vl = v - alpha*0.2/2;

    uout = [vr;vl];
    utraj = uout;

    k = k + 1;
end
