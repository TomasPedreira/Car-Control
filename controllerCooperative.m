function [uout] = controllerCooperative(xin)
    persistent utraj k
    if isempty(k)
        k = 1;
        utraj = [0;0;0;0];
    end

    extraCarRef = -5 + 10 * 0.005 * k;

    ref1 = [extraCarRef+30; -1];
    ref2 = [extraCarRef+30; -1];
    
    error_norm1 = norm(ref1 - xin(1:2));
    ref_aux1 = ref1 - xin(1:2);
    error_orientation1 = wrapToPi(atan2(ref_aux1(2), ref_aux1(1)) - xin(3));

    if abs(error_orientation1) > deg2rad(120)
        v1_km1 = 0;
        v1 = 0;
    else
        v1_km1 = 0.5*(utraj(1)+utraj(2));
        v1 = min(2, error_norm1 * 0.5);
        v1 = min(v1, v1_km1 + 0.005);
    end

    alpha1 = 10*atan2(2 * 0.2 * sin(error_orientation1), 0.4);
    % alpha1 = min(max(alpha1, deg2rad(-30)), deg2rad(30));
    
    v1r = v1 + alpha1*0.2/2;
    v1l = v1 - alpha1*0.2/2;
    
    error_norm2 = norm(ref2 - xin(4:5));
    ref_aux2 = ref2 - xin(4:5);
    error_orientation2 = wrapToPi(atan2(ref_aux2(2), ref_aux2(1)) - xin(6));
    
    if abs(error_orientation2) > deg2rad(120)
        v2_km1 = 0;
        v2 = 0;
    else
        v2_km1 = 0.5*(utraj(3)+utraj(4));
        v2 = min(2, error_norm2 * 0.5);
        v2 = min(v2, v2_km1 + 0.005);
    end

    alpha2 = 10*atan2(2 * 0.2 * sin(error_orientation2), 0.4);
    % alpha2 = min(max(alpha2, deg2rad(-30)), deg2rad(30));
    
    v2r = v2 + alpha2*0.2/2;
    v2l = v2 - alpha2*0.2/2;
    
    uout = [v1r;v1l;v2r;v2l];
    utraj = uout;
    
    k = k + 1;
end