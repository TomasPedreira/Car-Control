function [uout] = controllerMovie(xin)
    persistent  k ref1 ref2;

    if isempty(k)
        % Array Size
        k = 1;

        carRef = carReferences();
        ref1 = carRef(1:2, :);
        ref2 = carRef(3:4, :);
    end

    % 1st Car
    error_norm1 = norm(ref1(:,k) - xin(1:2));
    ref_aux1 = ref1(:,k) - xin(1:2);
    error_orientation1 = wrapToPi(atan2(ref_aux1(2), ref_aux1(1)) - xin(3));

    v1 = min(5, error_norm1 * 0.5);

    alpha1 = atan2(2 * 0.2 * sin(error_orientation1), 0.4);
    % alpha1 = min(max(alpha1, deg2rad(-30)), deg2rad(30)); % Limit steering angle

    v1r = v1 + alpha1*0.2/2;
    v1l = v1 - alpha1*0.2/2;

    % 2nd Car
    error_norm2 = norm(ref2(:,k) - xin(4:5));  % Distance to Reference
    ref_aux2 = ref2(:,k) - xin(4:5); % Position Error
    error_orientation2 = wrapToPi(atan2(ref_aux2(2), ref_aux2(1)) - xin(6));  % Orientation Error

    v2 = min(5, error_norm2 * 0.5);

    alpha2 = atan2(2 * 0.2 * sin(error_orientation2), 0.4);
    % alpha2 = min(max(alpha2, deg2rad(-30)), deg2rad(30)); % Limit steering angle

    v2r = v2 + alpha2*0.2/2;
    v2l = v2 - alpha2*0.2/2;

    uout = [v1r;v1l;v2r;v2l];

    k = k + 1;
end