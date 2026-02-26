function R = expr_PCM(w, theta)
    % w: rotation axis (3x1 unit vector)
    % theta: rotation angle (scalar)
    % Returns: R (3x3 rotation matrix)

    w = w(:);
    w = w/norm(w);
    w_hat = hat_PCM(w);

    R = eye(3) + w_hat * sin(theta) + (w_hat)^2 * (1 - cos(theta));
end