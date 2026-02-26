function T = expt_PCM(xi, theta)
    % xi: 6x1 twist vector [v; w]
    % theta: rotation angle (scalar)
    % Returns: T (4x4 transformation matrix)
    
    v = xi(1:3);
    w = xi(4:6);
    
    v = v(:);
    w = w(:);
    
    % Get rotation matrix using Rodrigues' formula
    R = expr_PCM(w, theta);
    
    % Calculate translation component
    w_hat = hat_PCM(w);
    p = (eye(3)*theta + (1-cos(theta))*w_hat + (theta-sin(theta))*w_hat^2) * v;
    
    % Construct 4x4 transformation matrix
    T = [R, p;
         0, 0, 0, 1];
end