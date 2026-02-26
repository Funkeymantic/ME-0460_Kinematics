function T = extp_PCM(xi, theta)
    % Computes the matrix exponential of a prismatic (translational) twist
    % xi:    6x1 twist vector [v; 0] where v is the translation direction
    % theta: displacement along v (scalar)
    % Returns: T (4x4 homogeneous transformation matrix)
    %
    % For a prismatic joint (w = 0):
    %   e^(xi_hat * theta) = [ I,   v*theta ]
    %                        [ 0,      1    ]

    v = xi(1:3);
    v = v(:);

    T = [eye(3),  v * theta;
         0, 0, 0,     1   ];
end
