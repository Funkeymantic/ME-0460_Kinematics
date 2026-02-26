function xi_hat = wedge_PCM(xi)
    % Returns the 4x4 matrix representation of a twist
    % xi: 6x1 twist vector [v; w]
    % Returns: xi_hat (4x4 matrix)
    
    xi = xi(:);
    v = xi(1:3);
    w = xi(4:6);
    
    w_hat = [0, -w(3), w(2);
             w(3), 0, -w(1);
             -w(2), w(1), 0];
    
    xi_hat = [w_hat, v;
              0, 0, 0, 0];
end