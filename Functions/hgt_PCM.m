function g = hgt_PCM(R, p)
    % Constructs a 4x4 Homogeneous Geometric Transformation (HGT) matrix
    % R: 3x3 rotation matrix
    % p: 3x1 position vector (origin of frame b expressed in frame a)
    % Returns: g (4x4 HGT matrix)
    %
    % g = [ R   p ]
    %     [ 0   1 ]

    p = p(:);
    g = [R,       p;
         0, 0, 0, 1];
end
