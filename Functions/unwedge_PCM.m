function xi = unwedge_PCM(M)
% Extracts twist coordinates from a 4x4 se(3) matrix
% Input:  M - 4x4 matrix in se(3) (skew-symmetric upper-left 3x3, vector upper-right)
% Output: xi - 6x1 twist vector [v; w]

w = [M(3,2); M(1,3); M(2,1)];  % extract from skew-symmetric part
v = M(1:3, 4);                  % translational velocity

xi = [v; w];
end