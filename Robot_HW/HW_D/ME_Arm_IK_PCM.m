function TH = ME_Arm_IK_PCM(x, y, z)
% ME_Arm_IK_PCM  Closed-form inverse kinematics for the ME Arm v3.0.
%
%   TH = ME_Arm_IK_PCM(x, y, z)
%
%   Solves the 3-DOF IK consistent with the forward kinematics derived in
%   RobotC_Martin_Kinematics.m (see gstthPCM_ME.m).  The FK relations are
%
%       r  = -L_BC sin(th2) + L_CD cos(th2 - th3) + L_GRIP
%       z  =  h_B + L_BC cos(th2) + L_CD sin(th2 - th3)
%       x  =  x_base - r sin(th1)
%       y  =  r cos(th1)
%
%   Inputs:
%       x, y, z  - tip position in the base/world frame (mm).
%
%   Outputs:
%       TH  - 1x3 vector [th1 th2 th3] of "kinematic" joint angles (rad)
%             suitable as input to gstthPCM_ME.  The motor-angle mapping
%             required by the assignment (theta_motor = -th1, th2,
%             th3 - th2) is applied downstream in ME_Arm_Traj_PCM.
%
%   Notes:
%       * Elbow-up branch selected (th3 from the principal asin branch).
%         For small home-relative motion this keeps th3 near 0.
%       * Singular on the shoulder axis (y = 0 AND x = x_base).  The
%         function flags unreachable targets via a warning and returns
%         NaN; calling code should ensure every waypoint lies inside the
%         reachable workspace.
%
%   Author: Paul C. Martin (PCM)   ME 4640 S26  HW D

    % --- Link dimensions (mm), identical to RobotC_Martin_Kinematics.m ---
    x_base = 150;
    h_B    =  55;
    L_BC   =  75;
    L_CD   =  80;
    L_GRIP =  62;

    % --- Base yaw:  (x - x_base) = -r sin(th1),  y = r cos(th1) -----------
    th1 = atan2(x_base - x, y);

    % --- Collapse to a planar 2-link problem at the shoulder pivot --------
    r_full = hypot(x - x_base, y);       % tip radius in the horizontal plane
    rp     = r_full - L_GRIP;            % wrist radius (gripper is radial)
    zp     = z - h_B;                    % wrist height above shoulder

    % --- Squared-sum identity collapses to a single equation in th3 -------
    % rp^2 + zp^2 = L_BC^2 + L_CD^2 - 2 L_BC L_CD sin(th3)
    s3 = (L_BC^2 + L_CD^2 - rp^2 - zp^2) / (2 * L_BC * L_CD);
    if s3 < -1 - 1e-9 || s3 > 1 + 1e-9
        warning('ME_Arm_IK_PCM:unreachable', ...
                'Target [%.2f %.2f %.2f] outside reach (sin(th3)=%.4f).', ...
                x, y, z, s3);
        TH = [NaN NaN NaN];
        return
    end
    s3  = max(-1, min(1, s3));
    th3 = asin(s3);                      % principal branch -> elbow-up

    % --- Solve the 2x2 linear system for (sin th2, cos th2) ---------------
    %     rp = a sin(th2) + b cos(th2)
    %     zp = b sin(th2) - a cos(th2)
    %   where a = L_CD sin(th3) - L_BC,  b = L_CD cos(th3).
    a     = L_CD*s3 - L_BC;
    b     = L_CD*cos(th3);
    denom = a*a + b*b;                   % = rp^2 + zp^2 by construction
    s2    = (a*rp + b*zp) / denom;
    c2    = (b*rp - a*zp) / denom;
    th2   = atan2(s2, c2);

    TH = [th1, th2, th3];
end
