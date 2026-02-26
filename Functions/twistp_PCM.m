function xi = twistp_PCM(v)
    % Returns the twist for a prismatic (translational) joint
    % v: 3x1 unit vector giving the direction of translation
    % Returns: xi = [v; 0] (6x1 twist vector)
    %
    % For a prismatic joint, the angular component w = 0
    % and the twist is xi = [v; 0]
    %
    % Note: v should be a unit vector (||v|| = 1)

    v = v(:);
    xi = [v; 0; 0; 0];
end
