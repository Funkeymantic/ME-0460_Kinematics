function xi = twistr_PCM(w, q)
    % w: rotation axis (3x1 vector)
    % q: point on the rotation axis (3x1 vector)
    % Returns: xi (6x1 twist vector)

    w = w(:);
    q = q(:);
    
    v = -cross(w, q);
    xi = [v; w];
end