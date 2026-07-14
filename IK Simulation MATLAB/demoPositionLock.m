function demoPositionLock(robot, ik, q0)
% DEMO 1 - POSITION LOCK
% The end-effector POSITION is held fixed at a sweet-spot point (sub-mm drift)
% while the tool ORIENTATION sweeps, driving large link self-motion.
% Shows the arm reconfiguring around a perfectly held point (precision pitch).
%
% Usage:  demoPositionLock(robot, ik, q0)
%         set StopTime = 12 in Simulink, then Run.

    xyz = [-0.300 0.150 0.200];        % sweet-spot point (found by workspace search)
    weights = [1 1 1 1 1 1];
    R0 = getTransform(robot, q0, 'Body6'); R0 = R0(1:3,1:3);

    nPoses = 300;
    a = linspace(0, 2*pi, nPoses);

    q = zeros(nPoses,6); guess = q0;
    for i = 1:nPoses
        T = trvec2tform(xyz);                 % POSITION locked at xyz
        yaw   = 0.6*sin(a(i));                % tool orientation sweeps...
        pitch = 0.6*cos(a(i));                % ...around two axes
        Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
        Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
        T(1:3,1:3) = Rz*Ry*R0;
        [qS,~] = ik('Body6', T, weights, guess);
        q(i,:) = qS; guess = qS;
    end

    t = linspace(0,12,nPoses)';
    jointData = [t, q];
    assignin('base','jointData',jointData);

    drift = 0;
    for i = 1:nPoses
        Ti = getTransform(robot, q(i,:), 'Body6');
        drift = max(drift, norm(Ti(1:3,4) - xyz(:)));
    end
    fprintf('[POSITION LOCK] ready. Max tip POSITION drift: %.5f m.\n', drift);
    fprintf('  -> Set Simulink StopTime = 12, then Run.\n');
end