function demoOrientationLock_XYZsweep(robot, ik, q0)
% DEMO 2 - ORIENTATION LOCK + X / Y / Z SWEEP
% The end-effector ORIENTATION is held fixed while the tip POSITION sweeps
% along X (out and back), then Y, then Z - one axis at a time.
% Shows controlled linear precision on each axis with a fixed tool angle.
%
% Usage:  demoOrientationLock_XYZsweep(robot, ik, q0)
%         set StopTime = 15 in Simulink, then Run.

    xyz0 = [-0.300 0.150 0.200];       % center point
    weights = [1 1 1 1 1 1];
    R0 = getTransform(robot, q0, 'Body6'); R0 = R0(1:3,1:3);   % LOCKED orientation

    amp  = 0.15;                       % +/- 6 cm sweep on each axis
    nSeg = 100;
    s = sin(linspace(0, 2*pi, nSeg));  % smooth out-and-back per axis

    offsets = [ [amp*s;  zeros(1,nSeg); zeros(1,nSeg)]' ;   % X sweep
                [zeros(1,nSeg); amp*s;  zeros(1,nSeg)]' ;   % Y sweep
                [zeros(1,nSeg); zeros(1,nSeg); amp*s ]' ];  % Z sweep

    nPoses = size(offsets,1);
    q = zeros(nPoses,6); guess = q0;
    for i = 1:nPoses
        p = xyz0 + offsets(i,:);
        T = trvec2tform(p);
        T(1:3,1:3) = R0;               % ORIENTATION locked
        [qS,~] = ik('Body6', T, weights, guess);
        q(i,:) = qS; guess = qS;
    end

    t = linspace(0, 15, nPoses)';      % ~5 s per axis
    jointData = [t, q];
    assignin('base','jointData',jointData);

    maxRot = 0;
    for i = 1:nPoses
        Ti = getTransform(robot, q(i,:), 'Body6');
        dR = R0' * Ti(1:3,1:3);
        maxRot = max(maxRot, acosd((trace(dR)-1)/2));
    end
    fprintf('[ORIENTATION LOCK] ready. Max ORIENTATION drift: %.3f deg.\n', maxRot);
    fprintf('  -> Set Simulink StopTime = 15, then Run.\n');
end