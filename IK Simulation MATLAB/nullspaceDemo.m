function nullspaceDemo(robot, ik, q0, xyz)
% Pins the end effector at xyz and sweeps the arm through self-motion poses.
% The tip stays fixed; the links flow around it.

    % Position pinned hard, orientation fully relaxed -> gives the arm
    % freedom to reconfigure while the tip stays put.
    weights = [0 0 0  1 1 1];

    % Target: fixed tip POSITION (orientation left free)
    tform = trvec2tform(xyz);

    % Sweep the base joint (J1) across a range; re-solve IK each step so
    % the tip stays anchored while the arm shape changes.
    nPoses   = 60;
    sweepDeg = linspace(-40, 40, nPoses);   % base swing range

    poses = zeros(nPoses, 6);
    guess = q0;
    for i = 1:nPoses
        % Bias the base angle, let IK solve the rest to keep the tip fixed
        guess(1) = deg2rad(sweepDeg(i));
        [qSol, info] = ik('Body6', tform, weights, guess);
        poses(i,:) = qSol;
        guess = qSol;                 % warm-start next solve (smoothness)
    end

    % Animate: tip should stay planted while the arm flows
    figure('Name','Nullspace self-motion demo');
    for i = 1:nPoses
        show(robot, poses(i,:), 'PreservePlot', false, 'FastUpdate', true);
        hold on;
        plot3(xyz(1), xyz(2), xyz(3), 'r.', 'MarkerSize', 30);  % the pinned point
        hold off;
        title(sprintf('Self-motion  %d/%d', i, nPoses));
        view(135, 25);
        drawnow;
    end
end