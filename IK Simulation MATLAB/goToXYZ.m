function anglesDeg = goToXYZ(robot, ik, weights, q0, xyz, s)
    % Solve IK for the target point
    Rhome = getTransform(robot, q0, 'Body6');
    T = trvec2tform(xyz);
    T(1:3,1:3) = Rhome(1:3,1:3);
    [qB, info] = ik('Body6', T, weights, q0);

    if ~strcmp(info.Status,'success')
        warning('Target may be unreachable (%s). Arm will get as close as it can.', info.Status);
    end

    % Build the smooth glide from home to target
    steps = 500;
    t = linspace(0,5,steps)';
    qTraj = zeros(steps,6);
    for k = 1:steps
        sBlend = (k-1)/(steps-1);
        blend = 10*sBlend^3 - 15*sBlend^4 + 6*sBlend^5;
        qTraj(k,:) = q0 + (qB - q0)*blend;
    end
    jointData = [t, qTraj];

    % Push into the workspace so the model's blocks can read it
    assignin('base','jointData',jointData);

    fprintf('Ready. Target [%.3f %.3f %.3f] -> press RUN in Simulink.\n', xyz);
    fprintf('Status: %s\n', info.Status);
    anglesDeg = rad2deg(qB);
    fprintf('Angles: %.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n', anglesDeg);

    % Send to the arm over serial (relative mode: this is a delta from current pose)
    msg = sprintf('%.2f,%.2f,%.2f,%.2f,%.2f,%.2f', anglesDeg);
    writeline(s, msg);
end