function moveBoth(s, delta)
% Move real arm AND simulation by the same relative delta from the current pose.
% delta = [d1 d2 d3 d4 d5 d6] in degrees.
%
% Uses a persistent 'currentPose' so the sim tracks the accumulated pose.

    persistent currentPose
    if isempty(currentPose)
        currentPose = [0 -5 120 0 160 0];   % your stable reference pose
    end

    % soft limit check before moving
    limitsDeg = [-180 180; -70 70; 120 240; -180 180; 60 300; -180 180];
    newPose = currentPose + delta;
    for j = 1:6
        if newPose(j) < limitsDeg(j,1) || newPose(j) > limitsDeg(j,2)
            fprintf('WARNING: J%d would hit limit (%.0f, target %.0f). Skipping move.\n', ...
                    j, limitsDeg(j, (delta(j)<0)+1), newPose(j));
            return;
        end
    end

    % --- send relative delta to the real arm ---
    msg = sprintf('%.2f,%.2f,%.2f,%.2f,%.2f,%.2f', delta);
    writeline(s, msg);

    % --- update the sim pose and animate it ---
    currentPose = currentPose + delta;
    poseRad = deg2rad(currentPose);

    steps = 100;
    t = linspace(0, 2, steps)';
    startRad = deg2rad(currentPose - delta);
    jointData = zeros(steps, 7);
    jointData(:,1) = t;
    for k = 1:steps
        b = (k-1)/(steps-1);
        blend = 10*b^3 - 15*b^4 + 6*b^5;
        jointData(k,2:7) = startRad + (poseRad - startRad)*blend;
    end
    assignin('base','jointData',jointData);

    set_param('Assem1','StopTime','2');
    set_param('Assem1','SimulationCommand','start');

    fprintf('Moved by [%s]. Sim pose now [%s]\n', ...
            num2str(delta), num2str(currentPose));
end