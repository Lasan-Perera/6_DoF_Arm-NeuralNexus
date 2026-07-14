function findDemoPoint(robot, ik, q0)
    weights = [1 1 1 1 1 1];
    R0 = getTransform(robot, q0, 'Body6'); R0 = R0(1:3,1:3);

    % Candidate tip positions to test (grid around the reachable zone)
    [X,Y,Z] = ndgrid(-0.30:0.05:-0.10, 0.15:0.05:0.35, -0.05:0.05:0.20);
    cands = [X(:) Y(:) Z(:)];

    best = struct('score',-inf);
    for c = 1:size(cands,1)
        xyz = cands(c,:);
        roll = linspace(-pi/6, pi/6, 40);    % modest +/-30 deg orientation sweep
        q = zeros(40,6); guess = q0; ok = true;
        for i = 1:40
            T = trvec2tform(xyz);
            Rz = [cos(roll(i)) -sin(roll(i)) 0; sin(roll(i)) cos(roll(i)) 0; 0 0 1];
            T(1:3,1:3) = Rz*R0;
            [qS,info] = ik('Body6',T,weights,guess);
            if ~strcmp(info.Status,'success'), ok=false; break; end
            q(i,:)=qS; guess=qS;
        end
        if ~ok, continue; end

        % Measure tip drift and link motion
        drift = 0; 
        for i=1:40
            Ti=getTransform(robot,q(i,:),'Body6');
            drift=max(drift,norm(Ti(1:3,4)-xyz(:)));
        end
        motion = sum(max(abs(diff(rad2deg(q)))));   % total link movement
        if drift < 0.005                            % tip held within 5mm
            score = motion;                         % maximize link motion
            if score > best.score
                best = struct('score',score,'xyz',xyz,'drift',drift,'motion',motion);
            end
        end
    end

    if isinf(best.score)
        fprintf('No point found with tip held under 5mm. Loosen threshold.\n');
    else
        fprintf('BEST DEMO POINT: [%.3f %.3f %.3f]\n', best.xyz);
        fprintf('  tip drift: %.4f m   link motion: %.1f deg total\n', best.drift, best.motion);
    end
end