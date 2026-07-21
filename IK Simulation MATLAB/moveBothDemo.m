function moveBothDemo(s, mode)
% Run a lock demo on BOTH the simulation and the real arm.
% mode = "position"  -> tip position held, orientation sweeps
% mode = "orientation" -> tip orientation held, position sweeps X/Y/Z
%
% Requires robot, ik, q0 in the base workspace.

    robot = evalin('base','robot');
    ik    = evalin('base','ik');
    q0    = evalin('base','q0');
    weights = [1 1 1 1 1 1];

    R0 = getTransform(robot, q0, 'Body6'); R0 = R0(1:3,1:3);
    xyz = [-0.300 0.150 0.200];           % the sweet-spot point

    nPoses = 200;
    q = zeros(nPoses,6); guess = q0;

    if mode == "position"
        a = linspace(0, 2*pi, nPoses);
        for i = 1:nPoses
            T = trvec2tform(xyz);
            yaw=0.6*sin(a(i)); pitch=0.6*cos(a(i));
            Rz=[cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
            Ry=[cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
            T(1:3,1:3)=Rz*Ry*R0;
            [qs,~]=ik('Body6',T,weights,guess); q(i,:)=qs; guess=qs;
        end
    else  % orientation
        amp=0.06; nSeg=round(nPoses/3); sName=sin(linspace(0,2*pi,nSeg));
        off=[[amp*sName;zeros(1,nSeg);zeros(1,nSeg)]';
             [zeros(1,nSeg);amp*sName;zeros(1,nSeg)]';
             [zeros(1,nSeg);zeros(1,nSeg);amp*sName]'];
        for i=1:size(off,1)
            T=trvec2tform(xyz+off(i,:)); T(1:3,1:3)=R0;
            [qs,~]=ik('Body6',T,weights,guess); q(i,:)=qs; guess=qs;
        end
        nPoses=size(off,1); q=q(1:nPoses,:);
    end

    % --- drive the sim ---
    t=linspace(0,12,nPoses)';
    jointData=[t, q];
    assignin('base','jointData',jointData);
    set_param('Assem1','StopTime','12');
    set_param('Assem1','SimulationCommand','start');

    % --- drive the arm: stream relative deltas at a handful of waypoints ---
    wpts = round(linspace(1, nPoses, 16));
    prev = rad2deg(q(1,:));
    for w = wpts
        target = rad2deg(q(w,:));
        delta = target - prev;
        writeline(s, sprintf('%.2f,%.2f,%.2f,%.2f,%.2f,%.2f', delta));
        prev = target;
        while true
            line = strtrim(readline(s));
            if line == "END", break; end
        end
    end
    fprintf('%s-lock demo done.\n', mode);
end