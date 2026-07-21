function drawSquare(s)
% Draw a vertical square in space (in the Y-Z plane, facing the arm).
% Traces the 4 corners in order, solving IK for each, on sim + arm.

    robot = evalin('base','robot');
    ik    = evalin('base','ik');
    q0    = evalin('base','q0');
    weights = [1 1 1 1 1 1];
    R0 = getTransform(robot, q0, 'Body6'); R0 = R0(1:3,1:3);

    % --- square definition (VERTICAL plane: fixed X, varying Y and Z) ---
    cx = -0.25;      % distance out from base (fixed) - TUNE to reachable space
    cy =  0.15;      % center left-right
    cz =  0.20;      % center height
    side = 0.10;     % square side length (meters) - TUNE

    h = side/2;
    % 4 corners of the square in the Y-Z plane, traced in order + close loop
    corners = [ cx, cy-h, cz-h;    % bottom-left
                cx, cy+h, cz-h;    % bottom-right
                cx, cy+h, cz+h;    % top-right
                cx, cy-h, cz+h;    % top-left
                cx, cy-h, cz-h ];  % back to start (close the square)

    % interpolate along each edge for smooth straight lines
    ptsPerEdge = 25;
    path = [];
    for e = 1:size(corners,1)-1
        a = corners(e,:); b = corners(e+1,:);
        for t = linspace(0,1,ptsPerEdge)
            path = [path; a + (b-a)*t];
        end
    end

    % solve IK along the whole path
    nP = size(path,1);
    q = zeros(nP,6); guess = q0;
    for i = 1:nP
        T = trvec2tform(path(i,:));
        T(1:3,1:3) = R0;                 % keep tool orientation fixed
        [qs,info] = ik('Body6', T, weights, guess);
        q(i,:) = qs; guess = qs;
        if ~strcmp(info.Status,'success')
            fprintf('WARN: corner/point %d may be unreachable (%s)\n', i, info.Status);
        end
    end

    % --- animate the sim ---
    t = linspace(0, 16, nP)';
    jointData = [t, q];
    assignin('base','jointData', jointData);
    set_param('Assem1','StopTime','16');
    set_param('Assem1','SimulationCommand','start');

    % --- drive the arm: stream waypoints as relative deltas ---
    wpts = round(linspace(1, nP, 20));
    prev = rad2deg(q(1,:));
    for w = wpts
        target = rad2deg(q(w,:));
        delta = target - prev;
        writeline(s, sprintf('%.2f,%.2f,%.2f,%.2f,%.2f,%.2f', delta));
        prev = target;
        pause(1.0);
    end

    fprintf('Square complete.\n');
end

function sendChunked(s, delta, maxStep, dly)
% Send a relative move split into small chunks so each completes in time.
%   delta   = [J1..J6] total relative move (deg)
%   maxStep = max degrees per chunk per joint (default 10)
%   dly     = delay between chunks in seconds (default 1)

    if nargin < 3, maxStep = 10; end
    if nargin < 4, dly = 1.0; end

    % how many chunks needed = based on the joint that moves the most
    nChunks = max(1, ceil(max(abs(delta)) / maxStep));

    stepDelta = delta / nChunks;      % equal small step per chunk

    for c = 1:nChunks
        writeline(s, sprintf('%.2f,%.2f,%.2f,%.2f,%.2f,%.2f', stepDelta));
        pause(dly);
    end
end