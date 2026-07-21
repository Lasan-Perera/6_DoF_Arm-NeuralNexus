function armBye(s)
% Goodbye wave: raise J3 up, then swing the base (J1) while J4/J5/J6 wave.
% All tunable parameters are grouped at the top.

    % ===================== TUNABLE PARAMETERS =====================
    basePose  = [0 -5 120 0 160 0];    % your stable start pose [J1..J6]

    % --- raise phase: how the arm lifts into the wave posture ---
    raiseJ3   = 20;      % how much to raise J3 (deg) from base   [+ = up]
    raiseJ2   = 30;       % optional shoulder lift (deg), 0 = none

    % --- wave phase amplitudes (deg swing each side) ---
    ampBase   = 25;      % J1 base swing left-right
    ampJ4     = 30;      % J4 wave
    ampJ5     = 30;      % J5 wave
    ampJ6     = 40;      % J6 wrist wave

    nWaves    = 3;       % how many left-right swings
    pausePer  = 2;     % seconds to hold between each waypoint (tune to move speed)
    % ==============================================================

    limitsDeg = [-180 180; -70 70; 120 240; -180 180; 60 300; -180 180];

    % ---- build the pose list ----
    seq = {};

    % 1) raise into wave posture (J3 up, optional J2)
    raisePose = basePose;
    raisePose(2) = basePose(2) + raiseJ2;
    raisePose(3) = basePose(3) + raiseJ3;
    seq{end+1} = raisePose;

    % 2) the waves: base swings one way, wrist joints swing together,
    %    then all swing the other way. Alternate each wave.
    for w = 1:nWaves
        p = raisePose;
        p(1) = raisePose(1) + ampBase;     % base right
        p(4) = raisePose(4) + ampJ4;
        p(5) = raisePose(5) + ampJ5;
        p(6) = raisePose(6) + ampJ6;
        seq{end+1} = p;

        p = raisePose;
        p(1) = raisePose(1) - ampBase;     % base left
        p(4) = raisePose(4) - ampJ4;
        p(5) = raisePose(5) - ampJ5;
        p(6) = raisePose(6) - ampJ6;
        seq{end+1} = p;
    end

    % 3) settle back to raised center, then home
    seq{end+1} = raisePose;
    seq{end+1} = basePose;

    % ---- clamp every pose to joint limits (protects sim + arm) ----
    for si = 1:numel(seq)
        for j = 1:6
            seq{si}(j) = max(limitsDeg(j,1), min(limitsDeg(j,2), seq{si}(j)));
        end
    end

    % ---- animate the sim (smooth glide through all poses) ----
    stepsPer = 80;
    allData = []; tCur = 0; prev = basePose;
    for si = 1:numel(seq)
        tgt = seq{si};
        for k = 1:stepsPer
            b = (k-1)/(stepsPer-1);
            blend = 10*b^3 - 15*b^4 + 6*b^5;
            p = prev + (tgt - prev)*blend;
            allData = [allData; tCur, deg2rad(p)];
            tCur = tCur + 0.02;
        end
        prev = tgt;
    end
    assignin('base','jointData',allData);
    set_param('Assem1','StopTime', num2str(tCur));
    set_param('Assem1','SimulationCommand','start');

    % ---- drive the real arm: one relative delta per pose ----
    prev = basePose;
    for si = 1:numel(seq)
        tgt = seq{si};
        delta = tgt - prev;
        writeline(s, sprintf('%.2f,%.2f,%.2f,%.2f,%.2f,%.2f', delta));
        prev = tgt;
        pause(pausePer);
    end

    fprintf('Bye wave done.\n');
end