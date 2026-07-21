function armWave(s)
% armWave  Goodbye wave sequence for the NeuralNexusArm.
%
%   armWave(s) sends a "bye" wave to the arm over serial port s:
%   raises J3 up, swings the base (J1) left-right while the wrist
%   joints (J4/J5/J6) wave, then lowers J3 and returns to the start pose.
%
%   The sequence is balanced so every joint nets to zero displacement
%   (returns to the starting pose), and every intermediate pose stays
%   within the joint limits.
%
%   Usage:
%       s = serialport("COM9", 115200);
%       armWave(s)
%
%   Tuning: edit the rows of 'seq' below. Each row is a RELATIVE delta
%   [J1 J2 J3 J4 J5 J6] in degrees. Keep each column summing to 0 so the
%   arm returns home. Adjust 'pausePer' if moves get cut off (a ~60 deg
%   move takes ~4 s at the current speed).

    pausePer = 4.5;    % seconds between moves - must exceed the longest move time

    % Relative deltas per move [J1 J2 J3 J4 J5 J6].
    % (Balanced: every column sums to 0 -> returns to start pose.)
    seq = [
        0    0   20    0    0    0;   % raise J3
        0    0   20    0    0    0;   % raise J3 more
        0    0   40    0    0    0;   % raise J3 to wave height
        30  -5  -50   20   30   40;   % wave 1: base right + wrist
       -60  10   10  -20  -30  -40;   % wave 2: base left
        60 -10  -10   20   30   40;   % wave 3: base right
       -60  10   10  -20  -30  -40;   % wave 4: base left
        30  -5    5    0    0    0;   % settle: base center, J2 home (wrist already balanced)
        0    0  -45    0    0    0;   % lower J3 back to start
    ];

    for i = 1:size(seq,1)
        writeline(s, sprintf('%.2f,%.2f,%.2f,%.2f,%.2f,%.2f', seq(i,:)));
        fprintf('Sent move %d: [%s]\n', i, num2str(seq(i,:)));
        pause(pausePer);
    end
    fprintf('Bye wave complete.\n');
end