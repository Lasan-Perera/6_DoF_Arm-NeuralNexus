function objectShowOff(s)
% objectShowOff  Demonstrates a simple robotic arm motion.
%
%   Sequence:
%       1. Raise the arm.
%       2. Rotate the wrist for display.
%       3. Return the wrist to its original orientation.
%       4. Lower the arm back to the home position.
%
%   Usage:
%       s = serialport("COM9",115200);
%       objectShowOff(s)

    pausePer = 4;   % Seconds between commands

    commands = {
        "0,-10,120,0,0,0"        % Raise arm
        "0,0,-10,40,40,-40"     % Wrist display motion
        "0,0,10,-40,-40,40"     % Return wrist
        "0,10,-60,0,0,0"        % Return to home
        "0,0,10,-40,-40,40"
        "0,0,-10,40,40,-40"
        "0,0,-60,0,0,0"
    };

    for i = 1:length(commands)
        writeline(s, commands{i});
        fprintf('Sent command %d: %s\n', i, commands{i});
        pause(pausePer);
    end

    fprintf('Show-off sequence complete.\n');
end