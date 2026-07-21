function objectPlaceFromBlackPole(s)
% objectPicking  Executes a simple object picking sequence.
%
%   objectPicking(s) sends the following commands to the robotic arm:
%       1. Close gripper
%       2. Open gripper
%       3. Move to pick position
%       4. Close gripper
%       5. Move to pick position again
%
%   Each command is separated by a 5-second delay.
%
%   Usage:
%       s = serialport("COM9", 115200);
%       objectPlaceFromBlackPole(s)

    pausePer2 = 4;   % seconds between commands

    commands = {
        "G,1"
        "G,0"
        "0,-60,70,0,-90,95"
        "G,1"
        "0,60,-70,0,90,-95"
    };

    for i = 1:length(commands)
        writeline(s, commands{i});
        fprintf('Sent command %d: %s\n', i, commands{i});
        pause(pausePer2);
    end

    fprintf('Object picking sequence complete.\n');
end


