function objectPickAndPlacePole(s)
% objectPickAndPlacePole  Picks an object and places it on a pole.
%
%   Sequence:
%       1. Ensure gripper is open.
%       2. Move to the object.
%       3. Close the gripper to grasp the object.
%       4. Return to the home position with the object.
%       5. Move to the pole position.
%       6. Open the gripper to release the object.
%       7. Return to the home position.
%
%   Usage:
%       s = serialport("COM9",115200);
%       objectPickAndPlacePole(s)

    pausePer = 4;     % Seconds between commands

    commands = {
        "G,1"                     % Ensure gripper is open
        "G,0"                     % Open gripper
        "0,-13,40,0,-50,0"        % Move to object
        "G,1"                     % Close gripper (pick object)
        "0,13,-40,0,50,0"         % Return home with object
        "G,0"                     % Release object
    };

    for i = 1:length(commands)
        writeline(s, commands{i});
        fprintf('Sent command %d: %s\n', i, commands{i});
        pause(pausePer);
    end

    fprintf('Object pick-and-place sequence complete.\n');
end