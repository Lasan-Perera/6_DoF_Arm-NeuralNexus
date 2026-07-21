function objectPickingFromBlackPole(s)
% objectPickingFromBlackPole  Picks an object from the black pole and
% returns it to the home position.
%
%   Sequence:
%       1. Ensure gripper is open.
%       2. Move above the black pole.
%       3. Align with the object.
%       4. Close the gripper.
%       5. Retract from the pole.
%       6. Return to the home position.
%
%   Usage:
%       s = serialport("COM9",115200);
%       objectPickingFromBlackPole(s)

    pausePer = 4;   % Seconds between commands

    commands = {
        "G,1"                    % Ensure gripper is open
        "0,-10,30,0,-40,0"       % Move towards black pole
        "0,-15,-5,0,30,0"         % Align with object
        "G,0"                    % Close gripper (pick object)
        "0,15,5,0,-30,0"         % Retract from pole
        "0,10,-30,0,40,0"        % Return to home position
    };

    for i = 1:length(commands)
        writeline(s, commands{i});
        fprintf('Sent command %d: %s\n', i, commands{i});
        pause(pausePer);
    end

    fprintf('Object picked successfully from the black pole.\n');
end