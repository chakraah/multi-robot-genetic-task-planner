function [robotsTrajectories trajectories] = trajectories_planner(sequence, trajectoryData, numColumns)
% Computes the trajectories for each robot's sequence
% Inputs:
%   sequence: The computed sequence of tasks for each robot
%   trajectoryData: The shortest trajectories computed between each site
%   numColumns: Number of columns in the grid
% Outputs:
%   trajectories: The trajectories for each robot

tic % start timer

% Initialize empty cell array for modified trajectories
robotsTrajectories = cell(1,length(sequence));

% Iterate over each robot's sequence
for robotIdx = 1:numel(sequence)
    
    fromNodes = sequence{robotIdx}(1:end-1);
    toNodes = sequence{robotIdx}(2:end);
    
    % Initialize an empty array for the modified trajectory
    robotTrajectory = [];
    
    % Iterate over each element in the sequence
    for siteIdx = 1:length(fromNodes)
        
        % Retrieve the trajectory from trajectoryData and append it to the modified trajectory
        robotTrajectory = [robotTrajectory trajectoryData(fromNodes(siteIdx), toNodes(siteIdx))];
    end
    
    % Append the modified trajectory to the robotsTrajectories cell array
    robotsTrajectories{robotIdx} = cell2mat(robotTrajectory);
    
    % Determine maximum trajectory length
    trajectoriesLengths = cellfun(@length, robotsTrajectories);
    [maxLength, maxIndex] = max(trajectoriesLengths);
    
    % Initialize the trajectories array
    trajectories = zeros(maxLength, 2 * length(robotsTrajectories));
    
    % Iterate over each trajectory
    for trajectoryIdx = 1:numel(robotsTrajectories)
        
        trajectory_i = cell2mat(robotsTrajectories(trajectoryIdx));
        % Convert the node to coordinates using the given function
        [xCoordinate, yCoordinate] = arrayfun(@(point) convertNodeToCoordinates(point, numColumns), trajectory_i);
        
        % Adjust the coordinates and assign them to the appropriate positions in the trajectories array
        trajectories(1:trajectoriesLengths(trajectoryIdx), 2 * trajectoryIdx - 1) = xCoordinate - 0.5;
        trajectories(1:trajectoriesLengths(trajectoryIdx), 2 * trajectoryIdx) = yCoordinate - 0.5;
    end
    
end

% Stop measuring execution time
elapsed_time = toc;

% Display the execution time
fprintf('Trajectories planning %.4f seconds.\n', elapsed_time);

end
%----------------------------------------------------------------------------------
function [xCoord, yCoord] = convertNodeToCoordinates(node, numColumns)
% Convert graph node to x and y coordinates
% Inputs:
%   node: graph node
%   numColumns: number of columns in the grid
% Outputs:
%   xCoord: x-coordinate corresponding to the given node
%   yCoord: y-coordinate corresponding to the given node

% Calculate the x and y coordinates based on the node and number of columns
xCoord = mod(node, numColumns);
yCoord = ceil(node / numColumns);

% Adjust x coordinate if it is 0 (mod result)
if (xCoord == 0)
    xCoord = numColumns;
end
end