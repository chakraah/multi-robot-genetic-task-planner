function [adjacencyMatrix numObstacles] = create_graph(gridMap, numColumns, numRows)
% Create a graph representation of a grid with obstacles
% Inputs:
%   numColumns: number of columns in the grid
%   numRows: number of rows in the grid
%   gridMap: Binary matrix representing the environment with obstacles
% Outputs:
%   adjacencyMatrix: adjacency matrix representing the graph
%   obstaclesNodes: array containing the nodes corresponding to obstacles


tic; % start timer

startNodes = [];
endNodes = [];
fullNodes = [];

[yCoords xCoords] = find(gridMap == 1);
yCoords = size(gridMap,1) - yCoords + 1;

numObstacles = length(xCoords);
obstaclesNodes = [];

% Convert obstacle coordinates to node indices
for obstacleIdx = 1:numObstacles
    obstaclesNodes(obstacleIdx) = convertCoordinatesToNode(xCoords(obstacleIdx), yCoords(obstacleIdx), numColumns);
end

% Iterate through rows and columns to create graph edges
for row = 1:numRows
    for column = 1:numColumns
        neighbors = getNeighbors(column, row, numColumns, numRows);
        currentNode = (row - 1) * numColumns + column;
        
        for idx = 1:length(neighbors)
             % Check if both nodes are not obstacles
            if (isempty(find(neighbors(idx) == obstaclesNodes, 1)) && isempty(find(currentNode == obstaclesNodes, 1)))
                startNodes = [startNodes currentNode];
                endNodes = [endNodes neighbors(idx)];
                fullNodes = [fullNodes (row - 1) * numColumns + column];
            end
        end
    end
end

% Initialize the adjacency matrix
adjacencyMatrix = inf(numColumns * numRows);

% Update the adjacency matrix with edge information
for idx = 1:length(startNodes)
    adjacencyMatrix(startNodes(idx), endNodes(idx)) = 1;
end

% Create a graph object and plot it
g = graph(startNodes, endNodes);
nexttile;
plot(g);
title('Graph Representation of the environment');

% Stop measuring execution time
elapsed_time = toc;

% Display the execution time
fprintf('Graph creation took %.4f seconds.\n', elapsed_time);

end

function node = convertCoordinatesToNode(xCoord, yCoord, numColumns)
% Convert x and y coordinates to graph node
% Inputs:
%   xCoord: x-coordinate of the point
%   yCoord: y-coordinate of the point
%   numColumns: number of columns in the grid
% Output:
%   node: graph node corresponding to the given coordinates

% Calculate the node index based on the coordinates and number of columns
node = (yCoord - 1) * numColumns + xCoord;
end

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

function neighbors = getNeighbors(xCoord, yCoord, numColumns, numRows)
% Get neighboring nodes of a given node
% Inputs:
%   xCoord: x-coordinate of the given node
%   yCoord: y-coordinate of the given node
%   numColumns: number of columns in the grid
%   numRows: number of rows in the grid
% Output:
%   neighbors: array containing the neighboring nodes

neighbors = [];

% Check if there is a neighbor to the right
if (xCoord < numColumns)
    neighbors = [neighbors (yCoord - 1) * numColumns + xCoord + 1];
end

% Check if there is a neighbor to the left
if (xCoord > 1)
    neighbors = [neighbors (yCoord - 1) * numColumns + xCoord - 1];
end

% Check if there is a neighbor below
if (yCoord < numRows)
    neighbors = [neighbors yCoord * numColumns + xCoord];
end

% Check if there is a neighbor above
if (yCoord > 1)
    neighbors = [neighbors (yCoord - 2) * numColumns + xCoord];
end
end
