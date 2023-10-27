function [costMatrix, trajectoryData] = compute_cost_matrix(adjacencyMatrix, numObstacles, sitesCoords, numCols, numRows)
% Computes the cost matrix between graph nodes and generates the corresponding trajectories.
% Inputs:
%   sites: graph nodes, starting with the depot
%   obstacles: list of obstacle nodes
%   numCols, numRows: dimensions of the graph
% Outputs:
%   costMatrix: matrix containing the costs between nodes
%   trajectory: cell array containing the trajectories between nodes
%   sitesCoords: coordinates of the nodes

tic % start timer

numSites = length(sitesCoords);
sitesNodes = zeros(1,numSites);

% Convert nodes to coordinates
for siteIndex = 1:numSites
    sitesNodes(siteIndex) = convertCoordinatesToNode(sitesCoords{siteIndex}(1), sitesCoords{siteIndex}(2), numCols);
end

%[adjacencyMatrix, numObstacles] = createGraph(numCols, numRows, obstaclesNodes);
costMatrix = zeros(numSites, numSites);
trajectoryData = cell(numSites, numSites);

for startSite = 1:numSites
    for endSite = 1:numSites
        [shortestPath, pathCost] = dijkstraAlgorithm(adjacencyMatrix, sitesNodes(startSite), sitesNodes(endSite), numObstacles);
        costMatrix(startSite, endSite) = pathCost;
        trajectoryData{startSite, endSite} = shortestPath;
    end
end

% Stop measuring execution time
elapsed_time = toc;

% Display the execution time
fprintf('Cost matrix computation took %.4f seconds.\n', elapsed_time);

end
%----------------------------------------------------------------------------------
function [shortestPath, shortestPathCost] = dijkstraAlgorithm(costMatrix, source, destination, numObstacles)
% Implementation of Dijkstra's algorithm to find the shortest path
% between two nodes in a graph with positive edge weights.
% Inputs:
%   costMatrix: adjacency matrix representing the graph
%   source: source node index
%   destination: destination node index
%   numObstacles: number of obstacles in the graph
% Outputs:
%   shortestPath: array containing the nodes in the shortest path
%   shortestPathCost: cost of the shortest path

numNodes = size(costMatrix, 1);
visitedNodes = zeros(1, numNodes);
distances = inf(1, numNodes);
previousNodes = numNodes + 1;

distances(source) = 0;

while sum(visitedNodes) ~= numNodes - numObstacles
    candidate = [];
    for node = 1:numNodes
        if visitedNodes(node) == 0
            candidate = [candidate distances(node)];
        else
            candidate = [candidate inf];
        end
    end
    [uIndex, u] = min(candidate);
    visitedNodes(u) = 1;
    for node = 1:numNodes
        if distances(u) + costMatrix(u, node) < distances(node)
            distances(node) = distances(u) + costMatrix(u, node);
            previousNodes(node) = u;
        end
    end
end

shortestPath = [destination];

while shortestPath(1) ~= source
    if previousNodes(shortestPath(1)) <= numNodes
        shortestPath = [previousNodes(shortestPath(1)) shortestPath];
    else
        error('Error in finding the shortest path.');
    end
end

shortestPathCost = distances(destination);
end
%----------------------------------------------------------------------------------
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
