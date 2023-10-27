function mission_planner_simulator(scenario)
% Simulates the mission planning process, computes robot trajectories, and displays them.
% Inputs:
%   scenario: The input scenario containing relevant data for mission planning.

% Get the number of robots
num_robots = size(scenario.robot_list,2);

% Define the grid size
grid_size = [scenario.maxY, scenario.maxX];

% Binarize the aerial image and create a grid map
scenario.grid_map = binarize_aerial_image(scenario.image_original_path, scenario.image_mask_path, grid_size, scenario.threshold);
scenario.grid_map = 1- scenario.grid_map;

% Create a graph and compute the cost matrix
[adjacency_matrix num_obstacles] = create_graph(scenario.grid_map, scenario.maxX, scenario.maxY);
[scenario.cost_matrix, trajectory_data] = compute_cost_matrix(adjacency_matrix, num_obstacles, scenario.sites_coordinates, scenario.maxX, scenario.maxY);

% Perform task allocation
[robots_allocation total_cost] = genetic_task_planner(scenario.task_list, scenario.robot_list, scenario.cost_matrix, scenario.sites_coordinates, scenario.maxX, scenario.maxY);

% Path planning
[robots_trajectories trajectories] = trajectories_planner(robots_allocation, trajectory_data, scenario.maxX);

% Create binary occupancy map
map = binaryOccupancyMap(scenario.maxX, scenario.maxY, 1);
setOccupancy(map, scenario.grid_map)

% Create a figure for robots trajectories
figure;
tiledlayout(1,num_robots)

for i=1:2:num_robots*2-1
    trajectoryData = trajectories(:,i:i+1);
    nexttile
    
    % Plot occupancy map and trajectories
    show(map);  title(''); xlabel(''); ylabel('');
    
    sitesX  = cellfun(@(x) x(1), scenario.sites_coordinates);
    sitesY  = cellfun(@(x) x(2), scenario.sites_coordinates);
    
    % Remove axis numbering
    xticklabels('')
    yticklabels('')
    set(gca, 'linewidth', 1.5)
    axesHandle = gca;
    axesHandle.XAxis.TickLength = [0 0];
    axesHandle.YAxis.TickLength = [0 0];
    datacursormode on
    
    hold on;
    
    xCoords = trajectoryData(:, 1);
    yCoords = trajectoryData(:, 2);
    
    plot(xCoords(xCoords > 0), yCoords(yCoords > 0), '--', 'LineWidth', 2, 'color', [0 0.4470 0.7410]);
    hold on;
    
    plot(sitesX(2:end) - 0.5, sitesY(2:end) - 0.5, 'o', 'MarkerSize', 7, 'LineWidth', 1.2, 'MarkerEdgeColor', 'b', 'MarkerFaceColor', [0.5, 0.5, 0.5]);
    hold on;
    
    plot(sitesX(1) - 0.5, sitesY(1) - 0.5, '^', 'LineWidth', 1.4, 'MarkerSize', 9, 'MarkerEdgeColor', 'b', 'MarkerFaceColor', [0.7, 0.2, 0.5]);
end

end

