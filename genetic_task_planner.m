function [bestSequence bestFitness] = genetic_task_planner(taskList, robotList, costMatrix, sitesCoords, X, Y)
% A centralized Task Allocation Algorithm for a Multi-Robot System with Sensing Specification
% Function call : genetic_task_planner(scenario1);
% September 2022

% 1- Parameters initialization

% Total number of robots and tasks in the scenario
numRobots = size(robotList, 2);
numTasks = sum(sum(taskList(2:end, :)));

% Genetic parameters
crossoverProb = 0.8;
mutationProb = 0.2;
if numRobots > 3 && numTasks > 20
    populationSize = 300;
    numGenerations = 150;
else
    populationSize = 150;
    numGenerations = 80;
end

% Objective function
objFun = 2;

% Number of retained parents and children
numParents = populationSize/2;
numChildren = numParents;

% Number of mutants to create
numMutants = mutationProb * populationSize;

% Initializing generation counter and cost values
currentGeneration = 1;
costValues = zeros(1, numGenerations);

% 2- Tasks decomposition

% Decompose the mission into tasks with one measurment
singleMeasureTasks = decomposeMission (taskList, numTasks);
% Identify suitable robots for each task
suitableRobots = identifySuitableRobots (singleMeasureTasks, robotList, numTasks);

% 3- Population initialization

% Initialize the population of solutions
parentPopulation = initializePopulation (populationSize, suitableRobots, numTasks);

% 4- Genetic Algorithm Process

tic % start timer

while currentGeneration <= numGenerations
    % Generating children by crossing the parents
    childrenPopulation = reproduction(parentPopulation, populationSize, crossoverProb, numTasks);
    
    % Calculate fitness of parent population
    fitnessValues = calculateFitness(parentPopulation, populationSize, costMatrix, numTasks, numRobots, objFun);
    
    % Selcet the best parents
    selectedParents = selection(parentPopulation, fitnessValues, numParents);
    
    % Calculate fitness of children population
    fitnessValues = calculateFitness(childrenPopulation, populationSize, costMatrix, numTasks, numRobots, objFun);
    
    % Select the best children
    selectedChildren = selection (childrenPopulation, fitnessValues, numChildren);
    
    % Create new population by combining selected parents and children
    newPopulation = [selectedParents , selectedChildren];
    
    % Mutate some solutions
    for i = 1:numMutants
        index = randi(populationSize);
        newPopulation{end+1} = mutation (newPopulation{index}, mutationProb, robotList, numTasks);
    end
    
    % Calculate fitness of new population
    newFitnessValues = calculateFitness(newPopulation, populationSize, costMatrix, numTasks, numRobots, objFun);
    
    % Select the best solution
    [bestFitness, bestIndex] = min(newFitnessValues);
    bestSolution = newPopulation{bestIndex};
    
    % Calculate cost of best solution
    bestSequence = generateSequence(bestSolution, numTasks, numRobots);
    costValues(currentGeneration) = bestFitness;
    
    % The new population become parent population for the next generation
    parentPopulation = newPopulation;
    
    % Increment the generation counter
    currentGeneration = currentGeneration + 1;   % Counter increment
end

% Stop measuring execution time
elapsed_time = toc;

% Display the execution time
fprintf('Task allocation took %.4f seconds.\n', elapsed_time);

% 5- solution display

figure; % Create a new figure
tiledlayout(1,numRobots);

% Iterating over each robot
for idxRobot = 1:numRobots
    nexttile
    % Tracing the sequence for the current robot
    traceSequence(costMatrix, sitesCoords, X, Y, bestSequence{idxRobot})
    title("Robot " + idxRobot + " sequence")
end

end
%----------------------------------------------------------------------------------
function single_measure_tasks = decomposeMission(task_list, num_tasks)
% decompTasks - Decompose the task_list to contain only one measurement per column.
% The number of columns corresponds to the number of tasks to be performed.

% Inputs:
% - tasks: a matrix containing the tasks to be decomposed
% - num_tasks: the number of tasks
%
% Returns:
% - single_measure_tasks: a matrix containing the decomposed tasks,
% where each task has only one measure

for task_idx = 1:num_tasks
    task = task_list(:, task_idx);
    num_measurements = sum(task(2:end));
    
    % Decompose task if more than one measurment is required
    if (num_measurements > 1)
        % Get indices of the measurements
        measurement_indices = find(task == 1);
        
        % Create new task column for each measurment
        single_measure_task = zeros(length(task), num_measurements);
        single_measure_task(measurement_indices, :) = eye(num_measurements);
        single_measure_task(1,:) = task(1);
        
        % Replace current task column with new task columns
        task_list = [task_list(:, 1:task_idx-1) single_measure_task task_list(:, task_idx+1:end)];
    end
end

% Return decomposed task matrix
single_measure_tasks = task_list(:, 2:end);

end
%----------------------------------------------------------------------------------
function suitable_robots = identifySuitableRobots(single_measure_tasks, robot_list, num_tasks)
% Determine the robots that can perform measurements based on the tasks

% Inputs:
% - single_measure_tasks: the matrix containing the tasks to be decomposed
% - robot_list: the list of each robot's capabilities
% - num_tasks: the number of tasks
%
% Returns:
% - suitable_robots: a cell containing for each task, the set of suitable
% robots that can perform it

% Initialize output cell array
suitable_robots = cell(3, num_tasks);

% Loop through each task
for task_idx = 1:num_tasks
    % Extract current task and measurement
    current_task = single_measure_tasks(:, task_idx);
    current_measure = find(current_task(2:end));
    
    % Find robots that can perform the measurement
    appropriate_robots = find(robot_list(current_measure, :));
    
    % store results in output cell array
    suitable_robots{1, task_idx} = current_task(1);  % Position
    suitable_robots{2, task_idx} = current_measure;  % Measurement
    suitable_robots{3, task_idx} = appropriate_robots;  % Robots that can perform the measurement
    
end

end
%----------------------------------------------------------------------------------
function initial_population = initializePopulation(population_size, suitable_robots, num_tasks)
% initialize_population - Initializes a population of candidate solutions
% Inputs:
% - population_size: the size of the population to generate
% - suitable_robots: a cell containing for each task, the set of suitable robots that can perform it
% - num_tasks: the number of tasks
%
% Returns:
% - initial_population: a set of initial candidates

% Pre-allocate memory for the population
initial_population = cell(1,population_size);

% Pre-allocate memory for a candidate
candidate_chromosome = zeros(3, num_tasks);

% Calculate the maximum number of robots available for a task
num_robots_per_task = cellfun(@length, suitable_robots(3,:));

for candidate_idx = 1:population_size
    % Randomly select a suitable robot for each task
    robotIndices = arrayfun(@(x) randi(x), num_robots_per_task);
    selected_robots = cellfun(@(x, y) x(y), suitable_robots(3,:), num2cell(robotIndices));
    
    % Saving the new individual in 'candidate'
    candidate_chromosome(1:2,:) = cell2mat(suitable_robots(1:2,:));
    candidate_chromosome(3,:) = selected_robots;
    
    % Randomly permute the tasks
    permutation = randperm(num_tasks);
    
    % Permute the tasks in 'candidate'
    candidate_chromosome = candidate_chromosome(:,permutation);
    
    % Add the generated candidate to the population
    initial_population{candidate_idx} = candidate_chromosome;
    
end

end
%----------------------------------------------------------------------------------
function candidate_sequence = generateSequence(candidate, num_tasks, num_robots)
% generate_sequence - Generates a sequence of tasks to robots from a given chromosome candidate.
% Inputs:
% - candidate: a candidate solution
% - num_tasks: the number of tasks
% - num_robots: the number of robots
%
% Returns:
% - candidate_sequence: the generated sequence

% Initialize sequence cell array
candidate_sequence = cell(1, num_robots);

% Define the depot as the starting and ending site of the sequence
depot = 1;

% Add tasks position to robots sequences
for task_idx = 1:num_tasks
    % Get the task position and the allocated robot from the chromosome candidate
    task_position = candidate(1, task_idx);
    allocated_robot = candidate(3, task_idx);
    
    % Add the task position to the robot's sequence
    candidate_sequence{allocated_robot} = [candidate_sequence{allocated_robot} task_position];
end

% Add the depot to the start and end of each robot's sequence
for robot_idx = 1:num_robots
    % Check if the robot has any tasks assigned to it
    if numel(candidate_sequence{robot_idx}) > 0
        candidate_sequence{robot_idx} = [depot candidate_sequence{robot_idx} depot];
    end
end

end
%----------------------------------------------------------------------------------
function [total_cost, robot_costs] = computeCost(candidate_sequence, cost_matrix, num_robots, objective_function)
% computeCost - Calculate the cost of a given allocation sequence of tasks
% Inputs:
% - sequence: the sequence that the cost will be calculated for
% - cost_matrix: a matrix of costs between task positions
% - num_robots: the number of robots
% - objective_function: an integer indicating the objective function to use:
%     * 1: MinMax cost function
%     * 2: MinSum cost function
%
% Outputs:
% - total_cost: the total cost of the mission
% - robot_costs: a vector of the cost of each robot's mission

% Initialize the vector that will hold the cost of each robot's mission
robot_costs = zeros(1,num_robots);

% Loop over each robot's sequence of tasks
for robot_idx = 1:num_robots
    % Extract the sequence of tasks for the current robot
    robot_sequence = candidate_sequence{robot_idx};
    
    % Check if the robot has any tasks assigned
    if length(robot_sequence) > 0
        % Calculate the cost of the robot's sequence of tasks by summing
        % the costs between consecutive tasks positions
        from = robot_sequence(1:end-1);
        to = robot_sequence(2:end);
        robot_costs(robot_idx) = sum(cost_matrix(sub2ind(size(cost_matrix), from, to)));
    end
end

% Calculate the total cost of the mission as the sum of each robot's cost
total_cost = sum(robot_costs);

% If the objective function is set to MinMax, calculate the mission cost as the maximum of each robot's cost
if objective_function == 1
    total_cost = max(robot_costs);
end

end
%----------------------------------------------------------------------------------
function fitness_values = calculateFitness(population, population_size, cost_matrix, num_tasks, num_robots, objective_function)
% calculateFitness - Calculate the fitness value for each individual in the population
% Inputs:
% - population: a cell representing the population of individuals
% - cost_matrix: a matrix of costs between task positions
% - num_robots: the number of robots
% - num_tasks: the number of tasks
%
% Output:
% - fitness_values: a column vector representing the fitness values of all individuals in the population

% Initialize fitness vector
fitness_values = zeros(1,population_size);

% Calculate the fitness value for each individual
for individual_idx = 1:size(population, 2)
    % Generate the task sequence for the current individual
    individual_sequence = generateSequence(population{individual_idx}, num_tasks, num_robots);
    
    % Calculate the fitness value of the sequence for the current individual
    fitness_values(individual_idx) = computeCost(individual_sequence, cost_matrix, num_robots, objective_function);
end

end
%--------------------------------------------------------------------------
function selected_population = selection(population, fitness_values, num_selected)
% selection - Select the best num_selected individuals from population
% Inputs:
% - population: a cell representing the population of individuals
% - fitness_values: a vector representing the fitness values of all individuals in the population
% - num_selected: the number of individuals to select
%
% Outputs:
% - selected_population: the population of selected individuals

% Sort the population in ascending order based on fitness
[~, idx] = sort(fitness_values, 'ascend');
sorted_population = population(idx);

% Select the top num_selected individuals
selected_population = sorted_population(1:num_selected);

end
%--------------------------------------------------------------------------
function [offspring1, offspring2] = crossover(chromosome1, chromosome2, crossover_rate, num_genes)
% crossover - Performs crossover between two chromosomes to produce two offspring
% Inputs:
% - chromosome1 & chromosome2: two chromosome parents
% - crossover_rate: crossover probability
% - num_tasks: number of tasks
%
% Outputs:
% - offspring1 & offspring2: the resulting offspring chromosomes after crossover

% Create copies of the input chromosomes
offspring1 = chromosome1;
offspring2 = chromosome2;

% Check if crossover should be performed based on the crossover rate
if rand() < crossover_rate
    
    % Select two random tasks from the chromosomes
    selected_tasks = randperm(num_genes, 2);
    
    % Find the corresponding tasks in each chromosome and choose one randomly
    task_in_chromosome1 = find(offspring2(2,selected_tasks(2)) == offspring1(2,:));
    task_in_chromosome1 = task_in_chromosome1(randi(numel(task_in_chromosome1)));
    
    task_in_chromosome2 = find(offspring1(2,selected_tasks(1)) == offspring2(2,:));
    task_in_chromosome2 = task_in_chromosome2(randi(numel(task_in_chromosome2)));
    
    % Swap the robot assignments for the selected tasks
    offspring1(3,[selected_tasks(1) task_in_chromosome1]) = offspring2(3,[task_in_chromosome2 selected_tasks(2)]);
    offspring2(3,[selected_tasks(2) task_in_chromosome2]) = offspring1(3,[task_in_chromosome1 selected_tasks(1)]);
    
    % Swap the order of the selected tasks in the chromosomes
    offspring1(:,[selected_tasks(1) task_in_chromosome1]) = offspring1(:,[task_in_chromosome1 selected_tasks(1)]);
    offspring2(:,[selected_tasks(2) task_in_chromosome2]) = offspring2(:,[task_in_chromosome2 selected_tasks(2)]);
end
end
%----------------------------------------------------------------------------------
function mutated_chromosome = mutation(chromosome, mutation_rate, robot_list, num_genes)
% mutation - Performs mutation of a chromosome
% Inputs:
% - chromosome: chromosome parent
% - mutation_rate: probability of the chromosome being mutated
%
% Outputs:
% - offspring: mutated chromosome

% Create a copy of the input parent
mutated_chromosome = chromosome;

% Check if mutation should be performed based on the mutation rate
if rand() < mutation_rate
    % Choose a random gene to mutate
    gene_to_mutate = randi(num_genes);
    
    % Get the measure and the allocated robot to the chosen gene
    measure = chromosome(2,gene_to_mutate);
    assigned_robot = chromosome(3,gene_to_mutate);
    
    % Find other robots that can perform the measure
    other_robots = find(robot_list(measure,:)==1);
    other_robots(other_robots == assigned_robot) = []; % Remove current robot
    
    % Change the robot assigned to the chosen gene if other robots exist
    if ~isempty(other_robots)
        mutated_chromosome(3,gene_to_mutate) = other_robots(randi(length(other_robots)));
    end
    
end

end
%----------------------------------------------------------------------------------
function new_population = reproduction(population, population_size, crossover_rate, num_tasks)
% reproduction - Generates a new population by crossing pairs of parents
% Inputs:
% - population: cell array of parent chromosomes
% - num_children: number of children to generate
% - num_parents: number of parents in the population
% - crossover_rate: crossover probability
% - num_tasks: number of tasks
%
% Outputs:
% - new_population: cell array of child chromosomes

% Pre-allocate memory for the new population
new_population = cell(1,population_size);

% Iterate over pairs of generated children
for pair_idx = 1:2:population_size
    % Select two distinct parents randomly
    parent_indices = randperm(population_size, 2);
    parent1 = population{parent_indices(1)};
    parent2 = population{parent_indices(2)};
    
    % Cross over the selected parents
    [child1,child2] = crossover(parent1, parent2, crossover_rate, num_tasks);
    
    % Add the new children to the new population
    new_population{pair_idx} = child1;
    new_population{pair_idx+1} = child2;
end

end
%----------------------------------------------------------------------------------
function traceSequence(cost_matrix, sites_coordinates, maxX, maxY, sequence)
% traceSequence - traces a sequence of visits for a robot
% Inputs:
% - cost_matrix: a matrix of costs between task positions
% - sites_coordinates: X and Y coordinates of each sites
% - (maxX,maxY) : the size of the environment
% - sequence: the sequence of visits for a robot

%figure(num_figure);
hold on;
axis([0 maxX+1 0 maxY+1]); % Set the axis limits of the plot

% Set the axis limits of the plot
num_sites = numel(sites_coordinates);

% Extract the X and Y coordinates from the site coordinates
X = cellfun(@(xy) xy(1), sites_coordinates);
Y = cellfun(@(xy) xy(2), sites_coordinates);

% Plot the site coordinates as circles
plot(X,Y,'o','MarkerSize',5,'MarkerFaceColor',[0 0.7 0.7],'MarkerEdgeColor','b');

% Generate labels for each site
labels = [];
labels = arrayfun(@(i) sprintf('a%d', i), 1:num_sites, 'UniformOutput', false);

% Add the labels to the plot at the corresponding coordinates
text(X, Y, labels,'VerticalAlignment','bottom','HorizontalAlignment','right');

for site_idx = 2:numel(sequence)
    % Draw lines between consecutive sites in the sequence
    line([X(sequence(site_idx-1)) X(sequence(site_idx))], [Y(sequence(site_idx-1)) Y(sequence(site_idx))],'Color',[0.3 0.5 0.7],'LineWidth',1);
end

end
