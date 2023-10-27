# Multi-Robot Genetic Task Planner for Inspection and Monitoring

----------------------------------------------------------------------------------

This MATLAB project is designed to simulate a multi-robot mission for inspection and monitoring. The project consists of considering a multiple mobile robots mission where each robot is equipped with specific sensors to perform measurements that are spatially distributed in different positions of an industrial environment. In this README, we'll provide instructions on how to use the project and perform mission simulations.

# Getting Started

**To get started with the mission simulation, follow the steps below:**

1) Clone or download this Git repository to your local machine.
2) Open MATLAB on your computer.
3) Set the MATLAB current directory to the root folder of this project.
4) Ensure that you have the following MATLAB functions in your workspace:
   
	* binarize_aerial_image
	* compute_cost_matrix
	* create_graph
	* genetic_task_planner
	* trajectories_planner
	* mission_planner_simulator
	* Load a scenario by using the load function in MATLAB. Replace "scenario" with the name of the scenario you want to use:

```bash
load('scenarios/scenario');
```

Launch the mission planner simulator using the following command:

```bash
mission_planner_simulator(scenario);
```

The mission_planner_simulator function will use the loaded scenario to perform a mission simulation and display the results.

# Functions Description

**Here is a brief description of the provided functions:**

* binarize_aerial_image: Function for binarizing an aerial image for mission planning.
* compute_cost_matrix: Calculates the cost matrix for path planning.
* create_graph: Creates a graph for path planning and optimization.
* genetic_task_planner: Implements a genetic algorithm to plan tasks for the mission.
* trajectories_planner: Plans trajectories for the mission using optimization.
* mission_planner_simulator: The main function to launch the mission simulation based on a loaded scenario.


# Data Files

**The project includes two data files:**

* images: Contains aerial images for mission planning.
* scenarios: Directory where you can store scenario files for different mission simulations (each scenario file in the 'scenarios' directory defines the parameters and setup for a specific mission simulation).

You can create your own scenarios and customize the mission parameters by creating scenario files in the 'scenarios' directory. These scenario files should define the necessary variables and settings for the specific mission you want to simulate.

## Reference

1. H. Chakraa, E. Leclercq, F. Guérin and D. Lefebvre, "A Centralized Task Allocation Algorithm for a Multi-Robot Inspection Mission With Sensing Specifications," in IEEE Access, vol. 11, pp. 99935-99949, 2023, doi: 10.1109/ACCESS.2023.3315130.

