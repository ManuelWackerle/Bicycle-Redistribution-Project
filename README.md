# Bicycle Redistribution Project
Case Studies, Project 3. Bicycle Redistribution Project

## How to use the Code
### Structure

#### Main Files
- `loaders` contains functions to read in data from an external source.
- `structure` contains the classes used to model and store information about a given instance of the problem.
- `solvers` contains methods use for producing initial solutions as well as for the VNS and LNS.
- `operators` is a collection of permutation operators used in the VNS to search neighbourhoods in the solution space.
- `utils` contains auxiliary methods used throughout the project such as plotters.
#### Folders
- `Executables` all the executable files including tests
- `Saved` all the saved output of plots, tests and statistics
- `MVG Code` the code for processing the MVG data 
- `Problem Instances` a collection of csv and pickle data from different data sets
#### Dependencies
| Package      | Version   |
|--------------|-----------|
| networkx     | 2.8.8     |
| numpy        | 1.21.4    |
| scipy        | 1.7.2     |
| matplotlib   | 3.5.0     |
| plotly       | 5.11.0    |
| pickleshare  | 0.7.5     |
| tqdm         | 4.64.1    |
| haversine    | 2.7.0     |


### Loading data and creating an instance of the problem
The code is written to work with input in the form of node information (stations or bins) as well as edge or distance information
- csv with node information must be in the form:  _number, id, delta, x_coord, y_coord_.
- cvs with the adjacency matrix must be in the form of a matrix with all the pairwise distances between nodes

These files can be read in by calling the load method from the loaders file. 
The Vehicle and ProblemInstance class for the structure file can then be used to create an instance of the problem. For example:
```
graph, node_info, depot = load_graph('nyc_instance', location='nyc')
vehicles = [Vehicle(capacity=15, vehicle_id=str(i)) for i in 5)]
problem = ProblemInstance(input_graph=graph, vehicles=vehicles, node_data=node_info, depot=depot, verbose=1)
```

### Finding solutions
To find an initial feasible solution use greedy (or random) method. This will set a sequence of stops for each vehicle. 
Then to determine the loading instructions on a given route a maximum flow computation is used.
The solution can then be improved by running VNS (and LNS) to look for local improvements to the routes. 
For the VNS select the neighbourhood operators and neighbourhood change method to use, for example:
```
solvers.greedy_routing(problem, randomness=False)
problem.calculate_loading_mf()

operators = [ops.inter_segment_swap, ops.intra_two_opt, ops.intra_segment_swap, ops.inter_two_opt]
change_method = slovers.change_nbh_cyclic
solvers.general_variable_nbh_search(problem, operators, change_method, timeout=120, verbose=1)
```
### Visualizing results
The utils folder contains two methods to visualize a set of routes, or, to print the list directly call `display_results()` from a problem instance.
```
utils.visualize_routes(problem.get_all_routes(), node_info)
utils.visualize_routes_go(problem.get_all_routes(), node_info)
problem.display_results(show_instructions=True)
```

### Running tests
The `Executables` folder contains a small set of example workflows as well as a set of tests with different arguments
The `Tests` folder contains the code for running the test that were used to measure and compare different performance indicators


## Mathematical Model
### Model and data assumptions:  
•	Static variant of the problem, where user activities are neglected during rebalancing.  
•	The entire domain is divided into subregions, and one station is assigned to each one of them.  
•	Effort for distributing bikes inside a subregion is not considered.  
•	The system of stations and routes are considered as a complete directed graph.  
•	Operational costs consist only of travel distances between stations, these are taken from a routing API.   

### Vehicle assumptions:  
•	All vehicles are homogeneous, i.e. all of them have the same load capacity, which can never be exceeded.  
•	All the vehicles start and finish from the single depot.  
•	Each vehicle is empty at the start of a trip and must be empty at the moment of arrival back to the depot.  
The assumptions are not crucial and can be easily omitted, but are used in the current version of the code

### Routing assumptions:  
•	Stations may be visited multiple times by the same or different vehicles.  
•	Monotonicity regarding pickup and delivery operations. A vehicle is only allowed to load bicycles at load stations and pickup bicycles from pickup stations. In other words, the number of bikes at each station is allowed to only decrease or only increase.  
