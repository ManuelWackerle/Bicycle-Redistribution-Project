"""
Use this file to run a standard VNS
"""
import random

from loaders import load_subset_from_ordered_nodes
from structure import ProblemInstance, Vehicle
import utils
import solvers
import operators as ops
import time


kwargs = {
    'nodes':                100,
    'num_vehicles':         5,
    'vehicle_capacity':     15,
    'vns_timeout':          60,  # seconds
}


# Load a subset of the data from MVG dataset and create the Vehicles and Problem Instance objects
graph, node_info = load_subset_from_ordered_nodes(nodes=kwargs['nodes'], cost='time', randomness=False)

vehicles = [Vehicle(capacity=kwargs['vehicle_capacity'], vehicle_id=str(i),
                    depot=str(random.randint(0, kwargs['nodes'] - 1))) for i in range(kwargs['num_vehicles'])]
problem = ProblemInstance(input_graph=graph, vehicles=vehicles, node_data=node_info, verbose=1)


# Create and initial set of solutions using the greedy algorithm and calculate the loading instrunctions
solvers.greedy_routing(problem, randomness=False)
problem.calculate_loading_mf()
print("\nInitial Solution using greedy alogrithm:")
problem.display_results(show_instructions=False)


# Run the VNS and time it
# operator_seq = [ops.inter_segment_swap, ops.intra_segment_swap, ops.inter_two_opt, ops.intra_two_opt]
operator_seq = [ops.inter_segment_swap, ops.intra_segment_swap, ops.intra_two_opt]
start_time = time.time()

distance_hist, time_hist, operation_hist = solvers.general_variable_nbh_search(
    problem, operator_seq, change_nbh=solvers.change_nbh_pipe, timeout=kwargs['vns_timeout'], verbose=0)
problem.calculate_loading_mf()

end_time = time.time()


# Show the final Results
time_run = round(end_time - start_time, 3)
time_out = 'Converged before timeout' if time_run < kwargs['vns_timeout'] else 'Stopped due to timeout'
print("\nSolution after applying VNS for {} seconds ({}):".format(time_run, time_out))
problem.display_results(show_instructions=True)


# Plot basic routes
utils.visualize_routes(problem.get_all_routes(), node_info)

# Plot routes in browser
utils.visualize_routes_go(problem.get_all_routes(), node_info)

# Plot VNS improvement vs time graph
utils.show_improvement_graph(distance_hist, time_hist, operation_hist, operator_seq, change_nbh_name='Pipe')
