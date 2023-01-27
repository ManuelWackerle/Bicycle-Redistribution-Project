"""
Use this file to run the standard VNS
"""
import time
from structure import ProblemInstance, Vehicle
from load_csv import load_subset_from_ordered_nodes
import utils
import vns


kwargs = {
    'nodes':                100,
    'num_vehicles':         5,
    'vehicle_capacity':     15,
    'vns_timeout':          60, #seconds
    'show_routes':          False,
    'ordered_nbhs': [vns.inter_segment_swap, vns.intra_segment_swap, vns.inter_two_opt, vns.intra_two_opt, vns.multi_remove_and_insert_station],
}



###_____________________________________________________________________________________________________________________
### Load a subset of the data from MVG dataset and create the Vehicles and Problem Instance objects


graph, node_info = load_subset_from_ordered_nodes(nodes=kwargs['nodes'], randomness=False)
vehicles = [Vehicle(capacity=kwargs['vehicle_capacity'], vehicle_id=str(i)) for i in range(kwargs['num_vehicles'])]
problem = ProblemInstance(input_graph=graph, vehicles=vehicles, node_data=node_info, verbose=0)


###_____________________________________________________________________________________________________________________
### Create and initial set of solutions using the greedy algorithm and calculate the loading instrunctions


vns.greedy_routing_v1(problem, randomness=False)
vns.calculate_loading_MF(problem)
print("\nInitial Solution using greedy alogrithm:")
problem.display_results(kwargs['show_routes'])


###_____________________________________________________________________________________________________________________
### Run the VNS and time it


start_time = time.time()

distance_hist, time_hist, operation_hist = vns.general_variable_nbh_search(
    problem, kwargs['ordered_nbhs'], change_nbh=vns.change_nbh_pipe, timeout=kwargs['vns_timeout'], verbose=0)

end_time = time.time()


###_____________________________________________________________________________________________________________________
### Show the final Results


time_run = round(end_time - start_time, 3)
time_out = 'Converged before timeout' if time_run < kwargs['vns_timeout'] else 'Stopped due to timeout'
print("\nSolution after applying VNS for {} seconds ({}):".format(time_run, time_out))
problem.display_results(kwargs['show_routes'])

# ### Plot basic routes
# utils.visualize_routes(problem.get_all_routes(), node_info)
#
# ### Plot routes in browser
# utils.visualize_routes_go(problem.get_all_routes(), node_info)
#
# ### Plot VNS improvement vs time graph
# utils.show_improvement_graph(distance_hist, time_hist, operation_hist, kwargs['ordered_nbhs'], change_nbh_name='Pipe')
#

###_____________________________________________________________________________________________________________________

