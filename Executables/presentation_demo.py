"""
Demo File used in the Presentation
"""
import random
import time
import numpy as np
import loaders
import utils
import operators as ops
import solvers
from structure import Vehicle, ProblemInstance

kwargs = {
    'num_vehicles': 5,
    'capacity': 15,
    'graph_size': 250,
    'timeout': 60,

    'ordered_nbhs': [ops.inter_segment_swap, ops.intra_two_opt, ops.intra_segment_swap, ops.inter_two_opt, ],

    'large_nbhs': [5, 7, 9, 11, 13, 15],
    'large_timeout': 300,
}

# =========================================================================================== BUILDING THE GRAPH

graph, node_info = loaders.load_subset_from_ordered_nodes(nodes=kwargs['graph_size'], randomness=False)
vehicles = [Vehicle(capacity=kwargs['capacity'], vehicle_id=str(i)) for i in range(kwargs['num_vehicles'])]
problem = ProblemInstance(input_graph=graph, vehicles=vehicles, node_data=node_info, depot='0', verbose=0)

print("BUILDING THE GRAPH")
utils.visualize_routes_go(problem.get_all_routes(), node_info)

input("Press Enter to continue...")

# =========================================================================================== RANDOM ROUTES

print("RANDOM ROUTES")
solvers.random_routing(problem)
problem.calculate_loading_mf()
utils.visualize_routes_go(problem.get_all_routes(), node_info)
problem.display_results(show_instructions=False)

input("Press Enter to continue...")
problem.reset()

# =========================================================================================== GREEDY ROUTES

print("GREEDY ROUTES")
solvers.greedy_routing(problem, randomness=False)
problem.calculate_loading_mf()
utils.visualize_routes_go(problem.get_all_routes(), node_info)
problem.display_results()

input("Press Enter to continue...")

# =========================================================================================== VNS

print("RUNNING VNS")

random.seed(888)
np.random.seed(369)
start1 = time.time()
distance_hist, time_hist, operation_hist = solvers.general_variable_nbh_search(
    problem, kwargs['ordered_nbhs'], change_nbh=solvers.change_nbh_cyclic, timeout=kwargs['timeout'], verbose=1)
end1 = time.time()

problem.display_results()
utils.visualize_routes_go(problem.get_all_routes(), node_info)
utils.show_improvement_graph(distance_hist, time_hist, operation_hist, kwargs['ordered_nbhs'],
                             change_nbh_name='cyclic')
print("VNS Duration: {}s".format(round(end1 - start1, 3)))

input("Press Enter to continue...")

# ========================================================================================== LNS

print("RUNNING LNS")
start1 = time.time()
solvers.large_nbh_search(
    problem, ordered_local_nbhs=kwargs['ordered_nbhs'], ordered_large_nbhs=kwargs['large_nbhs'],
    timeout=kwargs['timeout'], local_verbose=0, large_verbose=1)
end1 = time.time()

problem.display_results()
utils.visualize_routes_go(problem.get_all_routes(), node_info)
print("LNS Duration: {}s".format(round(end1 - start1, 3)))
