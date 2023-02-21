"""
Contains code in progress - MAY NOT WORK
This file is temporary and should be deleted when the project is complete
"""

import random
import time
import numpy as np

import load_csv
import utils
from Tests import collect_stats_lns, collect_stats_vns
import operators as ops
import vns
from structure import Vehicle, ProblemInstance

kwargs = {
    'num_vehicles':      5,
    'num_vehicles_max':  5,
    'num_vehicles_step': 1,
    'capacity':          15,
    'capacity_max':      15,
    'capacity_step':     2,
    'graph_size':        250,
    'graph_size_max':    250,
    'graph_size_step':   50,
    'graph_variations':  1,
    'trials_per_graph':  50,

    'timeout':           60,

    'ordered_nbhs': [ops.inter_segment_swap, ops.intra_two_opt, ops.intra_segment_swap, ops.inter_two_opt, ],
    'nbh_change_set': [vns.change_nbh_cyclic],
    # 'ordered_nbhs': [ops.intra_two_opt, ops.intra_segment_swap, ops.inter_two_opt, ops.inter_segment_swap, ops.multi_remove_and_insert_station_v2],
    # 'nbh_change_set': [vns.change_nbh_cyclic, vns.change_nbh_pipe, vns.change_nbh_sequential, vns.change_nbh_check_all],

    'large_nbhs': [0.1, 0.15, 0.20, 0.25, 0.30, 0.40],
    'large_timeout': 300,
}

np.random.seed(686)
random.seed(786)

#=========================================================================================== TEST 1

# print("TEST 1: Collecting Stats for varying vehicle numbers - VNS only")
# start = time.time()
# collect_stats_vns.run_test(kwargs)  # VNS Only
# stop = time.time()
# print("TEST 1 COMPLETE. (Runtime: {} minutes)\n".format(round((stop - start)/60, 1)))


#=========================================================================================== TEST 2

# print("TEST 2: Collecting Stats for varying capacities - VNS only")
# start = time.time()
# collect_stats_vns.run_test(kwargs)  # VNS Only
# stop = time.time()
# print("TEST 2 COMPLETE. (Runtime: {} minutes)\n".format(round((stop - start)/60, 1)))


#=========================================================================================== TEST 3

# print("TEST 3: Collecting Stats for different nbh change methods - VNS only")
# start = time.time()
# collect_stats_vns.run_test(kwargs)  # VNS Only
# stop = time.time()
# print("TEST 3 COMPLETE. (Runtime: {} minutes)\n".format(round((stop - start)/60, 1)))


#=========================================================================================== TEST 4

print("TEST 4: Collecting Stats for VNS vs LNS")
start = time.time()
collect_stats_lns.run_test(kwargs)  # VNS and LNS
stop = time.time()
print("TEST 4 COMPLETE. (Runtime: {} minutes)\n".format(round((stop - start)/60, 1)))

# ======================================================================================================================
# #      Vanilla VNS
#
# # graph, node_info = load_subset_from_ordered_nodes(nodes=kwargs['min_graph_size'], centeredness=6, randomness=False)
# # graph, node_info = load_graph('nyc_instance', location='nyc')
# graph, node_info, depot = load_csv.load_graph('nyc_instance_dummy', location='nyc_dummy')
#
#
# vehicles = [Vehicle(capacity=kwargs['capacity'], vehicle_id=str(i), distance_limit=999) for i in range(kwargs['num_vehicles'])]
# problem = ProblemInstance(input_graph=graph, vehicles=vehicles, node_data=node_info, depot=depot, verbose=1)
# start2 = time.time()
# vns.greedy_routing_v1(problem, randomness=True)
# # vns.random_routing(problem)
# problem.calculate_loading_MF()
# problem.remove_unused_stops()
# initial = problem.calculate_distances()
# problem.display_results()
# # visualize_routes(problem.get_all_routes(), node_info)
# utils.visualize_routes_go(problem.get_all_routes(), node_info)
#
# start1 = time.time()
# distance_hist, time_hist, operation_hist = vns.general_variable_nbh_search(
#     problem, kwargs['ordered_nbhs'], change_nbh=vns.change_nbh_cyclic, timeout=6000, verbose=1)
# end1 = time.time()
#
# distances = [problem.calculate_distance(v)/1000 for v in problem.vehicles]
# std = np.std(np.array(distances))
# distances = [round(d) for d in distances]
# final = problem.calculate_distances()
# impr = (1 - final / initial) * 100
# print("{:>4}, Improvement: {}%, {:>27}, std={}, time = {}s".format(round(final/1000), round(impr, 1),
#         str(distances), round(std, 3), round(end1 - start1, 3)))
#
# problem.display_results()
#
# # utils.visualize_routes(problem.get_all_routes(), node_info)
# utils.visualize_routes_go(problem.get_all_routes(), node_info)
# utils.show_improvement_graph(distance_hist, time_hist, operation_hist, kwargs['ordered_nbhs'],
#                              change_nbh_name='cyclic')
# end2 = time.time()
# # vns.remove_unused_stops(problem)
# # for v in problem.vehicles:
# # problem.plot_vehicle_route(problem.vehicles[0])
#
# print("Total Duration: {}s".format(round(end2 - start2, 3)))


# ======================================================================================================================




