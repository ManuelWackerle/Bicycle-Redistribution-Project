"""
This test can be used to compare the performance of LNS after applying it to the VNS results.
"""

import Tests.loops_for_testing as test
import operators as ops
import solvers as vns
import time


kwargs = {
    'num_vehicles':      5,
    'num_vehicles_max':  5,
    'num_vehicles_step': 1,
    'capacity':          15,
    'capacity_max':      15,
    'capacity_step':     2,
    'graph_size':        100,
    'graph_size_max':    300,
    'graph_size_step':   100,
    'graph_variations':  1,
    'trials_per_graph':  30,
    'timeout':           60,
    'ordered_nbhs': [ops.intra_two_opt, ops.intra_segment_swap, ops.inter_two_opt, ops.inter_segment_swap, ],
    'nbh_change_set': [vns.change_nbh_cyclic],
    'large_nbhs': [0.1, 0.15, 0.20, 0.30],
    'large_timeout': 300,
}


print("Collecting Stats for VNS vs LNS")
start = time.time()
test.test_loop_vns_lns(kwargs)
stop = time.time()
print("TEST COMPLETE. (Runtime: {} minutes)\n".format(round((stop - start)/60, 1)))


# ======================================================================================================================
# #      Vanilla VNS
#
# # graph, node_info = load_subset_from_ordered_nodes(nodes=kwargs['min_graph_size'], centeredness=6, randomness=False)
# # graph, node_info = load_graph('nyc_instance', location='nyc')
# graph, node_info, depot = loaders.load_graph('nyc_instance_dummy', location='nyc_dummy')
#
#
# vehicles = [Vehicle(capacity=kwargs['capacity'], vehicle_id=str(i), distance_limit=999) for i in range(kwargs['num_vehicles'])]
# problem = ProblemInstance(input_graph=graph, vehicles=vehicles, node_data=node_info, depot=depot, verbose=1)
# start2 = time.time()
# vns.greedy_routing(problem, randomness=True)
# # vns.random_routing(problem)
# problem.calculate_loading_mf()
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

