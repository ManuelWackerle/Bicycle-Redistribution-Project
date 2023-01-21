import time
import utils
from load_csv import load_graph
from load_csv import load_subset_from_ordered_nodes
from structure import ProblemInstance, Vehicle
from copy import deepcopy
import vns
from tests.test_with_various_graphs import run_test

import time
import utils
from load_csv import load_graph
from structure import ProblemInstance, Vehicle
from load_csv import load_subset_from_ordered_nodes
from copy import deepcopy
import os
import numpy as np
import vns
from matplotlib import pyplot as plt

"""
Use this file to load, test and run different solution approaches on the data.
"""
instances_dir = os.path.relpath('..\\..\\Problem Instances', os.path.dirname(os.path.abspath(os.getcwd())))
instance = "sample_graph_03.csv"
graph, node_info = load_subset_from_ordered_nodes(nodes=250, centeredness=5)
# graph, node_info = load_graph(os.path.splitext(instance)[0], path=instances_dir, use_adjacency_matrix=False)
instance_size = graph.number_of_nodes()

# Input vehicle information.
vehicles = []
for i in range(5):
    vehicles.append(Vehicle(capacity=20, vehicle_id=str(i)))

# Mount problem instance with and without zero demand nodes
problem = ProblemInstance(input_graph=graph, vehicles=vehicles, node_data=node_info, verbose=0)

# Solve the problem
vns.greedy_routing_v1(problem)
# problem_copy = deepcopy(problem)
initial_dist = problem.calculate_distances()

# Define local neighbours
ordered_nbhs = [vns.inter_two_opt, vns.intra_two_opt, vns.intra_or_opt, vns.multi_remove_and_insert_station]
# destruction_degrees = [0.05, 0.07, 0.10, 0.13]
# ordered_large_nbhs = [int(np.floor(instance_size * element)) for element in destruction_degrees]
ordered_large_nbhs = [1, 3, 5, 8, 10, 13, 15, 17, 20, 23, 25, 27, 30, 35, 40, 50]

# print("*** Showing improvement with LNS***")
# start = time.time()
#
# problem_copy = deepcopy(problem)
# distance_hist_lns, time_hist_lns, operation_hist_lns, time_shake, shake_effect = vns.large_nbh_search(problem_copy,
#                                                                                                       ordered_large_nbhs,
#                                                                                                       ordered_nbhs,
#                                                                                                       change_local_nbh=vns.change_nbh_sequential,
#                                                                                                       change_large_nbh=vns.change_nbh_pipe,
#                                                                                                       large_nbh_operator=vns.destroy_rebuild,
#                                                                                                       timeout=100,
#                                                                                                       large_verbose=0
#                                                                                                       )
# print("*** Final result using LNS ***")
# print("Time taken", time.time() - start)
# print("Reduction: ", (-problem.calculate_distances() + initial_dist) / initial_dist * 100, "%")
# problem.display_results()

print("*** Showing improvement with LNS multi_insert_remove***")
start = time.time()

problem_copy = deepcopy(problem)
distance_hist_lns_multi, time_hist_lns_multi, operation_hist_lns_multi, time_shake_multi, shake_effect_multi = vns.large_nbh_search(problem_copy,
                                                                                                      ordered_large_nbhs,
                                                                                                      ordered_nbhs,
                                                                                                      change_local_nbh=vns.change_nbh_sequential,
                                                                                                      change_large_nbh=vns.change_nbh_pipe,
                                                                                                      large_nbh_operator=vns.multi_remove_and_insert_station,
                                                                                                      timeout=2*60,
                                                                                                      large_timeout=15*60,

                                                                                                      large_verbose=0
                                                                                                      )
print("*** Final result using LNS ***")
print("Time taken", time.time() - start)
print("Reduction: ", (-problem.calculate_distances() + initial_dist) / initial_dist * 100, "%")
problem.display_results()

print("*** Showing improvement with VNS ***")
start_time = time.time()

problem_copy = deepcopy(problem)
distance_hist, time_hist, operation_hist = vns.general_variable_nbh_search(problem_copy,
                                                                           ordered_nbhs,
                                                                           change_nbh=vns.change_nbh_sequential,
                                                                           verbose=0, timeout=15*60
                                                                           )
time_hist = [element - start_time for element in time_hist]
print("*** Final Result without LNS ***")
print("Time taken: ", time.time() - start_time)
print("Reduction: ", (-problem_copy.calculate_distances() + initial_dist) / initial_dist * 100, "%")
problem_copy.display_results()
plt.plot(time_hist, distance_hist, color='blue', label="bare-vns", zorder=-1)
# plt.plot(time_hist_lns, distance_hist_lns, color='red', label="lns-vns-destroy")
plt.plot(time_hist_lns_multi, distance_hist_lns_multi, color='purple', label="lns-vns-multi")

# for i, time in enumerate(time_shake):
#     plt.axvline(time, ls="dashed", color='red')
#     plt.text(time, initial_dist, "LN changes? %s" % (not shake_effect[i]), rotation=-90,
#              verticalalignment='top', fontsize=12)

for i, time in enumerate(time_shake_multi):
    plt.axvline(time, ls="dashed", color='purple')
    plt.text(time, initial_dist, "LN changes? %s" % (not shake_effect_multi[i]), rotation=-90,
             verticalalignment='top', fontsize=12)

plt.xlabel("Time (s)")
plt.ylabel("Distance (m)")
plt.legend()
plt.show()
