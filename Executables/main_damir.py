"""
Contains code in progress - MAY NOT WORK
This file is temporary and should be deleted when the project is complete
"""
import time
import utils
from load_csv import load_graph
from load_csv import load_subset_from_ordered_nodes
from load_csv import load_from_pickle
from structure import ProblemInstance, Vehicle
from copy import deepcopy
import solvers
import operators as ops
from Tests.test_with_various_graphs import run_test

import time
import utils
from load_csv import load_graph
from structure import ProblemInstance, Vehicle
from load_csv import load_subset_from_ordered_nodes
from copy import deepcopy
import os
import numpy as np
import solvers
from matplotlib import pyplot as plt

kwargs = {
    'nodes': 40,
    'centeredness': 5,
    'number_of_vehicles': 41,
    'vehicle_capacity': 20,
    'ordered_nbhs': [ops.inter_two_opt, ops.intra_two_opt, ops.intra_or_opt, solvers.multi_remove_and_insert_station],
    'ordered_large_nbhs': [1, 3, 5, 8, 10],
    'local_timeout': 2*60,  # second
    'large_timeout': 60*60,  # second
    'show_instruction': False,
    'show_each_distance': True,
    'local_verbose': 0,
    'large_verbose': 1,
}

"""
Use this file to load, test and run different solution approaches on the data.
"""
# instances_dir = os.path.relpath('..\\..\\Problem Instances', os.path.dirname(os.path.abspath(os.getcwd())))
# instance = "sample_graph_03.csv"
# graph, node_info = load_subset_from_ordered_nodes(nodes=kwargs['nodes'], centeredness=kwargs["centeredness"])
graph, vehicle_capacity, vehicle_number = load_from_pickle(instance_name='37Guadalajara20.txt', force_balance='dummy')
node_info = None
# graph, node_info = load_graph(os.path.splitext(instance)[0], path=instances_dir, use_adjacency_matrix=False)
instance_size = graph.number_of_nodes()

# Input vehicle information.
vehicles = []
for i in range(kwargs["number_of_vehicles"]):
    vehicles.append(Vehicle(capacity=kwargs["vehicle_capacity"], vehicle_id=str(i)))

# Mount problem instance with and without zero demand nodes
problem = ProblemInstance(input_graph=graph, vehicles=vehicles, node_data=node_info, verbose=0)

solvers.greedy_routing_v1(problem)
initial_dist = problem.calculate_distances()

# Define local neighbours
ordered_nbhs = kwargs["ordered_nbhs"]
# destruction_degrees = [0.05, 0.07, 0.10, 0.13]
# ordered_large_nbhs = [int(np.floor(instance_size * element)) for element in destruction_degrees]
ordered_large_nbhs = kwargs["ordered_large_nbhs"]

"""
LNS with destroy_rebuild
"""
start = time.time()
problem_copy = deepcopy(problem)
distance_hist_lns, time_hist_lns, operation_hist_lns, time_shake, shake_effect = solvers.large_nbh_search(problem_copy,
                                                                                                          ordered_large_nbhs,
                                                                                                          ordered_nbhs,
                                                                                                          change_local_nbh=solvers.change_nbh_sequential,
                                                                                                          change_large_nbh=solvers.change_nbh_pipe,
                                                                                                          large_nbh_operator=solvers.destroy_rebuild,
                                                                                                          timeout=kwargs["local_timeout"],
                                                                                                          large_timeout=kwargs["large_timeout"],
                                                                                                          local_verbose=kwargs["local_verbose"],
                                                                                                          large_verbose=kwargs["large_verbose"]
                                                                                                          )
problem.display_results()



"""
LNS with multiple_remove_insert
"""
start = time.time()
problem_copy = deepcopy(problem)
distance_hist_lns_multi, time_hist_lns_multi, operation_hist_lns_multi, time_shake_multi, shake_effect_multi = solvers.large_nbh_search(problem_copy,
                                                                                                                                        ordered_large_nbhs,
                                                                                                                                        ordered_nbhs,
                                                                                                                                        change_local_nbh=solvers.change_nbh_sequential,
                                                                                                                                        change_large_nbh=solvers.change_nbh_sequential,
                                                                                                                                        large_nbh_operator=solvers.multi_remove_and_insert_station,
                                                                                                                                        timeout=kwargs["local_timeout"],
                                                                                                                                        large_timeout=kwargs["large_timeout"],
                                                                                                                                        local_verbose=kwargs["local_verbose"],
                                                                                                                                        large_verbose=kwargs["large_verbose"]
                                                                                                                                        )
print("*** Final result using LNS ***")
print(f"Time taken {(time.time() - start)/60} [m]")
print("Reduction: ", (-problem_copy.calculate_distances() + initial_dist) / initial_dist * 100, "%")
problem.display_results(kwargs["show_instruction"])
if kwargs["show_each_distance"]:
    for i, vehicle in enumerate(problem_copy.vehicles):
        print(f"Vehicle {i} has distance {round(problem_copy.calculate_distance(vehicle)/1000, 5)} km")

"""
VNS
"""
start_time = time.time()
problem_copy = deepcopy(problem)
distance_hist, time_hist, operation_hist = solvers.general_variable_nbh_search(problem_copy,
                                                                               ordered_nbhs,
                                                                               change_nbh=solvers.change_nbh_sequential,
                                                                               verbose=kwargs["local_verbose"], timeout=kwargs["local_timeout"]
                                                                               )
time_hist = [element - start_time for element in time_hist]
print("*** Final Result without LNS ***")
print(f"Time taken: {(time.time() - start_time)/60} [m]")
print("Reduction: ", (-problem_copy.calculate_distances() + initial_dist) / initial_dist * 100, "%")
problem_copy.display_results(kwargs["show_instruction"])
if kwargs["show_each_distance"]:
    for i, vehicle in enumerate(problem_copy.vehicles):
        print(f"Vehicle {i} has distance {round(problem_copy.calculate_distance(vehicle)/1000, 5)} km")


plt.plot([x/60 for x in time_hist], [x / 1000 for x in distance_hist], color='black', label="bare-vns", zorder=-1, lw=3)
plt.plot([x/60 for x in time_hist_lns], [x/1000 for x in distance_hist_lns], color='red', label="lns-vns-destroy")
plt.plot([x/60 for x in time_hist_lns_multi], [x / 1000 for x in distance_hist_lns_multi], color='blue', label="lns-vns-multi", lw=1)

# for i, time in enumerate(time_shake):
#     plt.axvline(time, ls="dashed", color='red')
#     plt.text(time, initial_dist, "LN changes? %s" % (not shake_effect[i]), rotation=-90,
#              verticalalignment='top', fontsize=12)
# for i, time in enumerate(time_shake_multi):
#     plt.axvline(time, ls="dashed", color='purple')
#     plt.text(time, initial_dist, "LN changes? %s" % (not shake_effect_multi[i]), rotation=-90,
#              verticalalignment='top', fontsize=12)

plt.xlabel("Time (m)")
plt.ylabel("Distance (km)")
plt.legend()
plt.show()
