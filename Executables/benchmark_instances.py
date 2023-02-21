import time
import utils

from load_csv import load_from_pickle
from load_csv import get_instances_names
from structure import ProblemInstance, Vehicle
from copy import deepcopy
import solvers
# from tests.test_with_various_graphs import run_test
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
import operators as ops

FOLDER_NAME = '../benchmarks_results'
os.makedirs(FOLDER_NAME, exist_ok=True)
instances_names = get_instances_names()

for instance_name in instances_names:
    print("Solving instance: " + instance_name.replace('.txt', '') + ".")

    graph, vehicle_capacity, vehicle_number = load_from_pickle(instance_name=instance_name, force_balance='dummy')
    print("Number of vehicles: " + str(vehicle_number))
    print("Vehicle capacity: " + str(vehicle_capacity))
    node_info = None

    kwargs = {
        'nodes': 40,
        'centeredness': 5,
        'number_of_vehicles': vehicle_number,
        'vehicle_capacity': vehicle_capacity,
        'ordered_nbhs': [ops.inter_two_opt, ops.intra_two_opt, ops.intra_or_opt, ops.destroy_local],
        'ordered_large_nbhs': [1, 3, 5, 8, 10],
        'local_timeout': 2 * 60,  # second
        'large_timeout': 60 * 60,  # second
        'show_instruction': False,
        'show_each_distance': True,
        'local_verbose': 1,
        'large_verbose': 1,
    }

    instance_size = graph.number_of_nodes()

    vehicles = []
    for i in range(kwargs["number_of_vehicles"]):
        vehicles.append(Vehicle(distance_limit = 100, capacity=kwargs["vehicle_capacity"], vehicle_id=str(i)))

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
    distance_hist_lns, time_hist_lns, operation_hist_lns, time_shake, shake_effect = vns.large_nbh_search(problem_copy,
                                                                                                          ordered_large_nbhs,
                                                                                                          ordered_nbhs,
                                                                                                          change_local_nbh=vns.change_nbh_sequential,
                                                                                                          change_large_nbh=solvers.change_nbh_pipe,
                                                                                                          large_nbh_operator=ops.destroy_rebuild,
                                                                                                          timeout=
                                                                                                          kwargs[
                                                                                                              "local_timeout"],
                                                                                                          large_timeout=
                                                                                                          kwargs[
                                                                                                              "large_timeout"],
                                                                                                          local_verbose=
                                                                                                          kwargs[
                                                                                                              "local_verbose"],
                                                                                                          large_verbose=
                                                                                                          kwargs[
                                                                                                              "large_verbose"]
                                                                                                          )
    problem.display_results()
    print('LNS:', time_hist_lns)
    """
    LNS with multiple_remove_insert
    """
    start = time.time()
    problem_copy = deepcopy(problem)
    distance_hist_lns_multi, time_hist_lns_multi, operation_hist_lns_multi, time_shake_multi, shake_effect_multi = vns.large_nbh_search(
        problem_copy,
        ordered_large_nbhs,
        ordered_nbhs,
        change_local_nbh=vns.change_nbh_sequential,
        change_large_nbh=solvers.change_nbh_pipe,
        large_nbh_operator=ops.multi_remove_and_insert_station,
        timeout=kwargs["local_timeout"],
        large_timeout=kwargs["large_timeout"],
        local_verbose=kwargs["local_verbose"],
        large_verbose=kwargs["large_verbose"]
    )
    print('LNS multi:', time_hist_lns_multi)
    print("*** Final result using LNS ***")
    print(f"Time taken {(time.time() - start) / 60} [m]")
    print("Reduction: ", (-problem_copy.calculate_distances() + initial_dist) / initial_dist * 100, "%")
    problem.display_results(kwargs["show_instruction"])
    if kwargs["show_each_distance"]:
        for i, vehicle in enumerate(problem_copy.vehicles):
            print(f"Vehicle {i} has distance {round(problem_copy.calculate_distance(vehicle) / 1000, 5)} km")

    """
    VNS
    """
    start_time = time.time()
    problem_copy = deepcopy(problem)
    distance_hist, time_hist, operation_hist = vns.general_variable_nbh_search(problem_copy,
                                                                               ordered_nbhs,
                                                                               change_nbh=solvers.change_nbh_sequential,
                                                                               verbose=kwargs["local_verbose"],
                                                                               timeout=kwargs["local_timeout"]
                                                                               )
    time_hist = [element - start_time for element in time_hist]
    print('VNS:', time_hist)
    print("*** Final Result without LNS ***")
    print(f"Time taken: {(time.time() - start_time) / 60} [m]")
    print("Reduction: ", (-problem_copy.calculate_distances() + initial_dist) / initial_dist * 100, "%")
    problem_copy.display_results(kwargs["show_instruction"])
    if kwargs["show_each_distance"]:
        for i, vehicle in enumerate(problem_copy.vehicles):
            print(f"Vehicle {i} has distance {round(problem_copy.calculate_distance(vehicle) / 1000, 5)} km")

    plt.plot([x for x in time_hist], [x / 1000 for x in distance_hist], color='black', label="bare-vns", zorder=-1,
             lw=3)
    plt.plot([x for x in time_hist_lns], [x / 1000 for x in distance_hist_lns], color='red',
             label="lns-vns-destroy")
    plt.plot([x for x in time_hist_lns_multi], [x / 1000 for x in distance_hist_lns_multi], color='blue',
             label="lns-vns-multi", lw=1)


    plt.xlabel("Time (s)")
    plt.ylabel("Distance (km)")
    plt.legend()

    plt.savefig(os.path.join(FOLDER_NAME, instance_name.replace('txt', 'png')))
    plt.clf()
