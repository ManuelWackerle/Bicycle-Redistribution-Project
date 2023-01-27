"""
Contains code in progress - MAY NOT WORK
This file is temporary and should be deleted when the project is complete
"""

import random
import time
from structure import ProblemInstance, Vehicle
from load_csv import load_subset_from_ordered_nodes
from copy import deepcopy
import numpy as np
import vns
import operators as ops

kwargs = {
    'nodes':                42,
    'centeredness':         5,
    'number_of_vehicles':   5,
    'vehicle_capacity':     20,
    'ordered_large_nbhs':   [1, 3, 5, 8, 10],
    'local_timeout':        2*60,   # seconds
    'large_timeout':        60*60,  # seconds
    'show_instruction':     False,
    'show_each_distance':   True,
    'local_verbose':        1,
    'large_verbose':        1,
    'ordered_nbhs': [ops.inter_segment_swap_fast, ops.intra_segment_swap_fast, ops.inter_two_opt_fast, ops.intra_two_opt_fast], # vns.multi_remove_and_insert_station],
}


np.random.seed(6134)
random.seed(7381)



if __name__ == '__main__':
    graph, node_info = load_subset_from_ordered_nodes(nodes=kwargs['nodes'], centeredness=kwargs["centeredness"], randomness=False)
    # graph, node_info = load_graph('nyc_instance', location='nyc')
    # graph, node_info = load_graph('nyc_instance_dummy', location='nyc_dummy')


    vehicles = [Vehicle(capacity=kwargs['vehicle_capacity'], vehicle_id=str(i)) for i in range(kwargs['number_of_vehicles'])]
    problem = ProblemInstance(input_graph=graph, vehicles=vehicles, node_data=node_info, verbose=1)

    vns.greedy_routing_v1(problem, randomness=True)
    problem.calculate_loading_MF()
    problem.remove_unused_stops()
    problem.display_results()
    # visualize_routes(problem.get_all_routes(), node_info)
    # utils.visualize_routes_go(problem.get_all_routes(), node_info)
    problem2 = deepcopy(problem)
    start1 = time.time()

    distance_hist, time_hist, operation_hist = vns.general_variable_nbh_search(
        problem, kwargs['ordered_nbhs'], change_nbh=vns.change_nbh_pipe, timeout=6000, verbose=1)
    end1 = time.time()

    problem.display_results(True)


    start2 = time.time()
    # utils.visualize_routes(problem.get_all_routes(), node_info)
    # utils.visualize_routes_go(problem.get_all_routes(), node_info)
    # utils.show_improvement_graph(distance_hist, time_hist, operation_hist, kwargs['ordered_nbhs'],
    #                              change_nbh_name='skewed sequential')
    end2 = time.time()
    # vns.remove_unused_stops(problem)
    # for v in problem.vehicles:
    # problem.plot_vehicle_route(problem.vehicles[0])

    # problem.display_results()
    print("Duration: {}s vs {}s".format(round(end1 - start1, 3), round(end2 - start2, 3)))


