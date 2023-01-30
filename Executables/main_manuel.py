"""
Contains code in progress - MAY NOT WORK
This file is temporary and should be deleted when the project is complete
"""

import random
import time
from structure import ProblemInstance, Vehicle
from load_csv import load_subset_from_ordered_nodes, load_graph
import numpy as np
from Tests import test_with_various_graphs
import operators as ops
import vns

kwargs = {
    'num_vehicles':      5,
    'capacity':          15,
    'min_graph_size':    550,
    'max_graph_size':    600,
    'graph_size_step':   100,
    'graph_variations':  1,
    'trials_per_graph':  20,
    'ordered_nbhs': [ops.intra_two_opt_fast, ops.intra_segment_swap_fast, ops.inter_two_opt_fast, ops.inter_segment_swap_fast], # vns.remove_and_insert_station]
    'nbh_change_set': [vns.change_nbh_cyclic, vns.change_nbh_pipe, vns.change_nbh_sequential, vns.change_nbh_check_all],
}


np.random.seed(6134)
random.seed(7381)



if __name__ == '__main__':

    

    # test_with_various_graphs.run_test(kwargs)


# ======================================================================================================================
#      Vanilla VNS

    # graph, node_info = load_subset_from_ordered_nodes(nodes=kwargs['nodes'], centeredness=kwargs["centeredness"], randomness=False)
    # graph, node_info = load_graph('nyc_instance', location='nyc')
    graph, node_info = load_graph('nyc_instance_dummy', location='nyc_dummy')


    vehicles = [Vehicle(capacity=kwargs['capacity'], vehicle_id=str(i)) for i in range(kwargs['num_vehicles'])]
    problem = ProblemInstance(input_graph=graph, vehicles=vehicles, node_data=node_info, verbose=1)


    vns.greedy_routing_v1(problem, randomness=True)
    problem.calculate_loading_MF()
    problem.remove_unused_stops()
    problem.display_results()
    # visualize_routes(problem.get_all_routes(), node_info)
    # utils.visualize_routes_go(problem.get_all_routes(), node_info)

    start1 = time.time()

    distance_hist, time_hist, operation_hist = vns.general_variable_nbh_search(
        problem, kwargs['ordered_nbhs'], change_nbh=vns.change_nbh_check_all, timeout=6000, verbose=1)
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

    problem.display_results()
    print("Duration: {}s vs {}s".format(round(end1 - start1, 3), round(end2 - start2, 3)))

# ======================================================================================================================




