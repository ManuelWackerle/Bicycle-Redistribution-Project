import time
import utils
from load_csv import load_graph
from load_csv import load_subset_from_ordered_nodes
from structure import ProblemInstance, Vehicle
from copy import deepcopy
import vns
from tests.test_with_various_graphs import run_test

"""
Use this file to load, test and run different solution approaches on the data.
"""


if __name__ == '__main__':

    # #Copy this into main
    # num_vehicles = 5
    # capacity = 15
    # min_graph_size = 200
    # max_graph_size = 200
    # graph_size_step = 10
    # graph_variations = 1
    # trials_per_graph = 50
    # run_test(num_vehicles, capacity, min_graph_size, max_graph_size, graph_size_step, graph_variations, trials_per_graph)


    ##___________________________________________________________________ VARIABLE NEIGHBOURHOOD SEARCH APPROACH
    # Load Problem Instance
    graph, node_info = load_subset_from_ordered_nodes(nodes=250, centeredness=5)

    # Imput vehicle information.
    vehicles = []
    for i in range(5):
        vehicles.append(Vehicle(capacity=20, vehicle_id=str(i)))

    # Mount problem instance with and without zero demand nodes
    problem = ProblemInstance(input_graph=graph, vehicles=vehicles, node_data=node_info, verbose=0)
    problem_no_zeros = deepcopy(problem)
    problem_no_zeros.remove_nodes_zero_demand()


    # Check that the problems are different.
    assert problem.model != problem_no_zeros.model, "The graphs are the same"

    # Compute runtime for the graph without the removed 0 demand nodes
    start1 = time.time()
    vns.greedy_routing_v1(problem)

    routes = problem.get_all_routes()
    utils.visualize_routes_go(routes, node_info)


    ordered_nbhs = [vns.inter_two_opt, vns.intra_two_opt, vns.intra_or_opt, vns.remove_and_insert_station]
    distance_hist, time_hist, operation_hist = vns.general_variable_nbh_search(
        problem, ordered_nbhs, change_nbh=vns.change_nbh_sequential, timeout=100, verbose=1)
    utils.show_improvement_graph(distance_hist, time_hist, operation_hist, ordered_nbhs,
                                 change_nbh_name='skewed sequential')

    routes = problem.get_all_routes()
    utils.visualize_routes_go(routes, node_info)


