import time
import utils
from load_csv import load_graph
from structure import ProblemInstance, Vehicle
from copy import deepcopy
import vns

"""
Use this file to load, test and run different solution approaches on the data.
"""


if __name__ == '__main__':
    ##___________________________________________________________________ VARIABLE NEIGHBOURHOOD SEARCH APPROACH
    # Load Problem Instance
    graph, node_info = load_graph('sample_graph_03')
    # graph, node_info = load_graph('sample_graph_06')

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
    # problem.display_results(False)

    ordered_nbhs = [vns.inter_two_opt, vns.intra_two_opt, vns.remove_and_insert_station]
    distance_hist, time_hist, operation_hist = vns.general_variable_nbh_search(
        problem, ordered_nbhs, change_nbh=vns.change_nbh_skewed_sequential,
        skew_param=100, timeout=30, verbose=1)

    # problem.display_results()
    utils.show_improvement_graph(distance_hist, time_hist, operation_hist, ordered_nbhs, change_nbh_name='skewed sequential')

    # end1 = time.time()
    #
    # # print(bcolors.OKGREEN + "Found route with length {}m".format(route_cost) + bcolors.ENDC)
    # print("VNS runtime with nodes of zero demand: %.4f s" % (end1 - start1))
    #
    # # Compute runtime for the instance with inactive 0 demand nodes
    # start2 = time.time()
    #
    # vns.greedy_routing_v1(problem_no_zeros)
    # # problem_no_zeros.display_results(False)
    #
    # ordered_nbhs = [vns.inter_two_opt, vns.intra_two_opt]
    # vns.general_variable_nbh_search(problem_no_zeros, ordered_nbhs, timeout=120)
    # # problem_no_zeros.display_results()
    #
    # end2 = time.time()

    # print(bcolors.OKGREEN + "Found route with length {}m".format(route_cost) + bcolors.ENDC)
    # print("VNS runtime with no zero-demand nodes: %.4f s" % (end2 - start2))

    # Print comparisson
    # print("Removing the nodes with zero demands speeds up the process by: %.4f percent" % (
    #     (1 - (end2 - start2) / (end1 - start1))*100))

    ##==========================================================================================================





    # Visualize - use only for very small graphs
    # display_graph(graph, node_info)
