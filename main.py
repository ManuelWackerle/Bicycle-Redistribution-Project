import time

import networkx as nx

import utils
from load_csv import load_graph
from structure import ProblemInstance, Vehicle
import vns

"""
Use this file to load, test and run different solution approaches on the data.
"""


if __name__ == '__main__':


    ##___________________________________________________________________ MINIMUM COST FLOW APPROACH
    # # Load graph
    # graph, node_info = load_graph('sample_graph_01',4) #only load first 4 rows
    #
    # # Load solver
    # mcf = MinimumCostFlow(graph, num_vehicles=2, vehicle_capacity=5, verbose=2)
    # start1 = time.time()
    #
    # flow_dict = mcf.min_cost_flow_INCORRECT()
    # mcf.remodel_flow_into_tsp(flow_dict)
    # _, route_cost = mcf.tsp_solver()
    #
    # print(bcolors.OKGREEN + "Found route with length {}m".format(route_cost) + bcolors.ENDC)
    # print("Algorithm duration: %.4f s"%(end1 - start1))
    #
    # end1 = time.time()
    ##==============================================================================================


    ##___________________________________________________________________ VARIABLE NEIGHBOURHOOD SEARCH APPROACH
    # Load Problem Instance
    graph, node_info = load_graph('sample_graph_04')
    # graph, node_info = load_graph('sample_graph_06')
    vehicles = []
    for i in range(5):
        vehicles.append(Vehicle(capacity=20, vehicle_id=str(i)))

    problem = ProblemInstance(input_graph=graph, vehicles=vehicles, node_data=node_info, verbose=0)

    start1 = time.time()
    ###__
    # for i in range(16, 17, 4):
    #     print("Searching instance {}".format(int((i - 4) / 4)))
    #     vns.greedy_routing_v1(problem, dist_weight=i / 10)
    #     vns.calculate_loading_MF(problem)
    #     problem.display_results(True)
    #
    #     # old_dist = 0
    #     # dist = problem.calculate_distances()
    #     # while old_dist != dist:
    #     #     old_dist = dist
    #     #     vns.intra_two_opt(problem, tolerance=0)
    #     #     dist = problem.calculate_distances()
    #     #     vns.calculate_loading_MF(problem)
    #     #     problem.display_results(False)
    #
    #     old_dist = 0
    #     dist = problem.calculate_distances()
    #     while old_dist != dist:
    #         old_dist = dist
    #         vns.inter_two_opt(problem, tolerance=0)
    #         dist = problem.calculate_distances()
    #         vns.calculate_loading_MF(problem)
    #         problem.display_results(False)

    vns.greedy_routing_v1(problem)
    problem.display_results(False)

    ordered_nbhs = [vns.inter_two_opt, vns.intra_two_opt]
    vns.general_variable_nbh_search(problem, ordered_nbhs, timeout=120)
    problem.display_results()

    end1 = time.time()

    # print(bcolors.OKGREEN + "Found route with length {}m".format(route_cost) + bcolors.ENDC)
    print("Algorithm duration: %.4f s"%(end1 - start1))

    ##==========================================================================================================





    # Visualize - use only for very small graphs
    # display_graph(graph, node_info)
