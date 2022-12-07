import time
from load_csv import load_graph
from mcf import MinimumCostFlow
from vns import VNS
from utils import *

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
    # Load graph
    # graph, node_info = load_graph('sample_graph_01_edited')
    graph, node_info = load_graph('sample_graph_03')

    # Load solver
    vns = VNS(graph, num_vehicles=10, vehicle_capacity=25, verbose=1)
    start2 = time.time()

    routes_initial = vns.greedy_routing_v1(budget=100000)
    visualize_routes(routes_initial, node_info)
    vns.calculate_loading_MF()
    vns.display_results()
    vns.remove_unused_stops()
    vns.display_results()

    end2 = time.time()

    # print(bcolors.OKGREEN + "Found route with length {}m".format(route_cost) + bcolors.ENDC)
    print("Algorithm duration: %.4f s"%(end2 - start2))

    ##==========================================================================================================





    # Visualize - use only for very small graphs
    # display_graph(graph, node_info)
