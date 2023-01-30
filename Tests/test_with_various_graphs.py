from copy import deepcopy

from load_csv import *
from structure import ProblemInstance, Vehicle
import vns
import operators as ops
import csv
import time

    # #Copy this into main
    # num_vehicles = 5
    # capacity = 15
    # min_graph_size = 10
    # max_graph_size = 400
    # graph_size_step = 10
    # graph_variations = 10
    # trials_per_graph = 10
    # ordered_nbhs = [ops.intra_segment_swap_fast, ops.intra_two_opt_fast, ops.inter_segment_swap_fast, ops.inter_two_opt_fast] # vns.remove_and_insert_station]
    # nbh_change_set = [vns.change_nbh_pipe, vns.change_nbh_sequential, vns.change_nbh_check_all]
    # run_test(num_vehicles, capacity, min_graph_size, max_graph_size, graph_size_step, graph_variations, trials_per_graph)


def run_test(kwargs):
    num_vehicles = kwargs['num_vehicles']
    capacity = kwargs['capacity']
    min_graph_size = kwargs['min_graph_size']
    max_graph_size = kwargs['max_graph_size']
    graph_size_step = kwargs['graph_size_step']
    graph_variations = kwargs['graph_variations']
    trials_per_graph = kwargs['trials_per_graph']
    ordered_nbhs = kwargs['ordered_nbhs']
    nbh_change_set = kwargs['nbh_change_set']

    now = datetime.datetime.now()
    root = os.path.dirname(os.path.abspath(os.getcwd()))
    folder = os.path.join(root, 'Saved/statistics/')
    file = open(folder + 'stats_' + now.strftime("%d-%m-%y_%H-%M-%S") + '.csv', 'w', newline='')
    writer = csv.writer(file, delimiter=',')
    for key, value in kwargs.items():
        writer.writerow([key, value])

    headings = ["graph_size", "graph_instance", "trial", "greedy_distance"]
    for nbh_change in nbh_change_set:
        if nbh_change == vns.change_nbh_cyclic:
            headings.append("vns_cyclic_distance")
            headings.append("vns_cyclic_time")

        elif nbh_change == vns.change_nbh_pipe:
            headings.append("vns_pipe_distance")
            headings.append("vns_pipe_time")

        elif nbh_change == vns.change_nbh_sequential:
            headings.append("vns_seq_distance")
            headings.append("vns_seq_time")

        elif nbh_change == vns.change_nbh_check_all:
            headings.append("vns_all_distance")
            headings.append("vns_all_time")

        else:
            print("Error, nbh_change method not recognised")
            raise ValueError
    writer.writerow(headings)

    count = 0
    for n in range(min_graph_size, max_graph_size + 1, graph_size_step):
        gdt, vdt, imt, tmt = 0, 0, 0, 0
        print("\nTEST {}. Problem has {} nodes and {} vehicles with capacity {}"
              .format(count, n, num_vehicles, capacity))
        print("       | greedy dist  | vns distance |  vns time  | improved |  change_nbh_method")
        count += 1

        for m in range(graph_variations): #try different graph variations
            #Load Problem Instance
            graph, node_info = load_subset_from_ordered_nodes(nodes=n, centeredness=m+5)
            # graph, node_info = load_graph('nyc_instance_dummy', location='nyc_dummy')

            vehicles = [Vehicle(capacity=capacity, vehicle_id=str(i)) for i in range(num_vehicles)]
            problem = ProblemInstance(input_graph=graph, vehicles=vehicles, node_data=node_info, verbose=1)

            print("--------- new graph ----------")
            for trial in range(trials_per_graph):
                vns.greedy_routing_v1(problem, dist_weight=2, randomness=True)
                greedy_distance = problem.calculate_distances()
                saved_problem = deepcopy(problem)

                gd = round(greedy_distance) / 1000
                results = [n, m, trial, gd]

                for nbh_change in nbh_change_set:
                    problem = deepcopy(saved_problem)

                    start1 = time.time()
                    vns.general_variable_nbh_search(problem, ordered_nbhs, change_nbh=nbh_change, timeout=300, verbose=0)
                    end1 = time.time()
                    distance = problem.calculate_distances()

                    vd = round(distance) / 1000
                    vt = round(end1 - start1, 3)
                    results.append(vd)
                    results.append(vt)

                    im = round((1 - distance / greedy_distance) * 100, 1)
                    print("{:5}: |{:11}km |{:11}km |{:10}s |{:8}% |  {}"
                          .format(m * trials_per_graph + trial, gd, vd, vt, im, nbh_change.__name__))

                writer.writerow(results)
                problem.reset()
                file.flush()
        # gdt = round(gdt/trials_per_graph/graph_variations, 3)
        # vdt = round(vdt/trials_per_graph/graph_variations, 3)
        # imt = round(imt/trials_per_graph/graph_variations, 3)
        # tmt = round(tmt/trials_per_graph/graph_variations, 3)
        # print("AVG -  |{:11}km |{:11}km |{:10}s |{:8}% |".format(gdt, vdt, tmt, imt))
