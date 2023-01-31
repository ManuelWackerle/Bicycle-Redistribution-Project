from copy import deepcopy
from load_csv import *
from structure import ProblemInstance, Vehicle
import vns
import operators as ops
import csv
import time


def run_test(kwargs):
    graph_size = kwargs.get('graph_size', 100)
    graph_size_max = kwargs.get('graph_size_max', 100)
    graph_size_step = kwargs.get('graph_size_step', 10)
    graph_variations = kwargs.get('graph_variations', 1)
    trials_per_graph = kwargs.get('trials_per_graph', 10)

    num_vehicles = kwargs.get('num_vehicles', 5)
    num_vehicles_max = kwargs.get('num_vehicles_max', 5)
    num_vehicles_step = kwargs.get('num_vehicles_step', 1)

    capacity = kwargs.get('capacity', 15)
    capacity_max = kwargs.get('capacity_max', 15)
    capacity_step = kwargs.get('capacity_step', 1)

    nbh_change_set = kwargs.get('nbh_change_set', [vns.change_nbh_cyclic])
    ordered_nbhs = kwargs.get('ordered_nbhs',
                              [ops.intra_two_opt_fast, ops.intra_segment_swap_fast, ops.inter_two_opt_fast,
                               ops.inter_segment_swap_fast])

    now = datetime.datetime.now()
    root = os.path.dirname(os.path.abspath(os.getcwd()))
    folder = os.path.join(root, 'Saved/statistics/')
    file = open(folder + 'stats_' + now.strftime("%d-%m-%y_%H-%M-%S") + '.csv', 'w', newline='')
    writer = csv.writer(file, delimiter=',')
    for key, value in kwargs.items():
        writer.writerow([key, value])

    headings = ["graph_size", "graph_instance", "num_vehicles", "capacity", "trial", "greedy_distance",]
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
    for n in range(graph_size, graph_size_max + 1, graph_size_step):
        print("\nNew graph size has {} nodes and {} vehicles with capacity {}"
              .format(count, n, num_vehicles, capacity))
        print("       | veh | cap | greedy dist  | vns distance |  vns time  | improved |  change_nbh_method")
        count += 1

        for m in range(graph_variations): #try different graph variations
            #Load Problem Instance
            graph, node_info = load_subset_from_ordered_nodes(nodes=n, centeredness=10 - m * 2)
            # graph, node_info = load_graph('nyc_instance_dummy', location='nyc_dummy')
            print("--------- new graph ----------")
            for v in range(num_vehicles, num_vehicles_max + 1, num_vehicles_step):
                for c in range(capacity, capacity_max + 1, capacity_step):
                    vehicles = [Vehicle(capacity=c, vehicle_id=str(i), distance_limit=0) for i in range(v)]
                    problem = ProblemInstance(input_graph=graph, vehicles=vehicles, node_data=node_info, verbose=0)

                    for trial in range(trials_per_graph):
                        vns.greedy_routing_v1(problem, dist_weight=2, randomness=True)
                        greedy_distance = problem.calculate_distances()
                        saved_problem = deepcopy(problem)

                        gd = round(greedy_distance) / 1000
                        results = [n, m, v, c, trial, gd]

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
                            print("{:5}: | {:3} | {:3} |{:11}km |{:11}km |{:10}s |{:8}% |  {}"
                                  .format(trial, v, c, gd, vd, vt, im, nbh_change.__name__))

                        writer.writerow(results)
                        problem.reset()
    file.flush()