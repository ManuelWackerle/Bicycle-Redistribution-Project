from copy import deepcopy
from load_csv import *
from structure import ProblemInstance, Vehicle
import solvers
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

    destruction_degrees = kwargs.get('large_nbhs', [])
    ordered_nbhs = kwargs.get('ordered_nbhs',
                              [ops.intra_two_opt, ops.intra_segment_swap, ops.inter_two_opt,
                               ops.inter_segment_swap])

    timeout = kwargs.get('timeout', 60)
    large_timeout = kwargs.get('large_timeout', 600)

    now = datetime.datetime.now()
    root = os.path.dirname(os.path.abspath(os.getcwd()))
    folder = os.path.join(root, 'Saved/statistics/')
    file = open(folder + 'stats_lns_' + now.strftime("%d-%m-%y_%H-%M-%S") + '.csv', 'w', newline='')
    writer = csv.writer(file, delimiter=',')
    for key, value in kwargs.items():
        writer.writerow([key, value])

    headings = ["graph_size", "graph_instance", "num_vehicles", "capacity", "trial", "greedy_distance",
                "vns_distance", "vns_time", "random_dist", "vns_distance", "vns_time"]
    writer.writerow(headings)

    for n in range(graph_size, graph_size_max + 1, graph_size_step):
        print("\nProblem Instance of size {} with {} vehicles of capacity {}"
              .format(n, num_vehicles, capacity))
        print("       | greedy dist  | vns distance |  vns time  | lns distance |  lns time  | improved |")
        large_nbhs = [int(np.floor(n * element)) for element in destruction_degrees]

        for m in range(graph_variations):  # try different graph variations
            # Load Problem Instance
            graph, node_info = load_subset_from_ordered_nodes(nodes=n, centeredness=10 - m * 2)
            # graph, node_info = load_graph('nyc_instance_dummy', location='nyc_dummy')

            print("--------- new graph ----------")
            for v in range(num_vehicles, num_vehicles_max + 1, num_vehicles_step):
                for c in range(capacity, capacity_max + 1, capacity_step):
                    vehicles = [Vehicle(capacity=c, vehicle_id=str(i), distance_limit=0) for i in range(v)]
                    problem = ProblemInstance(input_graph=graph, vehicles=vehicles, node_data=node_info, verbose=0)

                    for trial in range(trials_per_graph):
                        saved_problem = deepcopy(problem)
                        solvers.greedy_routing_v1(problem, dist_weight=2, randomness=True)
                        greedy_distance = problem.calculate_distances()


                        gd = round(greedy_distance) / 1000
                        results = [n, m, v, c, trial, gd]

                        start1 = time.time()
                        solvers.general_variable_nbh_search(problem, ordered_nbhs,
                                                            change_nbh=solvers.change_nbh_cyclic, timeout=300, verbose=0)
                        end1 = time.time()

                        distance = problem.calculate_distances()
                        vd = round(distance) / 1000
                        vt = round(end1 - start1, 3)
                        results.append(vd)
                        results.append(vt)

                        problem = saved_problem
                        solvers.random_routing(problem)
                        rd = round(problem.calculate_distances()) / 1000

                        start2 = time.time()
                        solvers.general_variable_nbh_search(problem, ordered_nbhs,
                                                            change_nbh=solvers.change_nbh_cyclic, timeout=300, verbose=0)
                        # vns.large_nbh_search(problem, large_nbhs, ordered_nbhs,
                        #     change_local_nbh=vns.change_nbh_cyclic,
                        #     change_large_nbh=vns.change_nbh_pipe,
                        #     large_nbh_operator=ops.destroy_rebuild,
                        #     timeout=timeout, large_timeout=large_timeout, local_verbose=0, large_verbose=0
                        # )
                        end2 = time.time()

                        distance = problem.calculate_distances()
                        ld = round(distance) / 1000
                        lt = round(end2 - start2, 3)
                        results.append(rd)
                        results.append(ld)
                        results.append(lt)
                        im = round((1 - distance / greedy_distance) * 100, 1)
                        print("{:5}: |{:11}km |{:11}km |{:10}s |{:11}km |{:10}s |{:8}% |"
                              .format(m * trials_per_graph + trial, gd, vd, vt, ld, lt, im))

                        writer.writerow(results)
                        problem.reset()
                    file.flush()
