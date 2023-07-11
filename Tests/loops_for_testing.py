"""
Contains a test loop which can be used in a variety of tests
"""

from copy import deepcopy
from loaders import *
from structure import ProblemInstance, Vehicle
import solvers
import operators as ops
import csv
import time

np.random.seed(686)
random.seed(786)

def test_loop_vns(kwargs):
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

    stopping_duration = kwargs.get('stop_duration', 40)
    loading_duration = kwargs.get('load_duration', 15)

    nbh_change_set = kwargs.get('nbh_change_set', [solvers.change_nbh_cyclic])
    ordered_nbhs = kwargs.get('ordered_nbhs',
                              [ops.intra_two_opt, ops.intra_segment_swap, ops.inter_two_opt,
                               ops.inter_segment_swap])

    now = datetime.datetime.now()
    root = os.path.dirname(os.path.abspath(os.getcwd()))
    folder = os.path.join(root, 'Saved/statistics/')
    file = open(folder + 'stats_' + now.strftime("%d-%m-%y_%H-%M-%S") + '.csv', 'w', newline='')
    writer = csv.writer(file, delimiter=',')
    for key, value in kwargs.items():
        writer.writerow([key, value])

    headings = ["graph_size", "graph_instance", "num_vehicles", "capacity", "trial", "greedy_cost",]
    for nbh_change in nbh_change_set:
        if nbh_change == solvers.change_nbh_cyclic:
            headings.append("vns_cyclic_cost")
            headings.append("vns_cyclic_time")

        elif nbh_change == solvers.change_nbh_pipe:
            headings.append("vns_pipe_cost")
            headings.append("vns_pipe_time")

        elif nbh_change == solvers.change_nbh_sequential:
            headings.append("vns_seq_cost")
            headings.append("vns_seq_time")

        elif nbh_change == solvers.change_nbh_check_all:
            headings.append("vns_all_cost")
            headings.append("vns_all_time")

        else:
            print("Error, nbh_change method not recognised")
            raise ValueError
    writer.writerow(headings)

    for n in range(graph_size, graph_size_max + 1, graph_size_step):
        print("\nProblem Instance of size {} with {} vehicles of capacity {}"
              .format(n, num_vehicles, capacity))
        print("       | veh | cap | greedy durt  | vns duration |  vns time  | improved |  change_nbh_method")

        for m in range(graph_variations): #try different graph variations
            #Load Problem Instance
            graph, node_info = load_subset_from_ordered_nodes(nodes=n, cost='time', centeredness=10 - m * 2)
            # graph, node_info = load_graph('nyc_instance_dummy', location='nyc_dummy')
            print("--------- new graph ----------")
            for v in range(num_vehicles, num_vehicles_max + 1, num_vehicles_step):
                for c in range(capacity, capacity_max + 1, capacity_step):
                    vehicles = [Vehicle(capacity=c, vehicle_id=str(i), stop_duration=stopping_duration,
                                        load_duration=loading_duration, distance_limit=0) for i in range(v)]
                    problem = ProblemInstance(input_graph=graph, vehicles=vehicles, node_data=node_info, verbose=0)

                    for trial in range(trials_per_graph):
                        solvers.greedy_routing(problem, dist_weight=2, randomness=True)
                        greedy_cost = problem.calculate_costs()
                        saved_problem = deepcopy(problem)

                        gd = round(greedy_cost / 60, 2)
                        results = [n, m, v, c, trial, round(greedy_cost)]

                        for nbh_change in nbh_change_set:
                            problem = deepcopy(saved_problem)

                            start1 = time.time()
                            solvers.general_variable_nbh_search(problem, ordered_nbhs, change_nbh=nbh_change, timeout=300, verbose=0)
                            end1 = time.time()
                            problem.calculate_loading_mf()
                            assert problem.allocated == problem.imbalance
                            cost = problem.calculate_costs()

                            vd = round(cost / 60,  2)
                            vt = round(end1 - start1, 3)
                            results.append(round(cost))
                            results.append(vt)

                            im = round((1 - cost / greedy_cost) * 100, 1)
                            print("{:5}: | {:3} | {:3} |{:10}min |{:10}min |{:10}s |{:8}% |  {}"
                                  .format(trial, v, c, gd, vd, vt, im, nbh_change.__name__))

                        writer.writerow(results)
                        problem.reset()
    file.flush()

def test_loop_vns_multi_init(kwargs):
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

    stopping_duration = kwargs.get('stop_duration', 40)
    loading_duration = kwargs.get('load_duration', 15)

    destruction_degrees = kwargs.get('large_nbhs', [])
    ordered_nbhs = kwargs.get('ordered_nbhs',
                              [ops.intra_two_opt, ops.intra_segment_swap, ops.inter_two_opt,
                               ops.inter_segment_swap])

    timeout = kwargs.get('timeout', 60)
    large_timeout = kwargs.get('large_timeout', 600)

    now = datetime.datetime.now()
    root = os.path.dirname(os.path.abspath(os.getcwd()))
    folder = os.path.join(root, 'Saved/statistics/')
    file = open(folder + 'stats_vns_multi_' + now.strftime("%d-%m-%y_%H-%M-%S") + '.csv', 'w', newline='')
    writer = csv.writer(file, delimiter=',')
    for key, value in kwargs.items():
        writer.writerow([key, value])

    headings = ["graph_size", "graph_instance", "num_vehicles", "capacity", "trial", "greedy_cost",
                "vns_cost", "vns_time", "random_dist", "vns_cost", "vns_time"]
    writer.writerow(headings)

    for n in range(graph_size, graph_size_max + 1, graph_size_step):
        print("\nProblem Instance of size {} with {} vehicles of capacity {}"
              .format(n, num_vehicles, capacity))
        print("       | greedy dist  | vns cost |  vns time  | random dist  | vns cost |  vns time  |")
        large_nbhs = [int(np.floor(n * element)) for element in destruction_degrees]

        for m in range(graph_variations):  # try different graph variations
            # Load Problem Instance
            graph, node_info = load_subset_from_ordered_nodes(nodes=n, cost='time', centeredness=10 - m * 2)
            # graph, node_info = load_graph('nyc_instance_dummy', location='nyc_dummy')

            print("--------- new graph ----------")
            for v in range(num_vehicles, num_vehicles_max + 1, num_vehicles_step):
                for c in range(capacity, capacity_max + 1, capacity_step):
                    vehicles = [Vehicle(capacity=c, vehicle_id=str(i), stop_duration=stopping_duration,
                                        load_duration=loading_duration, distance_limit=0) for i in range(v)]
                    problem = ProblemInstance(input_graph=graph, vehicles=vehicles, node_data=node_info, verbose=0)

                    for trial in range(trials_per_graph):
                        saved_problem = deepcopy(problem)
                        solvers.greedy_routing(problem, dist_weight=2, randomness=True)
                        greedy_cost = problem.calculate_costs()

                        gd = round(greedy_cost / 60, 2)
                        results = [n, m, v, c, trial, round(greedy_cost)]

                        start1 = time.time()
                        solvers.general_variable_nbh_search(problem, ordered_nbhs,
                                                            change_nbh=solvers.change_nbh_cyclic, timeout=300,
                                                            verbose=0)
                        end1 = time.time()
                        problem.calculate_loading_mf()
                        assert problem.allocated == problem.imbalance

                        cost = problem.calculate_costs()
                        vd = round(cost / 60,  2)
                        vt = round(end1 - start1, 3)
                        results.append(round(cost))
                        results.append(vt)

                        problem = saved_problem
                        solvers.random_routing(problem)
                        rd = round(problem.calculate_costs()) / 60

                        start2 = time.time()
                        solvers.general_variable_nbh_search(problem, ordered_nbhs,
                                                            change_nbh=solvers.change_nbh_cyclic, timeout=300,
                                                            verbose=0)
                        # vns.large_nbh_search(problem, large_nbhs, ordered_nbhs,
                        #     change_local_nbh=vns.change_nbh_cyclic,
                        #     change_large_nbh=vns.change_nbh_pipe,
                        #     large_nbh_operator=ops.destroy_rebuild,
                        #     timeout=timeout, large_timeout=large_timeout, local_verbose=0, large_verbose=0
                        # )
                        end2 = time.time()

                        cost = problem.calculate_costs()
                        ld = round(cost / 60,  2)
                        lt = round(end2 - start2, 3)
                        results.append(rd)
                        results.append(ld)
                        results.append(lt)
                        im = round((1 - cost / greedy_cost) * 100, 1)
                        print("{:5}: |{:10}min |{:10}min |{:10}s |{:10}min |{:10}min |{:10}s |"
                              .format(m * trials_per_graph + trial, round(greedy_cost), vd, vt, rd, ld, lt))

                        writer.writerow(results)
                        problem.reset()
                    file.flush()


def test_loop_vns_lns(kwargs):
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

    stopping_duration = kwargs.get('stop_duration', 40)
    loading_duration = kwargs.get('load_duration', 15)

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

    headings = ["graph_size", "graph_instance", "num_vehicles", "capacity", "trial", "greedy_cost",
                "vns_cost", "vns_time", "lns_cost", "lns_time"]
    writer.writerow(headings)

    for n in range(graph_size, graph_size_max + 1, graph_size_step):
        print("\nProblem Instance of size {} with {} vehicles of capacity {}"
              .format(n, num_vehicles, capacity))
        print("       | greedy dist  | vns cost |  vns time  | lns cost |  lns time  | improved |")
        large_nbhs = [int(np.floor(n * element)) for element in destruction_degrees]

        for m in range(graph_variations):  # try different graph variations
            # Load Problem Instance
            graph, node_info = load_subset_from_ordered_nodes(nodes=n, cost='time', centeredness=10 - m * 2)

            print("--------- new graph ----------")
            for v in range(num_vehicles, num_vehicles_max + 1, num_vehicles_step):
                for c in range(capacity, capacity_max + 1, capacity_step):
                    vehicles = [Vehicle(capacity=c, vehicle_id=str(i), stop_duration=stopping_duration,
                                        load_duration=loading_duration, distance_limit=0) for i in range(v)]
                    problem = ProblemInstance(input_graph=graph, vehicles=vehicles, node_data=node_info, verbose=0)

                    for trial in range(trials_per_graph):
                        solvers.greedy_routing(problem, dist_weight=2, randomness=True)
                        greedy_cost = problem.calculate_costs()

                        gd = round(greedy_cost / 60, 2)
                        results = [n, m, v, c, trial, round(greedy_cost)]

                        start1 = time.time()
                        solvers.general_variable_nbh_search(problem, ordered_nbhs, change_nbh=solvers.change_nbh_cyclic,
                                                            timeout=300, verbose=0)
                        end1 = time.time()
                        problem.calculate_loading_mf()
                        assert problem.allocated == problem.imbalance

                        cost = problem.calculate_costs()
                        vd = round(cost / 60,  2)
                        vt = round(end1 - start1, 3)
                        results.append(round(cost))
                        results.append(vt)

                        start2 = time.time()
                        solvers.large_nbh_search(problem, large_nbhs, ordered_nbhs,
                            change_local_nbh=solvers.change_nbh_cyclic,
                            change_large_nbh=solvers.change_nbh_pipe,
                            large_nbh_operator=ops.destroy_rebuild,
                            timeout=timeout, large_timeout=large_timeout, local_verbose=0, large_verbose=0
                        )
                        end2 = time.time()
                        problem.calculate_loading_mf()
                        assert problem.allocated == problem.imbalance

                        cost = problem.calculate_costs()
                        ld = round(cost / 60,  2)
                        lt = round(end2 - start2, 3)
                        results.append(round(cost))
                        results.append(lt)
                        im = round((1 - cost / greedy_cost) * 100, 1)
                        print("{:5}: |{:10}min |{:10}min |{:10}s |{:10}min |{:10}s |{:8}% |"
                              .format(m * trials_per_graph + trial, gd, vd, vt, ld, lt, im))

                        writer.writerow(results)
                        problem.reset()
                    file.flush()
