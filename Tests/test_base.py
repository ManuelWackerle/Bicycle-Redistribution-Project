"""
Abstract classes for test.
Overwrite the class to write a test.
"""

import csv
import os
import numpy as np
from tqdm import tqdm
import solvers
import operators as ops
from loaders import load_subset_from_ordered_nodes
from structure import ProblemInstance, Vehicle
from matplotlib import pyplot as plt


class TestBase(object):
    def __init__(self, *args, **kwargs) -> None:
        self.nodes = kwargs.get('nodes', 50)
        self.centeredness = kwargs.get('centeredness', 5)
        self.number_of_vehicles = kwargs.get('number_of_vehicles', 5)
        self.vehicle_capacity = kwargs.get('vehicle_capacity', 20)
        self.distance_limit = kwargs.get('distance_limit', 200000)
        self.time_station = kwargs.get('time_station', 0)
        self.time_load = kwargs.get('time_load', 0)
        self.randomness = kwargs.get('randomness', True)
        self.root = kwargs.get('root', os.path.join(os.getcwd(), 'results'))

    def get_problem_instance(self):
        graph, node_info = load_subset_from_ordered_nodes(nodes=self.nodes, centeredness=self.centeredness, randomness=self.randomness)
        vehicles = [Vehicle(capacity=self.vehicle_capacity, vehicle_id=str(i), distance_limit=self.distance_limit)
                    for i in range(self.number_of_vehicles)]
        problem = ProblemInstance(input_graph=graph, vehicles=vehicles, node_data=node_info, verbose=0, time_station=self.time_station, time_load=self.time_load)
        return problem

    def write_results_to_csv(self, filename, header, num_try):
        writer = csv.writer(open(os.path.join(self.root, filename), "w", newline=''))
        writer.writerow(header)
        results = []
        for _ in tqdm(range(num_try)):
            problem = self.get_problem_instance()
            result = self.solver(problem)
            writer.writerow(result)
            results.append(result)
        return results

    def read_results_from_csv(self, filename):
        results = []
        with open(os.path.join(self.root, filename), 'r') as f:
            reader = csv.reader(f)
            header = next(reader)
            for row in reader:
                results.append([float(x) for x in row])
        return header, results

    def get_stats(self, results):
        np_results = np.array(results)
        np_mean = np.round(np.mean(np_results, axis=0), decimals=2)
        np_std = np.round(np.std(np_results, axis=0), decimals=2)
        return np_mean.tolist(), np_std.tolist()

    def show_plot(self, results, header):
        np_results = np.array(results)
        np_mean = np.round(np.mean(np_results, axis=0), decimals=2)
        np_std = np.round(np.std(np_results, axis=0), decimals=2)
        plt.bar(header, np_mean, alpha=0.8, yerr=np_std)
        plt.show()

    def test_all(self, filename, header, num_try=100):
        results = self.write_results_to_csv(filename, header, num_try)
        self.show_plot(results, header)
        return results

    def read_and_show_plot(self, filename):
        header, results = self.read_results_from_csv(filename)
        self.show_plot(results, header)
        return results

    def solver(self, problem):
        raise Exception("Solver is not implemented.")


class TestLNS(TestBase):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.ordered_nbhs = kwargs.get('ordered_nbhs', [ops.intra_two_opt, ops.intra_segment_swap, ops.inter_two_opt, ops.inter_segment_swap])
        self.ordered_large_nbhs = kwargs.get('ordered_large_nbhs', [1, 3, 5, 10, 15, 20, 25, 30, 40, 50])
        self.change_local_nbh = kwargs.get('change_local_nbh', solvers.change_nbh_sequential)
        self.change_large_nbh = kwargs.get('change_large_nbh', solvers.change_nbh_pipe)
        self.large_nbh_operator = kwargs.get('large_nbh_operator', ops.multi_remove_and_insert_station)
        self.local_timeout = kwargs.get('local_timeout', 2*60)
        self.large_timeout = kwargs.get('large_timeout', 6*60)
        self.local_verbose = kwargs.get('local_verbose', 0)
        self.large_verbose = kwargs.get('large_verbose', 0)

    def run_greedy(self, problem):
        solvers.greedy_routing(problem)
        return problem

    def run_vns(self, problem):
        solvers.general_variable_nbh_search(
            problem, self.ordered_nbhs, change_nbh=self.change_local_nbh,
            verbose=self.local_verbose, timeout=self.local_timeout)
        return problem

    def run_lns(self, problem):
        solvers.large_nbh_search(problem,
                            self.ordered_large_nbhs,
                            self.ordered_nbhs,
                            change_local_nbh=self.change_local_nbh,
                            change_large_nbh=self.change_large_nbh,
                            large_nbh_operator=self.large_nbh_operator,
                            timeout=self.local_timeout,
                            large_timeout=self.large_timeout,
                            local_verbose=self.local_verbose,
                            large_verbose=self.large_verbose,
                            )