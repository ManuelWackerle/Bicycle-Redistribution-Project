"""
This is a test for checking machine performance.
This test shows the compute time average and standard diviation of greedy and vns.
"""

import os
import time
import csv
from tqdm import tqdm
from copy import deepcopy
import sys
sys.path.append(os.getcwd())

import solvers
from structure import Vehicle, ProblemInstance
import operators as ops
from loaders import load_subset_from_ordered_nodes

from tests.test_base import TestLNS
from utils import update_problem_with_all_window, update_problem_with_partial_window, get_graph_after_rebalance, get_total_imbalance_from_aux_graph


KWARGS = {
    'nodes': 100,
    'centeredness': 5,
    'number_of_vehicles': 5,
    'vehicle_capacity': 20,
    'ordered_nbhs': [ops.intra_two_opt,
                     ops.intra_segment_swap,
                     ops.inter_two_opt,
                     ops.inter_segment_swap],
    'distance_limit': 200000,  # metre
    'local_timeout': 2*60,  # second
    'local_verbose': 0,
    'change_local_nbh': solvers.change_nbh_cyclic,
    'root': os.path.join(os.getcwd(), 'results'),
    'read_only': False,
    'filename': 'test_window_model.csv',
    'num_try': 100,
    'max_window': 10,
}

class TestWindowModel(TestLNS):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.max_window = kwargs['max_window']

    def get_problem_instance(self, graph, node_info, delta=0):
        vehicles = [Vehicle(capacity=self.vehicle_capacity, vehicle_id=str(i), distance_limit=self.distance_limit)
                    for i in range(self.number_of_vehicles)]
        problem = ProblemInstance(input_graph=graph, vehicles=vehicles, node_data=node_info, verbose=0)
        return problem

    def solver(self, original_problem, window_problem):
        solvers.greedy_routing(window_problem, randomness=False)
        self.run_vns(window_problem)
        original_problem.vehicles = window_problem.vehicles
        original_problem.calculate_loading_mf()
        dist = original_problem.calculate_distances()
        total_bikes = get_total_imbalance_from_aux_graph(original_problem.model)
        aux_graph = get_graph_after_rebalance(original_problem)
        imbalance = get_total_imbalance_from_aux_graph(aux_graph)
        efficiency = (total_bikes - imbalance) / dist * 1000
        return dist, imbalance, efficiency

    def write_results_to_csv(self, filename, header, num_try):
        writer = csv.writer(open(os.path.join(self.root, filename), "w", newline=''))
        writer.writerow(header)
        results = []
        for _ in tqdm(range(num_try)):
            result = []
            original_graph, node_info = load_subset_from_ordered_nodes(nodes=self.nodes, centeredness=self.centeredness, randomness=self.randomness)
            original_problem = self.get_problem_instance(original_graph, node_info)
            for delta in range(self.max_window):
                find, window_graph = update_problem_with_all_window(deepcopy(original_graph), delta)
                if not find:
                    break
                window_problem = self.get_problem_instance(window_graph, node_info)
                dist, imbalance, efficiency = self.solver(original_problem, window_problem)
                result.append(dist)
                result.append(imbalance)
                result.append(efficiency)
            if not find:
                continue
            writer.writerow(result)
            results.append(result)
        return results


def main():
    test_instance = TestWindowModel(**KWARGS)
    if KWARGS.get('read_only'):
        header, results = test_instance.read_results_from_csv(KWARGS.get('filename', 'test_window_model.csv'))
    else:
        header=[x 
                for delta in range(KWARGS['max_window'])
                for x in [f'Window={delta}', f'Window={delta}', f'Window={delta}']]
        results = test_instance.write_results_to_csv(
            filename=KWARGS.get('filename', 'test_window_model.csv'),
            header=header,
            num_try=KWARGS.get('num_try', 100),
        )
    mean, std = test_instance.get_stats(results)

    print(''.join(['{:>10}'.format('Dist')] + ['{:>15}'.format(x) for i, x in enumerate(header) if i%3 == 0]))
    print(''.join(['{:>10}'.format('Mean')] + ['{:>15.2f}'.format(float(x)) for i, x in enumerate(mean) if i%3 == 0]))
    print(''.join(['{:>10}'.format('Std')] + ['{:>15.2f}'.format(float(x)) for i, x in enumerate(std) if i%3 == 0]))

    print(''.join(['{:>10}'.format('Imbalance')] + ['{:>15}'.format(x) for i, x in enumerate(header) if i%3 == 1]))
    print(''.join(['{:>10}'.format('Mean')] + ['{:>15.2f}'.format(float(x)) for i, x in enumerate(mean) if i%3 == 1]))
    print(''.join(['{:>10}'.format('Std')] + ['{:>15.2f}'.format(float(x)) for i, x in enumerate(std) if i%3 == 1]))

    print(''.join(['{:>10}'.format('Efficiency')] + ['{:>15}'.format(x) for i, x in enumerate(header) if i%3 == 2]))
    print(''.join(['{:>10}'.format('Mean')] + ['{:>15.2f}'.format(float(x)) for i, x in enumerate(mean) if i%3 == 2]))
    print(''.join(['{:>10}'.format('Std')] + ['{:>15.2f}'.format(float(x)) for i, x in enumerate(std) if i%3 == 2]))

if __name__ == '__main__':
    main()