"""
This is a test for checking machine performance.
This test shows the compute time average and standard diviation of greedy and vns.
"""

import os
import time
import csv
from tqdm import tqdm
from copy import deepcopy

import solvers
from structure import Vehicle, ProblemInstance
import operators as ops
from loaders import load_subset_from_ordered_nodes

from tests.test_base import TestLNS
from utils import update_problem_with_window, get_graph_after_rebalance, get_total_imbalance_from_aux_graph


KWARGS = {
    'nodes': 50,
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
    'num_try': 10,
    'max_window': 5,
}

class TestWindowModel(TestLNS):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.max_window = kwargs['max_window']

    def get_problem_instance(self, delta=0):
        graph, node_info = load_subset_from_ordered_nodes(nodes=self.nodes, centeredness=self.centeredness, randomness=self.randomness)
        window_graph = update_problem_with_window(graph, delta)
        vehicles = [Vehicle(capacity=self.vehicle_capacity, vehicle_id=str(i), distance_limit=self.distance_limit)
                    for i in range(self.number_of_vehicles)]
        problem = ProblemInstance(input_graph=window_graph, vehicles=vehicles, node_data=node_info, verbose=0)
        return problem

    def solver(self, problem):
        solvers.greedy_routing(problem, randomness=False)
        self.run_vns(problem)
        aux_graph = get_graph_after_rebalance(problem)
        imbalance = get_total_imbalance_from_aux_graph(aux_graph)
        return problem.calculate_distances(), imbalance

    def write_results_to_csv(self, filename, header, num_try):
        writer = csv.writer(open(os.path.join(self.root, filename), "w", newline=''))
        writer.writerow(header)
        results = []
        for _ in tqdm(range(num_try)):
            result = []
            for delta in range(self.max_window):
                problem = self.get_problem_instance(delta)
                dist, imbalance = self.solver(problem)
                result.append(dist)
                result.append(imbalance)
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
                for x in [f'Window={delta}', f'Window={delta}']]
        results = test_instance.write_results_to_csv(
            filename=KWARGS.get('filename', 'test_window_model.csv'),
            header=header,
            num_try=KWARGS.get('num_try', 100),
        )
    mean, std = test_instance.get_stats(results)

    print(''.join(['{:>10}'.format('Dist')] + ['{:>15}'.format(x) for i, x in enumerate(header) if i%2 == 0]))
    print(''.join(['{:>10}'.format('Mean')] + ['{:>15.2f}'.format(float(x)) for i, x in enumerate(mean) if i%2 == 0]))
    print(''.join(['{:>10}'.format('Std')] + ['{:>15.2f}'.format(float(x)) for i, x in enumerate(std) if i%2 == 0]))

    print(''.join(['{:>10}'.format('Imbalance')] + ['{:>15}'.format(x) for i, x in enumerate(header) if i%2 == 1]))
    print(''.join(['{:>10}'.format('Mean')] + ['{:>15.2f}'.format(float(x)) for i, x in enumerate(mean) if i%2 == 1]))
    print(''.join(['{:>10}'.format('Std')] + ['{:>15.2f}'.format(float(x)) for i, x in enumerate(std) if i%2 == 1]))

if __name__ == '__main__':
    main()