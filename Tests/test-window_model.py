"""
This is a test for a window model to see how it will affect our last route and imbalance.
This test will show the results in the following format:
```
------------------------------------------------------------
      Dist  Window=0  Window=1  Window=2  Window=3  Window=4
      Mean 226011.48 199532.96 167573.51 146517.96 130082.58
       Std  21428.47  32561.90  20143.69  22451.09  27301.54
------------------------------------------------------------
 Total Imb  Window=0  Window=1  Window=2  Window=3  Window=4
      Mean      0.00     41.80     75.00     98.60    119.40
       Std      0.00      4.85      8.59     13.97     15.13
------------------------------------------------------------
   Max Imb  Window=0  Window=1  Window=2  Window=3  Window=4
      Mean      0.00      1.00      2.00      3.00      4.00
       Std      0.00      0.00      0.00      0.00      0.00
------------------------------------------------------------
Efficiency  Window=0  Window=1  Window=2  Window=3  Window=4
      Mean      1.05      0.99      0.97      0.94      0.89
       Std      0.10      0.11      0.13      0.10      0.12
------------------------------------------------------------
```
where
    each column corresponds to window size,
    `Dist` is a total distance of all vehicles calculated by `calculate_distances`, 
    `Total Imb` is the total number of supply/demand (imbalance),
    `Max Imb` is the maximum number of supply/demand (imbalance) of one node (which should usually coincide with the number of window),
    `Efficiency` is calculated by (the total number of allocation)/(total distance) * 1000 
        (multiply by 1000 is just for making them more than 1 to show decimal 2)

You can change `kwargs` values used in the tests. The important one's are
    nodes:              The number of nodes (stations) in the problem.
    number_of_vehicles: The number of vehicles available for routing.
    vehicle_capacity:   The maximum capacity of each vehicle.
    ordered_nbhs:       A list of ordered neighborhoods (operator functions) used for local search.
    local_timeout:      The timeout value for the local search in seconds.
    change_local_nbh:   A function to change the local neighborhood during the search.
    read_only:          Specifies if the test is read-only (read results from CSV).
    filename:           The name of the CSV file for storing test results.
    num_try:            The number of times to perform the test.
    max_window:         The max window size you want to try
    window_model:       all window model or partial window model. Check original function to see the difference.
More kwargs, check the parent class.
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
from utils import (
    update_problem_with_all_window,
    update_problem_with_partial_window, 
    get_graph_after_rebalance, 
    get_total_imbalance_from_aux_graph,
    get_max_imbalance_from_aux_graph,
    assert_total_imbalance,
)


KWARGS = {
    'nodes': 50,
    'centeredness': 5,
    'number_of_vehicles': 5,
    'vehicle_capacity': 5,
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
    'window_model': update_problem_with_all_window,
}

class TestWindowModel(TestLNS):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.max_window = kwargs['max_window']
        self.window_model = kwargs['window_model']

    def get_problem_instance(self, graph, node_info, delta=0):
        vehicles = [Vehicle(capacity=self.vehicle_capacity, vehicle_id=str(i), distance_limit=self.distance_limit)
                    for i in range(self.number_of_vehicles)]
        problem = ProblemInstance(input_graph=graph, vehicles=vehicles, node_data=node_info, verbose=0)
        return problem

    def solver(self, original_problem, window_problem):
        solvers.greedy_routing(window_problem, randomness=False)
        self.run_vns(window_problem)
        original_problem.vehicles = window_problem.vehicles
        dist = original_problem.calculate_distances()
        total_bikes = get_total_imbalance_from_aux_graph(original_problem.model)
        aux_graph = get_graph_after_rebalance(original_problem)
        imbalance = get_total_imbalance_from_aux_graph(aux_graph)
        max_imbalance = get_max_imbalance_from_aux_graph(aux_graph, original_problem.depot)
        efficiency = (total_bikes - imbalance) / dist * 1000
        return dist, imbalance, max_imbalance, efficiency

    def write_results_to_csv(self, filename, header, num_try):
        writer = csv.writer(open(os.path.join(self.root, filename), "w", newline=''))
        writer.writerow(header)
        results = []
        for _ in tqdm(range(num_try)):
            result = []
            original_graph, node_info = load_subset_from_ordered_nodes(nodes=self.nodes, centeredness=self.centeredness, randomness=self.randomness)
            original_problem = self.get_problem_instance(original_graph, node_info)
            assert assert_total_imbalance(original_problem)
            for delta in range(self.max_window):
                find, window_graph = self.window_model(deepcopy(original_problem), delta)
                if not find:
                    break
                window_problem = self.get_problem_instance(window_graph, node_info)
                assert assert_total_imbalance(window_problem)
                dist, imbalance, max_imbalance, efficiency = self.solver(original_problem, window_problem)
                result += [dist, imbalance, max_imbalance, efficiency]
            if not find:
                continue
            writer.writerow(result)
            results.append(result)
        return results


def main():
    sections = ['Dist', 'Total Imb', 'Max Imb', 'Efficiency']
    N = len(sections)
    test_instance = TestWindowModel(**KWARGS)
    if KWARGS.get('read_only'):
        header, results = test_instance.read_results_from_csv(KWARGS.get('filename', 'test_window_model.csv'))
    else:
        header=[x 
                for delta in range(KWARGS['max_window'])
                for x in [f'Window={delta}' for _ in range(N)]]
        results = test_instance.write_results_to_csv(
            filename=KWARGS.get('filename', 'test_window_model.csv'),
            header=header,
            num_try=KWARGS.get('num_try', 100),
        )
    mean, std = test_instance.get_stats(results)

    print(''.join(['{:>10}'.format('----------')] + ['{:>10}'.format('----------') for i, x in enumerate(std) if i%N == 0]))
    for k, section in enumerate(sections):
        print(''.join(['{:>10}'.format(section)] + ['{:>10}'.format(x) for i, x in enumerate(header) if i%N == k]))
        print(''.join(['{:>10}'.format('Mean')] + ['{:>10.2f}'.format(float(x)) for i, x in enumerate(mean) if i%N == k]))
        print(''.join(['{:>10}'.format('Std')] + ['{:>10.2f}'.format(float(x)) for i, x in enumerate(std) if i%N == k]))
        print(''.join(['{:>10}'.format('----------')] + ['{:>10}'.format('----------') for i, x in enumerate(std) if i%N == k]))

if __name__ == '__main__':
    main()