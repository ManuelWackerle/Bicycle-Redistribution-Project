"""
This is a test for a window model to see how it will affect our last route and imbalance.
This test will show the results in the following format:
```
--------------------------------------------------------------------------------------------------------------
      Dist  Window=0  Window=1  Window=2  Window=3  Window=4  Window=5  Window=6  Window=7  Window=8  Window=9
      Mean 272688.25 225892.90 157897.60 113776.45 105047.05  98109.85  77643.20  75130.35  69290.85  68311.60
       Std   8005.05   4854.30  10939.50  12413.05  10877.75   1163.15   2097.20   9340.05  22771.05  13002.60
--------------------------------------------------------------------------------------------------------------
 Imbalance  Window=0  Window=1  Window=2  Window=3  Window=4  Window=5  Window=6  Window=7  Window=8  Window=9
      Mean      0.00     36.00    101.00    155.00    190.00    234.00    253.00    279.00    296.00    292.00
       Std      0.00      4.00      9.00      9.00      6.00     10.00      9.00     21.00     38.00      0.00
--------------------------------------------------------------------------------------------------------------
Efficiency  Window=0  Window=1  Window=2  Window=3  Window=4  Window=5  Window=6  Window=7  Window=8  Window=9
      Mean      1.60      1.77      2.13      2.49      2.37      2.07      2.37      2.16      2.03      2.18
       Std      0.01      0.02      0.03      0.11      0.22      0.03      0.06      0.43      0.01      0.28
--------------------------------------------------------------------------------------------------------------
```
where
    each column corresponds to window size,
    `Dist` is a total distance of all vehicles calculated by `calculate_distances`, 
    `Imbalance` is the total number of supply/demand,
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
    'num_try': 2,
    'max_window': 10,
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
                find, window_graph = self.window_model(deepcopy(original_graph), delta)
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

    print(''.join(['{:>10}'.format('----------')] + ['{:>10}'.format('----------') for i, x in enumerate(std) if i%3 == 0]))

    print(''.join(['{:>10}'.format('Dist')] + ['{:>10}'.format(x) for i, x in enumerate(header) if i%3 == 0]))
    print(''.join(['{:>10}'.format('Mean')] + ['{:>10.2f}'.format(float(x)) for i, x in enumerate(mean) if i%3 == 0]))
    print(''.join(['{:>10}'.format('Std')] + ['{:>10.2f}'.format(float(x)) for i, x in enumerate(std) if i%3 == 0]))
    print(''.join(['{:>10}'.format('----------')] + ['{:>10}'.format('----------') for i, x in enumerate(std) if i%3 == 0]))

    print(''.join(['{:>10}'.format('Imbalance')] + ['{:>10}'.format(x) for i, x in enumerate(header) if i%3 == 1]))
    print(''.join(['{:>10}'.format('Mean')] + ['{:>10.2f}'.format(float(x)) for i, x in enumerate(mean) if i%3 == 1]))
    print(''.join(['{:>10}'.format('Std')] + ['{:>10.2f}'.format(float(x)) for i, x in enumerate(std) if i%3 == 1]))
    print(''.join(['{:>10}'.format('----------')] + ['{:>10}'.format('----------') for i, x in enumerate(std) if i%3 == 0]))

    print(''.join(['{:>10}'.format('Efficiency')] + ['{:>10}'.format(x) for i, x in enumerate(header) if i%3 == 2]))
    print(''.join(['{:>10}'.format('Mean')] + ['{:>10.2f}'.format(float(x)) for i, x in enumerate(mean) if i%3 == 2]))
    print(''.join(['{:>10}'.format('Std')] + ['{:>10.2f}'.format(float(x)) for i, x in enumerate(std) if i%3 == 2]))
    print(''.join(['{:>10}'.format('----------')] + ['{:>10}'.format('----------') for i, x in enumerate(std) if i%3 == 0]))

if __name__ == '__main__':
    main()