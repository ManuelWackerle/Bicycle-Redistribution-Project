"""
This is a test for checking machine performance.
This test shows the compute time average and standard diviation of greedy and vns.
"""

import os
import time
import csv
from tqdm import tqdm

import solvers
import operators as ops

from test_base import TestLNS


kwargs = {
    'nodes': 250,
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
    'change_local_nbh': solvers.change_nbh_sequential,
    'root': os.path.join(os.getcwd(), 'results'),
    'read_only': False,
    'filename': 'test_machine_performance.csv',
    'num_try': 2,
}

class TestMachinePerformance(TestLNS):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

    def solver(self, problem):
        solvers.greedy_routing(problem, randomness=False)
        self.run_vns(problem)

    def write_results_to_csv(self, filename, header, num_try):
        writer = csv.writer(open(os.path.join(self.root, filename), "w", newline=''))
        results = []
        for _ in tqdm(range(num_try)):
            problem = self.get_problem_instance()
            st = time.time()
            self.solver(problem)
            results.append(time.time()-st)
        writer.writerow(results)
        return results
    
    def read_results_from_csv(self, filename):
        with open(os.path.join(self.root, filename), 'r') as f:
            reader = csv.reader(f)
            results = next(reader)
        return None, results


def main():
    test_instance = TestMachinePerformance(kwargs=kwargs)
    if kwargs.get('read_only'):
        results = test_instance.read_results_from_csv(kwargs.get('filename', 'test_machine_performance.csv'))
    else:
        results = test_instance.write_results_to_csv(
            filename=kwargs.get('filename', 'test_machine_performance.csv'),
            header=[],
            num_try=kwargs.get('num_try', 100),
        )
    mean, std = test_instance.get_stats(results)
    print('{:>12}[s] {:>12}[s]'.format('mean', 'std'))
    print('{:>12}[s] {:>12}[s]'.format(mean, std))

if __name__ == '__main__':
    main()