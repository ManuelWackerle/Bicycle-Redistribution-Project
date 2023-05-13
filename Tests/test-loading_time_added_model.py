"""
This is a test for checking machine performance.
This test shows the compute time average and standard diviation of greedy and vns.
"""

import os
import time
import itertools
from copy import deepcopy

import solvers
import operators as ops

from tests.test_base import TestLNS


kwargs = {
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
    'change_local_nbh': solvers.change_nbh_cyclic_time,
    'root': os.path.join(os.getcwd(), 'results'),
    'read_only': False,
    'filename': 'test_time_model.csv',
    'num_try': 2,
}

class TestLoadingTimeAddedModel(TestLNS):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

    def _solver(self, problem, time_load, time_station):
        problem.time_station = time_station
        problem.time_load = time_load
        st = time.time()
        solvers.time_model_vns(
            problem, self.ordered_nbhs, change_nbh=self.change_local_nbh, 
            verbose=self.local_verbose, timeout=self.local_timeout, plot=False)
        et = time.time()
        return problem.calculate_distances(), et - st

    def solver(self, problem):
        ret = []
        solvers.greedy_routing(problem, randomness=False)
        for time_station, time_load in itertools.product(range(0, 100, 40), range(0, 100, 40)):
            delivery_time, compute_time = self._solver(deepcopy(problem), time_load, time_station)
            ret.append(delivery_time)
            ret.append(compute_time)
        return ret


def main():
    test_instance = TestLoadingTimeAddedModel(**kwargs)
    if kwargs.get('read_only'):
        header, results = test_instance.read_results_from_csv(f'test_time_model.csv')
    else:
        header = [x 
                  for time_station, time_load in itertools.product(range(0, 100, 40), range(0, 100, 40))
                  for x in [f'D st={time_station}, lo={time_load}', f'C st={time_station}, lo={time_load}']]
        results = test_instance.write_results_to_csv(
            filename=f'test_time_model.csv',
            header=header,
            num_try=kwargs.get('num_try', 100),
        )
    mean, std = test_instance.get_stats(results)

    print(''.join(['{:>10}'.format('Delivery')] + ['{:>15}'.format(x) for i, x in enumerate(header) if i%2 == 0]))
    print(''.join(['{:>10}'.format('Mean')] + ['{:>15.2f}'.format(float(x)) for i, x in enumerate(mean) if i%2 == 0]))
    print(''.join(['{:>10}'.format('Std')] + ['{:>15.2f}'.format(float(x)) for i, x in enumerate(std) if i%2 == 0]))

    print(''.join(['{:>10}'.format('Compute')] + ['{:>15}'.format(x) for i, x in enumerate(header) if i%2 == 1]))
    print(''.join(['{:>10}'.format('Mean')] + ['{:>15.2f}'.format(float(x)) for i, x in enumerate(mean) if i%2 == 1]))
    print(''.join(['{:>10}'.format('Std')] + ['{:>15.2f}'.format(float(x)) for i, x in enumerate(std) if i%2 == 1]))

if __name__ == '__main__':
    main()