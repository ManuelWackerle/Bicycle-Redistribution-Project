"""
This is a test for a model with loading time and stopping time to see how the loading/stopping time changes the final routes.
This test will show the results in the following format:
```
  Delivery   D st=0, lo=0  D st=0, lo=40  D st=0, lo=80  D st=40, lo=0 D st=40, lo=40 D st=40, lo=80  D st=80, lo=0 D st=80, lo=40 D st=80, lo=80
      Mean      168845.50      168845.50      168845.50      168845.50      168845.50      168845.50      168845.50      168845.50      168845.50
       Std        9155.50        9155.50        9155.50        9155.50        9155.50        9155.50        9155.50        9155.50        9155.50
   Compute   C st=0, lo=0  C st=0, lo=40  C st=0, lo=80  C st=40, lo=0 C st=40, lo=40 C st=40, lo=80  C st=80, lo=0 C st=80, lo=40 C st=80, lo=80
      Mean           0.28           0.28           0.28           0.28           0.27           0.28           0.28           0.28           0.28
       Std           0.04           0.04           0.03           0.03           0.04           0.04           0.04           0.04           0.04
```
where `Delivery` is a total delivery time of all vehicles calculated by `calculate_distances` (without loading/stopping time because we want to check how this model change the route), 
and `Compute` is computational time of one greedy+vns. `st` means stopping time, and `lo` means loading time for each bike.

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

More kwargs, check the parent class.
"""

from tests.test_base import TestLNS
import operators as ops
import solvers
import os
import time
import itertools
from copy import deepcopy
import sys
sys.path.append(os.getcwd())


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

    print(''.join(['{:>10}'.format('Delivery')] + ['{:>15}'.format(x) for i, x in enumerate(header) if i % 2 == 0]))
    print(''.join(['{:>10}'.format('Mean')] + ['{:>15.2f}'.format(float(x)) for i, x in enumerate(mean) if i % 2 == 0]))
    print(''.join(['{:>10}'.format('Std')] + ['{:>15.2f}'.format(float(x)) for i, x in enumerate(std) if i % 2 == 0]))

    print(''.join(['{:>10}'.format('Compute')] + ['{:>15}'.format(x) for i, x in enumerate(header) if i % 2 == 1]))
    print(''.join(['{:>10}'.format('Mean')] + ['{:>15.2f}'.format(float(x)) for i, x in enumerate(mean) if i % 2 == 1]))
    print(''.join(['{:>10}'.format('Std')] + ['{:>15.2f}'.format(float(x)) for i, x in enumerate(std) if i % 2 == 1]))


if __name__ == '__main__':
    main()
