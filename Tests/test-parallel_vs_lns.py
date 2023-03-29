"""
This is a test for comparing LNS with parallel VNS and parallel LNS.
This test shows the plot and numbers of final distance and time average and standard diviations of each.
Change kwargs to test with another parameters. Especially, 'read_only' for reading results from file or not, 'num_try' for number of trials, and root for root directory of the project.
"""

import os
import time
from multiprocessing import Pool
from functools import partial

from copy import deepcopy
import solvers
import operators as ops
from matplotlib import pyplot as plt
import numpy as np

from test_base import TestLNS

kwargs = {
    'nodes': 250,
    'centeredness': 5,
    'number_of_vehicles': 5,
    'vehicle_capacity': 20,
    'distance_limit': 200000,  # meter
    'num_initial': 10,
    'num_processors': 5,
    'ordered_nbhs': [ops.intra_two_opt, ops.intra_segment_swap, ops.inter_two_opt, ops.inter_segment_swap],
    'ordered_large_nbhs': [1, 3, 5, 10, 15, 20, 25, 30, 40, 50],
    'change_local_nbh': solvers.change_nbh_sequential,
    'change_large_nbh': solvers.change_nbh_pipe,
    'large_nbh_operator': ops.multi_remove_and_insert_station,
    'local_timeout': 2*60,  # second
    'large_timeout': 6*60,  # second
    'local_verbose': 0,
    'large_verbose': 0,
    'randomness': True,
    'filename': 'test_parallel_lns_2.csv',
    'num_try': 2,
    'read_only': True,
    'root': os.path.join(os.getcwd(), 'results'),
}


class TestParallelLNS(TestLNS):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.num_processors = kwargs.get('num_processors', 5)
        self.num_initial = kwargs.get('num_initial', 10)

    def run_greedy_vns(self, problem):
        solvers.greedy_routing(problem, randomness=self.randomness)
        self.run_vns(problem)
        return problem

    def run_parallel_vns(self, problem):
        with Pool(self.num_processors) as p:
            results = p.map(self.run_greedy_vns, [deepcopy(problem) for _ in range(self.num_initial)])
            best_instance, best_distance = self.get_best_instance(results)
        return best_distance

    def reconstruct_and_vns(self, problem, num_removal):
        vehicles = self.large_nbh_operator(problem, num_removal)
        problem.vehicles = vehicles
        problem.calculate_loading_mf()
        self.run_vns(problem)
        return problem

    def get_best_instance(self, problem_instances):
        best_dist = 1e+10
        best_instance = None
        for instance in problem_instances:
            total = instance.calculate_distances()
            if total < best_dist:
                best_dist = total
                best_instance = instance
        return best_instance, best_dist

    def run_paralell_lns(self, problem):
        start_time = time.time()
        solvers.greedy_routing(problem)
        best_instance = problem
        best_distance = problem.calculate_distances()
        while time.time() - start_time < self.large_timeout:
            with Pool(self.num_processors) as p:
                results = p.map(partial(self.reconstruct_and_vns, deepcopy(best_instance)), self.ordered_large_nbhs)
                curr_best_instance, curr_best_distance = self.get_best_instance(results)
                if curr_best_distance < best_distance:
                    best_instance = curr_best_instance
                    best_distance = curr_best_distance
                else:
                    break
        return best_distance

    def run_greedy_lns(self, problem):
        solvers.greedy_routing(problem, randomness=self.randomness)
        self.run_lns(problem)
        return problem.calculate_distances()

    def compare_approaches(self, problem):
        sp = time.time()
        p_dist = self.run_parallel_vns(deepcopy(problem))
        p_time = time.time() - sp
        spl = time.time()
        p_lns_dist = self.run_paralell_lns(deepcopy(problem))
        p_lns_time = time.time() - spl
        sl = time.time()
        lns_dist = self.run_greedy_lns(deepcopy(problem))
        lns_time = time.time() - sl
        return p_dist, p_time, lns_dist, lns_time, p_lns_dist, p_lns_time

    def solver(self, problem):
        p_dist, p_time, lns_dist, lns_time, p_lns_dist, p_lns_time = self.compare_approaches(problem)
        result = [p_dist, p_time, lns_dist, lns_time, p_lns_dist, p_lns_time]
        return result
    
    def show_plot(self, results, header):
        np_results = np.array(results)
        np_mean = np.round(np.mean(np_results, axis=0), decimals=2)
        np_std = np.round(np.std(np_results, axis=0), decimals=2)

        colors = ['#F5A623', '#9013FE', '#7ED321', '#4A90E2', '#F8E71C', '#D0021B']
        x = ['Parallel', 'LNS', 'Par + LNS']
        plt.bar(x, np_mean[[0,2,4]]/1000, alpha=0.8, color=colors, yerr=np_std[[0,2,4]]/1000)
        plt.ylabel('Total Distance [km]')
        plt.show()
        plt.bar(x, np_mean[[1,3,5]], alpha=0.8, color=colors, yerr=np_std[[1,3,5]])
        plt.ylabel('Computational time [s]')
        plt.show()

def main():
    test_instance = TestParallelLNS(kwargs=kwargs)
    if kwargs.get('read_only'):
        results = test_instance.read_and_show_plot(kwargs.get('filename', 'test_parallel_lns.csv'))
    else:
        results = test_instance.test_all(
            filename=kwargs.get('filename', 'test_parallel_lns.csv'),
            header=['Parallel Dist [m]', 'Parallel Time [s]', 'LNS Dist [m]', 'LNS Time [s]', 'LNS + Paralell Dist [m]', 'LNS + Parallel Time [s]'],
            num_try=2,
        )

    mean, std = test_instance.get_stats(results)
    print('{:>5} {:>12} {:>12} {:>12} {:>12} {:>12} {:>12}'.format('', 'Par Dist [km]', 'Par Time [s]', 'LNS Dist [km]', 'LNS Time [s]', 'Par+LNS Dist [km]', 'Par+LNS Time [s]'))
    print('{:>5} {:>12.2f} {:>12.2f} {:>12.2f} {:>12.2f} {:>12.2f} {:>12.2f}'.format('Mean', mean[0]/1000, mean[1], mean[2]/1000, mean[3], mean[4]/1000, mean[5]))
    print('{:>5} {:>12.2f} {:>12.2f} {:>12.2f} {:>12.2f} {:>12.2f} {:>12.2f}'.format('Std', std[0]/1000, std[1], std[2]/1000, std[3], std[4]/1000, std[5]))
    

if __name__ == '__main__':
    main()