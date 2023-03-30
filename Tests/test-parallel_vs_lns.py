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
    'ordered_large_nbhs': [30, 20, 10, 5, 3, 1, 15, 25, 40, 50],
    'change_local_nbh': solvers.change_nbh_pipe,
    'change_large_nbh': solvers.change_nbh_pipe,
    'large_nbh_operator': ops.multi_remove_and_insert_station,
    'local_timeout': 2*60,  # second
    'large_timeout': 6*60,  # second
    'local_verbose': 0,
    'large_verbose': 0,
    'randomness': True,
    'filename': 'test_compare_parallel_lns_vns_greedy.csv',
    'num_try': 2,
    'read_only': False,
    'root': os.path.join(os.getcwd(), 'results'),
}


class TestParallelLNS(TestLNS):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.num_processors = kwargs.get('num_processors', 5)
        self.num_initial = kwargs.get('num_initial', 10)

    def run_greedy_vns(self, problem):
        self.run_vns(problem)
        return problem

    def run_parallel_vns(self, problem):
        with Pool(self.num_processors) as p:
            results = p.map(self.run_vns, [deepcopy(problem) for _ in range(self.num_initial)])
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

    def run_paralell_lns(self, problem, ordered_large_nbhs):
        start_time = time.time()
        best_instance = problem
        best_distance = problem.calculate_distances()
        while time.time() - start_time < self.large_timeout:
            with Pool(self.num_processors) as p:
                results = p.map(partial(self.reconstruct_and_vns, deepcopy(best_instance)), ordered_large_nbhs)
                curr_best_instance, curr_best_distance = self.get_best_instance(results)
                if curr_best_distance < best_distance:
                    best_instance = curr_best_instance
                    best_distance = curr_best_distance
                else:
                    break
        return best_distance

    def run_greedy_lns(self, problem):
        self.run_lns(problem)
        return problem.calculate_distances()

    def compare_approaches(self, problem):
        sg = time.time()
        solvers.greedy_routing(problem)
        g_dist = problem.calculate_distances()
        g_time = time.time() - sg
        s_vns = time.time()
        vns_p = self.run_vns(deepcopy(problem))
        vns_dist = vns_p.calculate_distances()
        vns_time = time.time() - s_vns
        sp = time.time()
        p_dist = self.run_parallel_vns(deepcopy(problem))
        p_time = time.time() - sp
        spl1 = time.time()
        p_lns_dist_1 = self.run_paralell_lns(deepcopy(problem), self.ordered_large_nbhs)
        p_lns_time_1 = time.time() - spl1
        spl2 = time.time()
        p_lns_dist_2 = self.run_paralell_lns(deepcopy(problem), self.ordered_large_nbhs[:5])
        p_lns_time_2 = time.time() - spl2
        sl = time.time()
        lns_dist = self.run_greedy_lns(deepcopy(problem))
        lns_time = time.time() - sl
        return g_dist, g_time, vns_dist, vns_time, p_dist, p_time, lns_dist, lns_time, p_lns_dist_1, p_lns_time_1, p_lns_dist_2, p_lns_time_2

    def solver(self, problem):
        g_dist, g_time, vns_dist, vns_time, p_dist, p_time, lns_dist, lns_time, p_lns_dist_1, p_lns_time_1, p_lns_dist_2, p_lns_time_2 = self.compare_approaches(problem)
        result = [g_dist, g_time, vns_dist, vns_time, p_dist, p_time, lns_dist, lns_time, p_lns_dist_1, p_lns_time_1, p_lns_dist_2, p_lns_time_2]
        return result
    
    def show_plot(self, results, header):
        np_results = np.array(results)
        np_mean = np.round(np.mean(np_results, axis=0), decimals=2)
        np_std = np.round(np.std(np_results, axis=0), decimals=2)

        colors = ['#F5A623', '#9013FE', '#7ED321', '#4A90E2', '#F8E71C', '#D0021B', 'r', 'b']
        x = ['Greedy', 'VNS', 'Parallel', 'LNS', 'Par + LNS (1)', 'Par + LNS (2)']
        plt.bar(x, np_mean[[0,2,4,6,8,10]]/1000, alpha=0.8, color=colors, yerr=np_std[[0,2,4,6,8,10]]/1000)
        plt.ylabel('Total Distance [km]')
        plt.show()
        plt.bar(x, np_mean[[1,3,5,7,9,11]], alpha=0.8, color=colors, yerr=np_std[[1,3,5,7,9,11]])
        plt.ylabel('Computational time [s]')
        plt.show()

def main():
    test_instance = TestParallelLNS(kwargs=kwargs)
    if kwargs.get('read_only'):
        results = test_instance.read_and_show_plot(kwargs.get('filename', 'test_parallel_lns.csv'))
    else:
        results = test_instance.test_all(
            filename=kwargs.get('filename', 'test_parallel_lns.csv'),
            header=['Greedy Dist [m]', 'Greedy Time [s]', 'VNS Dist [m]', 'VNS Time [s]', 'Parallel Dist [m]', 'Parallel Time [s]', 'LNS Dist [m]', 'LNS Time [s]', 'LNS + Paralell (1) Dist [m]', 'LNS + Parallel (2) Time [s]', 'LNS + Paralell (2) Dist [m]', 'LNS + Parallel (2) Time [s]'],
            num_try=kwargs.get('num_try', 100),
        )

    mean, std = test_instance.get_stats(results)
    print('{:>10} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10}'.format('Dist [km]', 'Greedy', 'VNS', 'Par', 'LNS', 'Par+LNS(1)', 'Par+LNS(2)'))
    print('{:>10} {:>10.2f} {:>10.2f} {:>10.2f} {:>10.2f} {:>10.2f} {:>10.2f}'.format('Mean', mean[0]/1000, mean[2]/1000, mean[4]/1000, mean[6]/1000, mean[8]/1000, mean[10]/1000))
    print('{:>10} {:>10.2f} {:>10.2f} {:>10.2f} {:>10.2f} {:>10.2f} {:>10.2f}'.format('Std', std[0]/1000, std[2]/1000, std[4]/1000, std[6]/1000, std[8]/1000, std[10]/1000))

    print('{:>10} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10}'.format('Time [s]', 'Greedy', 'VNS', 'Par', 'LNS', 'Par+LNS(1)', 'Par+LNS(2)'))
    print('{:>10} {:>10.2f} {:>10.2f} {:>10.2f} {:>10.2f} {:>10.2f} {:>10.2f}'.format('Mean', mean[1], mean[3], mean[5], mean[7], mean[9], mean[11]))
    print('{:>10} {:>10.2f} {:>10.2f} {:>10.2f} {:>10.2f} {:>10.2f} {:>10.2f}'.format('Std', std[1], std[3], std[5], std[7], std[9], std[11]))    

if __name__ == '__main__':
    main()
