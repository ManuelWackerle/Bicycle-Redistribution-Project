import sys
sys.path.append("/Users/hajime/workspace/tum/CaseStudies/Bicycle-Redistribution-Project/")

import os
import time
import csv
from tqdm import tqdm
from multiprocessing import Pool
from functools import partial
from loaders import load_graph, load_subset_from_ordered_nodes, load_from_pickle

from copy import deepcopy
import solvers
import operators as ops
from structure import ProblemInstance, Vehicle
from matplotlib import pyplot as plt
import numpy as np
from statistics import stdev

kwargs = {
    'nodes': 250,
    'centeredness': 5,
    'number_of_vehicles': 5,
    'vehicle_capacity': 20,
    'ordered_nbhs': [ops.intra_two_opt,
                     ops.intra_segment_swap,
                     ops.inter_two_opt,
                     ops.inter_segment_swap],
                    #  ops.multi_remove_and_insert_station,
                    #  ops.destroy_local],
    'distance_limit': 200000,  # meter
    'num_parallel': 10,
    'ordered_large_nbhs': [1, 3, 5, 10, 15, 20, 25, 30, 40, 50],
    'local_timeout': 2*60,  # second
    'large_timeout': 6*60,  # second
    'local_verbose': 0,
    'large_verbose': 0,
}

PATH = os.path.join(os.getcwd(), 'results')

def _get_problem_instance():
    graph, node_info = load_subset_from_ordered_nodes(nodes=kwargs['nodes'], centeredness=kwargs["centeredness"], randomness=True)
    vehicles = [Vehicle(capacity=kwargs['vehicle_capacity'], vehicle_id=str(i), distance_limit=kwargs["distance_limit"])
                for i in range(kwargs['number_of_vehicles'])]

    problem = ProblemInstance(input_graph=graph, vehicles=vehicles, node_data=node_info, verbose=0)
    return problem

def run_vns(problem):
    solvers.general_variable_nbh_search(
        problem, kwargs['ordered_nbhs'], change_nbh=vns.change_nbh_sequential,
        verbose=kwargs["local_verbose"], timeout=kwargs["local_timeout"])
    return problem

def run_greedy_vns(problem):
    vns.greedy_routing(problem, randomness=True)
    run_vns(problem)
    return problem

def run_parallel_lns(problem, num_removal):
    vehicles = ops.multi_remove_and_insert_station(problem, num_removal)
    problem.vehicles = vehicles
    problem.calculate_loading_mf()
    run_vns(problem)
    return problem

def get_best_instance(problem_instances):
    best_dist = 1e+10
    best_instance = None
    for instance in problem_instances:
        total = instance.calculate_distances()
        if total < best_dist:
            best_dist = total
            best_instance = instance
    return best_instance, best_dist

def parallel_approach(problem):
    with Pool(5) as p:
        results = p.map(run_vns, [deepcopy(problem) for _ in range(kwargs['num_parallel'])])
        best_instance, best_distance = get_best_instance(results)
        return best_distance

def lns_approach(problem):
    vns.greedy_routing(problem, randomness=True)
    vns.large_nbh_search(problem,
                        kwargs['ordered_large_nbhs'],
                        kwargs['ordered_nbhs'],
                        change_local_nbh=vns.change_nbh_sequential,
                        change_large_nbh=vns.change_nbh_pipe,
                        large_nbh_operator=ops.multi_remove_and_insert_station,
                        timeout=kwargs["local_timeout"],
                        large_timeout=kwargs["large_timeout"],
                        local_verbose=kwargs["local_verbose"],
                        large_verbose=kwargs["large_verbose"]
                        )
    return problem.calculate_distances()

def compare_approaches(problem):
    sp = time.time()
    parallel_distance = parallel_approach(deepcopy(problem))
    parallel_time_taken = time.time() - sp
    sl = time.time()
    lns_distance = lns_approach(deepcopy(problem))
    lns_time_taken = time.time() - sl
    return parallel_distance, parallel_time_taken, lns_distance, lns_time_taken

def paralell_large_nbh_search(problem):
    start_time = time.time()
    vns.greedy_routing(problem)
    best_instance = problem
    best_distance = problem.calculate_distances()
    while time.time() - start_time < kwargs['large_timeout']:
        with Pool(5) as p:
            results = p.map(partial(run_parallel_lns, deepcopy(best_instance)), kwargs['ordered_large_nbhs'])
            curr_best_instance, curr_best_distance = get_best_instance(results)
            if curr_best_distance < best_distance:
                best_instance = curr_best_instance
                best_distance = curr_best_distance
            else:
                break
    return best_distance/1000, (time.time() - start_time)/60

def write_results_to_csv(file_name, num_try=100):
    writer = csv.writer(open(os.path.join(PATH, file_name), "w", newline=''))
    # header = ['Parallel Dist [km]', 'Parallel Time [m]', 'LNS Dist [km]', 'LNS Time [m]']
    header = ['LNS + Paralell Dist [km]', 'LNS + Parallel Time [m]']
    writer.writerow(header)
    results = []
    for _ in tqdm(range(num_try)):
        problem = _get_problem_instance()
        result = paralell_large_nbh_search(problem)
        # parallel_distance, parallel_time_taken, lns_distance, lns_time_taken = compare_approaches(problem)
        # result = [parallel_distance/1000, parallel_time_taken/60, lns_distance/1000, lns_time_taken/60]
        writer.writerow(result)
        results.append(result)
    return results

def read_results_from_csv(file_name):
    hist = []
    with open(os.path.join(PATH, file_name), 'r') as f:
        reader = csv.reader(f)
        next(reader)  # ignore header
        for row in reader:
            hist.append([float(x) for x in row])
    return hist

def get_stats(results):
    np_results = np.array(results)
    np_mean = np.round(np.mean(np_results, axis=0), decimals=2)
    np_std = np.round(np.std(np_results, axis=0), decimals=2)
    return np_mean.tolist(), np_std.tolist()

def show_plot(results1, results2):
    np_results1 = np.array(results1)
    np_results2 = np.array(results2)
    np_mean1 = np.round(np.mean(np_results1, axis=0), decimals=2)
    np_mean2 = np.round(np.mean(np_results2, axis=0), decimals=2)
    np_mean = np.concatenate([np_mean1, np_mean2])
    np_std1 = np.round(np.std(np_results1, axis=0), decimals=2)
    np_std2 = np.round(np.std(np_results2, axis=0), decimals=2)
    np_std = np.concatenate([np_std1, np_std2])

    # n = np_results.shape[0]
    # x = np.arange(n, dtype=int)
    # plt.plot(x, np_results[:,0], marker='.', linestyle="None", color='r', label='Parallel')
    # plt.axhline(y=np_mean[0], color='r', linestyle='--')#, label='Parallel avg')
    # # plt.fill_between(x, np_mean[0]+np_std[0], np_mean[0]-np_std[0], alpha=0.5, color='r')
    # plt.plot(x, np_results[:,2], marker='.', linestyle="None", color='b', label='LNS')
    # plt.axhline(y=np_mean[2], color='b', linestyle='--')#, label='LNS avg')
    # # plt.fill_between(x, np_mean[2]+np_std[2], np_mean[2]-np_std[2], alpha=0.5, color='b')
    # plt.legend(title='Approaches')
    # plt.ylabel('Total Distance [km]')
    # plt.xlabel('Trial index')
    colors = ['#F5A623', '#9013FE', '#7ED321', '#4A90E2', '#F8E71C', '#D0021B']
    x = ['Parallel', 'LNS', 'Par + LNS']
    # fig, axs = plt.subplots(2, 1)
    plt.bar(x, np_mean[[0,2,4]], alpha=0.8, color=colors, yerr=np_std[[0,2,4]])
    # axs[0].set_title('Total Distance')
    plt.ylabel('Total Distance [km]')
    plt.show()
    # axs[0].set_ylim([0,600])
    plt.bar(x, np_mean[[1,3,5]], alpha=0.8, color=colors, yerr=np_std[[1,3,5]])
    plt.ylabel('Computational time [m]')
    # axs[1].set_ylim([0,3])
    # plt.bar(x, np_mean, alpha=0.8, color=colors, yerr=np_std)
    plt.show()

def main():
    filename = 'test_parallel_lns.csv'
    # results = write_results_to_csv(filename, num_try=100)
    pl_results = read_results_from_csv('test_parallel_lns.csv')
    co_results = read_results_from_csv('test_lns_vs_pal.csv')
    pl_mean, pl_std = get_stats(pl_results)
    co_mean, co_std = get_stats(co_results)
    # print(mean, std)
    print('{:>5} {:>12} {:>12} {:>12} {:>12} {:>12} {:>12}'.format('', 'Par Dist [km]', 'Par Time [m]', 'LNS Dist [km]', 'LNS Time [m]', 'Par+LNS Dist [km]', 'Par+LNS Time [m]'))
    print('{:>5} {:>12} {:>12} {:>12} {:>12} {:>12} {:>12}'.format('Mean', co_mean[0], co_mean[1]*60, co_mean[2], co_mean[3]*60, pl_mean[0], pl_mean[1]*60))
    print('{:>5} {:>12} {:>12} {:>12} {:>12} {:>12} {:>12}'.format('Std', co_std[0], co_std[1]*60, co_std[2], co_std[3]*60, pl_std[0], pl_std[1]*60))

    show_plot(co_results, pl_results)

    # print('----- compare results -----')
    # print(f'Pal: total distance {round(parallel_distance/1000, 3)} [km], time {round(parallel_time_taken/60, 3)} [m]')
    # print(f'LNS: total distance {round(lns_distance/1000, 3)} [km], time {round(lns_time_taken/60, 3)} [m]')
    

if __name__ == '__main__':
    main()