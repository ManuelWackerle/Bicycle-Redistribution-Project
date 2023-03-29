"""
This is a test for comparing vns operators.
This test shows plots of distance improvements and time for each operators.
Change kwargs to try different parameters.
"""

import os
import time
import csv
from loaders import load_subset_from_ordered_nodes

import solvers
import operators as ops
from structure import ProblemInstance, Vehicle
from matplotlib import pyplot as plt
import numpy as np

kwargs = {
    'nodes': 250,
    'centeredness': 5,
    'number_of_vehicles': 5,
    'vehicle_capacity': 20,
    'ordered_nbhs': [ops.intra_two_opt,
                     ops.intra_segment_swap,
                     ops.inter_two_opt,
                     ops.inter_segment_swap,
                     ops.multi_remove_and_insert_station,
                     ops.destroy_local],
    'nbhs_names': ['intra_two_opt',
                   'intra_seg_swap',
                   'inter_two_opt',
                   'inter_seg_swap',
                   'rem_ins_1',
                   'destroy_local'],
    'distance_limit': 200000,  # meter
    'num_try': 2,
    'patience': 5,
    'from_csv': True,
    'time_file_name': "nbhs_time_stats_2.csv",
    'dist_file_name': "nbhs_dist_stats_2.csv",
}

PATH = os.path.join(os.getcwd(), 'results')


def _get_problem_instance():
    graph, node_info = load_subset_from_ordered_nodes(nodes=kwargs['nodes'], centeredness=kwargs["centeredness"], randomness=True)
    vehicles = [Vehicle(capacity=kwargs['vehicle_capacity'], vehicle_id=str(i), distance_limit=kwargs["distance_limit"])
                for i in range(kwargs['number_of_vehicles'])]

    problem = ProblemInstance(input_graph=graph, vehicles=vehicles, node_data=node_info, verbose=0)
    return problem

def _get_operation_results(problem_instance, operation, patience=0):
    start_time = time.time()
    init_dist = problem_instance.calculate_distances()
    prev_dist = init_dist
    curr_patience = 0
    while True:
        new_vehicle_routes = operation(problem_instance)
        curr_dist = problem_instance.calculate_distances(new_vehicle_routes)
        if curr_dist < prev_dist:
            prev_dist = curr_dist
            curr_patience = 0
        elif curr_patience < patience:
            curr_patience += 1
        else:
            break
    time_taken = time.time() - start_time
    dist_improvement = (init_dist - prev_dist) / 1000
    return time_taken, dist_improvement

def _get_operations_results(operations, patience=0):
    times = []
    dists = []
    for operation in operations:
        # It should have the same problem_instance and greedyt
        problem_instance = _get_problem_instance()
        solvers.greedy_routing(problem_instance)
        time_taken, dist_improvement = _get_operation_results(problem_instance, operation, patience)
        times.append(time_taken)
        dists.append(dist_improvement)
    return times, dists

def write_results_to_csv(operations, num_try=100, patience=0, time_file_name="nbhs_time_stats.csv", dist_file_name="nbhs_dist_stats.csv"):
    """Get stats for each operation
    """

    time_writer = csv.writer(open(os.path.join(PATH, time_file_name), "w", newline=''))
    dist_writer = csv.writer(open(os.path.join(PATH, dist_file_name), "w", newline=''))
    header = [operation.__name__ for operation in operations]
    time_writer.writerow(header)
    dist_writer.writerow(header)
    time_hist = []
    dist_hist = []

    for _ in range(num_try):
        times, dists = _get_operations_results(operations, patience=patience)
        time_hist.append(times)
        dist_hist.append(dists)
        time_writer.writerow(times)
        dist_writer.writerow(dists)

    return time_hist, dist_hist

def read_results_from_csv(file_name):
    hist = []
    with open(os.path.join(PATH, file_name), 'r') as f:
        reader = csv.reader(f)
        next(reader)  # ignore header
        for row in reader:
            hist.append([float(x) for x in row])
    return hist

def show_stats(operation_names, hist, title, ylabel):
    colors = ['#F5A623', '#9013FE', '#7ED321', '#4A90E2', '#F8E71C', '#D0021B', 'r']
    np_hist = np.array(hist)[:,:-2]
    operation_names = np.array(operation_names)[:-2]
    np_mean = np.mean(np_hist, axis=0)
    np_std = np.std(np_hist, axis=0)
    print('{:>15} {:>12} {:>12} {:>12} {:>12}'.format(ylabel, operation_names[0], operation_names[1], operation_names[2], operation_names[3]))
    print('{:>15} {:>12.2f} {:>12.2f} {:>12.2f} {:>12.2f}'.format('Mean', np_mean[0], np_mean[1], np_mean[2], np_mean[3]))
    print('{:>15} {:>12.2f} {:>12.2f} {:>12.2f} {:>12.2f}'.format('Std', np_std[0], np_std[1], np_std[2], np_std[3]))
    plt.title(title)
    plt.ylabel(ylabel)
    plt.bar(operation_names, np_mean, alpha=0.8, color=colors[:len(operation_names)], yerr=np_std)
    plt.show()


def main():
    if kwargs['from_csv']:
        time_hist = read_results_from_csv(kwargs['time_file_name'])
        dist_hist = read_results_from_csv(kwargs['dist_file_name'])
    else:
        time_hist, dist_hist = write_results_to_csv(
            kwargs['ordered_nbhs'],
            num_try=kwargs['num_try'],
            patience=kwargs['patience'],
            time_file_name=kwargs['time_file_name'],
            dist_file_name=kwargs['dist_file_name'],
        )
    show_stats(kwargs['nbhs_names'], dist_hist, 'Distance improvement', 'Distance [km]')
    show_stats(kwargs['nbhs_names'], time_hist, 'Time Taken', 'Time [s]')

if __name__ == '__main__':
    main()