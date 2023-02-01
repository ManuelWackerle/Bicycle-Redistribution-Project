# import sys
# sys.path.append("/Users/hajime/workspace/tum/CaseStudies/Bicycle-Redistribution-Project/")

import os
import time
import csv
from load_csv import load_graph, load_subset_from_ordered_nodes, load_from_pickle

from copy import deepcopy
import vns
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
    'ordered_nbhs': [ops.inter_two_opt, ops.intra_two_opt, ops.intra_or_opt, ops.multi_remove_and_insert_station],
    'distance_limit': 200000,  # meter
    'num_try': 500,
    'patience': 1,
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
        vns.greedy_routing_v1(problem_instance)
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

def show_stats(operations, hist, title, ylabel):
    np_hist = np.array(hist)
    operation_names = np.array([x.__name__ for x in operations])
    np_mean = np.mean(np_hist, axis=0)
    np_std = np.std(np_hist, axis=0)
    plt.title(title)
    plt.ylabel(ylabel)
    plt.errorbar(operation_names, np_mean, np_std, linestyle='None', marker='^')
    plt.show()


def main():
    from_csv = False

    if from_csv:
        time_hist = read_results_from_csv('nbhs_time_stats.csv')
        dist_hist = read_results_from_csv('nbhs_dist_stats.csv')
    else:
        time_hist, dist_hist = write_results_to_csv(kwargs['ordered_nbhs'], num_try=kwargs['num_try'], patience=kwargs['patience'])
    show_stats(kwargs['ordered_nbhs'], dist_hist, 'Distance improvement', 'Distance [km]')
    show_stats(kwargs['ordered_nbhs'], time_hist, 'Time Taken', 'Time [s]')

if __name__ == '__main__':
    main()