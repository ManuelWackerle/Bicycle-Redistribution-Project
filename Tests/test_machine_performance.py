# import sys
# sys.path.append("/Users/hajime/workspace/tum/CaseStudies/Bicycle-Redistribution-Project/")

import os
import time
import csv
from tqdm import tqdm
from loaders import load_subset_from_ordered_nodes

import solvers
import operators as ops
from structure import ProblemInstance, Vehicle
import numpy as np


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
}

PATH = os.path.join(os.getcwd(), os.pardir, 'Saved', 'results')

def _get_problem_instance():
    graph, node_info = load_subset_from_ordered_nodes(nodes=kwargs['nodes'], centeredness=kwargs["centeredness"], randomness=False)
    vehicles = [Vehicle(capacity=kwargs['vehicle_capacity'], vehicle_id=str(i), distance_limit=kwargs["distance_limit"])
                for i in range(kwargs['number_of_vehicles'])]

    problem = ProblemInstance(input_graph=graph, vehicles=vehicles, node_data=node_info, verbose=0)
    return problem

def run_vns(problem):
    solvers.general_variable_nbh_search(
        problem, kwargs['ordered_nbhs'], change_nbh=solvers.change_nbh_sequential,
        verbose=kwargs["local_verbose"], timeout=kwargs["local_timeout"])
    return problem

def run_greedy_vns(problem):
    solvers.greedy_routing(problem, randomness=False)
    run_vns(problem)
    return problem

def write_results_to_csv(file_name, num_try=100):
    writer = csv.writer(open(os.path.join(PATH, file_name), "w", newline=''))
    results = []
    for _ in tqdm(range(num_try)):
        problem = _get_problem_instance()
        st = time.time()
        _ = run_greedy_vns(problem)
        results.append(time.time()-st)
    writer.writerow(results)
    return results

def read_results_from_csv(file_name):
    with open(os.path.join(PATH, file_name), 'r') as f:
        reader = csv.reader(f, quoting=csv.QUOTE_NONNUMERIC)
        return next(reader)

def get_stats(results):
    np_results = np.array(results)
    np_mean = np.round(np.mean(np_results), decimals=2)
    np_std = np.round(np.std(np_results), decimals=2)
    return np_mean.tolist(), np_std.tolist()

def main():
    filename = 'test_machine_performance.csv'
    results = write_results_to_csv(filename, num_try=100)
    # results = read_results_from_csv(filename)
    mean, std = get_stats(results)
    print('{:>12}[s] {:>12}[s]'.format('mean', 'std'))
    print('{:>12}[s] {:>12}[s]'.format(mean, std))

if __name__ == '__main__':
    main()