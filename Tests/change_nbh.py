"""
The goal of this test is to compare all the types of neighbourhood changes with different instances to see which ones
produce the best results.
"""

import utils
import os
import csv
import sys
import time
import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy
from tqdm import tqdm
from loaders import load_graph
from structure import (
    Vehicle,
    ProblemInstance
)
from solvers import (
    change_nbh_skewed_sequential,
    change_nbh_cyclic,
    change_nbh_pipe,
    change_nbh_sequential,
    greedy_routing,
    general_variable_nbh_search,
    inter_two_opt,
    intra_two_opt,
    remove_and_insert_station
)

# Create a list of all files that we want to test.
instances_dir = os.path.relpath('..\\..\\Problem Instances', os.path.dirname(os.path.abspath(os.getcwd())))
instances_to_test = os.listdir(instances_dir)

# We don't consider the whole dataset for the test.
instances_to_test.remove("felix_data.csv")
instances_to_test.remove("sample_graph_04.csv")
instances_to_test.remove("sample_graph_01.csv")
instances_to_test.remove("sample_graph_01_edited.csv")
instances_to_test.remove("ordered_nodes.csv")

# instances_to_test.remove("sample_graph_04_edited.csv")


# instances_to_test = ["sample_graph_02.csv"]

# Set list of all neighbourhood changes to test. The sequential nbh change is treated separately
list_of_nbh_changes = [change_nbh_sequential, change_nbh_cyclic, change_nbh_pipe]

# Set list of neighbourhoods to explore
ordered_nbhs = [inter_two_opt, intra_two_opt, remove_and_insert_station]

# Set maximum number of tries
max_tries = 10

# Initialize arrays for the plot of time
all_nodes = []
seq_time = [[] for i in range(max_tries)]
cycle_time = [[] for i in range(max_tries)]
skew_seq_time = [[] for i in range(max_tries)]
pipe_time = [[] for i in range(max_tries)]

# Initialize for plot of distance
seq_dist = [[] for i in range(max_tries)]
cycle_dist = [[] for i in range(max_tries)]
skew_seq_dist = [[] for i in range(max_tries)]
pipe_dist = [[] for i in range(max_tries)]
initial_distances = [[] for i in range(max_tries)]

# Initialize percentage of reduction
seq_percent = [[] for i in range(max_tries)]
cycle_percent = [[] for i in range(max_tries)]
skew_seq_percent = [[] for i in range(max_tries)]
pipe_percent = [[] for i in range(max_tries)]
initial_percent = [[] for i in range(max_tries)]

# Select 1 to print intermediate steps, 0 to not print them.
main_verbose = 1

# Select True to print pictures
plotFigures = True

# Prepare CSV file for writing
fout = open("change_nbh_intermediate_output.csv", "w", newline='')
writer = csv.writer(fout)

print("** Comparing Neighbourhood changes  **")
print("%30s %30s %20s %20s %20s %20s %20s" % ("Instance", "Change Operator", "Skew parameter", "Time taken (s)",
                                              "Initial Distance(km)",
                                              "Final Distance(km)", "% of reduction")) if main_verbose == 1 else None

fout_header = ["Instance", "Change Operator", "Skew parameter", "Time taken (s)",
                                              "Initial Distance(km)",
                                              "Final Distance(km)", "% of reduction"]
writer.writerow(fout_header)
# Loop over the whole set of sample graphs multiple times
for try_num in range(max_tries):
    print("** Try number ", try_num + 1, "of ", max_tries, "**")
    # Loop over all instances
    for instance in instances_to_test:
        # Load graph
        graph, node_info = load_graph(os.path.splitext(instance)[0], path=instances_dir, use_adjacency_matrix=False)
        if graph is None:
            sys.exit("Couldn't create graph")

        # Input vehicle information.
        vehicles = []
        for i in range(5):
            vehicles.append(Vehicle(capacity=20, vehicle_id=str(i)))

        # Mount problem instance
        problem = ProblemInstance(input_graph=graph, vehicles=vehicles, node_data=node_info, verbose=0)
        if try_num == 0:
            all_nodes.append(problem.model.number_of_nodes())
        else:
            pass

        # Compute initial solution
        greedy_routing(problem)
        initial_distance = problem.calculate_distances()
        initial_distances[try_num].append(initial_distance)

        # Save initial solution for later use
        problem_copy = deepcopy(problem)

        # Try sequential change
        start = time.time()
        distance_hist, time_hist, operation_hist = general_variable_nbh_search(problem, ordered_nbhs,
                                                                               change_nbh=change_nbh_sequential,
                                                                               timeout=120, verbose=0)
        end = time.time()
        # utils.show_improvement_graph(distance_hist, time_hist, operation_hist, ordered_nbhs,
        #                              change_nbh_name='sequential')

        seq_time[try_num].append(end-start)
        seq_dist[try_num].append(problem.calculate_distances())
        seq_percent[try_num].append(-(problem.calculate_distances()-initial_distance) / initial_distance * 100)

        # Print in terminal.
        print("%30s %30s %20s %20.3f %20.3f %20.3f %20.3f" % (instance, change_nbh_sequential.__name__, "NA",
                                                              end - start,
                                                              initial_distance / 1000,
                                                              problem.calculate_distances() / 1000,
                                                              (-problem.calculate_distances() + initial_distance) /
                                                              initial_distance * 100)) if main_verbose == 1 else None
        # Write on CSV file
        fout_data = [instance, change_nbh_sequential.__name__, "NA", end - start, initial_distance / 1000,
                     problem.calculate_distances() / 1000,
                     (-problem.calculate_distances() + initial_distance) / initial_distance * 100]
        writer.writerow(fout_data)

        # Reset the problem.
        problem = problem_copy

        # Try cycle change
        start = time.time()
        distance_hist, time_hist, operation_hist = general_variable_nbh_search(problem, ordered_nbhs, change_nbh=change_nbh_cyclic, timeout=120, verbose=0)
        end = time.time()
        # utils.show_improvement_graph(distance_hist, time_hist, operation_hist, ordered_nbhs, change_nbh_name='cyclic')

        cycle_time[try_num].append(end - start)
        cycle_dist[try_num].append(problem.calculate_distances())
        cycle_percent[try_num].append(-(problem.calculate_distances() - initial_distance) / initial_distance * 100)

        print("%30s %30s %20s %20.3f %20.3f %20.3f %20.3f" % (instance, change_nbh_cyclic.__name__, "NA", end - start,
                                                              initial_distance / 1000,
                                                              problem.calculate_distances() / 1000,
                                                              (-problem.calculate_distances() + initial_distance) /
                                                              initial_distance * 100)) if main_verbose == 1 else None

        # Write on CSV file
        fout_data = [instance, change_nbh_cyclic.__name__, "NA", end - start, initial_distance / 1000,
                     problem.calculate_distances() / 1000,
                     (-problem.calculate_distances() + initial_distance) / initial_distance * 100]
        writer.writerow(fout_data)

        # Reset the problem.
        problem = problem_copy

        # Try pipe change
        start = time.time()
        distance_hist, time_hist, operation_hist = general_variable_nbh_search(problem, ordered_nbhs, change_nbh=change_nbh_pipe, timeout=120, verbose=0)
        end = time.time()
        # utils.show_improvement_graph(distance_hist, time_hist, operation_hist, ordered_nbhs, change_nbh_name = 'pipe')

        pipe_time[try_num].append(end - start)
        pipe_dist[try_num].append(problem.calculate_distances())
        pipe_percent[try_num].append(-(problem.calculate_distances() - initial_distance) / initial_distance * 100)

        print("%30s %30s %20s %20.3f %20.3f %20.3f %20.3f" % (instance, change_nbh_pipe.__name__, "NA", end - start,
                                                              initial_distance / 1000,
                                                              problem.calculate_distances() / 1000,
                                                              (-problem.calculate_distances() + initial_distance) /
                                                              initial_distance * 100)) if main_verbose == 1 else None
        # Write on CSV file
        fout_data = [instance, change_nbh_pipe.__name__, "NA", end - start, initial_distance / 1000,
                     problem.calculate_distances() / 1000,
                     (-problem.calculate_distances() + initial_distance) / initial_distance * 100]
        writer.writerow(fout_data)

        # Reset the problem.
        problem = problem_copy

        # Try the skewed neighbourhood change for different skew parameters
        skew_param = 0.1
        start = time.time()
        distance_hist, time_hist, operation_hist = general_variable_nbh_search(problem, ordered_nbhs, change_nbh=change_nbh_skewed_sequential,
                                        skew_param=skew_param, verbose=0, timeout=120)
        end = time.time()
        # utils.show_improvement_graph(distance_hist, time_hist, operation_hist, ordered_nbhs, change_nbh_name = 'skewed sequential')

        skew_seq_time[try_num].append(end-start)
        skew_seq_dist[try_num].append(problem.calculate_distances())
        skew_seq_percent[try_num].append(-(problem.calculate_distances() - initial_distance) / initial_distance * 100)
        print("%30s %30s %20.3f %20.3f %20.3f %20.3f %20.3f" % (instance, change_nbh_skewed_sequential.__name__,
                                                                skew_param, end - start,
                                                                initial_distance / 1000,
                                                                problem.calculate_distances() / 1000,
                                                                (-problem.calculate_distances() + initial_distance)
                                                                / initial_distance * 100)) if main_verbose == 1 else None
        # Write on CSV file
        fout_data = [instance, change_nbh_skewed_sequential.__name__, skew_param, end - start,
                     initial_distance / 1000,
                     problem.calculate_distances() / 1000,
                     (-problem.calculate_distances() + initial_distance) / initial_distance * 100]

        writer.writerow(fout_data)

#Close file
fout.close()

fout = open("change_nbh_average_output.csv", "w", newline='')
writer = csv.writer(fout)

# fout_header = ["Instance", "Change Operator", "Skew parameter", "Avg. Time taken (s)", "Initial Distance(km)",
#                "Avg. Final Distance(km)", "Avg. % of reduction", "Std of % of reduction"]
fout_header = ["Instance", "Change Operator", "Initial Distance(km)", "Avg. Improvement(km)", "STD of Final Distance(km)"]
writer.writerow(fout_header)

# Print averages

for index, instance in enumerate(instances_to_test):
    # Print sequential change average
    fout_data = [instance, change_nbh_sequential.__name__,
                 np.mean(initial_distances, axis=0)[index] / 1000,
                 (np.mean(initial_distances, axis=0)[index] - np.mean(seq_dist, axis=0)[index]) / 1000,
                 np.std(seq_dist, axis=0)[index] / 1000]
    writer.writerow(fout_data)

    # Print pipe change average
    fout_data = [instance, change_nbh_pipe.__name__,
                 np.mean(initial_distances, axis=0)[index] / 1000,
                 (np.mean(initial_distances, axis=0)[index] - np.mean(pipe_dist, axis=0)[index]) / 1000,
                 np.std(pipe_dist, axis=0)[index] / 1000]
    writer.writerow(fout_data)

    # Print cycle change average
    fout_data = [instance, change_nbh_cyclic.__name__,
                 np.mean(initial_distances, axis=0)[index] / 1000,
                 (np.mean(initial_distances, axis=0)[index] - np.mean(cycle_dist, axis=0)[index]) / 1000,
                 np.std(cycle_dist, axis=0)[index] / 1000]
    writer.writerow(fout_data)

    # Print skew sequential change average
    fout_data = [instance, change_nbh_skewed_sequential.__name__,
                 np.mean(initial_distances, axis=0)[index] / 1000,
                 (np.mean(initial_distances, axis=0)[index] - np.mean(skew_seq_dist, axis=0)[index]) / 1000,
                 np.std(skew_seq_dist, axis=0)[index] / 1000]

    writer.writerow(fout_data)

fout.close()

if plotFigures is True:

    plt.figure()
    plt.title("Comparison of different Neighbourhood change operators")

    # Start plot for time
    plt.subplot(2, 1, 1)
    plt.ylabel('Time taken (s)')
    plt.errorbar(all_nodes, np.mean(seq_time, axis=0), yerr=np.std(seq_time, axis=0),
                 marker='o', markersize=5, label='Sequential')
    plt.errorbar(all_nodes, np.mean(cycle_time, axis=0), yerr=np.std(cycle_time, axis=0),
                 marker='o', markersize=5, label='Cyclic')
    plt.errorbar(all_nodes, np.mean(pipe_time, axis=0), yerr=np.std(pipe_time, axis=0),
                 marker='o', markersize=5, label='Pipe')
    plt.errorbar(all_nodes, np.mean(skew_seq_time, axis=0), yerr=np.std(skew_seq_time, axis=0),
                 marker='o', markersize=5, label='Skew')

    # Start plot for distance reductions
    plt.subplot(2, 1, 2)
    plt.xlabel('Number of nodes')
    plt.ylabel('Percentage of reduction')
    plt.errorbar(all_nodes, np.mean(seq_percent, axis=0), yerr=np.std(seq_percent, axis=0),
                 marker='o', markersize=5, label='Sequential')
    plt.errorbar(all_nodes, np.mean(cycle_percent, axis=0), yerr=np.std(cycle_percent, axis=0),
                 marker='o', markersize=5, label='Cyclic')
    plt.errorbar(all_nodes, np.mean(pipe_percent, axis=0), yerr=np.std(pipe_percent, axis=0),
                 marker='o', markersize=5, label='Pipe')
    plt.errorbar(all_nodes, np.mean(skew_seq_percent, axis=0), yerr=np.std(skew_seq_percent, axis=0),
                 marker='o', markersize=5, label='Skew')
    plt.legend()

    plt.show()
