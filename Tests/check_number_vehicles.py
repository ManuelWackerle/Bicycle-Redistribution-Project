"""
Check the optimal number of vehicles to rebalance a given station using an interpolation algorithm
"""
import solvers
from tqdm import tqdm
from structure import ProblemInstance, Vehicle
from scipy import optimize, interpolate
from numpy import argmin
import matplotlib.pyplot as plt
from loaders import load_subset_from_ordered_nodes
import operators as op


def validate_vehicles(array_num_vehicles: list):
    assert len(array_num_vehicles) != 0, "Can't have zero vehicles"
    assert all(n > 0 for n in array_num_vehicles), "Can't have a negative number of vehicles"
    assert all(isinstance(n, int) for n in array_num_vehicles), "Non-integer vehicles seems like a bad idea..."


def run_vns_different_vehicles(problem: ProblemInstance, num_vehicles_array: [int], ordered_nbhs: [], timeout=50):
    """
    Given an array of number of vehicles, compute the distances using VNS and return
    :param problem: Instance on which to test
    :param num_vehicles_array: list of number of vehicles to try
    :param timeout: time allowed in VNS
    :param ordered_nbhs: Ordered list of neighbourhood operators.
    """

    # Validate input array of numbers of vehicles
    validate_vehicles(num_vehicles_array)

    distances = {}
    pbar = tqdm(range(len(num_vehicles_array)))
    pbar.set_description("Number of vehicles evaluated")

    for index in pbar:
        # Add vehicles
        current_vehicles = []
        for index_vehicles in range(num_vehicles_array[index]):
            current_vehicles.append(Vehicle(capacity=20, vehicle_id=str(index_vehicles)))

        problem.vehicles = current_vehicles

        # Compute distance
        solvers.greedy_routing(problem)
        solvers.general_variable_nbh_search(problem, ordered_nbhs)
        distances[len(problem.vehicles)] = problem.calculate_distances()

    return distances


def minimize_interpolate_distances(distances):
    """
    Interpolate the distances to obtain a curve
    """
    vehicles = [int(num_str) for num_str in distances.keys()]
    dists = [int(num_str) for num_str in distances.values()]

    dist_fun = interpolate.interp1d(vehicles, dists)
    optimum_vehicles = optimize.minimize_scalar(dist_fun, bounds=[min(vehicles), max(vehicles)], method="bounded")
    check_optimum = [optimum_vehicles.fun, distances[(min(vehicles))], distances[(max(vehicles))]]
    min_idx = argmin(check_optimum)
    if min_idx == 0:
        optimum = optimum_vehicles
    elif min_idx == 1:
        optimum = min(vehicles)
    else:
        optimum = max(vehicles)

    return optimum


def check_optimal_number_vehicles(problem: ProblemInstance, num_vehicles: [int], ordered_nbhs, plot=False):

    distances = run_vns_different_vehicles(problem, num_vehicles, ordered_nbhs)
    vehicles = [int(num_str) for num_str in distances.keys()]
    dists = [int(num_str) / 1000 for num_str in distances.values()]

    # Plot
    if plot is True:
        plt.plot(vehicles, dists)
        plt.xlabel("Number of Vehicles")
        plt.ylabel("Total distance (km)")
        plt.show()

    optimum_num_vehicles = minimize_interpolate_distances(distances)

    return optimum_num_vehicles


if __name__ == '__main__':

    # Generate graph (randomly with a certain number of nodes)
    graph, node_info = load_subset_from_ordered_nodes(nodes=50, centeredness=5)

    # Mount problem instance with and without zero demand nodes
    problem = ProblemInstance(input_graph=graph, vehicles=[], node_data=node_info, verbose=0)

    # Define the list of neighbourhood operators
    ordered_nbhs = [op.inter_segment_swap, op.intra_two_opt, op.intra_segment_swap, op.inter_two_opt, ]

    # Compute the minimum number of vehicles and plot.
    argmin_vehicles = check_optimal_number_vehicles(problem, range(1, 50), ordered_nbhs, True)
    print("Number of vehicles that minimize the distance: ", argmin_vehicles)
