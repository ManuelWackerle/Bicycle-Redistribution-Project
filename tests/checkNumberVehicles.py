"""
Check the optimal number of vehicles to rebalance a given station using an interpolation algorithm
"""
import vns
import time
from structure import ProblemInstance, Vehicle
from scipy import optimize, interpolate
from numpy import argmin
import matplotlib.pyplot as plt


def test_vehicles(num_vehicles: []):
    assert len(num_vehicles) != 0, "Can't have zero vehicles"
    assert all(n > 0 for n in num_vehicles), "Negative vehicles? Really?"
    assert all(isinstance(n, int) for n in num_vehicles), "Non-integer vehicles seems like a bad idea..."


def bare_general_variable_nbh_search(problem_instance, ordered_nbhs: [], change_nbh=vns.change_nbh_sequential,
                                     timeout=10, skew_param=10, verbose=0):
    """
    General VNS with VND
    :param problem_instance: current array of routes for all vehicles
    :param ordered_nbhs: list of ordered neighbourhood operators
    :param change_nbh: What type of neighbourhood change we consider
    :param timeout: maximum execution time
    :param skew_param: see change_nbh_skewed_sequential
    :param verbose: control prints. 1 to print 0 to not print
    :return: best route found.
    """

    start_time = time.time()
    nbh_index = 0

    while nbh_index < len(ordered_nbhs) and time.time() < start_time + timeout:
        new_vehicle_routes = ordered_nbhs[nbh_index](problem_instance)
        vns.calculate_loading_MF(problem_instance)
        problem_instance.display_results(False) if verbose == 1 else None

        # new_routes = [None]*len(new_vehicle_routes)
        # for vehicle_index, vehicle in enumerate(new_vehicle_routes):
        #     new_routes[vehicle_index] = vehicle.route()

        # sequential_variable_nbh_descent(problem_instance, ordered_nbhs)
        if change_nbh == vns.change_nbh_cyclic:
            distances_old = problem_instance.calculate_distances()
            nbh_index = vns.change_nbh_cyclic(problem_instance, new_vehicle_routes, nbh_index, len(ordered_nbhs), verbose)
            if distances_old == problem_instance.calculate_distances():
                break

        elif change_nbh == vns.change_nbh_skewed_sequential:
            nbh_index = vns.change_nbh_skewed_sequential(problem_instance, new_vehicle_routes, nbh_index, skew_param,
                                                         verbose)
        else:
            nbh_index = change_nbh(problem_instance, new_vehicle_routes, nbh_index, verbose)


def run_vns_different_vehicles(problem: ProblemInstance, num_vehicles_array: [int], ordered_nbhs: [], timeout=50):
    """
    Given an array of number of vehicles, compute the distances using VNS and return
    :param problem: Instance on which to test
    :param num_vehicles_array: list of number of vehicles to try
    :param timeout: tima allowed in VNS
    """
    distances = {}
    for index in range(len(num_vehicles_array)):
        # Add vehicles
        current_vehicles = []
        for index_vehicles in range(num_vehicles_array[index]):
            current_vehicles.append(Vehicle(capacity=20, vehicle_id=str(index_vehicles)))

        problem.vehicles = current_vehicles

        # Compute distance
        vns.greedy_routing_v1(problem)
        bare_general_variable_nbh_search(problem, ordered_nbhs, timeout=timeout)
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
    test_vehicles(num_vehicles)
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