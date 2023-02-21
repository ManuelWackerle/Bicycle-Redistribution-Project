"""
Implementations of the variable neighbourhood search based on the article Variable Neighbourhood Search: basics and
variants, by P Hansen et al.
"""

import networkx as nx
from copy import deepcopy
import time
import random

import operators as ops
import numpy as np
import sys


"""
Neighbourhood operators. Given an initial route, each operator returns an array of new routes obtained by some 
modification of the initial route. We generate as many routes as possible in the given time.
The general type of these functions is:

def neighbourhood_operator ( initial_routes [[]], timeout ) -> list_of_modified_routes: [[[]]]
"""


def greedy_routing_v1(prob, dist_weight=3, tsp_weight=1, randomness=False):
    """
    Finds a set of vehicle routes with low cost based on a greedy approach
    :param prob: Problem instance that the greedy search should be applied to
    :param source: Start and end node (same for all vehicles in this version)
    :param dist_weight: ratio of movable bikes to distance scoring - the solution is very sensitive to this parameter
    :param tsp_weight: (== 1): ignores tsp solution, (!= 1): use tsp solution to suggest successor nodes by scaling the
    score function
    :modifies vehicle: modifies the vehicle routes and loads
    """
    prob.show_header("Searching for routes using basic greedy")
    graph = prob.model.copy()
    # mean = prob.mean_distance()
    mean = 15000  # hardcoded for now - just make it a variable in ProblemInstance
    choice_size = 5
    choices = list(range(0, choice_size))


    # Using a tsp solution to select successor nodes where possible
    successors = {}
    if tsp_weight != 1:
        guided = nx.algorithms.approximation.christofides(graph, weight='dist')
        for i in range(1, len(guided) - 2):
            s, n = guided[i], guided[i + 1]
            if s in successors:
                successors[s].add(n)
            else:
                successors[s] = set(n)

    if prob.imbalance == 0:
        prob.show_warning("bicycle imbalance is zero, greedy search has noting to do.")
    else:
        source_sup = graph.nodes[prob.depot]['sup']
        if source_sup > 0:
            for v in prob.vehicles:
                source_sup = graph.nodes[prob.depot]['sup']
                move = min(source_sup, v.capacity())
                v.add_stop(prob.depot, move)
                graph.nodes[prob.depot]['sup'] -= move
        else:
            for v in prob.vehicles:
                v.add_stop(prob.depot, 0)

        prob.allocated = 0
        while prob.allocated < prob.imbalance:
            for v in prob.vehicles:
                next_stop, next_move, next_score = None, 0, 0
                load = v.current_load()
                space = v.capacity() - load
                curr = v.current_stop()
                v_scores = [[0] for _ in range(choice_size)]

                for n in graph.nodes:
                    score, move = 0, 0
                    if n != curr:
                        dist = graph.edges[curr, n]['dist']
                        sup = graph.nodes[n]['sup']
                        dist = 1 if dist == 0 else dist
                        if sup > 0:
                            move = min(sup, space)
                            score = move * (mean / dist) ** dist_weight
                        elif sup < 0:
                            move = -min(-sup, load)
                            score = -move * (mean / dist) ** dist_weight
                        if tsp_weight != 1:
                            if curr in successors and n in successors[curr]:
                                score *= tsp_weight
                        if randomness:
                            if score >= v_scores[0][0]:
                                v_scores[0] = [score, n, move]
                                v_scores.sort()
                        elif score > next_score:
                            next_score, next_stop, next_move = score, n, move
                if randomness:
                    if v_scores[choice_size - 1][0] != 0:
                        weights = [i[0] ** 2 for i in v_scores]
                        indx = random.choices(choices, weights)[0]
                        next_stop = v_scores[indx][1]
                        next_move = v_scores[indx][2]

                if next_move != 0:
                    v.add_stop(next_stop, load + next_move)
                    graph.nodes[next_stop]['sup'] -= next_move
                if next_move < 0:
                    prob.allocated -= next_move
        for v in prob.vehicles:
            if v.current_stop() != prob.depot:
                v.add_stop(prob.depot, 0)


def random_routing(prob):
    """
    Finds a set of vehicle routes at random
    :param prob: Problem instance
    :modifies vehicle: modifies the vehicle routes and loads
    """
    graph = prob.model.copy()
    if prob.imbalance == 0:
        prob.show_warning("Error: bicycle imbalance is zero")
    else:
        source_sup = graph.nodes[prob.depot]['sup']
        if source_sup > 0:
            for v in prob.vehicles:
                source_sup = graph.nodes[prob.depot]['sup']
                move = min(source_sup, v.capacity())
                v.add_stop(prob.depot, move)
                graph.nodes[prob.depot]['sup'] -= move
        else:
            for v in prob.vehicles:
                v.add_stop(prob.depot, 0)

        prob.allocated = 0
        unbalanced = set(graph.nodes)

        while prob.allocated < prob.imbalance:
            for v in prob.vehicles:
                load = v.current_load()
                space = v.capacity() - load
                curr = v.current_stop()
                stop = random.sample(unbalanced, 1)[0]
                move = 0
                sup = graph.nodes[stop]['sup']
                if sup > 0:
                    move = min(sup, space)
                elif sup < 0:
                    move = -min(-sup, load)
                if move != 0:
                    v.add_stop(stop, load + move)
                    graph.nodes[stop]['sup'] -= move
                    if sup - move == 0:
                        unbalanced.remove(stop)
                if move < 0:
                    prob.allocated -= move

        for v in prob.vehicles:
            if v.current_stop() != prob.depot:
                v.add_stop(prob.depot, 0)

"""
General VNS.
"""

def sequential_variable_nbh_descent(problem_instance, ordered_nbhs: [], timeout=10) -> [[]]:
    """
    ** The idea behind Variable Neighbourhood descent, VND. **
    If a solution is a local optimizer with respect to several neighbourhood structures, then it is more likely to be a
    global optimizer. A sequential VND explores neighbourhoods in a sequence. Works as follows:
    1.  Several neighbourhood structures are ordered in a list, N = {N1,..., Nl_max}
    2.  Starting on a given solution, x, the sequential VND explores Nl for 1<=l<=lmax one after the other in the
        established order. The exploration of neighbourhoods is done in first improvement fashion
    3.  As soon as an improvement in a given neighbourhood occurs, the process is restarted from the first neighbourhood
        (and in the same order) with the new solution
    4.  The process stops when the current solution can not be improved with respect to any neighbourhood
    """
    loop = True
    best_routes = problem_instance.get_all_routes()
    while loop:

        current_nbh = 0  # Start at the first neighbourhood in the list.
        best_routes = problem_instance.get_all_routes()

        time_start = time.time()
        while current_nbh < len(ordered_nbhs) and time.time() < time_start + timeout:
            best_vehicles_in_nbh = ordered_nbhs[current_nbh](problem_instance)
            # modified_routes = local_search_first_improvement(best_routes, nbh_routes, cost_matrix)
            change_nbh_sequential(problem_instance, best_vehicles_in_nbh, current_nbh)

            loop = not (problem_instance.calculate_distances() ==
                        problem_instance.calculate_distances(best_vehicles_in_nbh))
            break
    return best_routes


def change_nbh_sequential(problem, modified_vehicles, nbh, nbh_last_success, ordered_nbhs, verbose) -> int:
    """
    Guides the vns heuristic when exploring a solution space. It decides which nbh will be explored next
    :param problem_instance: currently best instance
    :param modified_vehicles: candidate for better routes (in the vehicle structure)
    :param nbh: neighbourhood to explore
    :param verbose: controls if we want the prints or not
    :return:
    """

    if problem.calculate_distances() > problem.calculate_distances(modified_vehicles):
        print("Changing from neighbourhood ", nbh, "to neighbourhood 0") if verbose == 1 else None
        problem.vehicles = modified_vehicles
        problem.calculate_loading_MF()
        problem.remove_unused_stops()
        return 0

    else:
        print("Changing from neighbourhood ", nbh, "to neighbourhood ", end='') if verbose == 1 else None
        nbh = nbh + 1
        print(nbh) if verbose == 1 else None
        return nbh


def change_nbh_cyclic(problem, modified_vehicles, nbh, nbh_last_success, ordered_nbhs, verbose) -> int:
    """
    Cyclic neighbourhood change step: regardless of whether there is an improvement with respect to some nbh or not, the
    search is continued in the next neighbourhood in the list.
    """
    if problem.calculate_distances() > problem.calculate_distances(modified_vehicles):
        # print("Changing from neighbourhood ", nbh, "to neighbourhood ", nbh + 1) if verbose == 1 else None
        problem.vehicles = modified_vehicles
        nbh_last_success[0] = nbh

    nbh = (nbh + 1) % len(ordered_nbhs)
    if nbh == nbh_last_success[0]:
        nbh = len(ordered_nbhs)

    return nbh


def change_nbh_pipe(problem, modified_vehicles, nbh, nbh_last_success:[], ordered_nbhs, verbose) -> int:
    """
    We proceed in a pipe fashion through neighbourhoods and repeat until no improvement
    """
    if problem.calculate_distances(modified_vehicles) < problem.calculate_distances():
        problem.vehicles = modified_vehicles
        nbh_last_success[0] = nbh

    else:
        nbh_size = len(ordered_nbhs)
        nbh = (nbh + 1) % nbh_size
        if nbh == nbh_last_success[0]:
            nbh = nbh_size

    return nbh


def change_nbh_check_all(problem, modified_vehicles, nbh, nbh_last_success:[], ordered_nbhs, verbose) -> int:
    """
    At every iteration all neighbourhoods are checked and the best improvement taken
    """
    original_distance = problem.calculate_distances()
    best_distance = original_distance
    best_vehicles = problem.vehicles
    for operator in ordered_nbhs:
        new_vehicle_routes = operator(problem)
        new_distance = problem.calculate_distances(new_vehicle_routes)
        if new_distance < best_distance:
            best_distance = new_distance
            best_vehicles = new_vehicle_routes
    problem.vehicles = best_vehicles
    if best_distance == original_distance:
        return len(ordered_nbhs)
    else:
        return -1


def compare_vehicle_routes(a_set_of_routes: [[]], another_set_of_routes: [[]]) -> float:
    """
    We express the structural difference between two sets of routes as the number of different stations. The higher
    the number of non-common station, the less similar the stations are.
    """
    return np.count_nonzero(a_set_of_routes != another_set_of_routes)


def general_variable_nbh_search(problem_instance, ordered_nbhs: [], change_nbh=change_nbh_sequential,
                                timeout=10, plot=True, verbose=0):
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
    nbh_index = 0 if change_nbh != change_nbh_check_all else -1
    nbh_last_success = [0]
    new_vehicle_routes = None
    distance_hist = []
    time_hist = []
    operation_hist = []

    distance_hist.append(problem_instance.calculate_distances())
    time_start = time.time()
    time_hist.append(0)
    operation_hist.append(0)

    while nbh_index < len(ordered_nbhs) and time.time() < start_time + timeout:

        problem_instance.calculate_loading_MF()
        problem_instance.remove_unused_stops()

        if verbose == 1:
            name = "all" if nbh_index == -1 else ordered_nbhs[nbh_index].__name__
            print("Searching nbh: ", name)
            print('(before)  ', end= '')
            problem_instance.display_results(False)

        new_vehicle_routes = ordered_nbhs[nbh_index](problem_instance)
        nbh_index = change_nbh(problem_instance, new_vehicle_routes, nbh_index, nbh_last_success, ordered_nbhs, verbose)

        if verbose == 1:
            print('(after)   ', end= '')
            problem_instance.display_results(False)
        if plot:
            distance_hist.append(problem_instance.calculate_distances())
            time_hist.append(time.time() - time_start)
            operation_hist.append(nbh_index)

    return distance_hist, time_hist, operation_hist if plot else None


def large_nbh_search(problem_instance, ordered_large_nbhs: [int], ordered_local_nbhs: [],
                     change_large_nbh=change_nbh_pipe,
                     change_local_nbh=change_nbh_sequential,
                     large_nbh_operator=ops.destroy_rebuild,
                     large_timeout=60, timeout=10, skew_param=10, local_verbose=1, large_verbose=1):
    """
    Integrate multiple_remove and insert stations as Large neighbourhood.
    :param problem_instance: current array of routes for all vehicles
    :param ordered_large_nbhs: list of number of stations to remove for large neighbourhood operators.
    :param ordered_local_nbhs: list of ordered local neighbourhood operators
    :param change_large_nbh: What type of neighbourhood change we consider in the large neighbourhoods
    :param change_local_nbh: Neighbourhood change type in local improvement phase
    :param timeout: maximum execution time
    :param skew_param: see change_nbh_skewed_sequential
    :param local_verbose: control prints. 1 to print 0 to not print the changes inside the local improvement.
    :param large_verbose: control prints for change in large neighbourhoods.
    :return: best route found.
    """

    start_time = time.time()
    large_nbh_index = 0
    first_time = True  # To skip the shake in the first trial
    distance_hist = []
    time_hist = []
    operation_hist = []
    time_shake = []
    shake_effect = []
    idle_nbhs = [False] * (len(ordered_local_nbhs) + 1)
    """
    Outer loop controls the large neighbourhood change. We use the multiple insert and remove neighbourhood with
    varying number of stations (as given by ordered_large_nbhs) as large neighbourhood operator.

    Multiple remove insert acts as a large-scale shaking procedure.
    """
    while large_nbh_index < len(ordered_large_nbhs) and time.time() < start_time + large_timeout:
        if first_time is False:
            # time_shake.append(time.time() - start_time)
            time_shake.append(time.time())
            # new_vehicle_routes = multi_remove_and_insert_station(problem_instance, ordered_large_nbhs[large_nbh_index])
            old_vehicle_routes = problem_instance.vehicles
            new_vehicle_routes = large_nbh_operator(problem_instance, ordered_large_nbhs[large_nbh_index])
            # new_vehicle_routes = destroy_rebuild(problem_instance, ordered_large_nbhs[large_nbh_index])
            shake_effect.append(new_vehicle_routes == problem_instance.get_all_routes())
        else:
            old_vehicle_routes = problem_instance.vehicles
            new_vehicle_routes = problem_instance.vehicles
            first_time = False

        problem_instance.calculate_loading_MF()
        problem_instance.remove_unused_stops()
        problem_instance.display_results(False) if large_verbose == 1 else None

        """
        After the initial large neighbourhood shake, we use VND to locally improve the routes.
        """
        candidate_problem = deepcopy(problem_instance)
        candidate_problem.vehicles = new_vehicle_routes
        candidate_problem.calculate_loading_MF()

        distance_hist_local, time_hist_local, operation_hist_local = general_variable_nbh_search(candidate_problem,
                                                                                                 ordered_local_nbhs,
                                                                                                 change_nbh=change_local_nbh,
                                                                                                 timeout=timeout,
                                                                                                 verbose=local_verbose)

        """
        End of local improvement phase
        """

        if change_large_nbh == change_nbh_cyclic:
            distances_old = problem_instance.calculate_distances()
            large_nbh_index_old = large_nbh_index
            large_nbh_index = change_nbh_cyclic(problem_instance, candidate_problem.vehicles, large_nbh_index,
                                                len(ordered_large_nbhs), verbose=0)
            if large_verbose == 1 and large_nbh_index < len(ordered_large_nbhs):
                print("Large neighbourhood change remove ", ordered_large_nbhs[large_nbh_index_old],
                      "to remove", ordered_large_nbhs[large_nbh_index], "stations")
            if distances_old == problem_instance.calculate_distances():
                break

        elif change_large_nbh == change_nbh_pipe:
            large_nbh_index_old = large_nbh_index
            large_nbh_index = change_nbh_pipe(problem=problem_instance,
                                              modified_vehicles=candidate_problem.vehicles,
                                              nbh=large_nbh_index,
                                              nbh_last_success=[0],
                                              ordered_nbhs=ordered_large_nbhs,
                                              verbose=0)
            if large_verbose == 1 and large_nbh_index < len(ordered_large_nbhs):
                print("Large neighbourhood change remove ", ordered_large_nbhs[large_nbh_index_old],
                      "to remove", ordered_large_nbhs[large_nbh_index], "stations")
        else:
            large_nbh_index_old = large_nbh_index
            large_nbh_index = change_large_nbh(problem_instance, candidate_problem.vehicles, large_nbh_index, verbose=0)
            if large_verbose == 1 and large_nbh_index < len(ordered_large_nbhs):
                print("Large neighbourhood change remove ", ordered_large_nbhs[large_nbh_index_old],
                      "to remove", ordered_large_nbhs[large_nbh_index], "stations")

        distance_hist = distance_hist + distance_hist_local
        # time_hist = time_hist + [element + time_hist[-1] for element in time_hist_local]
        time_hist = time_hist + time_hist_local
        operation_hist = operation_hist + operation_hist_local
    time_hist = [element - start_time for element in time_hist]
    time_shake = [element - start_time for element in time_shake]

    return distance_hist, time_hist, operation_hist, time_shake, shake_effect
