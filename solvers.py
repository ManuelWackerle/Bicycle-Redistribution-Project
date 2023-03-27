"""
Implementations of the variable neighbourhood search based on the article Variable Neighbourhood Search: basics and
variants, by P Hansen et al.
"""

import networkx as nx
from copy import deepcopy
import time
import random
import operators as ops

"""
Code or generating an initial feasible solution. 
"""


def greedy_routing(prob, dist_weight=3, tsp_weight=1, randomness=False):
    """
    Finds a set of vehicle routes with low cost based on a greedy approach
    :param prob: Problem instance that the greedy search should be applied to
    :param dist_weight: ratio of movable bikes to distance scoring - the solution is very sensitive to this parameter
    :param tsp_weight: (== 1): ignores tsp solution, (!= 1): use tsp solution to suggest successor nodes by scaling the
    score function
    :param randomness: if False sets seed for reproducibility
    :modifies vehicle: modifies the vehicle routes and loads
    """
    graph = prob.model.copy()
    # mean = prob.mean_distance()
    mean = 15000  # approximate mean for Munich data
    choice_size = 5
    choices = list(range(0, choice_size))

    # Using a tsp solution to select successor nodes where possible
    successors = {}
    if tsp_weight != 1 and not graph.is_directed():
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
        load_all_at_depot(prob, graph)
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
                        index = random.choices(choices, weights)[0]
                        next_stop = v_scores[index][1]
                        next_move = v_scores[index][2]

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
        load_all_at_depot(prob, graph)
        unbalanced = set(graph.nodes)

        while prob.allocated < prob.imbalance:
            for v in prob.vehicles:
                load = v.current_load()
                space = v.capacity() - load
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


def load_all_at_depot(prob, graph_copy):
    source_sup = graph_copy.nodes[prob.depot]['sup']
    if source_sup > 0:
        for v in prob.vehicles:
            source_sup = graph_copy.nodes[prob.depot]['sup']
            move = min(source_sup, v.capacity())
            v.add_stop(prob.depot, move)
            graph_copy.nodes[prob.depot]['sup'] -= move
    else:
        for v in prob.vehicles:
            v.add_stop(prob.depot, 0)


"""
Code for the general VNS.
"""


def change_nbh_sequential(problem, modified_vehicles, nbh, nbh_last_success, ordered_nbhs, verbose) -> int:
    """
        Proceed in a sequential fashion through neighbourhoods and repeat until no improvement. 
        Whenever an operator returns a set of better routes return to first operator, otherwise use the next operator.
        [1,1,2,1, 1,2,3,1, 1,2,3,4,1 ... ]
    """

    if problem.calculate_distances() > problem.calculate_distances(modified_vehicles):
        print("Changing from neighbourhood ", nbh, "to neighbourhood 0") if verbose == 1 else None
        problem.vehicles = modified_vehicles
        problem.calculate_loading_mf()
        problem.remove_unused_stops()
        return 0

    else:
        print("Changing from neighbourhood ", nbh, "to neighbourhood ", end='') if verbose == 1 else None
        nbh = nbh + 1
        print(nbh) if verbose == 1 else None
        return nbh


def change_nbh_cyclic(problem, modified_vehicles, nbh, nbh_last_success, ordered_nbhs, verbose) -> int:
    """
        Proceed in a cyclic fashion through neighbourhoods and repeat until no improvement.
        keep cycling through all operators until none of them provide an improvement. 
        [1,2,3,4, 1,2,3,4, 1,2,3,4... ]
    """
    if problem.calculate_distances() > problem.calculate_distances(modified_vehicles):
        # print("Changing from neighbourhood ", nbh, "to neighbourhood ", nbh + 1) if verbose == 1 else None
        problem.vehicles = modified_vehicles
        nbh_last_success[0] = nbh

    nbh = (nbh + 1) % len(ordered_nbhs)
    if nbh == nbh_last_success[0]:
        nbh = len(ordered_nbhs)

    return nbh


def change_nbh_pipe(problem, modified_vehicles, nbh, nbh_last_success: [], ordered_nbhs, verbose) -> int:
    """
        Proceed in a pipe fashion through neighbourhoods and repeat until no improvement.
        If an operator provides an improvement, use it again, otherwise use the next operator.
        [1,1,1,1, 2,2,2, 3,3,3,3,3...  1,1, 2, 3, ... ]
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


def change_nbh_check_all(problem, modified_vehicles, nbh, nbh_last_success: [], ordered_nbhs, verbose) -> int:
    """
        At every iteration all neighbourhoods are checked and the best improvement taken.
        Compute the solutions by all operators and return the best. 
        [best(1,2,3,4), best(1,2,3,4), ... ]
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


def general_variable_nbh_search(problem_instance, ordered_nbhs: [], change_nbh=change_nbh_sequential,
                                timeout=10, plot=True, verbose=0):
    """
        General VNS with VND
        :param problem_instance: current array of routes for all vehicles
        :param ordered_nbhs: list of ordered neighbourhood operators
        :param change_nbh: What type of neighbourhood change we consider
        :param timeout: maximum execution time
        :param plot: plots the improvement graph for the operators at each iteration
        :param verbose: control prints. 1 to print 0 to not print
        :return: best route found.
    """

    start_time = time.time()
    nbh_index = 0 if change_nbh != change_nbh_check_all else -1
    nbh_last_success = [0]
    distance_hist = []
    time_hist = []
    operation_hist = []

    distance_hist.append(problem_instance.calculate_distances())
    time_start = time.time()
    time_hist.append(0)
    operation_hist.append(0)

    while nbh_index < len(ordered_nbhs) and time.time() < start_time + timeout:

        problem_instance.calculate_loading_mf()
        problem_instance.remove_unused_stops()

        if verbose == 1:
            name = "all" if nbh_index == -1 else ordered_nbhs[nbh_index].__name__
            print("Searching nbh: ", name)
            print('(before)  ', end='')
            problem_instance.display_results(False)

        new_vehicle_routes = ordered_nbhs[nbh_index](problem_instance)
        nbh_index = change_nbh(problem_instance, new_vehicle_routes, nbh_index, nbh_last_success, ordered_nbhs, verbose)

        if verbose == 1:
            print('(after)   ', end='')
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
                     large_timeout=60, timeout=10, local_verbose=1, large_verbose=1):
    """
        Integrate multiple_remove and insert stations as Large neighbourhood.
        Outer loop controls the large neighbourhood change. We use the multiple insert and remove neighbourhood with
        varying number of stations (as given by ordered_large_nbhs) as large neighbourhood operator.
        Multiple remove insert acts as a large-scale shaking procedure.
    
        :param problem_instance: current array of routes for all vehicles
        :param ordered_large_nbhs: list of number of stations to remove for large neighbourhood operators.
        :param ordered_local_nbhs: list of ordered local neighbourhood operators
        :param change_large_nbh: What type of neighbourhood change we consider in the large neighbourhoods
        :param change_local_nbh: Neighbourhood change type in local improvement phase
        :param large_nbh_operator: large neighbourhood operator to use
        :param large_timeout: maximum execution time for the outer LNS loop
        :param timeout: maximum execution time for the inner VNS loop
        :param local_verbose: control prints. 1 to print 0 to not print the changes inside the local improvement.
        :param large_verbose: control prints for change in large neighbourhoods.
        :return: best route found.
    """
    start_time = time.time()
    large_nbh_index = 0
    first_time = True  # To skip the shake in the first trial
    nbh_last_success = [0]
    distance_hist = []
    time_hist = []
    operation_hist = []
    time_shake = []
    shake_effect = []

    while large_nbh_index < len(ordered_large_nbhs) and time.time() < start_time + large_timeout:
        if first_time is False:
            # time_shake.append(time.time() - start_time)
            time_shake.append(time.time()-start_time)
            new_vehicle_routes = large_nbh_operator(problem_instance, ordered_large_nbhs[large_nbh_index])
            shake_effect.append(new_vehicle_routes == problem_instance.get_all_routes())
        else:
            new_vehicle_routes = problem_instance.vehicles
            first_time = False

        problem_instance.calculate_loading_mf()
        problem_instance.remove_unused_stops()
        problem_instance.display_results(False) if large_verbose == 1 else None

        # After the initial large neighbourhood shake, we use VND to locally improve the routes.
        candidate_problem = deepcopy(problem_instance)
        candidate_problem.vehicles = new_vehicle_routes
        candidate_problem.calculate_loading_mf()

        distance_hist_local, time_hist_local, operation_hist_local = \
            general_variable_nbh_search(candidate_problem, ordered_local_nbhs, change_nbh=change_local_nbh, 
                                        timeout=timeout, verbose=local_verbose)

        # End of local improvement phase
        if change_large_nbh == change_nbh_cyclic:
            distances_old = problem_instance.calculate_distances()
            large_nbh_index_old = large_nbh_index
            large_nbh_index = change_nbh_cyclic(problem_instance, candidate_problem.vehicles, large_nbh_index,
                                                nbh_last_success, ordered_large_nbhs, verbose=0)
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
            large_nbh_index = change_large_nbh(problem_instance, candidate_problem.vehicles,
                                               large_nbh_index, nbh_last_success, ordered_large_nbhs, verbose=0)
            if large_verbose == 1 and large_nbh_index < len(ordered_large_nbhs):
                print("Large neighbourhood change remove ", ordered_large_nbhs[large_nbh_index_old],
                      "to remove", ordered_large_nbhs[large_nbh_index], "stations")

        distance_hist = distance_hist + distance_hist_local
        time_hist = time_hist + [element + time_shake[-1] for element in time_hist_local] if len(time_shake) != 0 else time_hist_local
        operation_hist = operation_hist + operation_hist_local

    return distance_hist, time_hist, operation_hist, time_shake, shake_effect
