"""
Implementations of the variable neighbourhood search based on the article Variable Neighbourhood Search: basics and
variants, by P Hansen et al.
"""

import networkx as nx
from copy import deepcopy
import time
from tqdm import tqdm
import random
random.seed(8)

import numpy as np

"""
Neighbourhood operators. Given an initial route, each operator returns an array of new routes obtained by some 
modification of the initial route. We generate as many routes as possible in the given time.
The general type of these functions is:

def neighbourhood_operator ( initial_routes [[]], timeout ) -> list_of_modified_routes: [[[]]]
"""


def greedy_routing_v1(prob, source='0', dist_weight=3, tsp_weight=1):
    """
    Finds a set of vehicle routes with low cost based on a greedy approach
    :param prob: Problem instance that the greedy search should be applied to
    :param source: Starting node (same for all vehicles in this version)
    :param dist_weight: ratio of movable bikes to distance scoring - the solution is very sensitive to this parameter
    :param tsp_weight: (== 1): ignores tsp solution, (!= 1): use tsp solution to suggest successor nodes by scaling the
    score function
    :modifies vehicle: modifies the vehicle routes and loads
    """

    prob.show_header("Searching for routes using basic greedy")
    graph = prob.model.copy()
    mean = prob.mean_distance()

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
        source_sup = graph.nodes[source]['sup']
        if source_sup > 0:
            for v in prob.vehicles:
                source_sup = graph.nodes[source]['sup']
                move = min(source_sup, v.capacity())
                v.add_stop(source, move)
                graph.nodes[source]['sup'] -= move
        else:
            for v in prob.vehicles:
                v.add_stop(source, 0)

        prob.allocated = 0
        while prob.allocated < prob.imbalance:
            for v in prob.vehicles:
                next_stop, next_move, next_score = None, 0, 0
                load = v.current_load()
                space = v.capacity() - load
                curr = v.current_stop()
                for n in graph.nodes:
                    score, move = 0, 0
                    if n != curr:
                        dist = graph.edges[curr, n]['dist']
                        sup = graph.nodes[n]['sup']
                        if sup > 0:
                            move = min(sup, space)
                            score = move * (mean / dist) ** dist_weight
                        elif sup < 0:
                            move = -min(-sup, load)
                            score = -move * (mean / dist) ** dist_weight
                        if tsp_weight != 1:
                            if curr in successors and n in successors[curr]:
                                score *= tsp_weight
                        if score >= next_score:
                            next_stop, next_move = n, move
                            next_score = score
                if next_move != 0:
                    v.add_stop(next_stop, load + next_move)
                    graph.nodes[next_stop]['sup'] -= next_move
                if next_move < 0:
                    prob.allocated -= next_move
        for v in prob.vehicles:
            if v.current_stop() != source:
                v.add_stop(source, 0)


def greedy_routing_PILOT(prob):
    """
    Finds a set of vehicle routes with low cost based on a greedy approach.
    Version 1: greedy search using the PILOT technique
    :return routes: the set vehicle routes
    """

    pass  # Todo: implement


def calculate_loading_MF(prob, start_load=0, source='0'):
    """
    Given a set of vehicle routes, calculates optimal loading instructions for each route using a Maximum flow computation.
    Use this function if monotonicity is assumed.

    :return instructions: The instructions for how to load and unload the bicycles.
    """

    # Generate Max Flow graph
    prob.show_header("Generating Max flow graph")
    total_source, total_sink = 0, start_load
    mf_graph = nx.DiGraph()
    mf_graph.add_node('s')  # source node
    mf_graph.add_node('t')  # sink node
    for v, d in prob.model.nodes(data='sup'):
        # if v == source:
        #     total_source += d+start_load
        #     mf_graph.add_edge('s', v, capacity=d+start_load)
        #     mf_graph.add_edge(v, 't', capacity=start_load)
        # else:
        if d > 0:
            total_source += d
            mf_graph.add_edge('s', v, capacity=d)
        elif d < 0:
            total_sink -= d
            mf_graph.add_edge(v, 't', capacity=-d)

    for l in range(len(prob.vehicles)):
        v = prob.vehicles[l]
        path = v.route()
        prev_node = 0
        for r in range(len(path)):
            node = path[r]
            node_str = "{}-{}-{}".format(node, l, r)
            demand = prob.model.nodes[node]['sup']
            # if node == source:
            #     mf_graph.add_edge(node, node_str)
            #     mf_graph.add_edge(node_str, node)
            if demand > 0:
                mf_graph.add_edge(node, node_str)
            elif demand < 0:
                mf_graph.add_edge(node_str, node)
            if prev_node != 0:
                mf_graph.add_edge(prev_node, node_str, capacity=v.capacity())
            prev_node = node_str
    prob.show_info("Graph generated with {n} nodes and {e} edges. Source flow: {s}, Sink flow: {t}"
                   .format(n=len(mf_graph.nodes), e=len(mf_graph.edges), s=total_source, t=total_sink))

    # Solve Max Flow Problem
    prob.show_header("Solving the Max flow problem ")
    prob.imbalance = total_source - start_load
    if total_sink != total_source:
        prob.show_warning("mismatch in source and sink flow capacity, no exact solution can exist.")
        prob.imbalance = -1

    # This is where the magic happens
    # print(mf_graph.edges.data())
    value, data = nx.maximum_flow(mf_graph, 's', 't')  # , flow_func=nx.algorithms.flow.shortest_augmenting_path()
    #  TODO: investigate this algorithm exactly and see if it can be done better
    prob.allocated = value - start_load

    if value != total_source or value != total_sink:
        prob.show_warning(
            "Bikes can not be allocated to full capacity. Source flow: {s}, Sink flow: {t}, Allocated: {a}"
            .format(s=total_source, t=total_sink, a=value))
    else:
        prob.show_info("Bike allocation is exact. Total allocated bicycles: {}".format(value))

    prob.show_header("Generating instructions")
    for l in range(len(prob.vehicles)):
        loads = []
        path = prob.vehicles[l].route()
        for r in range(len(path) - 1):
            node = path[r]
            next = path[r + 1]
            node_str = "{}-{}-{}".format(node, l, r)
            next_str = "{}-{}-{}".format(next, l, r + 1)
            loads.append(data[node_str][next_str])
        prob.vehicles[l].set_loads(loads)


def remove_unused_stops(prob):
    """
    Given a set of vehicle routes, removes all the stops where the vehicle neither load nor unloads any bikes.
    """
    for v in prob.vehicles:
        remove = []
        prev_load = 0
        for s in range(1, len(v.route())-1):  # ignore first and last stops (source)
            load = v.loads()[s]
            if load == prev_load:
                remove.append(s)
            prev_load = load
        remove.reverse()
        for r in remove:
            v.remove_stop(r)


def intra_two_opt(prob, tolerance=0):
    swaps = []
    for l in range(len(prob.vehicles)):
        v = prob.vehicles[l]
        swaps.append([])
        route = v.route()
        b1, b2 = 0, 0
        for s1 in range(1, len(route) - 4):
            ri, rj = route[s1], route[s1 + 1]
            best, value, new_value = 0, 0, 0
            for s2 in range(s1 + 2, len(route) - 2):
                rk, rl = route[s2], route[s2 + 1]
                if ri != rk and ri != rl and rj != rk and rj != rl:
                    value = prob.model.edges[ri, rj]['dist'] + prob.model.edges[rk, rl]['dist']
                    new_value = prob.model.edges[ri, rk]['dist'] + prob.model.edges[rj, rl]['dist']
                    diff = value - new_value
                    if diff > best:
                        best = diff
                        b1, b2 = s1, s2

            if best > 0:
                swaps[l].append([best, b1, b2])

    original_vehicles = deepcopy(prob.vehicles)
    out = []
    for l in range(len(prob.vehicles)):
        # print(prob.vehicles[l].route())
        v = prob.vehicles[l]
        out.append(v)
        swaps[l].sort(reverse=True)
        mn, mx = len(v.route()) + 1, -1
        route = deepcopy(v.route())
        for s in range(len(swaps[l])):
            _, b1, b2 = swaps[l][s]
            if b2 < mn - 1 or mx + 1 < b1:
                v.set_route(route[:b1 + 1] + route[b2:b1:-1] + route[b2 + 1:])
                calculate_loading_MF(prob)
                if prob.allocated >= prob.imbalance - tolerance:
                    route = deepcopy(v.route())
                    mn, mx = min(b1, mn), max(b2, mx)
                else:
                    v.set_route(route)
    prob.vehicles = original_vehicles
    return out


def inter_two_opt(prob, tolerance=0):
    swaps = []
    clip = 5
    for l1, v1 in enumerate(prob.vehicles):
        route1 = prob.vehicles[l1].route()
        for l2 in range(l1 + 1, len(prob.vehicles)):
            route2 = prob.vehicles[l2].route()
            r1, r2, b1, b2 = 0, 0, 0, 0
            best, value, new_value = 0, 0, 0
            for s1 in range(clip, len(route1) - clip):
                ri, rj = route1[s1], route1[s1 + 1]
                for s2 in range(s1 - clip, min(s1 + clip, len(route2) - 1)):
                    rk, rl = route2[s2], route2[s2 + 1]
                    if ri != rk and ri != rl and rj != rk and rj != rl:
                        value = prob.model.edges[ri, rj]['dist'] + prob.model.edges[rk, rl]['dist']
                        new_value = prob.model.edges[ri, rl]['dist'] + prob.model.edges[rk, rj]['dist']
                        diff = value - new_value
                        if diff > best:
                            best = diff
                            r1, r2, b1, b2 = l1, l2, s1, s2
            if best > 0:
                swaps.append([best, r1, r2, b1, b2])

    swaps.sort(reverse=True)
    used = set()
    out = []
    original_vehicles = deepcopy(prob.vehicles)
    for s in range(len(swaps)):
        _, r1, r2, b1, b2 = swaps[s]
        if r1 not in used and r2 not in used:
            # Todo, implement index change tracking so that the other swaps can also be used and delete this check
            v1, v2 = prob.vehicles[r1], prob.vehicles[r2]
            route1 = deepcopy(v1.route())
            route2 = deepcopy(v2.route())
            v1.set_route(route1[:b1 + 1] + route2[b2 + 1:])
            v2.set_route(route2[:b2 + 1] + route1[b1 + 1:])
            calculate_loading_MF(prob)
            if prob.allocated >= prob.imbalance - tolerance:
                route1 = deepcopy(v1.route())
                route2 = deepcopy(v2.route())
                used.add(r1)
                used.add(r2)
            else:
                v1.set_route(route1)
                v2.set_route(route2)
    for v in prob.vehicles:
        out.append(v)
    prob.vehicles = original_vehicles
    return out


def remove_one_station_generator(vehicles, at_random=False):
    """generate routes by removing one station

    Write
        N = Number of trucks (=len(self.routes))
        Ln = Route length (=len(self.routes[n])-1)
        C = Number of candidates
    Note
        Ln depends on n in N. (route length might be different for each vehicles)
    :return
        Generator which generates vechiles with the shape (N, Ln) with total number C
    """
    idxes = []
    for i in range(len(vehicles)):
        if len(vehicles[i].route()) <= 2:
            # if route length is less than two, it contains only depot
            continue
        # ignore index 0 and -1 since both are depot
        for j in range(1, len(vehicles[i].route())-1):
            idxes.append([i, j])

    if at_random:
        random.shuffle(idxes)  # generate at random

    for i, j in idxes:
        candidate = deepcopy(vehicles)
        candidate[i].set_route(candidate[i].route()[:j] + candidate[i].route()[j+1:])
        candidate[i].set_loads(candidate[i].loads()[:j] + candidate[i].loads()[j+1:])
        yield candidate


def _get_rebalanced_graph(graph, vehicles):
    """given routes and instructions, returns graph after rebalance
    """
    for v in vehicles:
        prev_load = 0
        for s in range(len(v.route())-1):
            load = v.loads()[s]
            diff = prev_load - load
            prev_load = load
            graph.nodes[v.route()[s]]['sup'] += diff
    return graph


def insertU_nearest_generator(vehicles, unbalanced_stations, graph):
    """insert unbalanced stations to minimum distance position on each route

    return:
        Generator which generate insertU candidate
    """
    copied_vehicles = deepcopy(vehicles)
    random.shuffle(copied_vehicles)
    random.shuffle(unbalanced_stations)
    for u in unbalanced_stations:
        for vehicle in copied_vehicles:
            candidate = deepcopy(vehicles)
            route = vehicle.route()
            j = 1
            if route[0] == u or route[1] == u:
                continue
            distance = graph.edges[route[0], u]['dist'] + graph.edges[route[1], u]['dist']  # init
            for k in range(1, len(route) - 1):
                if route[j-1] == u or route[j] == u:
                    continue
                current_distance = graph.edges[route[j-1], u]['dist'] + graph.edges[route[j], u]['dist']
                if current_distance < distance:
                    distance = current_distance
                    j = k
            vehicle.set_route(vehicle.route()[:j] + [u] + vehicle.route()[j:])
            # set new load as 0
            vehicle.set_loads(vehicle.loads()[:j] + [0] + vehicle.loads()[j:])
            yield candidate


def insertU_generator(vehicles, unbalanced_stations, at_random=False):
    """generate routes by inserting unbalanced station

    Write
        N = Number of trucks (=len(self.routes))
        Ln = Route length (=len(self.routes[n])-1)
        C = Number of candidates
    Note
        Ln depends on n in N. (route length might be different for each vehicles)
    :return
        Generator which generates vechiles with the shape (N, Ln) with total number C
    """
    idxes = []
    for u in unbalanced_stations:
        for i in range(len(vehicles)):
            if len(vehicles[i].route()) <= 2:
                # if route length is less than two, it contains only depot
                continue
            for j in range(1, len(vehicles[i].route()) - 1):
                idxes.append([i, j, u])

    if at_random:
        random.shuffle(idxes)

    for i, j, u in idxes:
        candidate = deepcopy(vehicles)
        candidate[i].set_route(candidate[i].route()[:j] + [u] + candidate[i].route()[j:])
        candidate[i].set_loads(candidate[i].loads()[:j] + [0] + candidate[i].loads()[j:])  # set new load as 0
        yield candidate


def _get_loading_and_unbalanced_stations(problem_instance, vehicles):
    """Given vehicles, calculate best loading instructions and return unbalanced stations
    """
    calculate_loading_MF(problem_instance)
    graph = _get_rebalanced_graph(problem_instance.model.copy(), vehicles)
    unbalanced_stations = [x for x in graph.nodes if graph.nodes[x]['sup'] != 0]
    return unbalanced_stations


def remove_and_insert_station(problem_instance):
    """Remove and insert station VNS.
       The balanced feasibility is satisfied.
       As soon as it finds a feasible candidate, return

    :return
        vehicles
    """
    copied_problem_instance = deepcopy(problem_instance)

    for removed_vehicles in tqdm(remove_one_station_generator(copied_problem_instance.vehicles, at_random=True)):
        unbalanced_stations = _get_loading_and_unbalanced_stations(copied_problem_instance, removed_vehicles)
        if not unbalanced_stations:
            # if removal neighbor routes are possibly balanced, return them
            return removed_vehicles
        for inserted_vehicles in insertU_nearest_generator(removed_vehicles, unbalanced_stations, copied_problem_instance.model.copy()):
            # for inserted_vehicles in insertU_generator(removed_vehicles, unbalanced_stations, at_random=True):
            unbalanced_stations = _get_loading_and_unbalanced_stations(copied_problem_instance, inserted_vehicles)
            if not unbalanced_stations:
                return inserted_vehicles
    # if there is no candidate, return original
    return copied_problem_instance.vehicles

"""
General VNS.
"""

"""
** Summary of Paper explanation. **
The goal of the local search is to find local optimizers within neighbourhoods. In a first improvement strategy, as soon 
as a route in the neighbourhood with lower cost is found, it is set as the current route.
"""


# def local_search_first_improvement(current_routes, nbh_routes: [[[]]], cost_matrix: [[]]) -> [[]]:
#     """
#     As soon as an improving solution is detected in a neighbourhood, it is set as the new current solution
#     :param current_routes: routes to improve.
#     :param nbh_routes: array of routes in the neighbourhood.
#     :param cost_matrix: to compute total cost of routes.
#     :return:
#     """
#     while True:
#         index = 0
#         best_routes = current_routes
#
#         while index in range(len(nbh_routes)):
#             if compute_route_cost(nbh_routes[index], cost_matrix) < compute_route_cost(current_routes, cost_matrix):
#                 current_routes = nbh_routes[index]
#                 break
#         TODO: Change how to compute the route cost now using the built in function inside the ProblemInstance class
#         if compute_route_cost(best_routes, cost_matrix) <= compute_route_cost(current_routes, cost_matrix):
#             break
#
#     return best_routes


# def shake_solution(problem_instance, nbh_operator) -> [[]]:
#     """
#     Return a random solution from the given neighbourhood
#     :param problem_instance: array of routes to shake
#     :param nbh_operator: neighbourhood operator that takes a set of routes and returns a neighbouring route
#     :return: modified routes in the neighbourhood
#     """
#
#     list_of_candidate_routes = nbh_operator(problem_instance)
#     index = randint(0, len(list_of_candidate_routes))
#
#     return list_of_candidate_routes[index]


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


def change_nbh_sequential(problem_instance, modified_vehicles: [], nbh, verbose=0) -> int:
    """
    Guides the vns heuristic when exploring a solution space. It decides which nbh will be explored next
    :param problem_instance: currently best instance
    :param modified_vehicles: candidate for better routes (in the vehicle structure)
    :param nbh: neighbourhood to explore
    :param verbose: controls if we want the prints or not
    :return:
    """
    if problem_instance.calculate_distances() > problem_instance.calculate_distances(modified_vehicles):
        print("Changing from neighbourhood ", nbh, "to neighbourhood ", end='') if verbose == 1 else None
        problem_instance.vehicles = modified_vehicles
        nbh = 0
        print(nbh) if verbose == 1 else None
    else:
        print("Changing from neighbourhood ", nbh, "to neighbourhood ", end='') if verbose == 1 else None
        nbh = nbh + 1
        print(nbh) if verbose == 1 else None

    return nbh


def change_nbh_cyclic(problem_instance, modified_vehicles: [], nbh, number_of_nbhs, verbose=0) -> int:
    """
    Cyclic neighbourhood change step: regardless of whether there is an improvement with respect to some nbh or not, the
    search is continued in the next neighbourhood in the list.
    """
    if problem_instance.calculate_distances() > problem_instance.calculate_distances(modified_vehicles):
        print("Changing from neighbourhood ", nbh, "to neighbourhood ", nbh+1) if verbose == 1 else None
        problem_instance.vehicles = modified_vehicles

    nbh = (nbh + 1) % number_of_nbhs

    return nbh


def change_nbh_pipe(problem_instance, modified_vehicles: [], nbh, verbose=0) -> int:
    """
    If an improvement of the current solution occurs in some neighbourhood the search is continued in that
    neighbourhood. There is no returning to the beginning.
    """
    if problem_instance.calculate_distances() > problem_instance.calculate_distances(modified_vehicles):
        problem_instance.vehicles = modified_vehicles
    else:
        print("Changing from neighbourhood ", nbh, "to neighbourhood ", nbh + 1) if verbose == 1 else None
        nbh = nbh + 1

    return nbh


def compare_vehicle_routes(a_set_of_routes: [[]], another_set_of_routes: [[]]) -> float:
    """
    We express the structural difference between two sets of routes as the number of different stations. The higher
    the number of non-common station, the less similar the stations are.
    """
    return np.count_nonzero(a_set_of_routes != another_set_of_routes)


def change_nbh_skewed_sequential(problem_instance, modified_vehicles: [], nbh, skew_param=10, verbose=0) -> int:
    """
    We accept improving and non-improving solutions. The goal of this neighbourhood search is to allow the exploration
    of valleys far away from the current solution.

    """
    route_difference = \
        compare_vehicle_routes(problem_instance.get_all_routes(), problem_instance.get_all_routes(modified_vehicles))

    if problem_instance.calculate_distances(modified_vehicles) - problem_instance.calculate_distances() < \
            skew_param * route_difference:
        print("Changing from neighbourhood ", nbh, "to neighbourhood ", end='') if verbose == 1 else None
        problem_instance.vehicles = modified_vehicles
        nbh = 0
        print(nbh) if verbose == 1 else None
    else:
        print("Changing from neighbourhood ", nbh, "to neighbourhood ", end='') if verbose == 1 else None
        nbh = nbh + 1
        print(nbh) if verbose == 1 else None
    return nbh


def general_variable_nbh_search(problem_instance, ordered_nbhs: [], change_nbh=change_nbh_sequential,
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
        calculate_loading_MF(problem_instance)
        problem_instance.display_results(False) if verbose == 1 else None

        # new_routes = [None]*len(new_vehicle_routes)
        # for vehicle_index, vehicle in enumerate(new_vehicle_routes):
        #     new_routes[vehicle_index] = vehicle.route()

        # sequential_variable_nbh_descent(problem_instance, ordered_nbhs)
        if change_nbh == change_nbh_cyclic:
            distances_old = problem_instance.calculate_distances()
            nbh_index = change_nbh_cyclic(problem_instance, new_vehicle_routes, nbh_index, len(ordered_nbhs), verbose)
            if distances_old == problem_instance.calculate_distances():
                break

        elif change_nbh == change_nbh_skewed_sequential:
            nbh_index = change_nbh_skewed_sequential(problem_instance, new_vehicle_routes, nbh_index, skew_param,
                                                    verbose)
        else:
            nbh_index = change_nbh(problem_instance, new_vehicle_routes, nbh_index, verbose)
