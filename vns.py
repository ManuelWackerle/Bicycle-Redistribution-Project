"""
Implementations of the variable neighbourhood search based on the article Variable Neighbourhood Search: basics and
variants, by P Hansen et al.
"""

import networkx as nx
from copy import deepcopy
import time
from tqdm import tqdm
import random

import vns

from structure import Vehicle, ProblemInstance
import numpy as np
import sys


"""
Neighbourhood operators. Given an initial route, each operator returns an array of new routes obtained by some 
modification of the initial route. We generate as many routes as possible in the given time.
The general type of these functions is:

def neighbourhood_operator ( initial_routes [[]], timeout ) -> list_of_modified_routes: [[[]]]
"""


def greedy_routing_v1(prob, source='0', dist_weight=3, tsp_weight=1, randomness=False):
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


def greedy_routing_PILOT(prob):
    """
    Finds a set of vehicle routes with low cost based on a greedy approach.
    Version 1: greedy search using the PILOT technique
    :return routes: the set vehicle routes
    """

    pass  # Todo: implement


def calculate_loading_MF(prob, check_feasibility_only=False, start_load=0):
    """
    Given a set of vehicle routes, calculates the optimal loading instructions for each route using a Maximum flow computation.
    Use this function if monotonicity is assumed.
    :param prob: Problem instance
    :param source: Start and end node
    :param start_load: number of bicycles that
    :modifies vehilce.loads: The instructions for how to load and unload the bicycles.
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

    for l, vehicle in enumerate(prob.vehicles):
        prev_node = 0
        for r, node in enumerate(vehicle.route()):
            node_str = "{}-{}-{}".format(node, l, r)
            demand = prob.model.nodes[node]['sup']
            if demand > 0:
                mf_graph.add_edge(node, node_str)
            elif demand < 0:
                mf_graph.add_edge(node_str, node)
            if prev_node != 0:
                mf_graph.add_edge(prev_node, node_str, capacity=vehicle.capacity())
            prev_node = node_str
    prob.show_info("Graph generated with {n} nodes and {e} edges. Source flow: {s}, Sink flow: {t}"
                   .format(n=len(mf_graph.nodes), e=len(mf_graph.edges), s=total_source, t=total_sink))

    # Solve Max Flow Problem
    prob.show_header("Solving the Max flow problem ")
    prob.imbalance = total_source - start_load
    if total_sink != total_source:
        prob.show_warning("mismatch in source and sink flow capacity, no exact solution can exist.")
        prob.imbalance = -1

    value, data = nx.maximum_flow(mf_graph, 's', 't')  # , flow_func=nx.algorithms.flow.shortest_augmenting_path()
    prob.allocated = value - start_load

    if not check_feasibility_only:
        if value != total_source or value != total_sink:
            prob.show_warning(
                "Bikes can not be allocated to full capacity. Source flow: {s}, Sink flow: {t}, Allocated: {a}"
                .format(s=total_source, t=total_sink, a=value))
        else:
            prob.show_info("Bike allocation is exact. Total allocated bicycles: {}".format(value))

        prob.show_header("Generating instructions")
        for l, vehicle in enumerate(prob.vehicles):
            loads = []
            path = vehicle.route()
            prev = path[0]
            for r, node in enumerate(path[1:], 1):
                prev_str = "{}-{}-{}".format(prev, l, r - 1)
                node_str = "{}-{}-{}".format(node, l, r)
                loads.append(data[prev_str][node_str])
                prev = node
            vehicle.set_loads(loads)


def remove_unused_stops(prob):
    """
    Given a set of vehicle routes, removes all the stops where the vehicle neither load nor unloads any bikes.
    """
    for v in prob.vehicles:
        remove = []
        prev_load = 0
        for s in range(1, len(v.route()) - 1):  # ignore first and last stops (source)
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
                if ri != rk and ri != rl and rj != rk and rj != rl and ri != rj and rk != rl:
                    value = prob.model.edges[ri, rj]['dist'] + prob.model.edges[rk, rl]['dist']
                    new_value = prob.model.edges[ri, rk]['dist'] + prob.model.edges[rj, rl]['dist']
                    if prob.model.is_directed():
                        value += distance_between_stops(prob.model, route, s1 + 1, s2)
                        new_value += distance_between_stops(prob.model, route, s2, s1 + 1)
                    diff = value - new_value
                    if diff > best:
                        best = diff
                        b1, b2 = s1, s2

            if best > 0:
                swaps[l].append([best, b1, b2])

    original_vehicles = deepcopy(prob.vehicles)
    out = []
    for l, v in enumerate(prob.vehicles):
        out.append(v)
        swaps[l].sort(reverse=True)
        mn, mx = len(v.route()) + 1, -1
        route = deepcopy(v.route())
        for s in range(len(swaps[l])):
            _, b1, b2 = swaps[l][s]
            if b2 < mn - 1 or mx + 1 < b1:  # ignore interfering swaps
                v.set_route(route[:b1 + 1] + route[b2:b1:-1] + route[b2 + 1:])
                calculate_loading_MF(prob, check_feasibility_only=True)
                if prob.allocated >= prob.imbalance - tolerance:
                    route = deepcopy(v.route())
                    mn, mx = min(b1, mn), max(b2, mx)
                else:
                    v.set_route(route)
    prob.vehicles = original_vehicles
    return out


def intra_or_opt(prob, tolerance=0):
    swaps = []
    for l, v in enumerate(prob.vehicles):
        swaps.append([])
        route = v.route()
        b1, b2 = 0, 0
        for s1 in range(1, len(route) - 4):
            ri, r, rj = route[s1 - 1], route[s1], route[s1 + 1]
            best, value, new_value = 0, 0, 0
            for s2 in range(s1 + 2, len(route) - 2):
                rk, rl = route[s2], route[s2 + 1]
                if ri != rj and r != rk and r != rl:
                    dist_old = prob.model.edges[ri, r]['dist'] + prob.model.edges[r, rj]['dist'] + \
                               prob.model.edges[rk, rl]['dist']
                    dist_new = prob.model.edges[rk, r]['dist'] + prob.model.edges[r, rl]['dist'] + \
                               prob.model.edges[ri, rj]['dist']
                    diff = dist_old - dist_new
                    if diff > best:
                        best = diff
                        b1, b2 = s1, s2
            if best > 0:
                swaps[l].append([best, b1, b2])

    original_vehicles = deepcopy(prob.vehicles)
    for l, v in enumerate(prob.vehicles):
        # print(prob.vehicles[l].route())
        swaps[l].sort(reverse=True)
        mn, mx = len(v.route()) + 1, -1
        route = deepcopy(v.route())
        for s in range(len(swaps[l])):
            _, b1, b2 = swaps[l][s]
            if b2 < mn - 1 or mx + 1 < b1:  # ignore interfering swaps
                v.set_route(route[:b1] + route[b1 + 1:b2 + 1] + [route[b1]] + route[b2 + 1:])
                calculate_loading_MF(prob, check_feasibility_only=True)
                if prob.allocated >= prob.imbalance - tolerance:
                    route = deepcopy(v.route())
                    mn, mx = min(b1, mn), max(b2, mx)
                else:
                    v.set_route(route)
    out = deepcopy(prob.vehicles)
    prob.vehicles = original_vehicles
    return out


def inter_segment_swap(prob, max_segment_length=10, tolerance=0):
    swaps = []
    calculate_loading_MF(prob)

    for l1 in range(len(prob.vehicles) - 1):
        v1 = prob.vehicles[l1]
        route1 = v1.route()
        loads1 = v1.loads()
        m1, m2, a1, b1, a2, b2 = 0, 0, 0, 0, 0, 0
        for s1 in range(1, len(route1) - max_segment_length - 1):
            ri1, rj1 = route1[s1 - 1], route1[s1]
            load_change_dict = {}
            load1 = loads1[s1 - 1]

            for t1 in range(s1, s1 + max_segment_length):
                load_change_dict[load1 - loads1[t1]] = t1
            best, value, new_value = 0, 0, 0

            for l2 in range(l1 + 1, len(prob.vehicles)):
                v2 = prob.vehicles[l2]
                route2 = v2.route()
                loads2 = v2.loads()

                for s2 in range(1, len(route2) - max_segment_length - 1):
                    ri2, rj2 = route2[s2 - 1], route2[s2]
                    load2 = loads2[s2 - 1]

                    for t2 in range(s2 - 1, s2 + max_segment_length + 1):
                        load_change = load2 - loads2[t2]
                        if load_change in load_change_dict:
                            t1 = load_change_dict[load_change]
                            max_load1 = max(loads1[s1 - 1:t1 + 1]) - load1
                            min_load1 = min(loads1[s1 - 1:t1 + 1]) - load1
                            max_load2 = max(loads2[s2 - 1:t2 + 1]) - load2
                            min_load2 = min(loads2[s2 - 1:t2 + 1]) - load2
                            if load2 + max_load1 <= v2.capacity() and load2 + min_load1 >= 0 and \
                                    load1 + max_load2 <= v1.capacity() and load1 + min_load2 >= 0:
                                rk1, rl1 = route1[t1], route1[t1 + 1]
                                rk2, rl2 = route2[t2], route2[t2 + 1]
                                diff = 0
                                if s2 == t2 + 1 and ri1 != rl1 and ri2 != rj1 and rk1 != rj2:
                                    dist_old = prob.model.edges[ri1, rj1]['dist'] + prob.model.edges[rk1, rl1]['dist']
                                    dist_old += prob.model.edges[ri2, rj2]['dist']
                                    dist_new = prob.model.edges[ri1, rl1]['dist']
                                    dist_new += prob.model.edges[ri2, rj1]['dist'] + prob.model.edges[rk1, rj2]['dist']
                                    diff = dist_old - dist_new
                                elif ri1 != rj2 and rk2 != rl1 and ri2 != rj1 and rk1 != rl2:
                                    dist_old = prob.model.edges[ri1, rj1]['dist'] + prob.model.edges[rk1, rl1]['dist']
                                    dist_old += prob.model.edges[ri2, rj2]['dist'] + prob.model.edges[rk2, rl2]['dist']
                                    dist_new = prob.model.edges[ri1, rj2]['dist'] + prob.model.edges[rk2, rl1]['dist']
                                    dist_new += prob.model.edges[ri2, rj1]['dist'] + prob.model.edges[rk1, rl2]['dist']
                                    diff = dist_old - dist_new
                                if diff > best:
                                    best = diff
                                    m1, m2, a1, b1, a2, b2 = l1, l2, s1, t1, s2, t2
                        # elif -load_change in load_change_dict: Todo: add reverse insert check
            if best > 0:
                swaps.append([best, m1, m2, a1, b1, a2, b2])

    swaps.sort(reverse=True)
    vehicles = deepcopy(prob.vehicles)
    valid = [[] for i in range(len(prob.vehicles))]
    # Reformat list of swaps and ignore interfering indices
    for s in swaps:
        _, l1, l2, a1, b1, a2, b2 = s
        if non_overlapping(valid[l1], [a1, b1]) and non_overlapping(valid[l2], [a2, b2]):
            valid[l1].append([a1, b1, l2, a2, b2])
            valid[l2].append([a2, b2, l1, a1, b1])

    for l1, swaps in enumerate(valid):
        v1 = vehicles[l1]
        swaps.sort(reverse=True)
        for s in swaps:
            a1, b1, l2, a2, b2 = s
            v2 = prob.vehicles[l2]
            route1 = deepcopy(v1.route())
            route2 = deepcopy(v2.route())
            v1.set_route(route1[:a1] + route2[a2:b2 + 1] + route1[b1 + 1:])

    return vehicles


def non_overlapping(slices, new_slice):
    for slice in slices:
        s, t, _, _, _ = slice
        a, b = new_slice
        if b >= s - 1 and a <= t + 1:
            return False
    return True

def inter_two_opt(prob, tolerance=0):
    swaps = []
    clip = 5
    for l1, v1 in enumerate(prob.vehicles):
        route1 = v1.route()
        for l2 in range(l1 + 1, len(prob.vehicles)):
            route2 = prob.vehicles[l2].route()
            r1, r2, b1, b2 = 0, 0, 0, 0
            best, value, new_value = 0, 0, 0
            for s1 in range(clip, len(route1) - clip):
                ri, rj = route1[s1], route1[s1 + 1]
                for s2 in range(s1 - clip, min(s1 + clip, len(route2) - 1)):
                    rk, rl = route2[s2], route2[s2 + 1]
                    if ri != rk and ri != rl and rj != rk and rj != rl and ri != rj and rk != rl:
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
            calculate_loading_MF(prob, check_feasibility_only=True)
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


def intra_two_opt_v2(prob: ProblemInstance, tolerance=0):
    """
    Searches for intra two-opt switches that provide a decrease in route length
    param prob: Problem instance
    param tolerance: rebalancing tolerance allowed

    return swaps: array of possible swaps. A swap is given by [<distance improved>, <swap index 1>, <swap index 2>]
    """

    swaps = []
    for l, v in enumerate(prob.vehicles):
        swaps.append([])
        route = v.route()
        b1, b2 = 0, 0
        if v.modified():
            for s1 in range(1, len(route) - 4):
                ri, rj = route[s1], route[s1 + 1]
                best, dist_old, dist_new = 0, 0, 0
                for s2 in range(s1 + 2, len(route) - 2):
                    rk, rl = route[s2], route[s2 + 1]
                    if ri != rk and ri != rl and rj != rk and rj != rl:
                        dist_old = prob.model.edges[ri, rj]['dist'] + prob.model.edges[rk, rl]['dist']
                        dist_new = prob.model.edges[ri, rk]['dist'] + prob.model.edges[rj, rl]['dist']
                        diff = dist_old - dist_new
                        if prob.model.is_directed():
                            dist_old += distance_between_stops(prob.model, route, s1 + 1, s2)
                            dist_new += distance_between_stops(prob.model, route, s2, s1 + 1)
                        if diff > best:
                            best = diff
                            b1, b2 = s1, s2
                if best > 0:
                    swaps[l].append([best, b1, b2])

    prob.intialize_flow_graph()
    out = deepcopy(prob.vehicles)
    for l, v in enumerate(prob.vehicles):
        swaps[l].sort(reverse=True)
        mn, mx = len(v.route()) + 1, -1
        for s in range(len(swaps[l])):
            _, b1, b2 = swaps[l][s]
            if b2 < mn - 1 or mx + 1 < b1:  # ignore interfering swaps
                prob.verify_loading_on_swapped_route(b1, b2, l, tolerance=tolerance)
                if prob.allocated >= prob.imbalance - tolerance:
                    vout = out[l]
                    vout.set_route(vout.route()[:b1 + 1] + vout.route()[b2:b1:-1] + vout.route()[b2 + 1:])
                    mn, mx = min(b1, mn), max(b2, mx)
    return out


def inter_two_opt_v2(prob: ProblemInstance, tolerance=0):
    swaps = []
    clip = 5
    for l1, v1 in enumerate(prob.vehicles):
        route1 = v1.route()
        for l2 in range(l1 + 1, len(prob.vehicles)):
            route2 = prob.vehicles[l2].route()
            r1, r2, b1, b2 = 0, 0, 0, 0
            best, dist_old, dist_new = 0, 0, 0
            for s1 in range(clip, len(route1) - clip):
                ri, rj = route1[s1], route1[s1 + 1]
                for s2 in range(s1 - clip, min(s1 + clip, len(route2) - 1)):
                    rk, rl = route2[s2], route2[s2 + 1]
                    if ri != rk and ri != rl and rj != rk and rj != rl:
                        dist_old = prob.model.edges[ri, rj]['dist'] + prob.model.edges[rk, rl]['dist']
                        dist_new = prob.model.edges[ri, rl]['dist'] + prob.model.edges[rk, rj]['dist']
                        diff = dist_old - dist_new
                        if diff > best:
                            best = diff
                            r1, r2, b1, b2 = l1, l2, s1, s2
            if best > 0:
                swaps.append([best, r1, r2, b1, b2])

    prob.intialize_flow_graph()
    swaps.sort(reverse=True)
    used = set()
    out = deepcopy(prob.vehicles)
    for s in swaps:
        _, l1, l2, b1, b2 = s
        if l1 not in used and l2 not in used:
            # Todo, implement index change tracking so that the other swaps can also be used and delete this check
            prob.verify_loading_on_swapped_route(b1, b2, l1, l2, tolerance=tolerance)
            if prob.allocated >= prob.imbalance - tolerance:
                v1, v2 = out[l1], out[l2]
                v1_new = v1.route()[:b1 + 1] + v2.route()[b2 + 1:]
                v2_new = v2.route()[:b2 + 1] + v1.route()[b1 + 1:]
                v1.set_route(v1_new)
                v2.set_route(v2_new)
                used.add(l1)
                used.add(l2)
    return out

def distance_between_stops(graph, route, stop1, stop2):
    dist = 0
    step = 1 if stop1 < stop2 else -1
    for stop in range(stop1, stop2, step):
        u, v = route[stop], route[stop + step]
        if u != v: #u == v should never happen - this is a quick fix
            dist += graph.edges[u, v]['dist']
    return dist


def _delete_consecutive_same_station(vehicles):
    for k, vehicle in enumerate(vehicles):
        route = vehicle.route()
        N = len(route)
        if N < 5:
            print(f"Warning! The vehicle {k} has only {N} stations in the route.")
        remove_idxes = set()
        for i, station in enumerate(route):
            if i < N - 1 and station == route[i + 1]:
                remove_idxes.add(i)
        vehicle.set_route([s for i, s in enumerate(route) if i not in remove_idxes])
        vehicle.set_loads([l for i, l in enumerate(vehicle.loads()) if i not in remove_idxes])
    return vehicles


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
        for j in range(1, len(vehicles[i].route()) - 1):
            idxes.append([i, j])

    if at_random:
        random.shuffle(idxes)  # generate at random

    for i, j in idxes:
        candidate = deepcopy(vehicles)
        candidate[i].set_route(candidate[i].route()[:j] + candidate[i].route()[j + 1:])
        candidate[i].set_loads(candidate[i].loads()[:j] + candidate[i].loads()[j + 1:])
        yield candidate


def remove_multi_stations_generator(vehicles, at_random=False, num_removal=5):
    """generate routes by removing multiple stations

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
    N = len(vehicles)
    for i in range(N):
        if len(vehicles[i].route()) <= 5:
            # if route length is less than two, it contains only depot
            # if route is too short, don't remove from that route.
            continue
        # ignore index 0 and -1 since both are depot
        for j in range(1, len(vehicles[i].route()) - 1):
            idxes.append([i, j])

    if at_random:
        random.shuffle(idxes)  # generate at random

    nr = 0
    candidate = deepcopy(vehicles)
    removal_idxes = [[] for _ in range(N)]
    for i, j in idxes:
        if nr < num_removal:
            removal_idxes[i].append(j)
            nr += 1
        else:
            for n in range(N):
                original_route = candidate[n].route()
                idxes = [
                    k for k, station in enumerate(original_route)
                    if k not in removal_idxes[n]
                ]
                candidate[n].set_route([station for k, station in enumerate(original_route) if k in idxes])
                candidate[n].set_loads([load for k, load in enumerate(candidate[n].loads()) if k in idxes])
            # candidate[i].set_route(candidate[i].route()[:j] + candidate[i].route()[j+1:])
            # candidate[i].set_loads(candidate[i].loads()[:j] + candidate[i].loads()[j+1:])
            candidate = _delete_consecutive_same_station(candidate)
            yield candidate
            nr = 0
            removal_idxes = [[] for _ in range(N)]
            candidate = deepcopy(vehicles)


def remove_multi_stations_generator_v2(problem_instance, num_removal=5):
    """generate routes by removing multiple stations
    """
    vehicles = problem_instance.vehicles
    candidate = deepcopy(vehicles)
    longest_distance = 0
    bi = 0
    for i, vehicle in enumerate(vehicles):
        current_distance = problem_instance.calculate_distance(vehicle)
        if current_distance > longest_distance:
            longest_distance = current_distance
            bi = i

    station_idxes = list(range(1, len(vehicles[bi])))
    random.shuffle(station_idxes)
    nr = 0
    for j in station_idxes:
        if nr < num_removal:
            candidate[bi].set_route(candidate[bi].route()[:j] + candidate[i].route()[j + 1:])
            candidate[bi].set_loads(candidate[bi].loads()[:j] + candidate[i].loads()[j + 1:])
            nr += 1
        else:
            yield candidate
            nr = 0
            candidate = deepcopy(vehicles)


def remove_worst_meta_generator(vehicles, graph, num_removal=5, mode='worst', metric='dist', meta_parameter=5,
                                timeout=10):
    """generates routes by removing multiple stations, preferring the station with the highest distances to visit
        meta_parameter defines the preferences of the worst stations. The higher the value, the higher the chance that
        worse stations will be removed from the set of routes.
        if the meta_parameter is set to 1, then it makes the choice of stations uniformly random.

        input:
            • vehicles - Vehicles object
            • graph - graph object
            • num_removal - number of stations to be removed
            • mode - either 'worst' (remove the worst station according to the given metric) or 'random'
            • metric - metric of defining the 'worst' stations
            • meta_parameter - power of the uniform random (0,1) distribution
            • timeout - timeout for the generator

        returns (yields):
            • candidate - generator of vehicles
        """
    idxes = []
    distance_to_visit = []

    candidate = deepcopy(vehicles)

    for i in range(len(vehicles)):
        if len(vehicles[i].route()) <= 2:
            # if route length is less than two, it contains only depot
            continue
        # ignore index 0 and -1 since both are depot
        for j in range(1, len(vehicles[i].route()) - 1):
            idxes.append([i, j])
            station = candidate[i].route()[j]
            station_pre = candidate[i].route()[j - 1]
            station_post = candidate[i].route()[j + 1]

            distance_to_visit.append(
                graph.edges[station_pre, station][metric] + graph.edges[station, station_post][metric])

    if mode == 'worst':
        # sort the idxes according to their distance
        sorting_args = np.argsort(np.array(distance_to_visit))[::-1]
        idxes = np.array(idxes)
        idxes = idxes[sorting_args].tolist()
    elif mode == 'random':
        # uniform distribution
        meta_parameter = 1
    else:
        print(str(mode) + 'mode is not available. The ^worst^ mode is set.')
        sorting_args = np.argsort(np.array(distance_to_visit))[::-1]
        idxes = np.array(idxes)
        idxes = idxes[sorting_args].tolist()

    candidate = deepcopy(vehicles)

    # weighted distribution
    choose = lambda n, p: int(np.floor(n * np.random.uniform() ** p))

    time_start = time.time()
    while time.time() < time_start + timeout:
        idxes_copy = deepcopy(idxes)
        idxes_apply = []
        for _ in range(num_removal):
            idxes_apply.append(idxes_copy.pop(choose(len(idxes_copy), meta_parameter)))
        # idxes_apply (idxes of stations to be deleted) are unique because of the pop operation

        for i, j in idxes_apply:
            # remove one station per cycle (num_removal stations in total)
            # instead of removing a station from the list, we just mark it as deleted,
            # so we don't need to change the indexes of the later removals
            candidate[i].set_route(candidate[i].route()[:j] + ['x'] + candidate[i].route()[j + 1:])

        # remove 'x' elements from the routes
        for vehicle in candidate:
            route = vehicle.route()
            while (route.count('x')):
                route.remove('x')
            vehicle.set_route(route)

        # return a generator of a new set of routes
        yield candidate

        # reset the candidate
        candidate = deepcopy(vehicles)


def insert_regret_generator(vehicles, copied_problem_instance, mode='balance', verbose=0):
    """generates routes by inserting multiple stations, preferring the station which provide the lowest number
        of unbalanced stations after its insertion in the best arc.

        input:
            • vehicles - Vehicles object
            • copied_problem_instance - Problem object


        returns:
            • copied_vehicles - Vehicle object, which is balanced
        """

    graph = copied_problem_instance.model.copy()
    unbalanced_stations = get_unbalanced_stations(copied_problem_instance, vehicles)
    copied_vehicles = deepcopy(vehicles)
    while unbalanced_stations:
        best_positions_for_stations = []
        best_disbalance_for_stations = []
        regret_for_stations = []

        for station in unbalanced_stations:
            best_disbalance = sys.maxsize
            second_best_disbalance = sys.maxsize
            best_i_j = None
            second_best_i_j = None
            for i, vehicle in enumerate(copied_vehicles):
                route = vehicle.route()

                if len(vehicle.route()) <= 2:
                    # if route length is less than two, it contains only depot
                    continue
                # ignore index 0 and -1 since both are depot
                for j in range(1, len(vehicle.route()) - 1):
                    # print('i, j:', i, j)
                    route_modified = route[:j] + [station] + route[j:]
                    candidate = deepcopy(copied_vehicles)
                    candidate[i].set_route(route_modified)
                    candidate_problem_instance = deepcopy(copied_problem_instance)
                    candidate_problem_instance.vehicles = candidate
                    unbalanced_stations_candidate = get_unbalanced_stations(candidate_problem_instance,
                                                                            candidate)
                    if not unbalanced_stations_candidate:
                        second_best_disbalance = deepcopy(best_disbalance)
                        second_best_i_j = best_i_j
                        best_i_j = deepcopy([i, j])
                        best_disbalance = 0

                    else:
                        if best_disbalance > len(unbalanced_stations_candidate):
                            second_best_i_j = best_i_j
                            second_best_disbalance = deepcopy(best_disbalance)
                            best_i_j = deepcopy([i, j])
                            best_disbalance = len(unbalanced_stations_candidate)
                        else:
                            if second_best_disbalance > len(unbalanced_stations_candidate):
                                second_best_disbalance = len(unbalanced_stations_candidate)
                                second_best_i_j = deepcopy([i, j])
            regret = second_best_disbalance - best_disbalance
            regret_for_stations.append(regret)
            best_disbalance_for_stations.append(best_disbalance)
            best_positions_for_stations.append(best_i_j)

        if mode == 'balance':
            arg = np.argmin(np.array(best_disbalance_for_stations))
        elif mode == 'regret':
            arg = np.argmax(np.array(regret_for_stations))
        else:
            print(str(mode) + 'mode is not available. The ^balance^ mode is set.')
            arg = np.argmin(np.array(best_disbalance_for_stations))

        i, j = best_positions_for_stations[arg]
        inserted_station = unbalanced_stations[arg]
        route = copied_vehicles[i].route()
        route_update = route[:j] + [inserted_station] + route[j:]
        copied_vehicles[i].set_route(route_update)
        copied_problem_instance.vehicles = copied_vehicles

        if verbose:
            print('Disbalances:', best_disbalance_for_stations)
            print('Regrets:', regret_for_stations)
            print('Insert positions:', best_positions_for_stations)
            print('Inserted station:', inserted_station)
            print('Unbalanced stations (before):', unbalanced_stations)

        unbalanced_stations = get_unbalanced_stations(copied_problem_instance, copied_vehicles)

        if verbose:
            print('Unbalanced stations (after):', unbalanced_stations)

    return copied_vehicles


def insert_regret_generator_quick(vehicles, copied_problem_instance, mode='distance', metric='dist', meta_parameter=5,
                                  verbose=0, insert_ratio=0.5):
    """generates routes by inserting multiple stations, preferring the station which provide the lowest number
        of unbalanced stations after its insertion in the best arc.

        input:
            • vehicles - Vehicles object
            • copied_problem_instance - Problem object


        returns:
            • copied_vehicles - Vehicle object, which is balanced
        """

    graph = copied_problem_instance.model.copy()
    unbalanced_stations = get_unbalanced_stations(copied_problem_instance, vehicles)
    copied_vehicles = deepcopy(vehicles)
    choose = lambda n, p: int(np.floor(n * np.random.uniform() ** p))

    num_insert = min(int(np.ceil(len(unbalanced_stations) * insert_ratio)), len(unbalanced_stations))
    random.shuffle(unbalanced_stations)
    load_calculation_step = 3
    maximal_iteration_number = 1

    counter = 0
    while unbalanced_stations:
        counter += 1
        for station in unbalanced_stations[0:num_insert]:
            insertions = []
            distances = []
            for i, vehicle in enumerate(copied_vehicles):
                route = vehicle.route()

                if len(vehicle.route()) <= 2:
                    # if route length is less than two, it contains only depot
                    continue
                # ignore index 0 and -1 since both are depot
                for j in range(1, len(vehicle.route()) - 1):
                    if route[j - 1] == station or route[j] == station:
                        continue

                    current_distance = graph.edges[route[j - 1], station][metric] + graph.edges[station, route[j]][
                        metric]
                    insertions.append([i, j])
                    distances.append(current_distance)

            sorting_args = np.argsort(np.array(distances))
            insertions = np.array(insertions)
            insertions = insertions[sorting_args].tolist()

            if mode == 'distance':
                arg = choose(len(insertions), meta_parameter)
            else:
                print(str(mode) + 'mode is not available. The ^distance^ mode is set.')
                arg = choose(len(insertions), meta_parameter)

            best_i, best_j = insertions[arg]
            route = copied_vehicles[best_i].route()
            copied_vehicles[best_i].set_route(route[:best_j] + [station] + route[best_j:])

        if counter == maximal_iteration_number:
            break

        if counter % load_calculation_step == 0:
            # recalculate the unbalanced stations
            unbalanced_stations = get_unbalanced_stations(copied_problem_instance, copied_vehicles)

    return copied_vehicles


def insertU_nearest_generator(vehicles, unbalanced_stations, graph):
    """insert unbalanced stations to minimum distance position on each route
        return:
            Generator which generate insertU candidate
    """
    candidate = deepcopy(vehicles)
    random.shuffle(unbalanced_stations)
    for u in unbalanced_stations:
        best_distance = 1e+10  # init
        bi = 0
        bj = 0
        for i, vehicle in enumerate(candidate):
            route = vehicle.route()
            if len(route) < 2:
                continue
            for j in range(1, len(route) - 1):
                if route[j - 1] == u or route[j] == u:
                    continue
                current_distance = graph.edges[route[j - 1], u]['dist'] + graph.edges[route[j], u]['dist']
                if current_distance < best_distance:
                    best_distance = current_distance
                    bj = j
                    bi = i

        candidate[bi].set_route(candidate[bi].route()[:bj] + [u] + candidate[bi].route()[bj:])
        # set new load as 0
        candidate[bi].set_loads(candidate[bi].loads()[:bj] + [0] + candidate[bi].loads()[bj:])
    return candidate


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
            if len(route) < 2 or route[0] == u or route[1] == u:
                continue
            distance = graph.edges[route[0], u]['dist'] + graph.edges[u, route[1]]['dist']  # init
            for k in range(1, len(route) - 1):
                if route[j - 1] == u or route[j] == u:
                    continue
                current_distance = graph.edges[route[j-1], u]['dist'] + graph.edges[u, route[j]]['dist']
                if current_distance < distance:
                    distance = current_distance
                    j = k
            vehicle.set_route(vehicle.route()[:j] + [u] + vehicle.route()[j:])
            # set new load as 0
            vehicle.set_loads(vehicle.loads()[:j] + [0] + vehicle.loads()[j:])
            yield candidate


def insertU_nearest_v2(vehicles, unbalanced_stations, graph):
    """insert unbalanced stations to minimum distance position on each route

    return:
        Generator which generate insertU candidate
    """
    candidate = deepcopy(vehicles)
    random.shuffle(unbalanced_stations)
    for u in unbalanced_stations:
        best_distance = 1e+10  # init
        bi = 0
        bj = 0
        for i, vehicle in enumerate(candidate):
            route = vehicle.route()
            if len(route) < 2:
                continue
            for j in range(1, len(route) - 1):
                if route[j - 1] == u or route[j] == u:
                    continue
                current_distance = graph.edges[route[j-1], u]['dist'] + graph.edges[u, route[j]]['dist']
                if current_distance < best_distance:
                    best_distance = current_distance
                    bj = j
                    bi = i

        candidate[bi].set_route(candidate[bi].route()[:bj] + [u] + candidate[bi].route()[bj:])
        # set new load as 0
        candidate[bi].set_loads(candidate[bi].loads()[:bj] + [0] + candidate[bi].loads()[bj:])
    return candidate


def insertU_nearest_v3(vehicles, unbalanced_stations, graph):
    """insert unbalanced stations randomly to one of the minimum distance position on each route

    return:
        Generator which generate insertU candidate
    """

    def _insert_ordered(l, c, i, n):
        ret = l
        for k in range(len(l)):
            if c[i] < l[k][i]:
                ret = l[:k] + [c] + l[k:]
                break
        return ret[:n]

    probs = (10, 9, 8, 7, 6, 5, 4, 3, 2, 1)
    candidate = deepcopy(vehicles)
    for u in unbalanced_stations:
        top_n = [[0, 0, 1e+10], ]  # init
        for i, vehicle in enumerate(candidate):
            route = vehicle.route()
            if len(route) < 2:
                continue
            for j in range(1, len(route) - 1):
                if route[j - 1] == u or route[j] == u:
                    continue
                current_distance = graph.edges[route[j-1], u]['dist'] + graph.edges[u, route[j]]['dist']
                top_n = _insert_ordered(top_n, [i, j, current_distance], -1, len(probs))

        bi, bj, _ = random.choices(top_n, probs)[0]
        candidate[bi].set_route(candidate[bi].route()[:bj] + [u] + candidate[bi].route()[bj:])
        # set new load as 0
        candidate[bi].set_loads(candidate[bi].loads()[:bj] + [0] + candidate[bi].loads()[bj:])
    return candidate


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


def get_unbalanced_stations(problem_instance, vehicles):
    problem_instance_copy = deepcopy(problem_instance)
    problem_instance_copy.vehicles = vehicles
    calculate_loading_MF(problem_instance_copy)
    graph = _get_rebalanced_graph(problem_instance_copy.model.copy(), vehicles)
    unbalanced_stations = [x for x in graph.nodes if graph.nodes[x]['sup'] != 0]

    return unbalanced_stations


def _get_loading_and_unbalanced_stations(problem_instance, vehicles):
    """Given vehicles, calculate best loading instructions and return unbalanced stations
    """
    problem_instance.vehicles = vehicles
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

    # for removed_vehicles in tqdm(remove_one_station_generator(copied_problem_instance.vehicles, at_random=True)):
    for removed_vehicles in remove_one_station_generator(copied_problem_instance.vehicles, at_random=True):
        unbalanced_stations = _get_loading_and_unbalanced_stations(copied_problem_instance, removed_vehicles)
        if not unbalanced_stations:
            # if removal neighbor routes are possibly balanced, return them
            return removed_vehicles
        for inserted_vehicles in insertU_nearest_generator(removed_vehicles, unbalanced_stations,
                                                           copied_problem_instance.model.copy()):
            # for inserted_vehicles in insertU_generator(removed_vehicles, unbalanced_stations, at_random=True):
            unbalanced_stations = _get_loading_and_unbalanced_stations(copied_problem_instance, inserted_vehicles)
            if not unbalanced_stations:
                return inserted_vehicles
    # if there is no candidate, return original
    return problem_instance.vehicles


def destroy_rebuild(problem_instance, num_removal=3, verbose=1):
    """Destroy and rebuild the set of routes.

    return
        vehicles
    """
    start_time_outer = time.time()
    copied_problem_instance = deepcopy(problem_instance)
    print('Distance before applying the LN: '
          + str(copied_problem_instance.calculate_distances()) + '.')

    # the initial insert ratio
    insert_ratio = 0.5
    for removed_vehicles in remove_worst_meta_generator(copied_problem_instance.vehicles,
                                                        copied_problem_instance.model.copy(), mode='worst', meta_parameter=2,
                                                        num_removal=num_removal):
        unbalanced_stations = get_unbalanced_stations(copied_problem_instance, removed_vehicles)
        if not unbalanced_stations:
            # if removal neighbor routes are possibly balanced, return them
            return removed_vehicles

        start_time_inner = time.time()
        inserted_vehicles = insert_regret_generator_quick(removed_vehicles, copied_problem_instance, insert_ratio=insert_ratio)
        execution_time_inner = time.time() - start_time_inner

        if verbose:
            print('It took ' + str(execution_time_inner) + ' seconds to rebuild the set of routes.')

        unbalanced_stations = get_unbalanced_stations(copied_problem_instance, inserted_vehicles)
        if not unbalanced_stations:
            # calculate and print distance of the solution
            if verbose:
                execution_time_outer = time.time() - start_time_outer
                print('It took ' + str(execution_time_outer) + ' seconds to find a feasible solution.')

                output_problem_copy = deepcopy(copied_problem_instance)
                output_problem_copy.vehicles = inserted_vehicles
                print('LN returned a distinct feasible solution with distance '
                      + str(output_problem_copy.calculate_distances()) + '.')

                return inserted_vehicles
        if insert_ratio < 1:
            # the insert ratio for the next rebuild is increased if the previous rebuild wasn't successful
            insert_ratio += 0.05

    # if there is no candidate, return original
    return problem_instance.vehicles


def multi_remove_and_insert_station(problem_instance, num_removal=1):
    copied_problem_instance = deepcopy(problem_instance)

    start_time_outer = time.time()
    for removed_vehicles in remove_multi_stations_generator(copied_problem_instance.vehicles, at_random=True,
                                                            num_removal=num_removal):
        unbalanced_stations = _get_loading_and_unbalanced_stations(copied_problem_instance, removed_vehicles)
        if not unbalanced_stations:
            # if removal neighbor routes are possibly balanced, return them
            # print("Found feasible routes by removal.")
            return removed_vehicles
        # for inserted_vehicles in insertU_nearest_generator(removed_vehicles, unbalanced_stations, copied_problem_instance.model.copy()):
        #     unbalanced_stations = _get_loading_and_unbalanced_stations(copied_problem_instance, inserted_vehicles)
        #     if not unbalanced_stations:
        #         print("Found feasible routes by insertion.")
        #         return inserted_vehicles
        # vehicles = _delete_consecutive_same_station(removed_vehicles)
        vehicles = insertU_nearest_v3(removed_vehicles, unbalanced_stations, copied_problem_instance.model.copy())
        # vehicles = _delete_consecutive_same_station(vehicles)
        unbalanced_stations = _get_loading_and_unbalanced_stations(copied_problem_instance, vehicles)
        if not unbalanced_stations:
            # print(f"Found feasible routes by insertion.")
            # execution_time_outer = time.time() - start_time_outer
            # print('It took ' + str(execution_time_outer) + ' seconds to find a feasible solution.')
            return vehicles
    # if there is no candidate,
    # vehicles = removed_vehicles
    # for n in range(5):
    #     vehicles = insertU_nearest_v3(vehicles, unbalanced_stations, copied_problem_instance.model.copy())
    #     # vehicles = _delete_consecutive_same_station(vehicles)
    #     unbalanced_stations = _get_loading_and_unbalanced_stations(copied_problem_instance, vehicles)
    #     if not unbalanced_stations:
    #         print(f"Found feasible routes by {n+1} insertion.")
    #         return vehicles
    # if there is no candidate, return original
    # print("Did not find any feasible routes.")
    return problem_instance.vehicles


def multi_remove_and_insert_station_v2(problem_instance, num_removal=5):
    copied_problem_instance = deepcopy(problem_instance)

    for removed_vehicles in remove_multi_stations_generator(copied_problem_instance.vehicles, at_random=True,
                                                            num_removal=num_removal):
        unbalanced_stations = _get_loading_and_unbalanced_stations(copied_problem_instance, removed_vehicles)
        if not unbalanced_stations:
            # if removal neighbor routes are possibly balanced, return them
            return removed_vehicles
        inserted_vehicles = insertU_nearest_v3(removed_vehicles, unbalanced_stations,
                                               copied_problem_instance.model.copy())
        unbalanced_stations = _get_loading_and_unbalanced_stations(copied_problem_instance, inserted_vehicles)
        if not unbalanced_stations:
            return inserted_vehicles
    # if there is no candidate, return original
    return problem_instance.vehicles


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
        problem_instance.vehicles = modified_vehicles
        calculate_loading_MF(problem_instance)
        remove_unused_stops(problem_instance)
        nbh = 0
        print(nbh) if verbose == 1 else None
    else:
        nbh = nbh + 1

    return nbh


def change_nbh_cyclic(problem_instance, modified_vehicles: [], nbh, number_of_nbhs, verbose=0) -> int:
    """
    Cyclic neighbourhood change step: regardless of whether there is an improvement with respect to some nbh or not, the
    search is continued in the next neighbourhood in the list.
    """
    if problem_instance.calculate_distances() > problem_instance.calculate_distances(modified_vehicles):
        print("Changing from neighbourhood ", nbh, "to neighbourhood ", nbh + 1) if verbose == 1 else None
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
    distance_hist = []
    time_hist = []
    operation_hist = []

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

        distance_hist.append(problem_instance.calculate_distances())
        time_hist.append(time.time())
        operation_hist.append(nbh_index)

    return distance_hist, time_hist, operation_hist


def large_nbh_search(problem_instance, ordered_large_nbhs: [int], ordered_local_nbhs: [],
                     change_large_nbh=change_nbh_pipe,
                     change_local_nbh=change_nbh_sequential,
                     large_nbh_operator=destroy_rebuild,
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

        calculate_loading_MF(problem_instance)
        remove_unused_stops(problem_instance)
        problem_instance.display_results(False) if large_verbose == 1 else None

        """
        After the initial large neighbourhood shake, we use VND to locally improve the routes.
        """
        candidate_problem = deepcopy(problem_instance)
        candidate_problem.vehicles = new_vehicle_routes
        calculate_loading_MF(candidate_problem)

        distance_hist_local, time_hist_local, operation_hist_local = general_variable_nbh_search(candidate_problem,
                                                                                                 ordered_local_nbhs,
                                                                                                 change_nbh=change_local_nbh,
                                                                                                 timeout=timeout,
                                                                                                 skew_param=10,
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

        elif change_large_nbh == change_nbh_skewed_sequential:
            large_nbh_index_old = large_nbh_index
            large_nbh_index = change_nbh_skewed_sequential(problem_instance, candidate_problem.vehicles,
                                                           large_nbh_index,
                                                           skew_param, verbose=0)
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
