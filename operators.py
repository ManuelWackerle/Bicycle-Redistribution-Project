"""
Implementations of the neighbourhood operators.
"""
import random
import time
import numpy as np
from copy import deepcopy
from structure import ProblemInstance


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
                        value += _distance_between_stops(prob, route, s1 + 1, s2)
                        new_value += _distance_between_stops(prob, route, s2, s1 + 1)
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
                prob.calculate_loading_MF(check_feasibility_only=True)
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
                prob.calculate_loading_MF(check_feasibility_only=True)
                if prob.allocated >= prob.imbalance - tolerance:
                    route = deepcopy(v.route())
                    mn, mx = min(b1, mn), max(b2, mx)
                else:
                    v.set_route(route)
    out = deepcopy(prob.vehicles)
    prob.vehicles = original_vehicles
    return out


def inter_segment_swap(prob, max_segment_length=10):
    swaps = []
    prob.calculate_loading_MF()

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
            if best > 0:
                swaps.append([best, m1, m2, a1, b1, a2, b2])

    swaps.sort(reverse=True)
    vehicles = deepcopy(prob.vehicles)
    valid = [[] for _ in range(len(prob.vehicles))]
    # Reformat list of swaps and ignore interfering indices
    for s in swaps:
        _, l1, l2, a1, b1, a2, b2 = s
        if non_overlapping_old(valid[l1], [a1, b1]) and non_overlapping_old(valid[l2], [a2, b2]):
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


def non_overlapping_old(slices, new_slice):
    for pair in slices:
        s, t, _, _, _ = pair
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
            v1, v2 = prob.vehicles[r1], prob.vehicles[r2]
            route1 = deepcopy(v1.route())
            route2 = deepcopy(v2.route())
            v1.set_route(route1[:b1 + 1] + route2[b2 + 1:])
            v2.set_route(route2[:b2 + 1] + route1[b1 + 1:])
            prob.calculate_loading_MF(check_feasibility_only=True)
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
    param tolerance: re-balancing tolerance allowed

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
                            dist_old += _distance_between_stops(prob, route, s1 + 1, s2)
                            dist_new += _distance_between_stops(prob, route, s2, s1 + 1)
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
                    v_out = out[l]
                    v_out.set_route(v_out.route()[:b1 + 1] + v_out.route()[b2:b1:-1] + v_out.route()[b2 + 1:])
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


#      OLD METHODS - probably should be deleted at some point
# ======================================================================================================================
#      UPDATED METHODS - ideally faster and better readable


def _distance_between_stops(prob, route, stop1, stop2):
    """
    Calculates the directed distance on a route between two stops, stop1 and stop2 can be in reverse
    """
    dist = 0
    step = 1 if stop1 < stop2 else -1
    for stop in range(stop1, stop2, step):
        u, v = route[stop], route[stop + step]
        dist += prob.distance(u, v)
    return dist


def _relative_segment_length(prob: ProblemInstance) -> int:
    return len(prob.model.nodes) // len(prob.vehicles) // 5


def intra_two_opt_fast(prob: ProblemInstance):
    """
    Compares edges within the same route and Tests if swapping them reduces the total distance.
    Candidates are collected an executed on a deepcopy of the problem instance in a best improvement first fashion.
    Original Route:       [...   a x > > ... > > y b   ...]
    Candidates checked:   [...   a y < < ... < < x b   ...] (Note: inner segment traversed in reverse after swap)
    """

    prob.calculate_loading_MF()  # Ensure loading instructions are up to date.
    num_vehicles = len(prob.vehicles)
    swaps = [[] for _ in range(num_vehicles)]
    a_best, b_best = 0, 0

    # Loop through all routes
    for l, v in enumerate(prob.vehicles):
        route = v.route()
        loads = v.loads()

        # Loop through all candidate edges (a, x)
        for a in range(1, len(route) - 4):
            x = a + 1
            a_n, x_n = route[a], route[x]
            load_a = loads[a]
            max_load, min_load = max(load_a, loads[x]), min(load_a, loads[x])
            best = 0

            # Loop through all candidate edges (y, b)
            for y in range(x + 1, len(route) - 2):
                b = y + 1
                y_n, b_n = route[y], route[b]
                load_y = loads[y]

                max_load = max(max_load, loads[y])
                min_load = min(min_load, loads[y])
                max_load_diff = load_y - max_load
                min_load_diff = load_y - min_load
                if load_a + min_load_diff <= v.capacity() and load_a + max_load_diff >= 0:
                    old_dist = prob.distance(a_n, x_n) + prob.distance_route_segment(route, x, y) + prob.distance(y_n, b_n)
                    new_dist = prob.distance(a_n, y_n) + prob.distance_route_segment(route, y, x) + prob.distance(x_n, b_n)
                    diff = old_dist - new_dist
                    if diff > best:
                        best = diff
                        a_best, b_best = a, b
                else:
                    break

            # Save best candidate pair [(a, x), (y, b)] if one exists
            if best > 0:
                swaps[l].append([best, a_best, b_best])

    # Reformat list of swaps and ignore interfering sections in a best first fashion
    valid = [[] for _ in range(num_vehicles)]
    for l, v in enumerate(prob.vehicles):
        swaps[l].sort(reverse=True)  # Prioritise swaps with best improvements
        for s in swaps[l]:
            _, a, b = s
            if _non_overlapping(valid[l], [a, b]):
                valid[l].append([a, b])

    # Execute valid swaps in a deepcopy of vehicles and return it
    vehicles = deepcopy(prob.vehicles)
    for l, swap in enumerate(valid):
        v = vehicles[l]
        for s in swap:
            a, b = s
            route = v.route()
            v.set_route(route[:a + 1] + route[b - 1:a:-1] + route[b:])
    return vehicles


def inter_two_opt_fast(prob: ProblemInstance, max_length_alteration=-1):
    """
    Compares edges between different routes and Tests if swapping them reduces the total distance.
    Candidates are collected an executed on a deepcopy of the problem instance in a best improvement first fashion.
    Original Routes:       [...  - - a y # #    ...] [...  - - x b * *    ...]
    Candidates checked:    [...  - - a b * *    ...] [...  - - x y # #    ...]
    """

    prob.calculate_loading_MF()  # Ensure loading instructions are up to date.
    num_vehicles = len(prob.vehicles)
    swaps = []
    clip = _relative_segment_length(prob) // 2 if max_length_alteration < 0 else max_length_alteration
    save_l1, save_l2, best_y, best_b = 0, 0, 0, 0

    # Loop through all vehicle combinations
    for l1, v1 in enumerate(prob.vehicles):
        route1 = v1.route()
        loads1 = v1.loads()
        for l2 in range(l1 + 1, num_vehicles):
            route2 = prob.vehicles[l2].route()
            loads2 = prob.vehicles[l2].loads()

            # Loop through all edge swap combinations within the max length alteration
            for a in range(1, len(route1) - 3):
                y = a + 1
                a_n, y_n = route1[a], route1[y]
                load_a = loads1[a]
                best = 0
                for x in range(max(a - clip, 1), min(a + clip, len(route2) - 1)):
                    b = x + 1
                    x_n, b_n = route2[x], route2[b]
                    load_x = loads2[x]

                    if load_x == load_a:
                        old_dist = prob.distance(a_n, y_n) + prob.distance(x_n, b_n)
                        new_dist = prob.distance(a_n, b_n) + prob.distance(x_n, y_n)
                        diff = old_dist - new_dist
                        if diff > best:
                            best = diff
                            save_l1, save_l2, best_y, best_b = l1, l2, y, b
                if best > 0:
                    swaps.append([best, save_l1, save_l2, best_y, best_b])

    swaps.sort(reverse=True)  # Prioritise swaps with best improvements

    # Reformat list of swaps and ignore interfering sections in a best first fashion
    valid = [[len(prob.vehicles[l].route()) + 1 for l in range(num_vehicles)]]
    for s in swaps:
        _, l1, l2, y, b = s
        if y <= valid[0][l1] - 2 and b <= valid[0][l2] - 2:
            valid.append([l1, l2, y, b])
            valid[0][l1] = y
            valid[0][l2] = b

    # Execute valid swaps in a deepcopy of vehicles and return it
    vehicles = deepcopy(prob.vehicles)
    for s in range(1, len(valid)):
        l1, l2, y, b = valid[s]
        v1 = vehicles[l1]
        v2 = vehicles[l2]
        route1 = deepcopy(v1.route())
        route2 = v2.route()
        v1.set_route(route1[:y] + route2[b:])
        v2.set_route(route2[:b] + route1[y:])
    return vehicles


def _segment_swap_difference(prob: ProblemInstance, indices):
    l1, l2, a1, x1, y1, b1, a2, x2, y2, b2 = indices
    route1, route2 = prob.vehicles[l1].route(), prob.vehicles[l2].route()
    a_1, x_1, y_1, b_1 = route1[a1], route1[x1], route1[y1], route1[b1]
    a_2, x_2, y_2, b_2 = route2[a2], route2[x2], route2[y2], route2[b2]
    dist_old, dist_new = 0, 0
    
    if a1 == y1 and a2 == y2:
        print("WARNING: swapping empty segments - wasted computation. This shouldn't happen.")
    elif a1 == y1:  # If segment 1 empty
        dist_old = prob.distance(a_1, b_1) + prob.distance(a_2, x_2) + prob.distance(y_2, b_2)
        dist_new = prob.distance(a_1, x_2) + prob.distance(y_2, b_1) + prob.distance(a_2, b_2)
    elif a2 == y2:  # If segment 2 empty
        dist_old = prob.distance(a_1, x_1) + prob.distance(y_1, b_1) + prob.distance(a_2, b_2)
        dist_new = prob.distance(a_1, b_1) + prob.distance(a_2, x_1) + prob.distance(y_1, b_2)
    else:
        dist_old = prob.distance(a_1, x_1) + prob.distance(y_1, b_1) + prob.distance(a_2, x_2) + prob.distance(y_2, b_2)
        dist_new = prob.distance(a_1, x_2) + prob.distance(y_2, b_1) + prob.distance(a_2, x_1) + prob.distance(y_1, b_2)
        
    return dist_old - dist_new


def intra_segment_swap_fast(prob: ProblemInstance, max_segment_length=-1):
    """
    Compares segments within the same route and tests if swapping them reduces the total distance.
    Original Route:        [...   a1 x1  * * *  y1 b1    ...    a2 x2  # # #  y2 b2  ...]
    Candidates checked:    [...   a1 x2  # # #  y2 b1    ...    a2 x1  * * *  y1 b2  ...]
    """
    swaps = [[] for _ in prob.vehicles]
    prob.calculate_loading_MF()  # Ensure loading instructions are up to date
    num_vehicles = len(prob.vehicles)
    max_segment_length = _relative_segment_length(prob) if max_segment_length < 0 else max_segment_length
    a1_best, b1_best, a2_best, b2_best = 0, 0, 0, 0

    # Loop through all routes
    for l, v in enumerate(prob.vehicles):
        route = v.route()
        loads = v.loads()
        last = len(route)

        # Loop through all possible segments in route1 and record the loading values at each station
        for a1 in range(last - 4):
            x1 = a1 + 1
            load_change_dict = {0: a1}
            load_a1 = loads[a1]
            for y1 in range(x1, min(x1 + max_segment_length, last - 3)):
                load_change_dict[load_a1 - loads[y1]] = y1

            # Loop through all possible swap segments within the same route
            best = 0
            for a2 in range(x1 + 2, last - 2):
                x2 = a2 + 1
                load_a2 = loads[a2]
                for y2 in range(a2, min(x2 + max_segment_length, last - 1)):
                    b2 = y2 + 1
                    load_change = load_a2 - loads[y2]
                    if load_change in load_change_dict:
                        y1 = load_change_dict[load_change]
                        b1 = y1 + 1
                        if b1 <= a2 and (a1 != y1 or a2 != y2):
                            max_load1 = max(loads[a1:b1]) - load_a1
                            min_load1 = min(loads[a1:b1]) - load_a1
                            max_load2 = max(loads[a2:b2]) - load_a2
                            min_load2 = min(loads[a2:b2]) - load_a2
                            if load_a2 + max_load1 <= v.capacity() and load_a2 + min_load1 >= 0 and \
                                    load_a1 + max_load2 <= v.capacity() and load_a1 + min_load2 >= 0:

                                diff = _segment_swap_difference(prob, [l, l, a1, x1, y1, b1, a2, x2, y2, b2])
                                if diff > best:
                                    best = diff
                                    a1_best, b1_best, a2_best, b2_best = a1, b1, a2, b2
                            else:
                                break
            if best > 0:
                swaps[l].append([best, a1_best, b1_best, a2_best, b2_best])

    # Reformat list of swaps and ignore interfering indices
    valid = [[] for _ in range(num_vehicles)]
    for l, v in enumerate(prob.vehicles):
        swaps[l].sort(reverse=True)
        for s in swaps[l]:
            _, a1, b1, a2, b2 = s
            if _non_overlapping(valid[l], [a1, b1]) and _non_overlapping(valid[l], [a2, b2]):
                valid[l].append([a1, b1, a2, b2])
                valid[l].append([a2, b2, a1, b1])

    # Execute valid swaps in a deepcopy of vehicles and return it
    vehicles = deepcopy(prob.vehicles)
    for l, swaps in enumerate(valid):
        v = vehicles[l]
        swaps.sort(reverse=True)
        for s in swaps:
            a1, b1, a2, b2 = s
            route = v.route()
            original_route = prob.vehicles[l].route()
            v.set_route(route[:a1 + 1] + original_route[a2 + 1:b2] + route[b1:])
    return vehicles


def inter_segment_swap_fast(prob: ProblemInstance, max_segment_length=10):
    """
    Compares segments within the same route and tests if swapping them reduces the total distance.
    Original Routes:       [...   a1 x1  * * *  y1 b1   ...]  [...   a2 x2  # # #  y2 b2  ...]
    Candidates checked:    [...   a1 x2  # # #  y2 b1   ...]  [...   a2 x1  * * *  y1 b2  ...]
    """
    swaps = []
    prob.calculate_loading_MF()  # Ensure loading instructions are up to date
    num_vehicles = len(prob.vehicles)
    max_segment_length = _relative_segment_length(prob) if max_segment_length < 0 else max_segment_length
    save_l1, save_l2, a1_best, b1_best, a2_best, b2_best = 0, 0, 0, 0, 0, 0

    # Loop through all candidate route1
    for l1 in range(num_vehicles - 1):
        v1 = prob.vehicles[l1]
        route1 = v1.route()
        loads1 = v1.loads()
        last1 = len(route1)

        # Loop through all possible segments in route1 and record the loading values at each station
        for a1 in range(1, last1 - 2):
            x1 = a1 + 1
            load_change_dict = {0: a1}
            load1 = loads1[a1]

            for y1 in range(x1, min(x1 + max_segment_length, last1 - 1)):
                load_change_dict[load1 - loads1[y1]] = y1

            # Loop through all candidate route2
            for l2 in range(l1 + 1, num_vehicles):
                v2 = prob.vehicles[l2]
                route2 = v2.route()
                loads2 = v2.loads()
                last2 = len(route2)

                # Loop through all possible segments in route2 and look for matching load changes
                best = 0
                for a2 in range(1, last2 - 2):
                    x2 = a2 + 1
                    load2 = loads2[a2]

                    for y2 in range(a2, min(a2 + max_segment_length, last2 - 1)):
                        b2 = y2 + 1
                        load_change = load2 - loads2[y2]

                        # Verify that segments have equal load change and swapping does not exceed capacity constraints
                        if load_change in load_change_dict:
                            y1 = load_change_dict[load_change]
                            b1 = y1 + 1
                            if a1 != y1 or a2 != y2:
                                max_load1 = max(loads1[a1:b1]) - load1
                                min_load1 = min(loads1[a1:b1]) - load1
                                max_load2 = max(loads2[a2:b2]) - load2
                                min_load2 = min(loads2[a2:b2]) - load2
                                if load2 + max_load1 <= v2.capacity() and load2 + min_load1 >= 0 and \
                                        load1 + max_load2 <= v1.capacity() and load1 + min_load2 >= 0:

                                    diff = _segment_swap_difference(prob, [l1, l2, a1, x1, y1, b1, a2, x2, y2, b2])
                                    if diff > best:
                                        best = diff
                                        save_l1, save_l2 = l1, l2
                                        a1_best, b1_best, a2_best, b2_best = a1, b1, a2, b2
                                else:
                                    break
                if best > 0:
                    swaps.append([best, save_l1, save_l2, a1_best, b1_best, a2_best, b2_best])

    swaps.sort(reverse=True)
    vehicles = deepcopy(prob.vehicles)
    valid = [[] for _ in range(len(prob.vehicles))]

    # Reformat list of swaps and ignore interfering indices
    for s in swaps:
        _, l1, l2, a1, b1, a2, b2 = s
        if _non_overlapping(valid[l1], [a1, b1]) and _non_overlapping(valid[l2], [a2, b2]):
            valid[l1].append([a1, b1, l2, a2, b2])
            valid[l2].append([a2, b2, l1, a1, b1])

    # Execute valid swaps in a deepcopy of vehicles and return it
    for l1, swaps in enumerate(valid):
        v1 = vehicles[l1]
        swaps.sort(reverse=True)
        for s in swaps:
            a1, b1, l2, a2, b2 = s
            route1 = v1.route()
            route2 = prob.vehicles[l2].route()
            v1.set_route(route1[:a1 + 1] + route2[a2 + 1:b2] + route1[b1:])
    return vehicles


def _non_overlapping(slices, new_slice):
    """Determines if a new index pair overlaps with any of the indexes in the given set
        [...      x - - - y      ...] index pair in slices
        [...    a b              ...] Overlap
        [...              a b    ...] Overlap
        [...        a - b        ...] Overlap
        [...    a - - - - - b    ...] Overlap
        [...  a b                ...]   OK
        [...                a b  ...]   OK
    """
    for pair in slices:
        x, y = pair[0], pair[1]
        a, b = new_slice
        if b >= x and a <= y:
            return False
    return True


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
            candidate[bi].set_route(candidate[bi].route()[:j] + candidate[bi].route()[j + 1:])
            candidate[bi].set_loads(candidate[bi].loads()[:j] + candidate[bi].loads()[j + 1:])
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
    route_distance = []

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

            dist = 0
            route = vehicles[i].route()
            prev = route[0]
            for s in range(1, len(route)):
                dist += graph.edges[prev, route[s]][metric]
                prev = route[s]
            route_distance.append(dist)

    if mode == 'worst':
        # sort the idxes according to their distance
        sorting_args = np.argsort(np.array(distance_to_visit))[::-1]
        idxes = np.array(idxes)
        idxes = idxes[sorting_args].tolist()
    elif mode == 'random':
        # uniform distribution
        meta_parameter = 1
    elif mode == 'distance':
        # first, sort by distance improvement
        sorting_args = np.argsort(np.array(distance_to_visit))[::-1]
        idxes = np.array(idxes)
        idxes = idxes[sorting_args].tolist()
        # second, sort by distance of the respective route
        sorting_args = np.argsort(np.array(route_distance))[::-1]
        idxes = np.array(idxes)
        idxes = idxes[sorting_args].tolist()


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


# def insertU_nearest_generator(vehicles, unbalanced_stations, graph):
#     """insert unbalanced stations to minimum distance position on each route
#         return:
#             Generator which generate insertU candidate
#     """
#     candidate = deepcopy(vehicles)
#     random.shuffle(unbalanced_stations)
#     for u in unbalanced_stations:
#         best_distance = 1e+10  # init
#         bi = 0
#         bj = 0
#         for i, vehicle in enumerate(candidate):
#             route = vehicle.route()
#             if len(route) < 2:
#                 continue
#             for j in range(1, len(route) - 1):
#                 if route[j - 1] == u or route[j] == u:
#                     continue
#                 current_distance = graph.edges[route[j - 1], u]['dist'] + graph.edges[route[j], u]['dist']
#                 if current_distance < best_distance:
#                     best_distance = current_distance
#                     bj = j
#                     bi = i

#         candidate[bi].set_route(candidate[bi].route()[:bj] + [u] + candidate[bi].route()[bj:])
#         # set new load as 0
#         candidate[bi].set_loads(candidate[bi].loads()[:bj] + [0] + candidate[bi].loads()[bj:])
#     return candidate


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
    problem_instance_copy.calculate_loading_MF()
    graph = _get_rebalanced_graph(problem_instance_copy.model.copy(), vehicles)
    unbalanced_stations = [x for x in graph.nodes if graph.nodes[x]['sup'] != 0]

    return unbalanced_stations


def _get_loading_and_unbalanced_stations(problem_instance, vehicles):
    """Given vehicles, calculate best loading instructions and return unbalanced stations
    """
    problem_instance.vehicles = vehicles
    problem_instance.calculate_loading_MF()
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


def destroy_rebuild(problem_instance, num_removal=3, verbose=0):
    """Destroy and rebuild the set of routes.

    return
        vehicles
    """
    start_time_outer = time.time()
    copied_problem_instance = deepcopy(problem_instance)
    # print('Distance before applying the LN: '
    #       + str(copied_problem_instance.calculate_distances()) + '.')

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

def destroy_local(problem_instance, num_removal=3, num_removal_change_step = 5, verbose=1, timeout=10):
    """Destroy operator.

    return
        vehicles
    """
    copied_problem_instance = deepcopy(problem_instance)
    # the initial insert ratio
    num_removal_current = num_removal
    counter = 0
    start_time = time.time()
    for removed_vehicles in remove_worst_meta_generator(copied_problem_instance.vehicles,
                                                        copied_problem_instance.model.copy(), mode='distance', meta_parameter=2,
                                                        num_removal=num_removal):
        unbalanced_stations = get_unbalanced_stations(copied_problem_instance, removed_vehicles)

        couter += 1
        if counter % num_removal_change_step == 0 and num_removal_current > 1:
            # try to remove less stations next time, so there is a higher change to obtain a feasible solution
            num_removal_current -= 1

        if not unbalanced_stations:
            # if removal neighbor routes are possibly balanced, return them
            return removed_vehicles

        if time.time() - start_time > timeout:
            break

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
            return removed_vehicles
        vehicles = insertU_nearest_v3(removed_vehicles, unbalanced_stations, copied_problem_instance.model.copy())
        unbalanced_stations = _get_loading_and_unbalanced_stations(copied_problem_instance, vehicles)
        if not unbalanced_stations:
            # print(f"Found feasible routes by insertion.")
            # execution_time_outer = time.time() - start_time_outer
            # print('It took ' + str(execution_time_outer) + ' seconds to find a feasible solution.')
            return vehicles
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

