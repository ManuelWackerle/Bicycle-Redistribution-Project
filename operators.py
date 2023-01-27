"""
Implementations of the neighbourhood operators.
"""
import random
from copy import deepcopy
from structure import Vehicle, ProblemInstance


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


def inter_segment_swap(prob, max_segment_length=10, tolerance=0):
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
                        # elif -load_change in load_change_dict: Todo: add reverse insert check
            if best > 0:
                swaps.append([best, m1, m2, a1, b1, a2, b2])

    swaps.sort(reverse=True)
    vehicles = deepcopy(prob.vehicles)
    valid = [[] for i in range(len(prob.vehicles))]
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


def intra_two_opt_fast(prob:ProblemInstance, tolerance=0):
    """
    Compares edges within the same route and Tests if swapping them reduces the total distance.
    Candidates are collected an executed on a deepcopy of the problem instance in a best improvement first fashion.
    Original Route:       [...   a x > > ... > > y b   ...]
    Candidates checked:   [...   a y < < ... < < x b   ...]   (Note: inner segment traversed in opposite direction to original)
    """

    prob.calculate_loading_MF() #Ensure loading instructions are up to date.
    num_vehicles = len(prob.vehicles)
    swaps = [[] for _ in range(num_vehicles)]
    a_best, b_best = 0, 0

    # Loop through all routes
    for l, v in enumerate(prob.vehicles):
        route = v.route()
        loads = v.loads()

        #Loop through all candidate edges (a, x)
        for a_idx in range(1, len(route) - 4):
            x_idx = a_idx + 1
            a, x = route[a_idx], route[x_idx]
            load_a = loads[a_idx]
            max_load, min_load = max(load_a, loads[x_idx]), min(load_a, loads[x_idx])
            best = 0

            # Loop through all candidate edges (y, b)
            for y_idx in range(x_idx + 1, len(route) - 2):
                b_idx = y_idx + 1
                y, b = route[y_idx], route[b_idx]
                load_y = loads[y_idx]

                max_load = max(max_load, loads[y_idx])
                min_load = min(min_load, loads[y_idx])
                max_load_diff = load_y - max_load
                min_load_diff = load_y - min_load
                if load_a + min_load_diff <= v.capacity() and load_a + max_load_diff >= 0:

                    old_dist = prob.distance(a, x) + prob.distance_route_segment(route, x_idx, y_idx) + prob.distance(y, b)
                    new_dist = prob.distance(a, y) + prob.distance_route_segment(route, y_idx, x_idx) + prob.distance(x, b)
                    diff = old_dist - new_dist
                    if diff > best:
                        best = diff
                        a_best, b_best = a_idx, b_idx
                else:
                    break

            # Save best candidate pair [(a, x), (y, b)] if one exists
            if best > 0:
                swaps[l].append([best, a_best, b_best])

    # Reformat list of swaps and ignore interfering sections in a best first fashion
    valid = [[] for _ in range(num_vehicles)]
    for l, v in enumerate(prob.vehicles):
        swaps[l].sort(reverse=True) #Prioritse swaps with best improvements
        for s in swaps[l]:
            _, a_idx, b_idx = s
            if non_overlapping(valid[l], [a_idx, b_idx]):
                valid[l].append([a_idx, b_idx])

    # Execute valid swaps in a deepcopy of vehicles and return it
    vehicles = deepcopy(prob.vehicles)
    for l, swap in enumerate(valid):
        v = vehicles[l]
        for s in swap:
            a_idx, b_idx = s
            route = v.route()
            v.set_route(route[:a_idx + 1] + route[b_idx - 1:a_idx:-1] + route[b_idx:])
    return vehicles


def inter_two_opt_fast(prob, max_length_alteration=5, tolerance=0):
    """
    Compares edges between different routes and Tests if swapping them reduces the total distance.
    Candidates are collected an executed on a deepcopy of the problem instance in a best improvement first fashion.
    Original Routes:       [...  - - a y # #    ...] [...  - - x b * *    ...]
    Candidates checked:    [...  - - a b * *    ...] [...  - - x y # #    ...]
    """

    prob.calculate_loading_MF()  # Ensure loading instructions are up to date.
    num_vehicles = len(prob.vehicles)
    swaps = []
    clip = max_length_alteration
    save_l1, save_l2, best_y, best_b = 0, 0, 0, 0

    # Loop through all vehicle combinations
    for l1, v1 in enumerate(prob.vehicles):
        route1 = v1.route()
        loads1 = v1.loads()
        for l2 in range(l1 + 1, num_vehicles):
            route2 = prob.vehicles[l2].route()
            loads2 = prob.vehicles[l2].loads()

            # Loop through all edge swap combinations within the max length alteration
            for a_idx in range(1, len(route1) - 3):
                y_idx = a_idx + 1
                a, y = route1[a_idx], route1[y_idx]
                load_a = loads1[a_idx]
                best = 0
                for x_idx in range(max(a_idx - clip, 1), min(a_idx + clip, len(route2) - 1)):
                    b_idx = x_idx + 1
                    x, b = route2[x_idx], route2[b_idx]
                    load_x = loads2[x_idx]

                    if load_x == load_a:
                        old_dist = prob.distance(a, y)+ prob.distance(x, b)
                        new_dist = prob.distance(a, b)+ prob.distance(x, y)
                        diff = old_dist - new_dist
                        if diff > best:
                            best = diff
                            save_l1, save_l2, best_y, best_b  = l1, l2, y_idx, b_idx
                if best > 0:
                    swaps.append([best, save_l1, save_l2, best_y, best_b])

    swaps.sort(reverse=True) # Prioritise swaps with best improvements

    # Reformat list of swaps and ignore interfering sections in a best first fashion
    valid = [[len(prob.vehicles[l].route()) + 1 for l in range(num_vehicles)]]
    for s in swaps:
        _, l1, l2, y_idx, b_idx = s
        if y_idx <= valid[0][l1] - 2 and b_idx <= valid[0][l2] - 2:
            valid.append([l1, l2, y_idx, b_idx])
            valid[0][l1] = y_idx
            valid[0][l2] = b_idx

    # Execute valid swaps in a deepcopy of vehicles and return it
    vehicles = deepcopy(prob.vehicles)
    for s in range(1, len(valid)):
            l1, l2, y_idx, b_idx = valid[s]
            v1 = vehicles[l1]
            v2 = vehicles[l2]
            route1 = deepcopy(v1.route())
            route2 = v2.route()
            v1.set_route(route1[:y_idx] + route2[b_idx:])
            v2.set_route(route2[:b_idx] + route1[y_idx:])
    return vehicles


def intra_or_opt_fast(prob, tolerance=0):
    """
    Deprecated: use instead: intra_segment_swap(<problem>, <segment length>, tolerance=<tolerance>)
    """
    intra_segment_swap_fast(prob, 1, tolerance=tolerance)

def segment_swap_difference(prob:ProblemInstance, a1, x1, y1, b1, a2, x2, y2, b2):
    #We allow segement 2 to be empty
    if a2 == y2:
        dist_old = prob.distance(a1, x1) + prob.distance(y1, b1) + prob.distance(a2, x2)
        dist_new = prob.distance(a1, b1) + prob.distance(a2, x1) + prob.distance(y1, b2)
    else:
        dist_old = prob.distance(a1, x1) + prob.distance(y1, b1) \
                   + prob.distance(a2, x2) + prob.distance(y2, b2)
        dist_new = prob.distance(a1, x2) + prob.distance(y2, b1) \
                   + prob.distance(a2, x1) + prob.distance(y1, b2)
    return dist_old - dist_new


def intra_segment_swap_fast(prob:ProblemInstance, max_segment_length=10, tolerance=0):
    """
    Compares segments within the same route and Tests if swapping them reduces the total distance
    Original Route:        [...   a1 x1  * * *  y1 b1    ...    a2 x2  # # #  y2 b2  ...]
    Candidates checked:    [...   a1 x2  # # #  y2 b1    ...    a2 x1  * * *  y1 b2  ...]
    """
    swaps = [[] for _ in prob.vehicles]
    prob.calculate_loading_MF() #Ensure loading instructions are up to date
    num_vehicles = len(prob.vehicles)
    a1_best, b1_best, a2_best, b2_best = 0, 0, 0, 0

    # Loop through all routes
    for l, v in enumerate(prob.vehicles):
        route = v.route()
        loads = v.loads()
        last = len(route)

        # Loop through all possible segments in route1 and record the loading values at each station
        for a1_idx in range(last - 4):
            x1_idx = a1_idx + 1
            a1, x1 = route[a1_idx], route[x1_idx]
            load_change_dict = {}
            load_a1 = loads[a1_idx]
            for y1_idx in range(x1_idx, min(x1_idx + max_segment_length, last - 3)):
                load_change_dict[load_a1 - loads[y1_idx]] = y1_idx

            # Loop through all possible swap segments within the same route
            best = 0
            for a2_idx in range(x1_idx + 2, last - 2):
                x2_idx = a2_idx + 1
                a2, x2 = route[a2_idx], route[x2_idx]
                load_a2 = loads[a2_idx]

                for y2_idx in range(a2_idx, min(x2_idx + max_segment_length, last - 1)):
                    b2_idx = y2_idx + 1
                    load_change = load_a2 - loads[y2_idx]
                    if load_change in load_change_dict:
                        y1_idx = load_change_dict[load_change]
                        b1_idx = y1_idx + 1
                        if b1_idx <= a2_idx:
                            max_load1 = max(loads[a1_idx:b1_idx]) - load_a1
                            min_load1 = min(loads[a1_idx:b1_idx]) - load_a1
                            max_load2 = max(loads[a2_idx:b2_idx]) - load_a2
                            min_load2 = min(loads[a2_idx:b2_idx]) - load_a2
                            if load_a2 + max_load1 <= v.capacity() and load_a2 + min_load1 >= 0 and \
                                    load_a1 + max_load2 <= v.capacity() and load_a1 + min_load2 >= 0:
                                y1, b1 = route[y1_idx], route[b1_idx]
                                y2, b2 = route[y2_idx], route[b2_idx]
                                diff = segment_swap_difference(prob, a1, x1, y1, b1, a2, x2, y2, b2)
                                if diff > best:
                                    best = diff
                                    a1_best, b1_best, a2_best, b2_best = a1_idx, b1_idx, a2_idx, b2_idx
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
            if non_overlapping(valid[l], [a1, b1]) and non_overlapping(valid[l], [a2, b2]):
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


def inter_segment_swap_fast(prob:ProblemInstance, max_segment_length=10, tolerance=0):
    """
    Compares segments within the same route and Tests if swapping them reduces the total distance
    Original Routes:       [...   a1 x1  * * *  y1 b1   ...]  [...   a2 x2  # # #  y2 b2  ...]
    Candidates checked:    [...   a1 x2  # # #  y2 b1   ...]  [...   a2 x1  * * *  y1 b2  ...]
    """
    swaps = []
    prob.calculate_loading_MF() #Ensure loading instructions are up to date
    vehicle_indices = list(range(len(prob.vehicles)))
    random.shuffle(vehicle_indices)
    visited = 0
    save_l1, save_l2, a1_best, b1_best, a2_best, b2_best = 0, 0, 0, 0, 0, 0

    # Loop through all candidate route1
    for l1 in vehicle_indices[:-2]:
        visited += 1
        v1 = prob.vehicles[l1]
        route1 = v1.route()
        loads1 = v1.loads()
        last1 = len(route1)

        # Loop through all possible segments in route1 and record the loading values at each station
        for a1_idx in range(1, last1 - 2):
            x1_idx = a1_idx + 1
            a1, x1 = route1[a1_idx], route1[x1_idx]
            load_change_dict = {}
            load1 = loads1[a1_idx]

            for y1_idx in range(x1_idx, min(x1_idx + max_segment_length, last1 - 1)):
                load_change_dict[load1 - loads1[y1_idx]] = y1_idx

            # Loop through all candidate route2
            for l2 in vehicle_indices[visited:]:
                v2 = prob.vehicles[l2]
                route2 = v2.route()
                loads2 = v2.loads()
                last2 = len(route2)

                # Loop through all possible segments in route2 and look for matching load changes
                best = 0
                for a2_idx in range(1, last2 - 2):
                    x2_idx = a2_idx + 1
                    a2, x2 = route2[a2_idx], route2[x2_idx]
                    load2 = loads2[a2_idx]

                    for y2_idx in range(a2_idx, min(a2_idx + max_segment_length, last2 - 1)):
                        b2_idx = y2_idx + 1
                        load_change = load2 - loads2[y2_idx]

                        # Verify that segments have equal load change and swapping does not exceed capacity constraints
                        if load_change in load_change_dict:
                            y1_idx = load_change_dict[load_change]
                            b1_idx = y1_idx + 1
                            max_load1 = max(loads1[a1_idx:b1_idx]) - load1
                            min_load1 = min(loads1[a1_idx:b1_idx]) - load1
                            max_load2 = max(loads2[a2_idx:b2_idx]) - load2
                            min_load2 = min(loads2[a2_idx:b2_idx]) - load2
                            if load2 + max_load1 <= v2.capacity() and load2 + min_load1 >= 0 and \
                                    load1 + max_load2 <= v1.capacity() and load1 + min_load2 >= 0:
                                y1, b1 = route1[y1_idx], route1[b1_idx]
                                y2, b2 = route2[y2_idx], route2[b2_idx]
                                odd = a1, x1, y1, b1, a2, x2, y2, b2
                                if a1 == '0' and b1 == '0' or a2 == '0' and x1 == '0' or y1 == '0' and b2 == '0':
                                    print("We're are once again Fucked")
                                diff = segment_swap_difference(prob, a1, x1, y1, b1, a2, x2, y2, b2)
                                if diff > best:
                                    best = diff
                                    save_l1, save_l2 = l1, l2
                                    a1_best, b1_best, a2_best, b2_best = a1_idx, b1_idx, a2_idx, b2_idx
                            else:
                                break
                if best > 0:
                    swaps.append([best, save_l1, save_l2, a1_best, b1_best, a2_best, b2_best])

    swaps.sort(reverse=True)
    vehicles = deepcopy(prob.vehicles)
    valid = [[] for i in range(len(prob.vehicles))]

    # Reformat list of swaps and ignore interfering indices
    for s in swaps:
        _, l1, l2, a1, b1, a2, b2 = s
        if non_overlapping(valid[l1], [a1, b1]) and non_overlapping(valid[l2], [a2, b2]):
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


def non_overlapping(slices, new_slice):
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
