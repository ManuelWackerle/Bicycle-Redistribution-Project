"""
Implementations of the neighbourhood operators.

Neighbourhood operators. Given an initial route, each operator returns an array of new routes obtained by some
modification of the initial route. The general type of these functions is:
def neighbourhood_operator ( initial_routes [[]], **kwargs) -> list_of_modified_routes: [[[]]]
"""
import random
import sys
import time
import numpy as np
from copy import deepcopy
from structure import ProblemInstance


def intra_two_opt(prob: ProblemInstance):
    """
        Compares edges within the same route and Tests if swapping them reduces the total distance.
        Candidates are collected an executed on a deepcopy of the problem instance in a best improvement first fashion.
        Original Route:       [...   a x > > ... > > y b   ...]
        Candidates checked:   [...   a y < < ... < < x b   ...] (Note: inner segment traversed in reverse after swap)
        :param prob: the problem instance
    """

    prob.calculate_loading_mf()  # Ensure loading instructions are up to date.
    num_vehicles = len(prob.vehicles)
    swaps = [[] for _ in range(num_vehicles)]
    a_best, b_best = 0, 0

    # Loop through all routes
    for k, v in enumerate(prob.vehicles):
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
                    old_dist = prob.distance(a_n, x_n) + prob.distance_in_route(route, x, y) + prob.distance(y_n, b_n)
                    new_dist = prob.distance(a_n, y_n) + prob.distance_in_route(route, y, x) + prob.distance(x_n, b_n)
                    diff = old_dist - new_dist
                    if diff > best:
                        best = diff
                        a_best, b_best = a, b
                else:
                    break

            # Save best candidate pair [(a, x), (y, b)] if one exists
            if best > 0:
                swaps[k].append([best, a_best, b_best])

    # Reformat list of swaps and ignore interfering sections in a best first fashion
    valid = [[] for _ in range(num_vehicles)]
    for k, v in enumerate(prob.vehicles):
        swaps[k].sort(reverse=True)  # Prioritise swaps with best improvements
        for s in swaps[k]:
            _, a, b = s
            if _non_overlapping(valid[k], [a, b]):
                valid[k].append([a, b])

    # Execute valid swaps in a deepcopy of vehicles and return it
    vehicles = deepcopy(prob.vehicles)
    for k, swap in enumerate(valid):
        v = vehicles[k]
        for s in swap:
            a, b = s
            route, loads = v.route(), v.loads()
            v.set_route(route[:a + 1] + route[b - 1:a:-1] + route[b:])
            v.set_loads(loads[:a + 1] + loads[b - 1:a:-1] + loads[b:])
    return vehicles


def inter_two_opt(prob: ProblemInstance, max_length_alteration=-1):
    """
        Compares edges between different routes and Tests if swapping them reduces the total distance.
        Candidates are collected an executed on a deepcopy of the problem instance in a best improvement first fashion.
        Original Routes:       [...  - - a y # #    ...] [...  - - x b * *    ...]
        Candidates checked:    [...  - - a b * *    ...] [...  - - x y # #    ...]
        :param prob: the problem instance
        :param max_length_alteration: -1 for default, else sets max number of stops a route can be altered by
    """

    prob.calculate_loading_mf()  # Ensure loading instructions are up to date.
    num_vehicles = len(prob.vehicles)
    swaps = []
    clip = _relative_segment_length(prob) // 2 if max_length_alteration < 0 else max_length_alteration
    save_l1, save_l2, best_y, best_b = 0, 0, 0, 0

    # Loop through all vehicle combinations
    for l1, v1 in enumerate(prob.vehicles):
        route1 = v1.route()
        loads1 = v1.loads()
        length1 = len(route1)
        for l2 in range(l1 + 1, num_vehicles):
            route2 = prob.vehicles[l2].route()
            loads2 = prob.vehicles[l2].loads()
            length2 = len(route2)
            mid = (length2 - length1)//2

            # Loop through all edge swap combinations within the max length alteration
            for a in range(1, len(route1) - 3):
                y = a + 1
                a_n, y_n = route1[a], route1[y]
                load_a = loads1[a]
                best = 0
                for x in range(max(a + mid - clip, 1), min(a + mid + clip, len(route2) - 1)):
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
    valid = [[len(prob.vehicles[k].route()) + 1 for k in range(num_vehicles)]]
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
        route1, loads1 = deepcopy(v1.route()), deepcopy(v1.loads())
        route2, loads2 = v2.route(), v2.loads()
        v1.set_route(route1[:y] + route2[b:])
        v2.set_route(route2[:b] + route1[y:])
        v1.set_loads(loads1[:y] + loads2[b:])
        v2.set_loads(loads2[:b] + loads1[y:])
    return vehicles


def _relative_segment_length(prob: ProblemInstance) -> int:
    return len(prob.model.nodes) // len(prob.vehicles) // 5


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


def intra_segment_swap(prob: ProblemInstance, max_segment_length=-1):
    """
        Compares segments within the same route and tests if swapping them reduces the total distance.
        Original Route:        [...   a1 x1  * * *  y1 b1    ...    a2 x2  # # #  y2 b2  ...]
        Candidates checked:    [...   a1 x2  # # #  y2 b1    ...    a2 x1  * * *  y1 b2  ...]
        :param prob: the problem instance
        :param max_segment_length: -1 for default (1/5th route length) else sets max number of stops in a segment
    """
    swaps = [[] for _ in prob.vehicles]
    prob.calculate_loading_mf()  # Ensure loading instructions are up to date
    num_vehicles = len(prob.vehicles)
    max_segment_length = _relative_segment_length(prob) if max_segment_length < 0 else max_segment_length
    a1_best, b1_best, a2_best, b2_best = 0, 0, 0, 0

    # Loop through all routes
    for k, v in enumerate(prob.vehicles):
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

                                diff = _segment_swap_difference(prob, [k, k, a1, x1, y1, b1, a2, x2, y2, b2])
                                if diff > best:
                                    best = diff
                                    a1_best, b1_best, a2_best, b2_best = a1, b1, a2, b2
                            else:
                                break
            if best > 0:
                swaps[k].append([best, a1_best, b1_best, a2_best, b2_best])

    # Reformat list of swaps and ignore interfering indices
    valid = [[] for _ in range(num_vehicles)]
    for k, v in enumerate(prob.vehicles):
        swaps[k].sort(reverse=True)
        for s in swaps[k]:
            _, a1, b1, a2, b2 = s
            if _non_overlapping(valid[k], [a1, b1]) and _non_overlapping(valid[k], [a2, b2]):
                valid[k].append([a1, b1, a2, b2])
                valid[k].append([a2, b2, a1, b1])

    # Execute valid swaps in a deepcopy of vehicles and return it
    vehicles = deepcopy(prob.vehicles)
    for k, swaps in enumerate(valid):
        v = vehicles[k]
        swaps.sort(reverse=True)
        for s in swaps:
            a1, b1, a2, b2 = s
            route, loads = v.route(), v.loads()
            original_route, original_loads = prob.vehicles[k].route(), prob.vehicles[k].loads()
            v.set_route(route[:a1 + 1] + original_route[a2 + 1:b2] + route[b1:])
            v.set_loads(loads[:a1 + 1] + original_loads[a2 + 1:b2] + loads[b1:])
    return vehicles


def inter_segment_swap(prob: ProblemInstance, max_segment_length=10):
    """
        Compares segments within the same route and tests if swapping them reduces the total distance.
        Original Routes:       [...   a1 x1  * * *  y1 b1   ...]  [...   a2 x2  # # #  y2 b2  ...]
        Candidates checked:    [...   a1 x2  # # #  y2 b1   ...]  [...   a2 x1  * * *  y1 b2  ...]
        :param prob: the problem instance
        :param max_segment_length: -1 for default (1/5th route length) else sets max number of stops in a segment
    """
    swaps = []
    prob.calculate_loading_mf()  # Ensure loading instructions are up to date
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

                    # Use restrictions if lengths of routes differ too much
                    start, length = a2, max_segment_length
                    if last2 < 0.8 * last1:    # route2 too short
                        length = 1
                    elif last2 > 1.2 * last1:  # route2 too long
                        start = x2
                        length = 2 * max_segment_length

                    for y2 in range(start, min(a2 + length, last2 - 1)):
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
            route1, loads1 = v1.route(), v1.loads()
            route2, loads2 = prob.vehicles[l2].route(), prob.vehicles[l2].loads()
            v1.set_route(route1[:a1 + 1] + route2[a2 + 1:b2] + route1[b1:])
            v1.set_loads(loads1[:a1 + 1] + loads2[a2 + 1:b2] + loads1[b1:])
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
        n = len(route)
        # if n < 5:
        #     print(f"Warning! The vehicle {k} has only {n} stations in the route.")
        remove_idxes = set()
        for i, station in enumerate(route):
            if i < n - 1 and station == route[i + 1]:
                remove_idxes.add(i)
        vehicle.set_route([s for i, s in enumerate(route) if i not in remove_idxes])
        vehicle.set_loads([k for i, k in enumerate(vehicle.loads()) if i not in remove_idxes])
    return vehicles


def remove_multi_stations_generator(vehicles, at_random=False, num_removal=5):
    """generate routes by removing multiple stations

    Write
        k = Number of trucks (=len(self.routes))
        Ln = Route length (=len(self.routes[k])-1)
        C = Number of candidates
    Note
        Ln depends on n in k. (route length might be different for each vehicles)
    :return
        Generator which generates vehicles with the shape (k, Ln) with total number C
    """
    idxes = []
    k = len(vehicles)
    for i in range(k):
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
    removal_idxes = [[] for _ in range(k)]
    for i, j in idxes:
        if nr < num_removal:
            removal_idxes[i].append(j)
            nr += 1
        else:
            for n in range(k):
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
            removal_idxes = [[] for _ in range(k)]
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
            while route.count('x'):
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
                        best_i_j = deepcopy([i, j])
                        best_disbalance = 0

                    else:
                        if best_disbalance > len(unbalanced_stations_candidate):
                            second_best_disbalance = deepcopy(best_disbalance)
                            best_i_j = deepcopy([i, j])
                            best_disbalance = len(unbalanced_stations_candidate)
                        else:
                            if second_best_disbalance > len(unbalanced_stations_candidate):
                                second_best_disbalance = len(unbalanced_stations_candidate)
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
                                  insert_ratio=0.5):
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


def _get_rebalanced_graph(graph, vehicles):
    """
        Given routes and instructions, returns graph after rebalancing
    """
    for v in vehicles:
        prev_load = 0
        for s in range(len(v.route())-1):
            load = v.loads()[s]
            diff = prev_load - load
            prev_load = load
            graph.nodes[v.route()[s]]['sup'] += diff
    return graph


def insert_nearest_v3(vehicles, unbalanced_stations, graph):
    """
        Insert unbalanced stations randomly to one of the minimum distance position on each route
        return: generator which generate insertU candidate
    """

    def _insert_ordered(arr, c, index, n):
        ret = arr
        for k in range(len(arr)):
            if c[index] < arr[k][index]:
                ret = arr[:k] + [c] + arr[k:]
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

        bi, bj, _ = random.choices(top_n, probs[:len(top_n)])[0]
        candidate[bi].set_route(candidate[bi].route()[:bj] + [u] + candidate[bi].route()[bj:])
        # set new load as 0
        candidate[bi].set_loads(candidate[bi].loads()[:bj] + [0] + candidate[bi].loads()[bj:])
    return candidate


def get_unbalanced_stations(problem_instance, vehicles):
    problem_instance_copy = deepcopy(problem_instance)
    problem_instance_copy.vehicles = vehicles
    problem_instance_copy.calculate_loading_mf()
    graph = _get_rebalanced_graph(problem_instance_copy.model.copy(), vehicles)
    unbalanced_stations = [x for x in graph.nodes if graph.nodes[x]['sup'] != 0]

    return unbalanced_stations


def _get_loading_and_unbalanced_stations(problem_instance, vehicles):
    """
        Given vehicles, calculate best loading instructions and return unbalanced stations
    """
    problem_instance.vehicles = vehicles
    problem_instance.calculate_loading_mf()
    graph = _get_rebalanced_graph(problem_instance.model.copy(), vehicles)
    unbalanced_stations = [x for x in graph.nodes if graph.nodes[x]['sup'] != 0]
    return unbalanced_stations


def destroy_rebuild(problem_instance, num_removal=3, verbose=0):
    """
        Destroy and rebuild the set of routes.
        :return vehicles: set of vehicles with new routes after destroying and rebuilding routes
    """
    start_time_outer = time.time()
    copied_problem_instance = deepcopy(problem_instance)
    
    # the initial insert ratio
    insert_ratio = 0.5
    for removed_vehicles in remove_worst_meta_generator(copied_problem_instance.vehicles,
                                                        copied_problem_instance.model.copy(), mode='worst',
                                                        meta_parameter=2, num_removal=num_removal):
        unbalanced_stations = get_unbalanced_stations(copied_problem_instance, removed_vehicles)
        if not unbalanced_stations:  # if removal neighbor routes are possibly balanced, return them
            return removed_vehicles

        start_time_inner = time.time()
        inserted_vehicles = insert_regret_generator_quick(removed_vehicles, copied_problem_instance,
                                                          insert_ratio=insert_ratio)
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
                      + str(output_problem_copy.calculate_costs()) + '.')

            return inserted_vehicles
        if insert_ratio < 1:
            # the insert ratio for the next rebuild is increased if the previous rebuild wasn't successful
            insert_ratio += 0.05

    # if there is no candidate, return original
    return problem_instance.vehicles


def destroy_local(problem_instance, num_removal=3, num_removal_change_step=5, num_iter=20, timeout=10000):
    """
        Destroy operator.
        :return vehicles: set of vehicles with new routes after destroying and rebuilding routes
    """
    copied_problem_instance = deepcopy(problem_instance)
    # the initial insert ratio
    num_removal_current = num_removal
    counter = 0
    start_time = time.time()
    for removed_vehicles in remove_worst_meta_generator(copied_problem_instance.vehicles,
                                                        copied_problem_instance.model.copy(), mode='distance',
                                                        meta_parameter=2, num_removal=num_removal):
        unbalanced_stations = get_unbalanced_stations(copied_problem_instance, removed_vehicles)

        counter += 1
        if counter % num_removal_change_step == 0 and num_removal_current > 1:
            # try to remove less stations next time, so there is a higher change to obtain a feasible solution
            num_removal_current -= 1

        if not unbalanced_stations:
            # if removal neighbor routes are possibly balanced, return them
            return removed_vehicles

        if counter > num_iter:
            break

        if time.time() - start_time > timeout:
            break

    # if there is no candidate, return original
    return problem_instance.vehicles


def multi_remove_and_insert_station(problem_instance, num_removal=1):
    copied_problem_instance = deepcopy(problem_instance)

    for removed_vehicles in remove_multi_stations_generator(copied_problem_instance.vehicles, at_random=True,
                                                            num_removal=num_removal):
        unbalanced_stations = _get_loading_and_unbalanced_stations(copied_problem_instance, removed_vehicles)
        if not unbalanced_stations:
            # if removal neighbor routes are possibly balanced, return them
            return removed_vehicles
        vehicles = insert_nearest_v3(removed_vehicles, unbalanced_stations, copied_problem_instance.model.copy())
        unbalanced_stations = _get_loading_and_unbalanced_stations(copied_problem_instance, vehicles)
        if not unbalanced_stations:
            return vehicles
    # if there is no candidate, return original
    # print("Did not find any feasible routes.")
    return problem_instance.vehicles
