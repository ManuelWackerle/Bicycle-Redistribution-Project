import networkx as nx
import numpy as np
from copy import deepcopy

import utils
from utils import bcolors


class VNS(object):
    """
    Provides the basic version of the algorithms from the paper Balancing Bicycle Sharing Systems:
    A Variable Neighborhood Search Approach

    config format:
    - num_vehicles: the number of vehicles available
    - capacity: positive integer giving the vehicle capacity #TODO (OR array corresponding to each vehicle)
    - verbose: for debugging purposes. 0: execute silently, 1: display warnings only, 2: display in-between steps
    """

    def __init__(self, input_graph: nx.Graph, num_vehicles, vehicle_capacity, verbose=0):
        self.model = input_graph
        self.num_vehicles = num_vehicles
        self.capacity = vehicle_capacity
        self.routes = []  # Array of arrays corresponding to each vehicle and the sequence of vertices visited
        self.instructions = []  # Array of arrays corresponding to each vehicle and the number of bicyles to pick up
        self.distances = []
        self.imbalance = 0
        self.allocated = 0
        self._verbose = verbose

    def show_header(self, header_str):
        if self._verbose > 1:
            print(bcolors.HEADER + header_str + bcolors.ENDC)

    def show_info(self, info_string):
        if self._verbose > 1:
            print(bcolors.OKBLUE + info_string + bcolors.ENDC)

    def show_warning(self, warn_string):
        if self._verbose > 0:
            print(bcolors.WARNING + "Warning: " + warn_string + bcolors.ENDC)

    def mean_distance(self):
        distances = utils.edge_data_as_numpy(self.model, 'dist')
        return distances.mean()

    def greedy_routing_v1(self, budget, source='0'):
        """
        Finds a set of vehicle routes with low cost based on a greedy approach.
        Version 1: most basic

        :return routes: the set vehicle routes
        """

        self.show_header("Searching for routes using basic greedy")
        graph = self.model.copy()
        mean = self.mean_distance()

        # Initialize tracker variables
        stops = 0
        vehicles, distances, loads, space, remove, current = [], [], [], [], [], []
        for l in range(self.num_vehicles):
            vehicles.append(l)
            self.routes.append([source])
            distances.append(0)  # total route distance
            loads.append(0)  # number of bikes on vehicle
            space.append(self.capacity)  # remaining space on vehicle
            current.append(source)

        while len(vehicles) != 0:
            if stops > len(graph.nodes) * 2:
                remove = vehicles
            else:
                for l in vehicles:
                    # while distance < budget and stops < len(graph.nodes):  # within distance budget
                    next, next_score, to_move = None, 0, 0
                    for n, dict in graph.nodes.items():
                        score, move = 0, 0
                        if n != current[l]:
                            dist = graph.edges[current[l], n]['dist']
                            sup = graph.nodes[n]['sup']
                            if sup > 0:
                                move = -min(sup, space[l])
                                score = -(move) * (mean / dist)
                            elif sup < 0:
                                move = min(-sup, loads[l])
                                score = (move) * (mean / dist)
                            if score >= next_score:
                                next, next_score, to_move = n, score, move
                    self.routes[l].append(next)
                    distances[l] += graph.edges[current[l], next]['dist']
                    current[l] = next
                    loads[l] -= to_move
                    space[l] += to_move
                    graph.nodes[next]['sup'] += to_move
                    stops += 1
                    if distances[l] > budget:
                        remove.append(l)
            for l in remove:
                vehicles.remove(l)
                while loads[l] > 0 and stops < 10:
                    next, next_score, to_move = None, 0, 0
                    for n, dict in graph.nodes.items():
                        score, move = 0, 0
                        if n != current[l]:
                            dist = graph.edges[current[l], n]['dist']
                            sup = graph.nodes[n]['sup']
                            if sup < 0:
                                move = min(-sup, loads[l])
                                score = move / dist
                            if score >= next_score:
                                next, next_score, to_move = n, score, move
                    self.routes[l].append(next)
                    distances[l] += graph.edges[current[l], next]['dist']
                    current[l] = next
                    loads[l] -= to_move
                    space[l] += to_move
                    stops += 1
                    graph.nodes[next]['sup'] += to_move
                self.routes[l].append('0')
                distances[l] += graph.edges[current[l], source]['dist']
            remove = []
            self.distances = distances
        routes_str = "Routes:"
        if self._verbose > 1:
            for r in range(len(self.routes)):
                routes_str += "\nvehicle {}: {}, distance = {}".format(r, str(self.routes[r]), distances[r])
        self.show_info(routes_str)
        return self.routes

    def greedy_routing_v2(self):
        """
        Finds a set of vehicle routes with low cost based on a greedy approach.
        Version 1: more advanced greedy search taking load balances into account

        :return routes: the set vehicle routes
        """

        pass  # Todo: implement

    def greedy_routing_v3(self, budget, source='0', DEBUG=False):

        self.show_header("Searching for routes using basic greedy")
        graph = self.model.copy()

        N = len(graph.nodes)  # number of stations (including depot)
        Q = self.capacity  # vehicle capacity (vehicles are homogeneous)
        K = self.num_vehicles  # number of vehicles

        cost_limit = budget  # cost limit for a route of one vehicle
        cost_matrix = utils.compute_cost_matrix(graph)
        if np.any((np.diagonal(cost_matrix))):
            raise ValueError('Cost matrix diagonal should be zero.')


        # 5 means that 5 bicycles should be picked up, and -3 means that 3 bikes should be delivered
        demand_array = utils.compute_demand_array(graph)

        if np.sum(demand_array) != 0:
            raise ValueError('Average demand is not equal to zero.')

        if demand_array[int(source)] != 0:
            raise ValueError('Depot should be balanced.')

        routes = []
        distances = []
        for vehicle in range(K):
            if DEBUG:
                print("Route " + str(vehicle) + " is considered.")

            # initial state for a vehicle is set
            route = []
            current_station = int(source)
            route.append(current_station)
            vehicle_load = 0
            route_cost = 0

            counter = 0

            while True:
                if DEBUG:
                    print(str(counter) + " iteration of " + str(vehicle) + " route.")
                    print("Current route:", route)
                counter += 1
                if DEBUG:
                    print("Demand array:", demand_array)
                bike_delta_array = np.zeros(N)
                gain_ratio = np.zeros(N)
                for target_station in range(N):
                    if DEBUG:
                        print("Target station " + str(target_station) + " with a disbalance of " + str(
                            demand_array[target_station]) + " is considered.")
                    if target_station == current_station:
                        if DEBUG:
                            print("[=] This station is the same as current station.")
                        # cannot choose the next station the same as the current station
                        # gain_ratio is set to zero
                        continue
                    if demand_array[target_station] > 0:  # pickup
                        if DEBUG:
                            print("[=] It is a pickup station.")

                        if (route_cost + cost_matrix[current_station, target_station] +
                                cost_matrix[target_station, 0] > cost_limit):
                            # costs will be exceeded even if visiting only a pickup station
                            # gain_ratio is set to zero
                            # move to the next target station
                            continue

                        demand_array_tmp = deepcopy(demand_array)
                        target_station_tmp = deepcopy(target_station)
                        route_delivery = deepcopy(route)

                        route_delivery_cost = route_cost + cost_matrix[current_station, target_station_tmp]
                        route_delivery.append(target_station_tmp)

                        bike_delta = min(demand_array_tmp[target_station_tmp], Q - vehicle_load)
                        # demand_array_tmp[target_station_tmp] -= bike_delta
                        deliverable_bikes = 0
                        while True:
                            # gain_ratio_delivery is set to zero for every station after each iteration
                            gain_ratio_delivery = np.zeros(N)
                            deliverable_bikes_array = np.zeros(N)

                            for delivery_station in range(N):
                                if delivery_station == target_station_tmp:
                                    # cannot choose the next after next station as the next station
                                    # gain_ratio_delivery is set to zero
                                    continue
                                if demand_array[delivery_station] >= 0:
                                    # balanced stations and pickup stations are not considered
                                    continue
                                else:
                                    # only delivery stations are considered
                                    if (route_delivery_cost +
                                            cost_matrix[target_station_tmp, delivery_station] +
                                            cost_matrix[delivery_station, 0] <= cost_limit):
                                        deliverable_bikes_array[delivery_station] = demand_array[delivery_station]
                                        if deliverable_bikes_array[delivery_station] > 0:
                                            raise ValueError(
                                                'deliverable_bikes_array value cannot be larger than zero.')
                                        gain_ratio_delivery[delivery_station] = deliverable_bikes_array[
                                                                                    delivery_station] / \
                                                                                cost_matrix[
                                                                                    target_station_tmp, delivery_station]

                            if DEBUG:
                                pass
                                # print("Delivery station have the following deliverable bikes:", deliverable_bikes_array)
                                # print("Delivery station have the following gain ratios:", gain_ratio_delivery)

                            if not (np.any(gain_ratio_delivery)):
                                if deliverable_bikes == 0:
                                    # no feasible station that could be visited was found
                                    # break the "while True" cycle for a delivery route
                                    # gain_ratio (not gain_ratio_delivery) is set to zero
                                    break
                                elif deliverable_bikes < 0:
                                    # gain_ratio[target_station] is positive (pickup)
                                    bike_delta_array[target_station] = min(-deliverable_bikes, bike_delta)
                                    gain_ratio[target_station] = bike_delta_array[target_station] / cost_matrix[
                                        current_station, target_station]
                                    break
                                else:
                                    raise ValueError('deliverable_bikes cannot be larger than zero.')
                                # TODO some stuff should be added before break
                            else:
                                next_delivery_station = np.argmax(np.abs(gain_ratio_delivery))

                                # deliverable_bikes has a negative value (because it's a delivery)
                                deliverable_bikes += deliverable_bikes_array[next_delivery_station]

                                route_delivery.append(next_delivery_station)

                                route_delivery_cost += cost_matrix[target_station_tmp, next_delivery_station]

                                # vehicle_load_estimated will be reduced after visiting a delivery station

                                # move to the next station
                                target_station_tmp = next_delivery_station


                    elif demand_array[target_station] < 0:  # delivery
                        if DEBUG:
                            print("[=] It is a delivery station.")
                        if (route_cost + cost_matrix[current_station, target_station] +
                                cost_matrix[target_station, 0] <= cost_limit):
                            bike_delta_array[target_station] = -min(-demand_array[target_station], vehicle_load)

                            # gain_ratio[target_station] is negative (delivery)
                            gain_ratio[target_station] = bike_delta_array[target_station] / cost_matrix[
                                current_station, target_station]
                    else:
                        if DEBUG:
                            print("[=] This station is already balanced.")
                        # the station doesn't require a visit, as it is already balanced
                        # gain_ratio is set to zero
                        continue

                if DEBUG:
                    pass
                    # print("Target station have the following bike delta:", bike_delta_array)
                    # print("Target station have the following gain rations:", gain_ratio)

                if not (np.any(gain_ratio)):
                    # no feasible station that could be visited was found
                    # break the "while True" cycle for a route
                    if route[-1] != 0:
                        # if a vehicle is not currently at the depot, it should return
                        route.append(0)
                        route_cost += cost_matrix[current_station, 0]
                    if vehicle_load != 0:
                        print('Vehicle load should be zero when the route is finished.')
                        print('Current vehicle load:', vehicle_load)
                        # raise ValueError('Vehicle load should be zero when the route is finished.')
                        # TODO For some reason, it is possible that load at the end of the route is not zero.
                    break

                else:
                    # choose a station with the highest gain ratio (a sign doesn't matter)
                    next_station = np.argmax(np.abs(gain_ratio))
                    route.append(next_station)
                    route_cost += cost_matrix[current_station, next_station]
                    if DEBUG:
                        print("Station " + str(next_station) + " is appended to the route " + str(vehicle) + ".")

                    vehicle_load += bike_delta_array[next_station]

                    before_demand = demand_array[next_station]
                    demand_array[next_station] -= bike_delta_array[next_station]
                    after_demand = demand_array[next_station]

                    if after_demand * before_demand < 0:
                        raise ValueError('Sign of a station demand was changed after a trip.')

                    if vehicle_load > Q:
                        raise ValueError('Vehicle load is exceeded.')

                    if vehicle_load < 0:
                        raise ValueError('Vehicle load is negative.')

                    current_station = next_station
                    # TODO add vehicle load
                counter += 1

                if np.sum(demand_array) + vehicle_load != 0:
                    pass
                    # print(np.sum(demand_array))
                    # print(vehicle_load)
                    # raise ValueError('The number of bikes changed during the computation.')
            route = list(map(str, route))
            distances.append(route_cost)
            routes.append(route)
            print("Route #", vehicle, ": ", route)

        if np.any(demand_array):
            print("Balance is not reached. Stations have the following demand:")
            print(demand_array)
        else:
            print("Balance is reached.")

        self.distances = distances
        self.routes = routes

        return self.routes


    def greedy_routing_PILOT(self):
        """
        Finds a set of vehicle routes with low cost based on a greedy approach.
        Version 1: greedy search using the PILOT technique

        :return routes: the set vehicle routes
        """

        pass  # Todo: implement

    def calculate_loading_MF(self):
        """
        Given a set of vehicle routes, calculates optimal loading instructions for each route using a Maximum flow computation.
        Use this function if mononicity is assumed.

        :return instructions: The instructions for how to load and unload the bicycles.
        """

        # Generate Max Flow graph
        self.show_header("Generating Max flow graph")
        total_source, total_sink = 0, 0
        mf_graph = nx.DiGraph()
        mf_graph.add_node('s')  # source node
        mf_graph.add_node('t')  # sink node
        for v, d in self.model.nodes(data='sup'):
            if d > 0:
                total_source += d
                mf_graph.add_edge('s', v, capacity=d)
            elif d < 0:
                total_sink -= d
                mf_graph.add_edge(v, 't', capacity=-d)
        for p in range(len(self.routes)):
            path = self.routes[p]
            prev_node = 0
            for r in range(len(path)):
                node = path[r]
                node_str = "{}-{}-{}".format(node, p, r)
                demand = self.model.nodes[node]['sup']
                if demand > 0:
                    mf_graph.add_edge(node, node_str)
                elif demand < 0:
                    mf_graph.add_edge(node_str, node)
                if prev_node != 0:
                    mf_graph.add_edge(prev_node, node_str, capacity=self.capacity)
                prev_node = node_str
        self.show_info("Graph generated with {n} nodes and {e} edges. Source flow: {s}, Sink flow: {t}"
                       .format(n=len(mf_graph.nodes), e=len(mf_graph.edges), s=total_source, t=total_sink))

        # Sovle Max Flow Problem
        self.show_header("Solving the Max flow problem ")
        self.imbalance = total_source
        if total_sink != total_source:
            self.show_warning("mismatch in source and sink flow capacity, no exact solution can exist.")
            self.imbalance = -1

        # This is where the magic happens
        value, dict = nx.maximum_flow(mf_graph, 's',
                                      't')  # , flow_func=nx.algorithms.flow.shortest_augmenting_path) #TODO: investigate this algorithm exactly and see if it can be done better
        self.allocated = value

        if value != total_source or value != total_sink:
            self.show_warning(
                "Bikes can not be allocated to full capacity. Source flow: {s}, Sink flow: {t}, Allocated: {a}"
                .format(s=total_source, t=total_sink, a=value))
        else:
            self.show_info("Bike allocation is exact. Total allocated bicycles: {}".format(value))

        self.show_header("Generating instructions")
        self.instructions = []
        for p in range(len(self.routes)):
            self.instructions.append([])
            path = self.routes[p]
            for r in range(len(path) - 1):
                node = path[r]
                next = path[r + 1]
                node_str = "{}-{}-{}".format(node, p, r)
                next_str = "{}-{}-{}".format(next, p, r + 1)
                self.instructions[p].append(dict[node_str][next_str])

        routes_str = "Routes with loading instructions:"
        if self._verbose > 1:
            for r in range(len(self.routes)):
                routes_str += "\nvehicle {}: load = {}" \
                    .format(r, self.instructions[r])
        self.show_info(routes_str)

    def calculate_loading_LP(self, w):
        """
        Given a set of vehicle routes, calculates optimal loading instructions for each route using a Linear Program computation.
        Use this function for the general case in which monotonicity is not assumed.

        :param w: does nothing
        :return instructions: The instructions for how to load and unload the bicycles
        """
        pass

    def set_routes(self, routes, instr=None):
        success = True
        if len(routes) != self.num_vehicles:
            self.show_warning("Set routes failed. Number of routes needs to match number of vehicles ({})"
                              .format(self.num_vehicles))
            success = False
        else:
            for l in range(len(routes)):
                route = routes[l]
                stops = len(route)
                if stops < 3:
                    self.show_warning("Set routes failed. Routes need to contain at least 3 stops")
                    success = False
                    break
        if success:
            self.routes = routes
            self.recalculate_distance()
            if instr is not None:
                if len(instr) != len(routes):
                    self.show_warning("instr need to be an array of the same size as routes")
                else:
                    self.instructions = instr

    def remove_unused_stops(self):
        """
        Given a set of vehicle routes, simply removes all the stops where the vehicle neither load nor unloads any bikes.
        """
        for l in range(len(self.instructions)):
            remove = []
            node = None
            distance = 0
            prev_load = 0
            for s in range(1, len(self.instructions[l])):
                node = self.routes[l][s]
                load = self.instructions[l][s]
                if load == prev_load:
                    remove.append(s)
                prev_load = load
            remove.reverse()
            for r in remove:
                del self.instructions[l][r]
                del self.routes[l][r]
        self.recalculate_distance()

    def recalculate_distance(self):
        self.distances = []
        for l in range(len(self.routes)):
            route = self.routes[l]
            dist = 0
            prev = route[0]
            for s in range(1, len(route)):
                if prev == route[s]:
                    self.show_warning("same route twice in sequence - might be a mistake")
                else:
                    dist += self.model.edges[prev, route[s]]['dist']
                prev = route[s]
            self.distances.append(dist)

    def display_results(self, show_instructions=True):
        """
        Displays the information in self.routes and self.instructions in a human readable way
        """
        results = "Results\t\t"
        if show_instructions:
            results += "total distance || instructions   <station>: <load/unload bikes> (<total on vehicle>)"
            for l in range(len(self.routes)):
                line = "\nVehicle #{:<3} {:>11}km |".format(l, self.distances[l] / 1000)
                prev_load = 0
                for s in range(len(self.instructions[l])):
                    load = self.instructions[l][s]
                    diff = load - prev_load
                    prev_load = load
                    instr = 'l' if diff >= 0 else 'u'
                    # a, b = self.routes[l][s], self.routes[l][s+1]
                    # dist = self.model.edges[a, b]['dist'] if a != b else 0
                    line += "|{:>4}: ".format(self.routes[l][s]) + instr + "{:<2}({:2})".format(abs(diff), load)
                results += line + "|   0: u{:<2}( 0) |".format(0)
        d = sum(self.distances) / 1000
        success = bcolors.OKGREEN if self.allocated == self.imbalance else bcolors.FAIL
        results += bcolors.BOLD + bcolors.OKGREEN + "\nTotal Distance:{:9}km".format(d) + bcolors.ENDC + " ||  "
        results += success + bcolors.BOLD + " Total Rebalanced: {}/{}".format(self.allocated,
                                                                              self.imbalance) + bcolors.ENDC
        print(results)

    def reset(self):
        self.routes = []
        self.instructions = []
        self.distances = []
        self.imbalance = 0
        self.allocated = 0
