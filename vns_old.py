import networkx as nx
import math
from haversine import haversine, Unit
import numpy as np
from copy import deepcopy
import utils
from utils import bcolors
import time


class VNS(object):
    """
    Provides the basic version of the algorithms from the paper Balancing Bicycle Sharing Systems:
    A Variable Neighborhood Search Approach

    config format:
    - num_vehicles: the number of vehicles available
    - capacity: positive integer giving the vehicle capacity #TODO (OR array corresponding to each vehicle)
    - verbose: for debugging purposes. 0: execute silently, 1: display warnings only, 2: display in-between steps
    """

    def __init__(self, input_graph: nx.Graph, num_vehicles, vehicle_capacity, node_data=None, verbose=0):
        self._verbose = verbose

        # input variables
        self.model = input_graph
        self.node_data = node_data
        self.num_vehicles = num_vehicles
        self.capacity = vehicle_capacity

        # tracking variables
        self.total_source = 0
        self.total_sink = 0
        self.imbalance = 0
        self.allocated = 0
        self._initialize_tracking_variables(input_graph)
        self.mf_graph = None

        # result variables
        self.routes = []
        self.instructions = []
        self.distances = []

        # G.
        self.neighbourhoods = []  # keys of neighbourhoods to be searched.
        self.nh_dict = {"remove_station": self.remove_one_station}  # functions associated to neighbourhoods
        self.current_nh = 0

    def show_header(self, header_str):
        if self._verbose > 1:
            print(bcolors.HEADER + header_str + bcolors.ENDC)

    def show_info(self, info_string):
        if self._verbose > 1:
            print(bcolors.OKBLUE + info_string + bcolors.ENDC)

    def show_warning(self, warn_string):
        if self._verbose > 0:
            print(bcolors.WARNING + "Warning: " + warn_string + bcolors.ENDC)

    def _initialize_tracking_variables(self, input_graph: nx.Graph):
        """
        Loops once through all the input data to collect additional information about the graph.
        Setting
        """
        for n, data in input_graph.nodes.items():
            d = data['sup']
            if d > 0:
                self.total_source += d
            elif d < 0:
                self.total_sink -= d
        if self.total_sink == self.total_source:
            self.imbalance = self.total_source
        else:
            pass
            # TODO: handle case where sink & source don't match, e.g. by adding an additional node to the graph or changing depot value

    def mean_distance(self):
        distances = utils.edge_data_as_numpy(self.model, 'dist')
        return distances.mean()

    def centre_node(self):
        """
        Finds the node nearest to the centre of the graph (the mean over all node positions)

        :return centre_node: the central node of the graph
        """
        positions = utils.dict_data_as_numpy(self.node_data, 'pos')
        centre = positions.mean(axis=0)
        best, centre_node = math.inf, '0'
        for n, data in self.node_data.items():
            dist = int(round(haversine(centre, data['pos'], unit=Unit.METERS)))
            if dist < best:
                best = dist
                centre_node = n
        return centre_node

    def furthest_nodes(self, number=0):
        """
        Given a source node returns a list with <num.vehicles> nodes that are far away from the source node and each other

        :return furthest_nodes: array of nodes from self.model
        """

        centre = self.centre_node()
        furthest_nodes, best = [], '0'
        number = self.num_vehicles if number == 0 else number
        for l in range(number):
            furthest = 0
            for n in self.model.nodes:
                sup = self.model.nodes[n]['sup']
                if sup > 0 and n != centre and n not in furthest_nodes:
                    dist = self.model.edges[centre, n]['dist'] ** 2
                    for s in furthest_nodes:
                        dist += self.model.edges[s, n]['dist'] ** 2
                    if dist > furthest:
                        furthest = dist
                        best = n
            furthest_nodes.append(best)
        return furthest_nodes

    def recenter_routes(self, source='0'):
        """
        Given a set of routes with arbitrary, different source nodes inserts the new source node if necessary
        and premutes the routes to start at the new source

        :return loads: the starting loads necessary at the new source node in order for the solution to remain feasible
        """

        loads = []
        for l in range(self.num_vehicles):
            route = self.routes[l]
            loads.append(0)
            split = 0
            if source not in route:
                best, load = math.inf, 0
                for s in range(len(route) - 1):
                    dist = self.model.edges[source, route[s]]['dist'] + self.model.edges[source, route[s + 1]]['dist']
                    if dist < best:
                        best = dist
                        split = s
                        loads[l] = self.instructions[l][s]
                left = route[1:split]
                right = route[split:]
                self.routes[l] = [source] + right + left + [source]
            else:
                split = route.index(source)
                loads[l] = self.instructions[l][split]
                left = route[1:split]
                right = route[split:]
                self.routes[l] = right + left + [source]
        self.recalculate_distance()
        return loads

    def greedy_routing_v1(self, source='0', varied_starts=None, a=3):

        """
        Finds a set of vehicle routes with low cost based on a greedy approach.
        Version 1: most basic

        :return routes: the set vehicle routes
        """

        self.show_header("Searching for routes using basic greedy")
        graph = self.model.copy()
        mean = self.mean_distance()
        varied = varied_starts is not None

        # Initialize tracker variables
        stops = 0

        vehicles, distances, loads, space, remove, current = [], [], [], [], [], []
        self.instructions = []
        for l in range(self.num_vehicles):
            vehicles.append(l)
            distances.append(0)  # total route distance
            if varied:
                start = varied_starts[l]
                self.routes.append([start])
                current.append(varied_starts[l])
                loads.append(min(self.model.nodes[start]['sup'], self.capacity))
                self.instructions.append([loads[l]])
                space.append(self.capacity - loads[l])
                graph.nodes[start]['sup'] -= loads[l]
            else:
                self.routes.append([source])
                current.append(source)
                loads.append(0)  # number of bikes on vehicle
                space.append(self.capacity)  # remaining space on vehicle
                self.instructions.append([loads[l]])

        if self.imbalance == 0:
            self.show_warning("bicycle imbalance is zero, greedy search has noting to do.")

        while self.allocated < self.imbalance:
            for l in vehicles:
                next, next_score, to_move = None, 0, 0
                curr = current[l]
                for n in graph.nodes:
                    score, move = 0, 0
                    if n != curr:
                        dist = graph.edges[curr, n]['dist']
                        sup = graph.nodes[n]['sup']
                        if sup > 0:
                            move = -min(sup, space[l])
                            score = -move * (mean / dist) ** a
                        elif sup < 0:
                            move = min(-sup, loads[l])
                            score = move * (mean / dist) ** a
                        if score >= next_score:
                            next, next_score, to_move = n, score, move
                if to_move != 0:
                    self.routes[l].append(next)
                    distances[l] += graph.edges[current[l], next]['dist']
                    current[l] = next
                    loads[l] -= to_move
                    space[l] += to_move
                    self.instructions[l].append(loads[l])
                    graph.nodes[next]['sup'] += to_move
                if to_move > 0:
                    self.allocated += to_move
        for l in vehicles:
            if varied and current[l] != varied_starts[l]:
                self.routes[l].append(varied_starts[l])
                distances[l] += graph.edges[current[l], varied_starts[l]]['dist']
            elif not varied and current[l] != source:
                self.routes[l].append(source)
                distances[l] += graph.edges[current[l], source]['dist']
        self.distances = distances
        routes_str = "Routes:"
        if self._verbose > 1:
            for r in range(len(self.routes)):
                routes_str += "\nvehicle {}: {}, distance = {}".format(r, str(self.routes[r]), distances[r])
        self.show_info(routes_str)
        return self.routes

    def greedy_routing_v2(self, source='0', a=3):
        """
        Finds a set of vehicle routes with low cost based on a greedy approach.
        Version 1: more advanced greedy search taking load balances into account

        :return routes: the set vehicle routes
        """
        self.show_header("Searching for routes using tsp iteration")
        graph = self.model.copy()
        guided = nx.algorithms.approximation.christofides(graph, weight='dist')
        mean = self.mean_distance()

        # Initialize tracker variables
        vehicles, loads, space, remove, current = [], [], [], [], []
        self.instructions = []
        successors = {}
        for l in range(self.num_vehicles):
            vehicles.append(l)
            self.distances.append(0)  # total route distance
            self.routes.append([source])
            current.append(source)
            loads.append(0)  # number of bikes on vehicle
            space.append(self.capacity)  # remaining space on vehicle
            self.instructions.append([loads[l]])

        for i in range(1, len(guided) - 2):
            s, n = guided[i], guided[i + 1]
            if s in successors:
                successors[s].add(n)
            else:
                successors[s] = set(n)
            # if n in successors:
            #     successors[n].add(s)
            # else:
            #     successors[n] = set(s)

        if self.imbalance == 0:
            self.show_warning("bicycle imbalance is zero, greedy search has noting to do.")
        # source_sup = graph.nodes[source]['sup']
        # if source_sup > 0:
        #     for l in vehicles:
        #         source_sup = graph.nodes[source]['sup']
        #         move = min(source_sup, self.capacity)
        #         loads[l] += move
        #         space[l] -= move
        #         self.instructions[l][0] = move
        #         graph.nodes[source]['sup'] -= move
        while self.allocated < self.imbalance:
            for l in vehicles:
                next, next_score, to_move = None, 0, 0
                curr = current[l]
                for n in graph.nodes:
                    score, move = 0, 0
                    if n != curr:
                        dist = graph.edges[curr, n]['dist']
                        sup = graph.nodes[n]['sup']
                        if sup > 0:
                            move = -min(sup, space[l])
                            score = -move * (mean / dist) ** a
                        elif sup < 0:
                            move = min(-sup, loads[l])
                            score = move * (mean / dist) ** a
                        if curr in successors and n in successors[curr]:
                            score *= 5
                        if score >= next_score:
                            next, next_score, to_move = n, score, move
                if to_move != 0:
                    self.routes[l].append(next)
                    self.distances[l] += graph.edges[curr, next]['dist']
                    current[l] = next
                    loads[l] -= to_move
                    space[l] += to_move
                    self.instructions[l].append(loads[l])
                    graph.nodes[next]['sup'] += to_move
                if to_move > 0:
                    self.allocated += to_move

        for l in vehicles:
            self.routes[l].append(source)
            self.distances[l] += graph.edges[current[l], source]['dist']

        routes_str = "Routes:"
        if self._verbose > 1:
            for r in range(len(self.routes)):
                routes_str += "\nvehicle {}: {}, distance = {}".format(r, str(self.routes[r]), self.distances[r])
        self.show_info(routes_str)
        return self.routes

    def tsp_segmenting_routing(self, source='0'):
        """
        Finds a set of vehicle routes with low cost based on segmenting a tsp solution

        :return routes: the set vehicle routes
        """

        self.show_header("Searching for routes using tsp segmenting")
        graph = self.model.copy()
        path = nx.algorithms.approximation.christofides(graph, weight='dist')
        # print(path)
        path_len = len(path) - 1
        s_idx = path.index(source)
        e_idx = (s_idx - 1) % path_len
        segments = {}
        node_str = "{}-{}"
        load, into = 0, 1
        last_idx = s_idx
        while s_idx != e_idx:
            load += self.model.nodes[path[s_idx]]['sup']
            while load < 0 or self.capacity < load:
                out = 0 if load < 0 else 1
                segments[node_str.format(path[last_idx], path[s_idx])] = [into, out, path[last_idx:s_idx + 1]]
                last_idx = s_idx
                into = out
                load = load - self.capacity if out else load + self.capacity
            s_idx = (s_idx + 1) % path_len
        segments[node_str.format(path[last_idx], path[e_idx])] = [into, 0, path[last_idx:e_idx + 1]]

        tsp_graph = nx.DiGraph()
        for node in segments.keys():
            tsp_graph.add_node(node)
        for j, dir_j in segments.items():
            s_j, e_j = j.split('-')
            for k, dir_k in segments.items():
                if j != k:
                    s_k, e_k = k.split('-')
                    if dir_j[1] ^ dir_k[0]:
                        dist = self.model.edges[e_j, s_k]['dist']
                        tsp_graph.add_edge(j, k, dist=dist)
                    else:
                        tsp_graph.add_edge(j, k, dist=500000)
                    if dir_j[0] ^ dir_k[1]:
                        dist = self.model.edges[e_k, s_j]['dist']
                        tsp_graph.add_edge(k, j, dist=dist)
                    else:
                        tsp_graph.add_edge(k, j, dist=500000)

        seq = nx.algorithms.approximation.greedy_tsp(tsp_graph, weight='dist')
        self.routes = [[]]
        dist = 0
        for s in range(len(seq) - 1):
            dist += tsp_graph.edges[seq[s], seq[s + 1]]['dist']
            self.routes[0] += segments[seq[s]][2]
            # print(segments[seq[s]][2])
        # print(dist)
        self.routes[0] += source
        self.recalculate_distance()

    def tsp_rerouting(self):
        for l in range(self.num_vehicles):
            subgraph = self.model.subgraph(self.routes[l])
            graph = subgraph.to_undirected()
            new_seq = nx.algorithms.approximation.christofides(graph, weight='dist')
            new_seq.pop()
            split = new_seq.index('0')
            left = new_seq[split:]
            right = new_seq[:split]
            new = left + right + ['0']
            # print("New sequence calculated\nold: {}\nnew: {}".format(self.routes[l], new))
            self.routes[l] = new
        self.recalculate_distance()

    def greedy_routing_v3(self, budget, source='0', DEBUG=False):

        self.show_header("Searching for routes using advanced greedy.")
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
                    print("Demand array:", demand_array)
                    print("Vehicle load:", vehicle_load)
                bike_delta_array = np.zeros(N)  # load and delivery instructions for all target stations
                gain_ratio = np.zeros(N)  # gain ratio for all target stations
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
                                cost_matrix[target_station, int(source)] > cost_limit):
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
                        deliverable_bikes = 0  # the number of bikes that can be delivered after a pickup is done

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
                                                                                    delivery_station] / cost_matrix[
                                                                                    target_station_tmp, delivery_station]

                            if not (np.any(gain_ratio_delivery)):
                                # no feasible delivery station was found
                                # delivery root search ends here
                                if DEBUG:
                                    print("Delivery route:", route_delivery)
                                if deliverable_bikes == 0:
                                    # no feasible station that could be visited was found
                                    # break the "while True" cycle for a delivery route
                                    # gain_ratio (not gain_ratio_delivery) is set to zero
                                    break
                                elif deliverable_bikes < 0:
                                    # gain_ratio[target_station] is positive (pickup)
                                    bike_delta_array[target_station] = min(-deliverable_bikes - vehicle_load,
                                                                           bike_delta)
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
                                cost_matrix[target_station, int(source)] <= cost_limit):
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
                    if route[-1] != int(source):
                        # if a vehicle is not currently at the depot, it should return
                        route.append(int(source))
                        route_cost += cost_matrix[current_station, int(source)]
                    if vehicle_load != 0:
                        print('Vehicle load should be zero when the route is finished.')
                        print('Current vehicle load:', vehicle_load)
                        raise ValueError('Vehicle load should be zero when the route is finished.')
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

    def remove_one_station(self, patience):
        """return remove station neighbor matrix whose each row corresponds to removing one station from the original route

        Write
            N = Number of trucks (=len(self.routes))
            Cn = Number of candidate routes (=len(self.routes[n]))
            Ln = Route length (=len(self.routes[n])-1)
        Note
            Cn, Ln depends on n in N. (candidate routes and route length might differ for each truck)
        :return
            (N, Cn, Ln) matrix (python list)
        """
        retval = []
        p = 0
        for i in range(len(self.routes)):
            if self.routes[i] == 1:
                continue
            for j in range(1, len(self.routes[i]) - 1):
                candidate = deepcopy(self.routes)
                candidate[i] = candidate[i][:j] + candidate[i][j + 1:]
                retval.append(candidate)
                p += 1
                if p > patience:
                    break
        return retval

    def _get_rebalanced_graph(self):
        """given routes and instructions, returns graph after rebalance
        """
        graph = self.model.copy()
        for l in range(len(self.routes)):
            prev_load = 0
            for s in range(len(self.instructions[l])):
                load = self.instructions[l][s]
                diff = prev_load - load
                prev_load = load
                graph.nodes[self.routes[l][s]]['sup'] += diff
        return graph

    def insert_unbalanced_station(self, patience):
        """adding unbalanced station is considered as the neighbor

        Write
            N = Number of trucks (=len(self.routes))
            C = Number of candidate routes (=len(self.routes[n])*len(unbalanced_stations))
            Ln = Route length (=len(self.routes[n])+1)
        Note
            Cn, Ln depends on n in N. (candidate routes and route length might differ for each truck)
        :return
            (C, N, Ln) matrix (python list)
        """
        graph = self._get_rebalanced_graph()
        unbalanced_stations = [x for x in graph.nodes if graph.nodes[x]['sup'] != 0]
        retval = []
        p = 0
        for i in range(len(self.routes)):
            for j in range(1, len(self.routes[i]) - 1):
                for u in unbalanced_stations:
                    candidate = deepcopy(self.routes)
                    candidate[i] = candidate[i][:j] + [u] + candidate[i][j:]
                    retval.append(candidate)
                    p += 1
                    if p > patience:
                        break
        return retval

    def calculate_loading_MF(self, start_load=0, source='0'):
        """
        Given a set of vehicle routes, calculates optimal loading instructions for each route using a Maximum flow computation.
        Use this function if mononicity is assumed.

        :return instructions: The instructions for how to load and unload the bicycles.
        """

        # Generate Max Flow graph
        self.show_header("Generating Max flow graph")
        total_source, total_sink = 0, start_load
        mf_graph = nx.DiGraph()
        mf_graph.add_node('s')  # source node
        mf_graph.add_node('t')  # sink node
        for v, d in self.model.nodes(data='sup'):
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

        for p in range(len(self.routes)):
            path = self.routes[p]
            prev_node = 0
            for r in range(len(path)):
                node = path[r]
                node_str = "{}-{}-{}".format(node, p, r)
                demand = self.model.nodes[node]['sup']
                # if node == source:
                #     mf_graph.add_edge(node, node_str)
                #     mf_graph.add_edge(node_str, node)
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
        self.imbalance = total_source - start_load
        if total_sink != total_source:
            self.show_warning("mismatch in source and sink flow capacity, no exact solution can exist.")
            self.imbalance = -1

        # This is where the magic happens
        value, data = nx.maximum_flow(mf_graph, 's', 't')  # , flow_func=nx.algorithms.flow.shortest_augmenting_path) #TODO: investigate this algorithm exactly and see if it can be done better
        self.allocated = value - start_load

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
                self.instructions[p].append(data[node_str][next_str])

        routes_str = "Routes with loading instructions:"
        if self._verbose > 1:
            for r in range(len(self.routes)):
                routes_str += "\nvehicle {}: load = {}" \
                    .format(r, self.instructions[r])
        self.show_info(routes_str)

    def verify_loading_setup(self, source='0'):
        # Generate Max Flow graph
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
        self.imbalance = total_source

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
        self.mf_graph = mf_graph

    def verify_loading_on_route(self, l, b1, b2, tolerance=0, undo=False):
        ri, rj = self.routes[l][b1], self.routes[l][b1 + 1]
        rk, rl = self.routes[l][b2], self.routes[l][b2 + 1]
        ri_str, rj_str = "{}-{}-{}".format(ri, l, b1), "{}-{}-{}".format(rj, l, b1 + 1)
        rk_str, rl_str = "{}-{}-{}".format(rk, l, b2), "{}-{}-{}".format(rl, l, b2 + 1)
        if undo:
            self.mf_graph.remove_edge(ri_str, rk_str)
            self.mf_graph.remove_edge(rj_str, rl_str)
            self.mf_graph.add_edge(ri_str, rj_str, capacity=self.capacity)
            self.mf_graph.add_edge(rk_str, rl_str, capacity=self.capacity)
        else:
            self.mf_graph.remove_edge(ri_str, rj_str)
            self.mf_graph.remove_edge(rk_str, rl_str)
            self.mf_graph.add_edge(ri_str, rk_str, capacity=self.capacity)
            self.mf_graph.add_edge(rj_str, rl_str, capacity=self.capacity)

        path = self.routes[l]
        prev_node = "{}-{}-{}".format(path[b1 + 1], l, b1 + 1)
        for m in range(b1 + 2, b2 + 1):
            node = path[m]
            node_str = "{}-{}-{}".format(node, l, m)
            if undo:
                self.mf_graph.remove_edge(node_str, prev_node)
                self.mf_graph.add_edge(prev_node, node_str, capacity=self.capacity)
            else:
                self.mf_graph.remove_edge(prev_node, node_str)
                self.mf_graph.add_edge(node_str, prev_node, capacity=self.capacity)
            prev_node = node_str

        if not undo:
            # Sovle Max Flow Problem
            value, data = nx.maximum_flow(self.mf_graph, 's', 't') #TODO: investigate this algorithm exactly and see if it can be done better
            self.allocated = value
            if self.allocated < self.imbalance - tolerance:
                self.verify_loading_on_route(l, b1, b2, tolerance, True)


    def calculate_loading_LP(self, w):
        """
        Given a set of vehicle routes, calculates optimal loading instructions for each route using a Linear Program computation.
        Use this function for the general case in which monotonicity is not assumed.

        :param w: does nothing
        :return instructions: The instructions for how to load and unload the bicycles
        """
        pass

    def two_opt(self, routes):
        swaps = []
        for l in range(len(routes)):
            swaps.append([])
            route = routes[l]
            b1, b2 = 0, 0
            for s1 in range(1, len(route) - 4):
                ri, rj = route[s1], route[s1 + 1]
                best, value, new_value = 0, 0, 0
                for s2 in range(s1 + 2, len(route) - 2):
                    rk, rl = route[s2], route[s2 + 1]
                    if ri != rk and ri != rl and rj != rk and rj != rl:
                        # a = self.model.edges[ri, rj]['dist']
                        # b = self.model.edges[rk, rl]['dist']
                        # c = self.model.edges[ri, rk]['dist']
                        # d = self.model.edges[rl, rj]['dist']
                        # e = self.model.edges[ri, rl]['dist']
                        # f = self.model.edges[rk, rj]['dist']
                        value = self.model.edges[ri, rj]['dist'] + self.model.edges[rk, rl]['dist']
                        new_value = self.model.edges[ri, rk]['dist'] + self.model.edges[rl, rj]['dist']
                        diff = value - new_value
                        if diff > best:
                            best = diff
                            b1, b2 = s1, s2

                if best > 0:
                    swaps[l].append([best, b1, b2])
            # if b1 is not None:
            #     new_routes[l] = route[:b1+1] + route[b2:b1:-1] + route[b2+1:]
            # else:
            #     self.show_warning("no 2-opt improvement found for vehicle #{}".format(l))
            #     new_routes[l] = routes[l]
        return swaps

    def perform_valid_swaps(self, swaps, tolerance=0):
        for l in range(self.num_vehicles):
            # print(swaps)
            swaps[l].sort(reverse=True)
            mn, mx = len(self.routes[l]) + 1, -1
            route = deepcopy(self.routes[l])
            for s in range(len(swaps[l])):
                _, b1, b2 = swaps[l][s]
                if b2 < mn - 1 or mx + 1 < b1:
                    self.verify_loading_on_route(l, b1, b2, tolerance)
                    if self.allocated >= self.imbalance - tolerance:
                        self.routes[l] = route[:b1 + 1] + route[b2:b1:-1] + route[b2 + 1:]
                        route = deepcopy(self.routes[l])
                        mn, mx = min(b1, mn), max(b2, mx)
                    else:
                        self.routes[l] = route
        self.recalculate_distance()

    def node_swap(self):
        for l1 in range(self.num_vehicles):
            path1 = self.routes[l1]
            best, value, new_value = 0, 0, 0
            sj, sk, sl2 = 0, 0, 0
            for j in range(1, len(path1)-1):
                pre1, out, post1 = path1[j-1], path1[j], path1[j+1]
                for l2 in range(self.num_vehicles):
                    if l1 != l2:
                        path2 = self.routes[l2]
                        for k in range(1, len(path2)):
                            pre2, post2 = path2[k - 1], path2[k]
                            if pre1 != post1 and pre2 != out and out != post2:
                                value = self.model.edges[pre1, out]['dist']
                                value += self.model.edges[out, post1]['dist']
                                value += self.model.edges[pre1, post1]['dist']
                                new_value = self.model.edges[pre2, out]['dist'] + self.model.edges[out, post2]['dist'] - self.model.edges[pre2, post2]['dist']
                                diff = value - new_value
                                if diff > best:
                                    best = diff
                                    sl2 = l2
                                    sj = j
                                    sk = k
            self.routes[l1] = path1[:sj] + path1[sj+1:]
            self.routes[sl2] = self.routes[sl2][:sk] + [path1[sj]] + self.routes[sl2][sk:]
        self.recalculate_distance()


    def recalculate_distance(self, only_subroute=-1):
        for l in range(len(self.routes)):
            if only_subroute == -1 or only_subroute == l:
                route = self.routes[l]
                dist = 0
                prev = route[0]
                for s in range(1, len(route)):
                    if prev == route[s]:
                        self.show_warning("same route twice in sequence - might be a mistake")
                    else:
                        dist += self.model.edges[prev, route[s]]['dist']
                    prev = route[s]
                self.distances[l] = dist

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
            self.distances = [0] * self.num_vehicles
            self.recalculate_distance()
            if instr is not None:
                if len(instr) != len(routes):
                    self.show_warning("instr need to be an array of the same size as routes")
                else:
                    self.instructions = instr

    def remove_unused_stops(self):
        """
        Given a set of vehicle routes, removes all the stops where the vehicle neither load nor unloads any bikes.
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

    def display_results(self, show_instructions=True):
        """
        Displays the information in self.routes and self.instructions in a human readable way
        """
        results = "Results\t\t"
        if show_instructions:
            results += "total distance || instructions   <station>: <load/unload bikes> (<total on vehicle>)"
            for l in range(len(self.routes)):
                line = "\nVehicle #{:<3} {:>11}km |".format(l, self.distances[l] / 1000)
                prev_load, last = 0, 0
                for s in range(len(self.instructions[l])):
                    load = self.instructions[l][s]
                    diff = load - prev_load
                    prev_load = load
                    instr = 'l' if diff >= 0 else 'u'
                    # a, b = self.routes[l][s], self.routes[l][s+1]
                    # dist = self.model.edges[a, b]['dist'] if a != b else 0
                    line += "|{:>3}: ".format(self.routes[l][s]) + instr + "{:<2}({:2})".format(abs(diff), load)
                    last = s
                results += line + "|{:>3}: ".format(self.routes[l][last + 1]) + "u{:<2}( 0)|".format(prev_load)
            results += "\n"
        d = sum(self.distances) / 1000
        success = bcolors.OKGREEN if self.allocated == self.imbalance else bcolors.FAIL
        results += bcolors.BOLD + bcolors.OKGREEN + "Total Distance:{:9}km".format(d) + bcolors.ENDC + " ||  "
        results += success + bcolors.BOLD + " Total Rebalanced: {}/{}".format(self.allocated,
                                                                              self.imbalance) + bcolors.ENDC
        print(results)

    def reset(self):
        self.routes = []
        self.instructions = []
        self.distances = []
        self.allocated = 0

    """
    Implementation of the Variable Neighbourhood Search for the Vehicle routing problem.
    Basic idea:
    0 Initialization:
        0.1 Generate initial feasible solution and compute the objective value.
        0.2 Set solution as current best.
        0.3 Choose list of neighbourhoods to explore in a specific order.
            Ns for shaking, Nk for local search.

    Set s = 1
        1 Shaking:
            1.1 Randomly create a solution in the sth neighbourhood of current best.
            1.2 Set k = 1
                    1.2.1 Explore the kth NH.

    set_neighbourhoods. Sets the neighbourhoods to be used.
    """

    def set_neighbourhoods(self, neighbourhood_keys):
        self.neighbourhoods = neighbourhood_keys
        self.current_nh = 0

    """
    _shake : Generates a solution inside the given neighbourhood.
    INPUT
        nh: the neighbourhood in which we want to produce a new route as a function that takes a route and return the shake.

    OUTPUT
        return: Set of routes produced from the original route that lies in the neighbourhood specified by _nh_type
    """

    def _shake(self, nh):

        routes_in_neighbourhood = self.nh_dict[self.neighbourhoods[nh]](patience=5)

        return routes_in_neighbourhood

    """
    _best_improvement: Local search in neighbourhood. Create a copy with modified routes and compare maximum flow
    INPUT
        routes_in_nh: list of all routes to be searched
    OUTPUT
        solution_best_local: best feasible solution found inside the neighbourhood
    """

    def _best_improvement(self, routes_in_nh):
        print("Trying to improve\n")
        index_best = 0
        # TODO: Need to create a copy of vns with the modified route to compare. How?
        instance_2_explore = VNS(self.model, self.num_vehicles, self.capacity, self._verbose)

        for index_current in range(len(routes_in_nh)):
            instance_2_explore.routes = routes_in_nh[index_current]
            self.calculate_loading_MF()
            instance_2_explore.calculate_loading_MF()
            if sum(self.distances) < sum(instance_2_explore.distances):
                index_best = index_current
            print(index_current, index_best)

        return index_best

    """
    _nh_change: Controls what neighbourhood is searched. When the solution in the nh is better than the current best, we 
                update the solution and restart from first nh. Else, we move to the next nh.
    REMARK
        The order in which the neighbourhoods are searched is important, but fixed. So given the order, we always explore 
        the one immediately after.  
    INPUTS
        solution_current: current minimization value.
        solution_nh: solution found in nh.
        nh_num: neighbourhood number that we are searching

    OUTPUTS
        return: best solution and new neighbourhood to search
    """

    def _nh_change(self, imbalance_current):

        if self.imbalance < imbalance_current:
            self.current_nh = 1
        else:
            self.current_nh = self.current_nh + 1

    """
    gvns : Implementation of General Variable Neighbourhood Search.
        solution_initial: Tour produced by greedy algorithms.
        timeout: Maximum allowed execution time
        nhs: ordered list of neighbourhoods to explore  
    """

    # noinspection SpellCheckingInspection
    def gvns(self, timeout):

        timeout_start = time.time()
        while time.time() < timeout_start + timeout:
            # Start search from first nh.
            nh_current = 0
            while nh_current <= len(self.neighbourhoods):
                # Neighbourhood shaking
                routes_to_explore = self._shake(nh_current)

                # Local search. We find the best route in the neighbourhood
                best_route_in_nh = self._best_improvement(routes_to_explore)

                # TODO: Check for feasibility of best route.
                # Update current route
                self.routes = routes_to_explore[best_route_in_nh]

                # Neighbourhood change
                # self._nh_change()