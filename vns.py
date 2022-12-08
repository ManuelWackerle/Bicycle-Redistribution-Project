import networkx as nx
import math
from haversine import haversine, Unit
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
    def __init__(self, input_graph: nx.Graph, num_vehicles, vehicle_capacity, node_data=None, verbose=0):
        self._verbose = verbose

        #input variables
        self.model = input_graph
        self.node_data = node_data
        self.num_vehicles = num_vehicles
        self.capacity = vehicle_capacity

        #tracking variables
        self.total_source = 0
        self.total_sink = 0
        self.imbalance = 0
        self.allocated = 0
        self._initialize_tracking_variables(input_graph)

        #result variables
        self.routes = []
        self.instructions = []
        self.distances = []




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
                for s in range(len(route)-1):
                    dist = self.model.edges[source, route[s]]['dist'] + self.model.edges[source, route[s+1]]['dist']
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


    def greedy_routing_v1(self, source='0', varied_starts=None, x1=3):
        """
        Finds a set of vehicle routes with low cost based on a greedy approach.
        Version 1: most basic

        :return routes: the set vehicle routes
        """

        self.show_header("Searching for routes using basic greedy")
        graph = self.model.copy()
        mean = self.mean_distance()
        varied = varied_starts is not None

        #Initialize tracker variables
        vehicles, distances, loads, space, remove, current = [], [], [], [], [], []
        self.instructions = []
        for l in range(self.num_vehicles):
            vehicles.append(l)
            distances.append(0) #total route distance
            if varied:
                start = varied_starts[l]
                self.routes.append([start])
                current.append(varied_starts[l])
                loads.append(min(self.model.nodes[start]['sup'],self.capacity))
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
                            score = -move * (mean / dist) ** x1
                        elif sup < 0:
                            move = min(-sup, loads[l])
                            score = move * (mean / dist) ** x1
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

    def greedy_routing_v2(self, source='0', x1=3, x2=2):
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

        for i in range(1, len(guided)-2):
            s, n = guided[i], guided[i+1]
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
                            score = -move * (mean / dist) ** x1
                        elif sup < 0:
                            move = min(-sup, loads[l])
                            score = move * (mean / dist) ** x1
                        if curr in successors and n in successors[curr]:
                            score *= x2
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
        print(path)
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
        for s in range(len(seq)-1):
            dist += tsp_graph.edges[seq[s], seq[s+1]]['dist']
            self.routes[0] += segments[seq[s]][2]
            print(segments[seq[s]][2])
        print(dist)
        self.routes[0] += source
        self.recalculate_distance()


    def tsp(self):
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


    def greedy_routing_PILOT(self):
        """
        Finds a set of vehicle routes with low cost based on a greedy approach.
        Version 1: greedy search using the PILOT technique

        :return routes: the set vehicle routes
        """

        pass #Todo: implement


    def calculate_loading_MF(self, start_load=0, source='0'):
        """
        Given a set of vehicle routes, calculates optimal loading instructions for each route using a Maximum flow computation.
        Use this function if mononicity is assumed.

        :return instructions: The instructions for how to load and unload the bicycles.
        """

        #Generate Max Flow graph
        self.show_header("Generating Max flow graph")
        total_source, total_sink = 0, start_load
        mf_graph = nx.DiGraph()
        mf_graph.add_node('s') #source node
        mf_graph.add_node('t') #sink node
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

        #Sovle Max Flow Problem
        self.show_header("Solving the Max flow problem ")
        self.imbalance = total_source - start_load
        if total_sink != total_source:
            self.show_warning("mismatch in source and sink flow capacity, no exact solution can exist.")
            self.imbalance = -1

        #This is where the magic happens
        value, data = nx.maximum_flow(mf_graph, 's', 't') #, flow_func=nx.algorithms.flow.shortest_augmenting_path) #TODO: investigate this algorithm exactly and see if it can be done better
        self.allocated = value - start_load

        if value != total_source or value != total_sink:
            self.show_warning("Bikes can not be allocated to full capacity. Source flow: {s}, Sink flow: {t}, Allocated: {a}"
                              .format(s=total_source, t=total_sink, a=value))
        else:
            self.show_info("Bike allocation is exact. Total allocated bicycles: {}".format(value))

        self.show_header("Generating instructions")
        self.instructions = []
        for p in range(len(self.routes)):
            self.instructions.append([])
            path = self.routes[p]
            for r in range(len(path)-1):
                node = path[r]
                next = path[r+1]
                node_str = "{}-{}-{}".format(node, p, r)
                next_str = "{}-{}-{}".format(next, p, r+1)
                self.instructions[p].append(data[node_str][next_str])

        routes_str = "Routes with loading instructions:"
        if self._verbose > 1:
            for r in range(len(self.routes)):
                routes_str += "\nvehicle {}: load = {}"\
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


    def two_opt(self, routes):
        new_routes = []
        for l in range(len(routes)):
            new_routes.append([])
            route = routes[l]
            best, b1, b2 = 0, None, None
            for s1 in range(1, len(route)-4):
                ri, rj = route[s1], route[s1 + 1]
                for s2 in range(s1 + 2, len(route)-2):
                    rk, rl = route[s2], route[s2 + 1]
                    if s2 < s1 - 1 or  s1 + 1 < s2:
                        if ri != rk and ri != rl and rj != rk and rj != rl:
                            value = self.model.edges[ri, rj]['dist'] + self.model.edges[rk, rl]['dist']
                            switch = self.model.edges[ri, rk]['dist'] + self.model.edges[rl, rj]['dist']
                        if value - switch > best:
                            best = value - switch
                            b1, b2 = s1, s2
            if b1 is not None:
                new_routes[l] = route[:b1+1] + route[b2:b1:-1] + route[b2+1:]
            else:
                self.show_warning("no 2-opt improvement found for vehicle #{}".format(l))
        return new_routes


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


    def recalculate_distance(self):
        self.distances = []
        for l in range(len(self.routes)):
            route = self.routes[l]
            dist = 0
            prev = route[0]
            for s in range(1,len(route)):
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
                line = "\nVehicle #{:<3} {:>11}km |".format(l, self.distances[l]/1000)
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
                results += line + "|{:>3}: ".format(self.routes[l][last+1]) + "u{:<2}( 0)|".format(prev_load)
            results += "\n"
        d = sum(self.distances)/1000
        success = bcolors.OKGREEN if self.allocated == self.imbalance else bcolors.FAIL
        results += bcolors.BOLD + bcolors.OKGREEN + "Total Distance:{:9}km".format(d) + bcolors.ENDC + " ||  "
        results += success + bcolors.BOLD + " Total Rebalanced: {}/{}".format(self.allocated, self.imbalance) + bcolors.ENDC
        print(results)



    def reset(self):
        self.routes = []
        self.instructions = []
        self.distances = []
        self.allocated = 0



