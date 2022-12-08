import networkx as nx
import numpy as np

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
        self.routes = [] #Array of arrays corresponding to each vehicle and the sequence of vertices visited
        self.instructions = [] #Array of arrays corresponding to each vehicle and the number of bicyles to pick up
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

        #Initialize tracker variables
        stops = 0
        vehicles, distances, loads, space, remove, current = [], [], [], [], [], []
        for l in range(self.num_vehicles):
            vehicles.append(l)
            self.routes.append([source])
            distances.append(0) #total route distance
            loads.append(0) #number of bikes on vehicle
            space.append(self.capacity) #remaining space on vehicle
            current.append(source)

        while len(vehicles) != 0:
            if stops > len(graph.nodes)*2:
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

        pass #Todo: implement


    def greedy_routing_PILOT(self):
        """
        Finds a set of vehicle routes with low cost based on a greedy approach.
        Version 1: greedy search using the PILOT technique

        :return routes: the set vehicle routes
        """

        pass #Todo: implement

    def remove_one_station(self):
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
        retval = [
            [
                self.routes[:j] + self.routes[j+1:]  # remove j-th station from list
                for j in range(len(self.routes[i]))
            ]
            for i in range(len(self.routes))
        ]
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

    def insert_unbalanced_station(self):
        """adding unbalanced station is considered as the neighbor

        Write
            N = Number of trucks (=len(self.routes))
            Cn = Number of candidate routes (=len(self.routes[n])*len(unbalanced_stations))
            Ln = Route length (=len(self.routes[n])+1)
        Note
            Cn, Ln depends on n in N. (candidate routes and route length might differ for each truck)
        :return
            (N, Cn, Ln) matrix (python list)
        """
        graph = self._get_rebalanced_graph()
        unbalanced_stations = [x for x in graph.nodes if  graph.nodes[x]['sup'] != 0]
        retval = [
            [
                route[:j] + [u] + route[j:]  # insert unbalanced station to j-th position
                for u in unbalanced_stations
                for j in range(1, len(route))
            ] for route in self.routes
        ]
        return retval

    def calculate_loading_MF(self):
        """
        Given a set of vehicle routes, calculates optimal loading instructions for each route using a Maximum flow computation.
        Use this function if mononicity is assumed.

        :return instructions: The instructions for how to load and unload the bicycles.
        """

        #Generate Max Flow graph
        self.show_header("Generating Max flow graph")
        total_source, total_sink = 0, 0
        mf_graph = nx.DiGraph()
        mf_graph.add_node('s') #source node
        mf_graph.add_node('t') #sink node
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

        #Sovle Max Flow Problem
        self.show_header("Solving the Max flow problem ")
        self.imbalance = total_source
        if total_sink != total_source:
            self.show_warning("mismatch in source and sink flow capacity, no exact solution can exist.")
            self.imbalance = -1

        #This is where the magic happens
        value, dict = nx.maximum_flow(mf_graph, 's', 't') #, flow_func=nx.algorithms.flow.shortest_augmenting_path) #TODO: investigate this algorithm exactly and see if it can be done better
        self.allocated = value

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
                self.instructions[p].append(dict[node_str][next_str])

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
            for s in range(1,len(route)):
                if prev == route[s]:
                    self.show_warning("same route twice in sequence - might be a mistake")
                else:
                    dist += self.model.edges[prev, route[s]]['dist']
                prev = route[s]
            self.distances.append(dist)


    def display_results(self,show_instructions=True):
        """
        Displays the information in self.routes and self.instructions in a human readable way
        """
        results = "Results\t\t"
        if show_instructions:
            results += "total distance || instructions   <station>: <load/unload bikes> (<total on vehicle>)"
            for l in range(len(self.routes)):
                line = "\nVehicle #{:<3} {:>11}km |".format(l, self.distances[l]/1000)
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
        d = sum(self.distances)/1000
        success = bcolors.OKGREEN if self.allocated == self.imbalance else bcolors.FAIL
        results += bcolors.BOLD + bcolors.OKGREEN + "\nTotal Distance:{:9}km".format(d) + bcolors.ENDC + " ||  "
        results += success + bcolors.BOLD + " Total Rebalanced: {}/{}".format(self.allocated, self.imbalance) + bcolors.ENDC
        print(results)



    def reset(self):
        self.routes = []
        self.instructions = []
        self.distances = []
        self.imbalance = 0
        self.allocated = 0



