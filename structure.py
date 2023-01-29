"""
Definitions of the base classes used in the problem.
"""
from copy import deepcopy

import networkx as nx
from utils import *


class Vehicle(object):
    """
    Vehicle: contains vehicle identification, capacity and vehicle route .
    """
    def __init__(self, capacity: int, vehicle_id: str, distance_limit: int):
        """
        Initializes the vehicle
        :param capacity: Maximum number of bikes that the vehicle can carry at any time
        :param vehicle_id: License plate or other identifier of vehicle
        """
        self._capacity = capacity
        self._id = vehicle_id
        self._route = []
        self._loads = []
        self._distance = 0
        self._current_position = None
        self._current_load = 0
        self._modified = True
        self._distance_limit = distance_limit

    def add_stop(self, stop, load):
        self._route.append(stop)
        self._loads.append(load)
        self._current_position = stop
        self._current_load = load

    def remove_stop(self, stop_indx):
        del self._route[stop_indx]
        del self._loads[stop_indx]
        # self._current_position = None
        # self._current_load = 0

    def current_stop(self):
        return self._current_position

    def current_load(self):
        return self._current_load

    def set_route(self, route):
        self._route = route
        self._modified = True

    def modified(self):
        modified = self._modified
        self._modified = False
        return modified

    def route(self):
        return self._route

    def set_loads(self, loads):
        self._loads = loads

    def loads(self):
        return self._loads

    def capacity(self):
        return self._capacity

    def id(self):
        return self._id

    def distance_limit(self):
        return self._distance_limit

    def reset(self):
        self._route = []
        self._loads = []
        self._distance = 0
        self._current_position = None
        self._current_load = 0

class ProblemInstance:
    """
    ProblemInstance: contains map and vehicles operating.
        model: graph of possible bike locations.
        vehicles: list of vehicles operating.
        imbalance: number of non-balanced bikes. -1 if not computed.
        allocated: number of bikes that the current routes allocate.
        verbose: for debugging purposes. 0: execute silently, 1: display warnings only, 2: display in-between steps
    """
    def __init__(self, input_graph: nx.Graph, vehicles: [], node_data=None, verbose=0):
        self._verbose = verbose

        # problem instance variables
        self.model = input_graph
        self.node_data = node_data
        self.vehicles = vehicles
        self.depot = '0'

        # tracking variables
        self.total_source = 0
        self.total_sink = 0
        self.imbalance = 0
        self.allocated = 0
        self._initialize_tracking_variables(input_graph)
        self.mf_graph = None

        # G.
        # self.neighbourhoods = []  # keys of neighbourhoods to be searched.
        # self.nh_dict = {"remove_station": self.remove_one_station}  # functions associated to neighbourhoods
        # self.current_nh = 0

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
            # TODO: handle case where sink & source don't match, e.g. by adding an additional
            #  node to the graph or changing depot value

    def distance_route_segment(self, route:[], start_indx, stop_indx):
        dist = 0
        step = 1 if start_indx < stop_indx else -1 #calculate
        for i in range(start_indx, stop_indx, step):
            u, v = route[i], route[i + step]
            dist += self.model.edges[u, v]['dist']
        return dist

    def distance(self, node1, node2): #Wrapper for distance function
        return self.model.edges[node1, node2]['dist']

    def calculate_distances(self, vehicles=None):
        total = 0
        if vehicles is None:
            vehicles = self.vehicles
        for v in vehicles:
            total += self.calculate_distance(v)
        return total

    def calculate_distance(self, vehicle):
        dist = 0
        prev = vehicle.route()[0]
        for s in range(1, len(vehicle.route())):
            dist += self.model.edges[prev, vehicle.route()[s]]['dist']
            prev = vehicle.route()[s]
        vehicle.set_distance = dist
        return dist

    def check_distance_limits(self):
        """check distance limit for all vehicles

        Returns:
            Bool: Result of feasibility check for all vehicles
            List: Distance for each vehicles
        """
        distances = []
        constraint = []
        for vehicle in self.vehicles:
            dist = self.calculate_distance(vehicle)
            distances.append(dist)
            constraint.append(dist <= vehicle.distance_limit())
        return all(constraint), distances

    def get_all_capacities(self) -> dict:
        """
        :return: Array of capacities of the vehicles
        """
        vehicle_capacities = {}
        for vehicle in self.vehicles:
            vehicle_capacities[vehicle.id] = vehicle.capacity()

        return vehicle_capacities

    def get_all_routes(self, vehicles=None):
        if vehicles is None:
            vehicles = self.vehicles

        current_routes = [None]*len(vehicles)

        for vehicle_index, vehicle in enumerate(vehicles):
            current_routes[vehicle_index] = vehicle.route()

        return current_routes

    def assign_routes_to_vehicles(self, routes):
        """
        Set the routes for each vehicle
        :param  routes: array of Route elements to be set for the vehicles
        """
        assert len(routes) == len(self.vehicles), "Number of routes and vehicles don't match, can't set routes.\n"

        # Check compatibility of vehicles and routes. For each route, we must have a vehicle with sufficient capacity
        assert set(self.get_all_capacities()) == set(routes[:, 0]), "Vehicle capacities and route loads don't match"

        unused_vehicles = self.vehicles
        unassigned_routes = routes

        for route in unassigned_routes:
            for vehicle in unused_vehicles:
                if vehicle.capacity == route._min_capacity_needed:
                    # Assign route to vehicle
                    vehicle.route = route

                    # Delete route and vehicle from unused
                    unused_vehicles.delete(vehicle)
                    unassigned_routes.delete(route)

                    # Move on
                    break

        return len(unassigned_routes)

    def compute_imbalance(self, auxiliary_graph=None):
        if auxiliary_graph is None:
            auxiliary_graph = self.model
        for node, node_data in auxiliary_graph.nodes.items():
            sup = auxiliary_graph.nodes[node]['sup']
            if sup > 0:
                self.imbalance += sup

    def mean_distance(self):
        distances = edge_data_as_numpy(self.model, 'dist')
        return distances.mean()

    def intialize_flow_graph(self):
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

        for l, vehicle in enumerate(self.vehicles):
            route = vehicle.route()
            prev_node = 0
            for r, node in enumerate(route):
                node_str = "{}-{}".format(l, r)
                demand = self.model.nodes[node]['sup']
                if demand > 0:
                    mf_graph.add_edge(node, node_str)
                elif demand < 0:
                    mf_graph.add_edge(node_str, node)
                if prev_node != 0:
                    mf_graph.add_edge(prev_node, node_str, capacity=vehicle.capacity())
                prev_node = node_str
        self.mf_graph = mf_graph

    def calculate_loading_MF(self, check_feasibility_only=False, start_load=0):
        """
        Given a set of vehicle routes, calculates the optimal loading instructions for each route using a Maximum flow computation.
        Use this function if monotonicity is assumed.
        :param prob: Problem instance
        :param start_load: number of bicycles that
        :modifies vehilce.loads: The instructions for how to load and unload the bicycles.
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

        for l, vehicle in enumerate(self.vehicles):
            prev_node = 0
            for r, node in enumerate(vehicle.route()):
                node_str = "{}-{}-{}".format(node, l, r)
                demand = self.model.nodes[node]['sup']
                if demand > 0:
                    mf_graph.add_edge(node, node_str)
                elif demand < 0:
                    mf_graph.add_edge(node_str, node)
                if prev_node != 0:
                    mf_graph.add_edge(prev_node, node_str, capacity=vehicle.capacity())
                prev_node = node_str
        self.show_info("Graph generated with {n} nodes and {e} edges. Source flow: {s}, Sink flow: {t}"
                       .format(n=len(mf_graph.nodes), e=len(mf_graph.edges), s=total_source, t=total_sink))

        # Solve Max Flow Problem
        self.show_header("Solving the Max flow problem ")
        self.imbalance = total_source - start_load
        if total_sink != total_source:
            self.show_warning("mismatch in source and sink flow capacity, no exact solution can exist.")
            self.imbalance = -1

        value, data = nx.maximum_flow(mf_graph, 's', 't')  # , flow_func=nx.algorithms.flow.shortest_augmenting_path()
        self.allocated = value - start_load

        if not check_feasibility_only:
            if value != total_source or value != total_sink:
                self.show_warning(
                    "Bikes can not be allocated to full capacity. Source flow: {s}, Sink flow: {t}, Allocated: {a}"
                        .format(s=total_source, t=total_sink, a=value))
            else:
                self.show_info("Bike allocation is exact. Total allocated bicycles: {}".format(value))

            self.show_header("Generating instructions")
            for l, vehicle in enumerate(self.vehicles):
                loads = []
                path = vehicle.route()
                prev = path[0]
                for r, node in enumerate(path[1:], 1):
                    prev_str = "{}-{}-{}".format(prev, l, r - 1)
                    node_str = "{}-{}-{}".format(node, l, r)
                    loads.append(data[prev_str][node_str])
                    prev = node
                vehicle.set_loads(loads)

    def remove_unused_stops(self):
        """
        Given a set of vehicle routes, removes all the stops where the vehicle neither load nor unloads any bikes.
        """
        for v in self.vehicles:
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

    def verify_loading_on_swapped_route(self, b1, b2, l1, l2=None,  tolerance=0):
        l2 = l1 if l2 is None else l2
        v1, v2 = self.vehicles[l1], self.vehicles[l2]
        ri, rj = "{}-{}".format(l1, b1), "{}-{}".format(l1, b1 + 1)
        rk, rl = "{}-{}".format(l2, b2), "{}-{}".format(l2, b2 + 1)

        self.mf_graph.remove_edge(ri, rj)
        self.mf_graph.remove_edge(rk, rl)
        if l1 == l2:
            self.mf_graph.add_edge(ri, rk, capacity=v1.capacity())
            self.mf_graph.add_edge(rj, rl, capacity=v1.capacity())

            prev = rj
            for m in range(b1 + 2, b2 + 1):
                node = "{}-{}".format(l1, m)
                self.mf_graph.remove_edge(prev, node)
                self.mf_graph.add_edge(node, prev, capacity=v1.capacity())
                prev = node
        else:
            self.mf_graph.add_edge(ri, rl, capacity=v1.capacity())
            self.mf_graph.add_edge(rk, rj, capacity=v2.capacity())

            ## uncomment to adjust for different vehicle capacities
            # if v1.capacity != v2.capacity:
            #     prev = "{}-{}".format(l1, b1 + 1)
            #     for m1 in range(b1 + 2, len(v1.route())):
            #         node = "{}-{}".format(l1, m1)
            #         self.mf_graph.edges[prev, node]['capacity'] = v2.capacity
            #         prev = node
            #     prev = "{}-{}".format(l2, b2 + 1)
            #     for m2 in range(b2 + 2, len(v2.route())):
            #         node = "{}-{}".format(l2, m2)
            #         self.mf_graph.edges[prev, node]['capacity'] = v1.capacity
            #         prev = node

        # Sovle Max Flow Problem
        self.allocated, data = nx.maximum_flow(self.mf_graph, 's', 't')

        if self.allocated < self.imbalance - tolerance: #if not sufficiently balanced undo changes to graph
            self.mf_graph.add_edge(ri, rj, capacity=v1.capacity())
            self.mf_graph.add_edge(rk, rl, capacity=v2.capacity())
            if l1 == l2:
                self.mf_graph.remove_edge(ri, rk)
                self.mf_graph.remove_edge(rj, rl)

                prev = rj
                for m in range(b1 + 2, b2 + 1):
                    node = "{}-{}".format(l1, m)
                    self.mf_graph.remove_edge(node, prev)
                    self.mf_graph.add_edge(prev, node, capacity=v1.capacity())
                    prev = node
            else:
                self.mf_graph.remove_edge(ri, rl)
                self.mf_graph.remove_edge(rk, rj)

                ## uncomment to adjust for different vehicle capacities
                # if v1.capacity != v2.capacity:
                #     prev = "{}-{}".format(l1, b1 + 1)
                #     for m1 in range(b1 + 2, len(v1.route())):
                #         node = "{}-{}".format(l1, m1)
                #         self.mf_graph.edges[prev, node]['capacity'] = v1.capacity
                #         prev = node
                #     prev = "{}-{}".format(l2, b2 + 1)
                #     for m2 in range(b2 + 2, len(v2.route())):
                #         node = "{}-{}".format(l2, m2)
                #         self.mf_graph.edges[prev, node]['capacity'] = v2.capacity
                #         prev = node

    # def centre_node(self):
    #     """
    #     Finds the node nearest to the centre of the graph (the mean over all node positions)
    #
    #     :return centre_node: the central node of the graph
    #     """
    #     positions = dict_data_as_numpy(self.node_data, 'pos')
    #     centre = positions.mean(axis=0)
    #     best, centre_node = math.inf, '0'
    #     for n, data in self.node_data.items():
    #         dist = int(round(haversine(centre, data['pos'], unit=Unit.METERS)))
    #         if dist < best:
    #             best = dist
    #             centre_node = n
    #     return centre_node

    # def furthest_nodes(self, number=0):
    #     """
    #     Given a source node returns a list with <num.vehicles> nodes that are far away from the source and each other
    #
    #     :return furthest_nodes: array of nodes from self.model
    #     """
    #
    #     centre = self.centre_node()
    #     furthest_nodes, best = [], '0'
    #     number = self.num_vehicles if number == 0 else number
    #     for l in range(number):
    #         furthest = 0
    #         for n in self.model.nodes:
    #             sup = self.model.nodes[n]['sup']
    #             if sup > 0 and n != centre and n not in furthest_nodes:
    #                 dist = self.model.edges[centre, n]['dist'] ** 2
    #                 for s in furthest_nodes:
    #                     dist += self.model.edges[s, n]['dist'] ** 2
    #                 if dist > furthest:
    #                     furthest = dist
    #                     best = n
    #         furthest_nodes.append(best)
    #     return furthest_nodes

    # def tsp_bound(self):
    #     graph = deepcopy(self.model)
    #     print(graph.is_directed())
    #     m = len(graph.nodes)
    #     capacity = self.vehicles[0].capacity()
    #     print(m)
    #     for n in self.model.nodes:
    #         while graph.nodes[n]['sup'] > capacity:
    #             graph.nodes[n]['sup'] -= capacity
    #             graph.add_node(str(m))
    #             for o in graph.nodes:
    #                 if o != str(m) and o != n:
    #                     d = graph.edges[n, o]['dist']
    #                     graph.add_edge(str(m), o, dist=d)
    #                     d = graph.edges[o, n]['dist']
    #                     graph.add_edge(o, str(m), dist=d)
    #             graph.add_edge(str(m), n, dist=999999)
    #             graph.add_edge(str(n), m, dist=999999)
    #             m += 1
    #     print(m)
    #     seq = nx.algorithms.approximation.christofides(graph, weight='dist')
    #     dist = 0
    #     for s in range(1, len(seq)):
    #         dist += graph.edges[seq[s-1], seq[s]]['dist']
    #     print("TSP soltion: {} => lower bound = {}".format(dist, dist/1.5))


    def display_results(self, show_instructions=True):
        """
        Displays the information in routes and instructions in a human-readable way
        """
        results = "Results\t\t"

        if show_instructions:
            results += "total distance || instructions   <station>: <load/unload bikes> (<total on vehicle>)"
            for v in self.vehicles:
                dist = self.calculate_distance(v)
                line = "\nVehicle #{:<3} {:>12}km |".format(v.id(), round(dist/1000, 3))
                prev_load, last = 0, -1
                for s in range(len(v.route())-1):
                    load = v.loads()[s]
                    diff = load - prev_load
                    prev_load = load
                    instr = 'l' if diff >= 0 else 'u'
                    # a, b = self.routes[l][s], self.routes[l][s+1]
                    # dist = self.model.edges[a, b]['dist'] if a != b else 0
                    line += "|{:>3}: ".format(v.route()[s]) + instr + "{:<2}({:2})".format(abs(diff), load)
                    last = s
                results += line + "|{:>3}: ".format(v.route()[last + 1]) + "u{:<2}( 0)|".format(prev_load)
            results += "\n"
        d = round(self.calculate_distances()/1000, 3)
        success = bcolors.OKGREEN if self.allocated == self.imbalance else bcolors.FAIL
        results += bcolors.BOLD + bcolors.OKGREEN + "Total Distance:{:10}km".format(d) + bcolors.ENDC + " ||  "
        results += success + bcolors.BOLD + " Total Rebalanced: {}/{}".format(self.allocated,
                                                                              self.imbalance) + bcolors.ENDC
        print(results)

    def plot_vehicle_route(self, vehicle):
        route = vehicle.route()
        loads = vehicle.loads()
        loads = loads if len(loads) == len(route) else loads + [0]
        stops = list(range(len(route)))

        plt.plot(stops, loads)
        running_supply = 0
        virtual_loads = [0]*len(route)
        for n, n_id in enumerate(route[1:]):
            supply = self.model.nodes[n_id]['sup']
            running_supply += supply
            colour = 'g' if loads[n+1] - loads[n] == supply else 'y'
            plt.bar(n+1, supply, color=colour, alpha=0.5)
            virtual_loads[n+1] = running_supply
        # plt.plot(stops, virtual_loads)
        plt.xlabel('Vehicle {}'.format(vehicle.id()))
        plt.ylabel(['Load Values', 'Supply Values'])

        # Add horizontal markers for max and min load values
        min_load = 0
        max_load = vehicle.capacity()
        plt.axhline(min_load, color='b', linestyle='--')
        plt.axhline(max_load, color='b', linestyle='--')
        plt.show()

    def remove_nodes_zero_demand(self):
        """
        Nodes with zero demand need not be considered in the rebalancing problem with monotonicity assumption,
        so we can remove them from the graph before operation.
        """
        for node, demand in enumerate(self.model.nodes()):
            if demand == 0:
                self.model.remove_node(node)

    def reset(self):
        for v in self.vehicles:
            v.reset()
        self.allocated = 0


