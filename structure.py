"""
Definitions of the base classes used in the problem.
"""
from utils import *


class Vehicle(object):
    """
    Vehicle: contains vehicle identification, capacity and vehicle route information.
    """
    def __init__(self, capacity: int, vehicle_id: str, distance_limit=100):
        """
        Initializes the vehicle class
        :param capacity: Maximum number of bikes that the vehicle can carry at any time
        :param vehicle_id: License plate or other identifier of vehicle
        :param distance_limit: -not used-
        """
        self._capacity = capacity
        self._id = vehicle_id
        self._route = []
        self._loads = []
        self._distance = 0
        self._current_position = None
        self._current_load = 0
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

    def update_distance(self, dist):
        self._distance = dist

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

    def distance(self):
        return self._distance

    def reset(self):
        self._route = []
        self._loads = []
        self._distance = 0
        self._current_position = None
        self._current_load = 0


class ProblemInstance:
    """
    ProblemInstance: contains map (graph information) and operating vehicles
    """
    def __init__(self, input_graph: nx.Graph, vehicles: [], node_data=None, depot='0', verbose=0):
        """
            :param input_graph: the model/graph of possible bike locations.
            :param vehicles: array of vehicles
            :param node_data: a dictionary of dictionaries of node information; position and id
            :param depot: id of the node to be used as the depot
            :param verbose: for debugging purposes. 0: execute silently, 1: display warnings only, 2: display all steps
        """

        # problem instance variables
        self.model = input_graph
        self.node_data = node_data
        self.vehicles = vehicles
        self.depot = depot

        # tracking variables
        self.total_source = 0
        self.total_sink = 0
        self.imbalance = 0
        self.allocated = 0
        self.mf_graph = None
        self.average_distance = 0

        self._initialize_tracking_variables(input_graph)
        self._verbose = verbose

    def show_info(self, info_string):
        if self._verbose > 1:
            print(bcolors.OKBLUE + info_string + bcolors.ENDC)

    def show_warning(self, warn_string):
        if self._verbose > 0:
            print(bcolors.WARNING + "Warning: " + warn_string + bcolors.ENDC)

    def _initialize_tracking_variables(self, input_graph: nx.Graph):
        """
        Loops once through all the input data to collect additional information about the graph.
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
            self.imbalance = max(self.total_source, self.total_sink)
            self.model.nodes[self.depot]['sup'] = self.total_sink - self.total_source

    def distance(self, node1, node2):  # wrapper for distance function
        return self.model.edges[node1, node2]['dist']

    def distance_in_route(self, route: [], start_index, stop_index):
        """
            Calculates the directed distance on a route between two stops, stop1 and stop2 can be in reverse
            :param route: The route as an array of stops
            :param start_index: position index of the start node
            :param stop_index: position index of the stop node
            :return dist: the distance travelled by traversing the segment of route from start to stop
        """
        dist = 0
        step = 1 if start_index < stop_index else -1
        for i in range(start_index, stop_index, step):
            u, v = route[i], route[i + step]
            dist += self.distance(u, v)
        return dist

    def calculate_distances(self, vehicles=None):
        """
            Given a set of vehicles with corresponding routes, calculate the total distance travelled
            :param vehicles: array of vehicle objects
            :return total: the total distance travelled in metres
        """
        total = 0
        if vehicles is None:
            vehicles = self.vehicles
        for v in vehicles:
            total += self.calculate_distance(v)
        self.average_distance = total/len(vehicles)
        return total

    def calculate_distance(self, vehicle):
        """
            Given a single vehicles with a corresponding route, calculate the distance travelled by the vehicle
            :param vehicle: an object of class Vehicle
            :return dist: the distance travelled in metres
        """
        dist = 0
        prev = vehicle.route()[0]
        for s in range(1, len(vehicle.route())):
            dist += self.distance(prev, vehicle.route()[s])
            prev = vehicle.route()[s]
        vehicle.update_distance(dist)
        return dist

    def check_distance_limits(self):
        """
            Check distance limit for all vehicles
            :return satisfied: Result of feasibility check for all vehicles
            :return distances: distance for each vehicle
        """
        distances = []
        constraint = []
        for vehicle in self.vehicles:
            dist = self.calculate_distance(vehicle)
            distances.append(dist)
            constraint.append(dist <= vehicle.distance_limit())
        satisfied = all(constraint)
        return satisfied, distances

    def get_all_capacities(self) -> dict:
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

    def calculate_loading_mf(self, check_feasibility_only=False, start_load=0):
        """
        Given a set of vehicle routes, calculates the optimal loading instructions for each route using a
        maximum flow computation. Use this function if monotonicity is assumed.
        :param check_feasibility_only: if true will not calculate explicit loading instructions
        :param start_load: number of bicycles at the depot at the start of the tours
        :modifies vehicle.loads: The instructions for how to load and unload the bicycles.
        """

        # Generate max flow graph
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

        for k, vehicle in enumerate(self.vehicles):
            prev_node = 0
            for r, node in enumerate(vehicle.route()):
                node_str = "{}-{}".format(k, r)
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

        # Solve max flow problem
        self.imbalance = total_source - start_load
        if total_sink != total_source:
            self.show_warning("mismatch in source and sink flow capacity, no exact solution can exist.")
            self.imbalance = -1

        value, data = nx.maximum_flow(mf_graph, 's', 't')  # , flow_func=nx.algorithms.flow.shortest_augmenting_path()
        self.allocated = value - start_load

        if not check_feasibility_only:
            if value != total_source or value != total_sink:
                self.show_warning(
                    "Bikes can not be allocated to full capacity. Source flow: {s},"
                    " Sink flow: {t}, Allocated: {a}".format(s=total_source, t=total_sink, a=value))
            else:
                self.show_info("Bike allocation is exact. Total allocated bicycles: {}".format(value))

            # Generate loading instructions
            for k, vehicle in enumerate(self.vehicles):
                loads = []
                path = vehicle.route()
                for r, _ in enumerate(path[1:], 1):
                    prev_str = "{}-{}".format(k, r - 1)
                    node_str = "{}-{}".format(k, r)
                    loads.append(data[prev_str][node_str])
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

    def display_results(self, show_instructions=True):
        """
            Displays the information in routes and instructions in a human-readable way
            :param show_instructions: set to True to show loading instructions at each stop
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

    def _validate_edges(self, edges: nx.classes.reportviews.OutEdgeView):
        """
        Check that a provided list of edges is subset of the edges in the graph.
        """
        assert all(edge in self.model.edges for edge in edges)

    def add_noise(
            self,
            edges: nx.classes.reportviews.OutEdgeView = None,
            edge_atr: str = 'dist', distr=np.random.normal, distr_kwargs={'loc': 0, 'scale': 1}):
        """
        Adds noise to the cost matrix of the problem instance in specified edges
        :param edges: Set of NetworkX edges to add noise to
        :param edge_atr: name of the edge attribute to add noise to
        :param distr: Distribution from which to sample the noise
        :param distr_kwargs: Keyword arguments for the specified distribution
        :param random_state: Pass a specific value to ensure reproductibility of experiment
        """

        if edges is None:
            edges = self.model.edges()
        else:
            self._validate_edges(edges)

        noise = distr(**distr_kwargs, size=len(edges))
        for num, edge in enumerate(edges):
            self.model[edge[0]][edge[1]][edge_atr] = self.model[edge[0]][edge[1]][edge_atr] + noise[num]

    def reset(self):
        for v in self.vehicles:
            v.reset()
        self.allocated = 0
