"""
Definitions of the base classes used in the problem.
"""
import networkx as nx
from utils import bcolors
from utils import edge_data_as_numpy


class Route (object):
    """
    Contains the information of the stops in a given route and the what to do at each station. The dimensions of the
    array of stops and the array of instrutions must match. The i-th component of the instructions is the number of
    bikes to load (>0) or unload (<0) at the i-th station.
    """
    def __init__(self, stops_in_route=None, instructions=None):
        """
        Initialize a route.
        :param stops_in_route: Array of stops in the route, in the order they are visited. Each station is a string.
        :param instructions:  Array of loading instructions for each stop. Dimension must agree with stops in the route.
        :param min_capacity_needed: The minimum capacity that a vehicle needs to perform the route.
        """
        if instructions is not None and len(instructions) != len(stops_in_route):
            raise Exception("Loading instructions do not match.\n")

        self._stops = stops_in_route
        self._distance = 0
        self._loading_instructions = instructions
        self._min_capacity_needed = 0

    def add_stop(self, station: str):
        self._stops.append(station)

    def remove_stop(self, station: str): #stops are not unique, this will remove the first stop in the path
        if station in self._stops:
            self._stops.remove(station)
        else:
            raise Exception("Stop to delete: %s is not in route\n" % station)

    def add_instructions(self, instructions: dict):
        for station in instructions.keys():
            self._loading_instructions[station] = instructions[station]

    def remove_instructions(self, instructions: dict):
        for station in instructions.keys():
            if station in self._stops:
                del self._loading_instructions[station]

    def get_route_stops(self) -> []:
        return self._stops

    def set_full_route(self, route: [], instructions=None):
        self._stops = route
        if instructions is not None:
            self._loading_instructions = instructions

    def compute_capacity_needed(self):
        self._min_capacity_needed = max(self._loading_instructions)

    def reset_route(self):
        self._stops = []
        self._loading_instructions = {}


class Vehicle(object):
    """
    Vehicle: contains vehicle identification, capacity and vehicle route .
    """
    def __init__(self, capacity: int, vehicle_id: str):
        """
        Initializes the vehicle.
        :param capacity: Maximum number of bikes that the vehicle can carry at any time.
        :param vehicle_id: License plate or other identifier of vehicle
        :param vehicle_current_position: Position of the vehicle as element in route array. This is an int, not a str.
        :param vehicle_current_load: Current number of bikes in the vehicle.
        :param vehicle_route: Route assigned to the vehicle.
        """
        self._capacity = capacity
        self._id = vehicle_id
        self._route = []
        self._loads = []
        self._distance = 0
        self._current_position = None
        self._current_load = 0

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
        return  self._current_position

    def current_load(self):
        return  self._current_load

    def set_route(self, route):
        self._route = route

    def route(self):
        return self._route

    def set_loads(self, loads):
        self._loads = loads

    def loads(self):
        return  self._loads

    def capacity(self):
        return self._capacity

    def id(self):
        return self._id



class ProblemInstance:
    """
    ProblemInstace: contains map and vehicles operating.
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
        self.vehicles = vehicles

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
            # TODO: handle case where sink & source don't match, e.g. by adding an additional node to the graph or changing depot value

    def calculate_distances(self, vehicles = None):
        total = 0
        if vehicles == None:
            vehicles = self.vehicles
        for v in vehicles:
            total += self.calculate_distance(v)
        return total

    def calculate_distance(self, vehicle):
        dist = 0
        prev = vehicle.route()[0]
        for s in range(1, len(vehicle.route())):
            if prev == vehicle.route()[s]:
                print("Warning: same route twice in  sequence - might be a mistake")
            else:
                dist += self.model.edges[prev, vehicle.route()[s]]['dist']
            prev = vehicle.route()[s]
        vehicle.set_distance = dist
        return dist

    def get_all_capacities(self) -> dict:
        """
        :return: Array of capacities of the vehicles
        """
        vehicle_capacities = {}
        for vehicle in self.vehicles:
            vehicle_capacities[vehicle.id] = vehicle.capacity

        return vehicle_capacities

    def assign_routes_to_vehicles(self, routes):
        """
        Set the routes for each vehicle.
        :param  routes: array of Route elements to be set for the vehicles.
        """
        assert len(routes) == len(self.vehicles), "Number of routes and vehicles don't match, can't set routes.\n"

        # Check compatibility of vehicles and routes. For each route, we must have a vehicle with suff. capacity
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

    def compute_imbalance(self, auxiliary_graph = None):
        if auxiliary_graph is None:
            auxiliary_graph = self.model
        for node, node_data in auxiliary_graph.nodes.items():
            sup = auxiliary_graph.nodes[node]['sup']
            if sup > 0:
                self.imbalance += sup

    def mean_distance(self):
        distances = edge_data_as_numpy(self.model, 'dist')
        return distances.mean()

    # def centre_node(self):
    #     """
    #     Finds the node nearest to the centre of the graph (the mean over all node positions)
    #
    #     :return centre_node: the central node of the graph
    #     """
    #     positions = utils.dict_data_as_numpy(self.node_data, 'pos')
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
    #     Given a source node returns a list with <num.vehicles> nodes that are far away from the source node and each other
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

    def display_results(self, show_instructions=True):
        """
        Displays the information in self.routes and self.instructions in a human readable way
        """
        results = "Results\t\t"
        if show_instructions:
            results += "total distance || instructions   <station>: <load/unload bikes> (<total on vehicle>)"
            for v in self.vehicles:
                dist = self.calculate_distance(v)
                line = "\nVehicle #{:<3} {:>12}km |".format(v.id(), dist/1000)
                prev_load, last = 0, 0
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
        d = self.calculate_distances()/1000
        success = bcolors.OKGREEN if self.allocated == self.imbalance else bcolors.FAIL
        results += bcolors.BOLD + bcolors.OKGREEN + "Total Distance:{:10}km".format(d) + bcolors.ENDC + " ||  "
        results += success + bcolors.BOLD + " Total Rebalanced: {}/{}".format(self.allocated,
                                                                              self.imbalance) + bcolors.ENDC
        print(results)

    def reset(self):
        self.routes = []
        self.instructions = []
        self.distances = []
        self.allocated = 0

