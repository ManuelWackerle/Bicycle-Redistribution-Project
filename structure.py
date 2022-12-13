"""
Definitions of the base classes used in the problem.
"""
import networkx as nx


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
        self._loading_instructions = instructions
        self._min_capacity_needed = 0

    def add_stop(self, station: str):
        self._stops.append(station)

    def remove_stop(self, station: str):
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
    def __init__(self, capacity: int, vehicle_id: str, vehicle_current_position = 0, vehicle_current_load = 0, vehicle_route: Route = None):
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
        self._route = vehicle_route
        self._current_position = vehicle_current_position
        self._current_load = vehicle_current_load

    def go_to_next_station(self):
        self._current_position += 1

    def load_or_unload(self):
        self._current_load += self.route._loading_instructions[self._current_position]


class ProblemInstance:
    """
    ProblemInstace: contains map and vehicles operating.
        model: graph of possible bike locations.
        vehicles: list of vehicles operating.
        imbalance: number of non-balanced bikes. -1 if not computed.
        allocated: number of bikes that the current routes allocate.
    """
    def __init__(self, input_graph: nx.Graph, vehicles: []):
        self.model = input_graph
        self.vehicles = vehicles
        self.imbalance = -1
        self.allocated = -1

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

    def compute_imbalance(self):

        for node, node_data in self.model.nodes.items():
            self.imbalance += self.model.nodes[node]['sup']

        self.imbalance = abs(self.imbalance)
