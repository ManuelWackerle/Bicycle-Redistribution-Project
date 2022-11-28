import networkx as nx

from utils import *

class MinimumCostFlow(object):
    """
    Provides the original minimum cost flow approach.
    An optimal flow is calculated for each bike and this is reformulated into a TSP problem.

    config format:
    - num_vehicles: the number of vehicles available
    - capacity: positive integer giving the vehicle capacity #TODO (OR array corresponding to each vehicle)
    - verbose: for debugging purposes. 0: execute silently, 1: display warnings only, 2: display in-between steps
    """
    def __init__(self, input_graph: nx.Graph, num_vehicles, vehicle_capacity, verbose=0):
        self.model = input_graph
        self.tsp_model = None
        self.num_vehicles = num_vehicles
        self.capacity = vehicle_capacity
        self.route = [] #The sequence of vertices visited
        self.instructions = [] #Array of the number of bicyles to pick up
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

    def flip_demand(self):
        for n, attr in self.model.nodes.items():
            self.model.nodes[n]['sup'] = -attr['sup']

    def min_cost_flow_INCORRECT(self):
        """
        Uses the standard NetworkX min cost flow calculation. This strictly incorrect.
        Calculates a cost per moved bike instead of per vehicle, only provides an approximation.

        :return flow_dict: the set vehicle routes
        """
        self.show_header("Calculating min-cost flow")

        #use standard NetworkX min flow calculation
        self.flip_demand() #NetworkX uses demand instead of supply values
        directed = nx.to_directed(self.model)
        flow_dict = nx.min_cost_flow(directed, demand='sup', weight='dist')
        #This calculation will throw an error if there is a mismatch in supply and demand values
        string_output = "Flows:"
        if self._verbose > 1:
            for source,value in flow_dict.items():
                for target,flow in value.items():
                    if flow != 0:
                        string_output += "\nPath: {}-{}, Num bikes: {}".format(source,target,flow)
        self.show_info(string_output)

        return flow_dict


    def min_cost_flow(self):
        """
        Calculate a minimum cost flow using an indicator variable in the cost function

        :return routes: the set vehicle routes
        """
        pass #Todo: implement


    def remodel_flow_into_tsp(self, flow_dict):
        """
        Takes a set of flows to be visited and generates a directed graph from them stored in self.tsp_model.
        Flows become nodes with the distances between them corresponding to the distances in the original graph.

        :param flow_dict: dictionary of dictionaries keyed by nodes such that flow_dict[u][v] is the flow edge (u, v).
        """
        self.show_header("Remodelling flow into TSP formulation")
        digraph = nx.DiGraph()
        node_arr = []
        for source, pair in flow_dict.items():
            for target, flow in pair.items():
                if flow != 0:
                    node_str = "{}-{}-{}"
                    digraph.add_node(node_str.format(source,target,0))
                    node_arr.append([source,target,0])
                    for i in range((flow-1)//self.capacity):
                        node_str = "{}-{}-{}"
                        digraph.add_node(node_str.format(source,target,i+1))
                        node_arr.append([source,target,i+1])
        size = len(node_arr)
        for i in range(size):
            for j in range(size):
                if i != j:
                    s_i, t_i, m_i = node_arr[i]
                    s_j, t_j, m_j = node_arr[j]
                    i_str = "{}-{}-{}".format(s_i, t_i, m_i)
                    j_str = "{}-{}-{}".format(s_j, t_j, m_j)
                    if self.model.has_edge(t_i, s_j):
                        edge_weight = self.model.edges[t_i, s_j]['dist']
                    elif t_i == s_j:
                        edge_weight = 0
                    else:
                        self.show_warning("input graph G should be complete (and metric)")
                        break #uncomment next line if G is not complete:
                        # edge_weight = nx.shortest_path_length(G,t_i,s_j,weight='weight')
                    digraph.add_edge(i_str, j_str, weight=edge_weight)
        self.tsp_model = metric_completion(digraph)
        self.show_info('Generated TSP model with {} nodes.'.format(len(self.tsp_model.nodes)))


    def tsp_solver(self, source=None):
        """
        Solves the TSP problem for a single vehicle using the standard NetworkX function.
        Reformulates the solution into a route, stored in self.route.

        :param source: set starting node for tsp tour
        :return route: sequence of nodes to visit
        :return route_cost: the total distance of the route
        """
        self.show_header("Solving TSP. This may take VERY long if upwards of 8 nodes.")
        seq = nx.algorithms.approximation.asadpour_atsp(self.tsp_model, source=source)

        self.show_header("Extracting Route")
        for i in range(len(seq)-1):
            path = seq[i].split('-')
            self.route.append(int(path[0]))
            self.route.append(int(path[1]))

        route_cost = 0
        prev = '0'
        edge_list = []
        for s in self.route:
            step = str(s)
            if self.model.has_edge(prev, step):
                route_cost += self.model.edges[prev, step]['dist']
                edge_list.append((prev,step))
            else:
                self.show_warning("input graph G should be complete (and metric)")
                route_cost = -1
                break # uncomment next line if G is not complete:
                # edge_weight = nx.shortest_path_length(G,t_i,s_j,weight='weight')
            prev = step
        self.show_info("Route: {}\nTotal route distance: {}m".format(self.route, route_cost))
        return self.route, route_cost


    def mtsp_solver(self):
        """
        Solves the TSP problem for a MULTIPLE vehicles... would be nice.
        """
        pass #TODO: research and implement