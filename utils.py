from matplotlib import rc
import matplotlib.pyplot as plt
import networkx as nx
import pickle
import numpy as np
import random
import plotly.graph_objects as go
import os
import datetime
import time
import os.path
from copy import deepcopy
from operator import itemgetter
import itertools


rc('font', **{'family': 'lmodern', 'serif': ['Latin  Modern'], 'size': 22})
rc('text', usetex=True)

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "lmodern",
    "font.sans-serif": "Latin  Modern",
    "font.size": 24
})


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def visualize_routes(routes, node_data, save_figure=False):
    """
        Plot routing information on the model using matplotlib
        :param routes: an array of routes, each route a sequence of stations visited
        :param node_data: a dictionary of dictionaries of node information, containing at least positional information
        :param save_figure: save the figure in the Saved folder as 'routes_<date_time>.png'
    """
    station_color = 'grey'
    random.seed(0)
    get_colors = lambda n: ["#%06x" % random.randint(0, 0xFFFFFF) for _ in range(n)]
    route_color = get_colors(len(routes))

    for i in range(len(node_data)):
        x, y = node_data[str(i)]['pos']
        plt.scatter(x, y, color=station_color)

    for route_num, route in enumerate(routes):
        for j, station in enumerate(route):
            if j == 0:
                continue
            previous_station = route[j]
            current_station = route[j - 1]
            x_line = [node_data[str(previous_station)]['pos'][0], node_data[str(current_station)]['pos'][0]]
            y_line = [node_data[str(previous_station)]['pos'][1], node_data[str(current_station)]['pos'][1]]
            plt.plot(x_line, y_line, color=route_color[route_num], zorder=-1)
    plt.axis('equal')

    if save_figure:
        now = datetime.datetime.now()
        print('Plot generated: ' + now.strftime("%d-%m-%Y_%H-%M-%S"))
        root = os.path.dirname(os.path.abspath(os.getcwd()))
        folder = os.path.join(root, 'Saved', 'plots')
        plt.savefig(folder + '/routes_' + now.strftime("%d-%m-%y_%H-%M-%S"))
        time.sleep(1)

    plt.show()


def visualize_routes_go(routes, node_data):
    """
        Plot routing information in a browser window using go
        :param routes: an array of routes, each route a sequence of stations visited
        :param node_data: a dictionary of dictionaries of node information, containing at least positional information
    """
    random.seed(0)
    x_stations = []
    y_stations = []

    for i in range(len(node_data)):
        x_stations.append(node_data[str(i)]['pos'][0])
        y_stations.append(node_data[str(i)]['pos'][1])
    x_stations = np.array(x_stations)
    y_stations = np.array(y_stations)

    fig = go.Figure()

    for route_num, route in enumerate(routes):
        route = list(map(int, route))
        x_route = []
        y_route = []
        for visit, station in enumerate(route):
            x_route.append(x_stations[station])
            y_route.append(y_stations[station])

        fig.add_trace(go.Scattermapbox(
            mode="lines",
            lon=x_route,
            lat=y_route,
            line=dict(width=3),
            marker={'size': 10},
            name='Route #' + str(route_num)))

    fig.add_trace(go.Scattermapbox(
        mode="markers",
        lon=x_stations,
        lat=y_stations,
        marker={'size': 12,
                'color': 'black'},
        name='Stations'))

    fig.update_layout(
        margin={'l': 0, 't': 0, 'b': 0, 'r': 0},
        mapbox={
            'center': {'lon': (max(x_stations) + min(x_stations)) / 2, 'lat': (max(y_stations) + min(y_stations)) / 2},
            'style': "stamen-terrain",
            'zoom': 11})

    fig.show()


def nodes_data_as_numpy(graph: nx.Graph, data_str):
    nodes = []
    for n, data in graph.nodes.items():
        nodes.append(data[data_str])
    return np.array(nodes)


def edge_data_as_numpy(graph: nx.Graph, data_str):
    edges = []
    for e, data in graph.edges.items():
        edges.append(data[data_str])
    return np.array(edges)


def save_object(item, save_as="pickle_file"):
    file_path = "Saved/" + save_as + ".p"
    pickle.dump(item, open(file_path, "wb"))


def extract_saved_object(file_name):
    return pickle.load(open("Saved/" + file_name + ".p", "rb"))


def show_improvement_graph(distance_hist, time_hist, operation_hist, ordered_nbhs, change_nbh_name):
    """
        Plot distance improvement per computation time for each operator during a VNS run
        :param distance_hist: array of the distances at each iteration
        :param time_hist: array of time points at the end of each iteration
        :param operation_hist: array of which operator was used at each iteration
        :param ordered_nbhs: array of the set of operators used
        :param change_nbh_name: type of neighbourhood change method used during the VNS run
    """

    fig, ax = plt.subplots(figsize=(19, 13))
    colors = ['#F5A623', '#9013FE', '#7ED321', '#4A90E2', '#F8E71C', '#D0021B', 'w']

    distance_hist = [x / 1000 for x in distance_hist]
    operation_dict = {k: [[], []] for k in set(operation_hist)}
    for d, t, o in zip(distance_hist, time_hist, operation_hist):
        operation_dict[o][0].append(t)
        operation_dict[o][1].append(d)

    plt.plot(time_hist, distance_hist, color='lightgray')

    for k, v in operation_dict.items():
        if k >= len(ordered_nbhs):
            continue
        plt.plot(v[0], v[1], color=colors[k], marker='o', linestyle='None', label=ordered_nbhs[k].__name__)

    # for i in range(len(time_hist) - 2):
    #     plt.plot([time_hist[i], time_hist[i + 1]], [distance_hist[i], distance_hist[i]],
    #              color=colors[operation_hist[i]], lw=4)
    #     plt.plot([time_hist[i+1], time_hist[i+1]], [distance_hist[i], distance_hist[i+1]],
    #              color=colors[operation_hist[i]], lw=4)

    plt.xlabel('Computational time [s]')
    plt.ylabel('Distance [km]')
    plt.legend(title='Operation where:')
    plt.title('improvement of distance with ' + change_nbh_name + ' neighborhood search')
    now = datetime.datetime.now()
    print('Plot generated: ' + now.strftime("%d-%m-%Y_%H-%M-%S"))
    ax.set_position([0.1, 0.1, 0.8, 0.8])
    root = os.path.dirname(os.path.abspath(os.getcwd()))
    folder = os.path.join(root, 'Saved', 'plots')
    fig.savefig(folder + '/improvement_gr_' + now.strftime("%d-%m-%y_%H-%M-%S"))
    time.sleep(1)
    plt.show()


def fix_balance_after_removal_by_combination(removal_sup_total, removal_sup_nodes, removal_dem_total, removal_dem_nodes):
    """
    Adjusts the balance of supply and demand after removing nodes by combination.

    Args:
        removal_sup_total (list): List of supply values for the nodes being removed.
        removal_sup_nodes (list): List of supply nodes being removed.
        removal_dem_total (list): List of demand values for the nodes being removed.
        removal_dem_nodes (list): List of demand nodes being removed.

    Returns:
        tuple: A tuple containing the adjusted removal_sup_nodes, removal_dem_nodes, and a flag indicating if a balance was found.
    """
    # TODO refactor
    find = True
    diff = sum(removal_sup_total) + sum(removal_dem_total)
    if diff > 0:
        find = False
        try:
            i = removal_sup_total.index(diff)
            removal_sup_nodes.pop(i)
            removal_sup_total.pop(i)
            find = True
        except ValueError:
            N = len(removal_sup_nodes)
            for c in reversed(range(2, N+1)):
                for subset_idxes in itertools.combinations(list(range(N)), c):
                    if diff == sum(itemgetter(*subset_idxes)(removal_sup_total)):
                        find = True
                        removal_sup_nodes = [x for i, x in enumerate(removal_sup_nodes) if i not in subset_idxes]
                        removal_sup_total = [x for i, x in enumerate(removal_sup_total) if i not in subset_idxes]
                        break
                if find:
                    break
    elif diff < 0:
        find = False
        try:
            i = removal_dem_total.index(diff)
            removal_dem_total.pop(i)
            removal_dem_total.pop(i)
            find = True
        except ValueError:
            N = len(removal_dem_nodes)
            for c in reversed(range(2, N+1)):
                for subset_idxes in itertools.combinations(list(range(N)), c):
                    if diff == sum(itemgetter(*subset_idxes)(removal_dem_total)):
                        find = True
                        removal_dem_nodes = [x for i, x in enumerate(removal_dem_nodes) if i not in subset_idxes]
                        removal_dem_total = [x for i, x in enumerate(removal_dem_total) if i not in subset_idxes]
                        break
                if find:
                    break
    return removal_sup_nodes, removal_dem_nodes, find


def fix_balance_after_removal_by_inside_reduction(removal_sup_nodes, removal_dem_nodes, sup_nodes_outside, dem_nodes_outside, graph, depot_fixing):
    """This function adjusts the balance in a graph after the removal of nodes based on the reduction strategy. It takes the following parameters:

    Args:
        removal_sup_total (list): The supply to be removed for each node in the graph.
        removal_sup_nodes (list): Node identifiers corresponding to the supply nodes to be removed.
        removal_dem_total (list): The total demand to be removed for each node in the graph.
        removal_dem_nodes (list): Node identifiers corresponding to the demand nodes to be removed.
        graph: The graph object representing the network.
    Returns
        removal_sup_nodes (list): Updated node identifiers corresponding to the supply nodes to be removed.
        removal_dem_nodes (list): Updated node identifiers corresponding to the demand nodes to be removed.
    
    It adjusts the balance of supply and demand by choosing as much node as possible and reducing supply/demand at the end.
    Example:
        Suppose
            supply nodes' supply: [1, 1, 1, 2, 2] --> total supply 7
            demand nodes' demand: [-2, -2, -2]    --> total demand -6.
        Since total supply is (in absolute value) larger than total demand, choose all demand nodes.
        Then choose as much supply nodes as possible while total supply is 6 (=abs(total demand)) by
            [1, ] -> total 1 < 6, so choose first node from supply
            [1, 1, ] -> total 2 < 6, so choose second node from supply
            [1, 1, 1, ] -> total 3 < 6, so choose third node from supply
            [1, 1, 1, 2] -> total 5 < 6, so choose fourth node from supply
            [1, 1, 1, 2, 2] -> total 7 > 6, so choose fifth node from supply, but reduce supply
                => [1, 1, 1, 2, 1] -> total 6=6
        return nodes with supply/demand [1, 1, 1, 2, 1] + [-2, -2, -2]
    """
    # TODO refactor
    removal_idxes = []
    removal_sup_total = [graph.nodes[node]['sup'] for node in removal_sup_nodes]
    removal_dem_total = [graph.nodes[node]['sup'] for node in removal_dem_nodes]
    diff = sum(removal_sup_total) + sum(removal_dem_total) + depot_fixing
    tmp = 0
    if diff > 0:
        max_val = abs(sum(removal_dem_total))
        removal_sup_total, removal_sup_nodes = zip(*sorted(zip(removal_sup_total, removal_sup_nodes)))
        for i in range(len(removal_sup_nodes)):
            if removal_sup_total[i] <= max_val:
                max_val -= removal_sup_total[i]
                removal_idxes.append(i)
            else:
                tmp = max_val
                graph.nodes[removal_sup_nodes[i]]['sup'] -= max_val
                break
        removal_sup_nodes = [x for j, x in enumerate(removal_sup_nodes) if j in removal_idxes]
        removal_sup_total = [x for j, x in enumerate(removal_sup_total) if j in removal_idxes]
    elif diff < 0:
        max_val = sum(removal_sup_total)
        removal_dem_total, removal_dem_nodes = zip(*sorted(zip(removal_dem_total, removal_dem_nodes), reverse=True))
        for i in range(len(removal_dem_nodes)):
            if abs(removal_dem_total[i]) <= max_val:
                max_val -= abs(removal_dem_total[i])
                removal_idxes.append(i)
            else:
                tmp = -max_val
                graph.nodes[removal_dem_nodes[i]]['sup'] += max_val
                break
        removal_dem_nodes = [x for j, x in enumerate(removal_dem_nodes) if j in removal_idxes]
        removal_dem_total = [x for j, x in enumerate(removal_dem_total) if j in removal_idxes]
    assert sum(removal_dem_total) + sum(removal_sup_total) + tmp == 0
    N = min(len(sup_nodes_outside), len(dem_nodes_outside))
    sup_nodes_outside = sup_nodes_outside[:N]
    dem_nodes_outside = dem_nodes_outside[:N]
    return removal_sup_nodes, removal_dem_nodes, sup_nodes_outside, dem_nodes_outside


def fix_balance_after_removal_by_outside_increase(sup_nodes_inside, dem_nodes_inside, sup_nodes_outside, dem_nodes_outside, graph, delta, _depot_fixing):
    total_sup_inside = sum(graph.nodes[node]['sup'] for node in sup_nodes_inside) + delta * len(sup_nodes_outside)
    total_dem_inside = sum(graph.nodes[node]['sup'] for node in dem_nodes_inside) - delta * len(dem_nodes_outside)
    diff = total_sup_inside + total_dem_inside + _depot_fixing
    # delta = 2*delta
    if diff > 0:
        for node in sup_nodes_outside:
            if diff >= delta:
                diff -= delta
                graph.nodes[node]['sup'] += delta
            else:
                graph.nodes[node]['sup'] += diff
                diff = 0
                break
    elif diff < 0:
        diff = -diff
        for node in dem_nodes_outside:
            if diff >= delta:
                diff -= delta
                graph.nodes[node]['sup'] -= delta
            else:
                graph.nodes[node]['sup'] -= diff
                diff = 0
                break
    return sup_nodes_inside, dem_nodes_inside, sup_nodes_outside, dem_nodes_outside


def update_problem_with_all_window(problem, delta=0, balance_fix='outside'):
    """Apply window and remove node within the window while keeping total imbalance
       For nodes within the window --> the nodes are removed.
       For nodes outside the window --> the demand/supply is reduced by window size.
    """
    if not delta > 0:
        # if we do not apply window, just return the original graph.
        return True, problem.model
    # TODO refactor
    sup_nodes_inside = []
    dem_nodes_inside = []
    sup_nodes_outside = []
    dem_nodes_outside = []
    removal_nodes = []
    graph = problem.model
    depot_fixing = 0
    for node in graph.nodes:
        if abs(graph.nodes[node]['sup']) <= delta:
            if node == problem.depot:
                depot_fixing = graph.nodes[node]['sup']
                graph.nodes[node]['sup'] = 0
            elif graph.nodes[node]['sup'] > 0:
                sup_nodes_inside.append(node)
            elif graph.nodes[node]['sup'] < 0:
                dem_nodes_inside.append(node)
            else:
                removal_nodes.append(node)
        elif graph.nodes[node]['sup'] > delta:
            sup_nodes_outside.append(node)
        elif graph.nodes[node]['sup'] < -delta:
            dem_nodes_outside.append(node)

    if balance_fix == 'outside':
        sup_nodes_inside, dem_nodes_inside, sup_nodes_outside, dem_nodes_outside = fix_balance_after_removal_by_outside_increase(sup_nodes_inside, dem_nodes_inside, sup_nodes_outside, dem_nodes_outside, graph, delta, depot_fixing)
    elif balance_fix == 'inside':
        sup_nodes_inside, dem_nodes_inside, sup_nodes_outside, dem_nodes_outside = fix_balance_after_removal_by_inside_reduction(sup_nodes_inside, dem_nodes_inside, sup_nodes_outside, dem_nodes_outside, graph, depot_fixing)
    else:
        raise Exception("unexpected balance fixing method.")

    removal_nodes += sup_nodes_inside
    removal_nodes += dem_nodes_inside
    graph.remove_nodes_from(removal_nodes)

    for node in sup_nodes_outside:
        graph.nodes[node]['sup'] -= delta
    
    for node in dem_nodes_outside:
        graph.nodes[node]['sup'] += delta

    return True, graph


def update_problem_with_partial_window(graph, delta=0):
    """Apply window to nodes which has supply/node within the window while keeping total imbalance
       For nodes within the window --> the nodes are removed.
       For nodes outside the window --> keep the demand/supply
    """
    # TODO refactor
    removal_sup_nodes = []
    removal_sup_total = []
    removal_dem_nodes = []
    removal_dem_total = []
    removal_nodes = []
    for node in graph.nodes:
        if node == '0':
            # keep depot
            continue
        elif abs(graph.nodes[node]['sup']) <= delta:
            if graph.nodes[node]['sup'] > 0:
                removal_sup_nodes.append(node)
                removal_sup_total.append(graph.nodes[node]['sup'])
            elif graph.nodes[node]['sup'] < 0:
                removal_dem_nodes.append(node)
                removal_dem_total.append(graph.nodes[node]['sup'])
            else:
                removal_nodes.append(node)
    
    removal_sup_nodes, removal_dem_nodes, find = fix_balance_after_removal_by_combination(removal_sup_total, removal_sup_nodes, removal_dem_total, removal_dem_nodes)

    if find:
        removal_nodes += removal_sup_nodes
        removal_nodes += removal_dem_nodes
        graph.remove_nodes_from(removal_nodes)

    return find, graph


def get_graph_after_rebalance(problem):
    # TODO refactor
    """Given routes and loading instructions, return the graph of which supply/demand is updated.
    """
    vehicles = problem.vehicles
    auxiliary_graph = deepcopy(problem.model)
    for vehicle in vehicles:
        prev = 0
        m = len(vehicle.route())-1
        auxiliary_graph.nodes[vehicle.route()[prev]]['sup'] -= vehicle.loads()[prev]
        if len(vehicle.route()) != len(vehicle.loads()) + 1:
            m = min(m, len(vehicle.loads()))
            print(f"len routes {len(vehicle.route())}, len loads {len(vehicle.loads())}")
        for curr in range(1, m):
            auxiliary_graph.nodes[vehicle.route()[curr]]['sup'] -= vehicle.loads()[curr] - vehicle.loads()[prev]
            prev = curr
        auxiliary_graph.nodes[vehicle.route()[curr+1]]['sup'] += vehicle.loads()[curr]
    return auxiliary_graph


def assert_total_imbalance(problem):
    """
    """
    return sum(problem.model.nodes[node]['sup'] for node in problem.model.nodes)


def get_total_imbalance_from_aux_graph(aux_graph, depot_node):
    """Given graph given by get_graph_after_rebalance function, return total supply/demand.
    """
    return sum(abs(aux_graph.nodes[node]['sup']) for node in aux_graph.nodes if node != depot_node)


def get_max_imbalance_from_aux_graph(aux_graph, depot_node):
    """Given graph given by get_graph_after_rebalance function, return max supply/demand. (to check the windowing)
    """
    abs_val = [abs(aux_graph.nodes[node]['sup']) for node in aux_graph.nodes if node != depot_node]
    return max(abs_val)

