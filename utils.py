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


def update_problem_with_window(graph, delta=0):
    """apply window and remove node within the window while keeping total imbalance
    """
    removal_sup_nodes = []
    removal_sup_total = []
    removal_dem_nodes = []
    removal_dem_total = []
    removal_nodes = []
    for node in graph.nodes:
        if node == '0':
            # deep depot
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
        elif graph.nodes[node]['sup'] > delta:
            graph.nodes[node]['sup'] -= delta
        else:
            graph.nodes[node]['sup'] += delta
    diff = sum(removal_sup_total) + sum(removal_dem_total)
    if diff > 0:
        try:
            i = removal_dem_total.index(diff)
            removal_sup_nodes.pop(i)
            removal_sup_total.pop(i)
        except ValueError:
            N = len(removal_sup_nodes)
            find = False
            for c in reversed(range(2, N+1)):
                for subset_idxes in itertools.combinations(list(range(N)), c):
                    if diff == sum(itemgetter(*subset_idxes)(removal_sup_total)):
                        find = True
                        removal_sup_nodes = [x for i, x in enumerate(removal_sup_nodes) if i not in subset_idxes]
                        removal_sup_total = [x for i, x in enumerate(removal_sup_total) if i not in subset_idxes]
                        break
                if find:
                    break
            if not find:
                print("could not find fully balanced")
                return graph
    elif diff < 0:
        try:
            i = removal_dem_total.index(diff)
            removal_sup_nodes.pop(i)
            removal_sup_total.pop(i)
        except ValueError:
            N = len(removal_dem_nodes)
            find = False
            for c in reversed(range(2, N+1)):
                for subset_idxes in itertools.combinations(list(range(N)), c):
                    if diff == sum(itemgetter(*subset_idxes)(removal_dem_total)):
                        find = True
                        removal_dem_nodes = [x for i, x in enumerate(removal_dem_nodes) if i not in subset_idxes]
                        removal_dem_total = [x for i, x in enumerate(removal_dem_total) if i not in subset_idxes]
                        break
                if find:
                    break
            if not find:
                print("could not find fully balanced")
                return graph

    removal_nodes += removal_sup_nodes
    removal_nodes += removal_dem_nodes
    graph.remove_nodes_from(removal_nodes)

    return graph


def get_graph_after_rebalance(problem):
    """get original graph, apply loading and get graph after rebalance
    """
    graph = problem.model
    vehicles = problem.vehicles
    auxiliary_graph = deepcopy(graph)
    for vehicle in vehicles:
        prev = 0
        for curr in range(1, len(vehicle.route())-1):
            auxiliary_graph.nodes[vehicle.route()[curr]]['sup'] -= vehicle.loads()[curr] - vehicle.loads()[prev]
            prev = curr
    return auxiliary_graph


def get_total_imbalance_from_aux_graph(aux_graph):
    total = 0
    for node in aux_graph.nodes:
        total += abs(aux_graph.nodes[node]['sup'])
    return total
