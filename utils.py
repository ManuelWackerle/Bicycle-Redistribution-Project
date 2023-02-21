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
from os.path import exists

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


def display_graph(Graph, node_pos=None, edge_list=None, title=None):
    header = "Graph" if title is None else title
    use_pos = node_pos is not None
    pos = {} if use_pos else None
    labels = {}
    for node, attr in Graph.nodes.items():
        labels[node] = "{}".format(node)  # attr['sup'])
        if use_pos:
            pos[node] = node_pos[node]['pos']
    nx.draw_networkx(Graph, pos=pos, edgelist=edge_list, node_size=300, node_color='#FEECD2', labels=labels)
    plt.title(title)
    plt.draw()
    plt.show()


def visualize_routes(routes, node_data):
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
    # plt.savefig('Saved/routes.png')
    plt.show()


def visualize_routes_go(routes, node_data):
    station_color = 'blue'
    random.seed(0)
    get_colors = lambda n: ["#%06x" % random.randint(0, 0xFFFFFF) for _ in range(n)]
    route_color = get_colors(len(routes))

    x_stations = []
    y_stations = []

    for i in range(len(node_data)):
        x_stations.append(node_data[str(i)]['pos'][0])
        y_stations.append(node_data[str(i)]['pos'][1])
    x_stations = np.array(x_stations)
    y_stations = np.array(y_stations)

    # create a plot
    fig = go.Figure()

    # plot stations

    routes_buttons = []

    for route_num, route in enumerate(routes):
        route = list(map(int, route))
        x_route = []
        y_route = []
        for visit, station in enumerate(route):
            x_route.append(x_stations[station])
            y_route.append(y_stations[station])

        # routes_buttons.append(dict(
        #     args=[ {
        #         'type':'Scattermapbox',
        #         'mode':'lines',
        #         'lon':x_route,
        #         'lat':y_route,
        #         'marker':{'size': 10},
        #         'margin': {'l': 0, 't': 0, 'b': 0, 'r': 0},
        #         'mapbox':{
        #     'center': {'lon': 10, 'lat': 10},
        #     'style': "stamen-terrain",
        #     'center': {'lon': -20, 'lat': -20},
        #     'zoom': 1}
        #     }],
        #
        #     label='Route #' + str(route_num),
        #     method='update'))

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

    # fig.update_layout(
    #     updatemenus=[
    #         dict(
    #             # type="buttons",
    #             buttons=routes_buttons)
    #     ]
    # )
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
    # if not exists(file_path):
    pickle.dump(item, open(file_path, "wb"))
    # else: #Todo: automatically rename file with incrementing number and save
    #     raise Fi/leExistsError


def extract_saved_object(file_name):
    return pickle.load(open("Saved/" + file_name + ".p", "rb"))


def show_improvement_graph(distance_hist, time_hist, operation_hist, ordered_nbhs, change_nbh_name):
    # fig = plt.figure(figsize=(9, 6))
    fig, ax = plt.subplots(figsize=(19, 13))
    # get_colors = lambda n: ["#%06x" % random.randint(0, 0xFFFFFF) for _ in range(n)]
    # colors = get_colors(len(ordered_nbhs))
    colors = ['#F5A623', '#9013FE', '#7ED321', '#4A90E2', '#F8E71C', '#D0021B', 'w']

    distance_hist = [x / 1000 for x in distance_hist]
    operation_dict = {k: [[], []] for k in set(operation_hist)}
    for d, t, o in zip(distance_hist, time_hist, operation_hist):
        operation_dict[o][0].append(t)
        operation_dict[o][1].append(d)

    plt.plot(time_hist, distance_hist, color='lightgray')


    for k,v in operation_dict.items():
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





