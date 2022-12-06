import matplotlib.pyplot as plt
import networkx as nx
import pickle
import numpy as np
from os.path import exists

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
        labels[node] = "{}".format(node) #attr['sup'])
        if use_pos:
            pos[node] = node_pos[node]['pos']
    nx.draw_networkx(Graph, pos=pos, edgelist=edge_list, node_size=300, node_color='#FEECD2', labels=labels)
    plt.title(title)
    plt.draw()
    plt.show()


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


def dict_data_as_numpy(dict: dict, data_str):
    arr = []
    for e, data in dict.items():
        arr.append(data[data_str])
    return np.array(arr)


def reformat_pos(graph: nx.Graph):
    x_list = nodes_data_as_numpy(graph, "")


def metric_completion(graph):
    G_metric = nx.DiGraph()
    G_metric.add_nodes_from(graph.nodes.items())
    for n in graph.nodes:
        for m in graph.nodes:
            if n != m:
                dist = nx.shortest_path_length(graph, n, m, weight='weight')
                G_metric.add_edge(n, m, weight=dist)
    return G_metric


def print_graph(Graph):
    print("Nodes: {}".format(Graph.nodes.data()))
    print("Edges: {}".format(Graph.edges.data()))


def save_object(item, save_as="pickle_file"):
    file_path = "Saved/" + save_as + ".p"
    # if not exists(file_path):
    pickle.dump(item, open(file_path, "wb"))
    # else: #Todo: automatically rename file with incrementing number and save
    #     raise FileExistsError


def extract_saved_object(file_name):
    return pickle.load( open( "Saved/" + file_name + ".p", "rb"))

