import networkx as nx
from haversine import haversine, Unit
import csv
from utils import bcolors


def load_graph(graph_name, truncate_after=-1, flip_demands=False):
    """
         Loads data from a csv file into a NetworkX graph.

         :param graph_name: name of csv file containing graph data
         :param truncate_after: set number of rows to load, or -1 for all.
         :return graph: a NetworkX graph representation of the data
         :return node_data: dictionary of additional node information, bin_id and position
         """
    graph = nx.Graph()
    data = csv.reader(open('Problem Instances/' + graph_name + '.csv', "r"))
    header = next(data)
    node_data = {}
    row_count = 0
    total_supply = 0
    for row in data:
        row_count += 1
        if 0 < truncate_after < row_count:
            break
        n, bin_id, s_str, x_str, y_str = row
        supply, x, y = int(s_str), float(x_str), float(y_str)
        total_supply += supply
        graph.add_node(n, sup=supply)
        for node, dict in node_data.items():
            dist = int(round(haversine((x,y), dict['pos'], unit=Unit.METERS))) #Great circle distance between two coordinates on a earth
            graph.add_edge(node, n, dist=dist)
        node_data[n] = {'bin_id': bin_id, 'pos': (x, y)}
    if total_supply != 0:
        print(bcolors.WARNING + "Warning: mismatch in supply and demand values, some algorithms may not work" + bcolors.ENDC)

    return graph, node_data


