import os.path

import networkx as nx
from haversine import haversine, Unit
import csv
from utils import bcolors

import random
import pandas as pd
from itertools import combinations

munich_lat = [47.8, 48.5]
munich_long = [11.1, 12.1]


def load_graph(graph_name, path=None, truncate_after=-1):
    """
         Loads data from a csv file into a NetworkX graph.

         :param graph_name: name of csv file containing graph data
         :param truncate_after: set number of rows to load, or -1 for all.
         :return graph: a NetworkX graph representation of the data
         :return node_data: dictionary of additional node information, bin_id and position
         """

    graph = nx.Graph()
    if path is None:
        data = csv.reader(open('Problem Instances/' + graph_name + '.csv', "r"))
    else:
        data = csv.reader(open(os.path.join(path, graph_name + '.csv'), "r"))

    data_drop = False
    header = next(data)
    node_data = {}
    row_count, total_supply = 0, 0
    for row in data:
        row_count += 1
        if 0 < truncate_after < row_count:
            break
        n, bin_id, s_str, lat_str, long_str = row
        supply, x, y = int(s_str), float(long_str), float(lat_str)
        total_supply += supply
        if munich_long[0] < x < munich_long[1] and munich_lat[0] < y < munich_lat[1]:
            graph.add_node(n, sup=supply)
            for node, dict in node_data.items():
                dist = int(round(haversine((x,y), dict['pos'], unit=Unit.METERS))) #Great circle distance between two coordinates on a earth
                graph.add_edge(node, n, dist=dist)
            node_data[n] = {'bin_id': bin_id, 'pos': (x, y)}
        else:
            data_drop = True
    if total_supply != 0:
        print(bcolors.WARNING + "Warning: mismatch in supply and demand values, some algorithms may not work" + bcolors.ENDC)
    if data_drop:
        print(bcolors.WARNING + "Warning: some data points ignored because position is too far out of Munich" + bcolors.ENDC)

    return graph, node_data


# WARNING: only use for very small graphs, otherwise it takes too long.
# def load_random_sub_graph(graph_name, number_nodes, truncate_after=-1):
#     """
#          Selects random subset of csv data points to create subgraphs with a certain number of nodes
#
#          :param graph_name: name of csv file containing graph data
#          :param truncate_after: set number of rows to load, or -1 for all.
#          :return graph: a NetworkX graph representation of the data
#          :return node_data: dictionary of additional node information, bin_id and position
#          """
#
#     graph = nx.Graph()
#     filename = 'Problem Instances/' + graph_name + '.csv'
#
#     data = pd.read_csv(filename)
#     required_total_supply = 0
#
#     rows =[0]
#
#     index_delta_pairs = [(i, d) for i, d in zip(data["number"], data["delta"])]
#     candidates = combinations(index_delta_pairs, number_nodes)
#     for c in candidates:
#         if sum(d for i, d in c) == required_total_supply:
#             candidate = c
#
#     for row, delta in candidate:
#         rows.append(row)
#
#     data_drop = False
#     node_data = {}
#     row_count, total_supply = 0, 0
#     for row_index in rows:
#         row_count += 1
#         if 0 < truncate_after < row_count:
#             break
#
#         row = data.iloc[row_index]
#         n = str(row.number)
#         bin_id = str(row.id)
#         s_str = str(row.delta)
#         lat_str = str(row.x_coord)
#         long_str = str(row.y_coord)
#
#         supply, x, y = int(s_str), float(long_str), float(lat_str)
#         total_supply += supply
#         if munich_long[0] < x < munich_long[1] and munich_lat[0] < y < munich_lat[1]:
#             graph.add_node(n, sup=supply)
#             for node, dict in node_data.items():
#                 dist = int(round(haversine((x,y), dict['pos'], unit=Unit.METERS))) #Great circle distance between two coordinates on a earth
#                 graph.add_edge(node, n, dist=dist)
#             node_data[n] = {'bin_id': bin_id, 'pos': (x, y)}
#         else:
#             data_drop = True
#     if total_supply != 0:
#         print(bcolors.WARNING + "Warning: mismatch in supply and demand values, some algorithms may not work" + bcolors.ENDC)
#     if data_drop:
#         print(bcolors.WARNING + "Warning: some data points ignored because position is too far out of Munich" + bcolors.ENDC)
#
#     return graph, node_data
