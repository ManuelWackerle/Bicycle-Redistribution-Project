import networkx as nx
from haversine import haversine, Unit
import csv
from utils import *

def load_graph(graph_name, truncate_after=-1):
    """
         Loads data from a csv file into a NetworkX graph.

         :param graph_name: name of csv file containing graph data
         :param truncate_after: set number of rows to load, or -1 for all.
         :return graph: a NetworkX graph representation of the data
         :return node_data: dictionary of additional node information, bin_id and position
         """
    # munich_lat = [47.8, 48.5]
    # munich_long = [11.1, 12.1]
    # munich_lat = [47.975, 48.33]
    # munich_long = [11.279, 11.825]
    munich_lat = [46.8, 49.5]
    munich_long = [10.1, 13.1]

    graph = nx.Graph()
    data = csv.reader(open('Problem Instances/' + graph_name + '.csv', "r"))
    # writer = csv.writer(open('Problem Instances/sample_graph_05.csv', 'w', newline=''), delimiter=',')
    data_drop = False
    header = next(data)
    node_data = {}
    row_count, total_supply = 0, 0
    # count = 0
    for row in data:
        row_count += 1
        if 0 < truncate_after < row_count:
            break
        n, bin_id, s_str, lat_str, long_str = row
        supply, x, y = int(s_str), float(long_str), float(lat_str)
        total_supply += supply
        if munich_long[0] < x < munich_long[1] and munich_lat[0] < y < munich_lat[1]:
            graph.add_node(n, sup=supply)
            # writer.writerow([str(count), bin_id, s_str, lat_str, long_str])
            # count += 1
            for node, data in node_data.items():
                dist = int(round(haversine((x, y), data['pos'], unit=Unit.METERS))) #Great circle distance between two coordinates on a earth
                graph.add_edge(node, n, dist=dist)
            node_data[n] = {'bin_id': bin_id, 'pos': (x, y)}
        else:
            data_drop = True
    if total_supply != 0:
        print(bcolors.WARNING + "Warning: mismatch in supply and demand values, some algorithms may not work" + bcolors.ENDC)
    if data_drop:
        print(bcolors.WARNING + "Warning: some data points ignored because position is too far out of Munich" + bcolors.ENDC)

    return graph, node_data

