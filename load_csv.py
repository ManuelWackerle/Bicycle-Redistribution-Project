import networkx as nx
from haversine import haversine, Unit
import csv
from utils import *
import numpy as np

def load_graph(graph_name, use_adjacency_matrix=True, truncate_after=-1):
    """
         Loads data from a csv file into a NetworkX graph.

         :param graph_name: name of csv file containing graph data
         :param truncate_after: set number of rows to load, or -1 for all.
         :return graph: a NetworkX graph representation of the data
         :return node_data: dictionary of additional node information, bin_id and position
         """
    munich_lat = [47.8, 48.5]
    munich_long = [11.1, 12.1]
    # munich_lat = [47.975, 48.33]
    # munich_long = [11.279, 11.825]
    # munich_lat = [46.8, 49.5]
    # munich_long = [10.1, 13.1]

    graph = nx.Graph()
    adjacency_dict = {}

    if use_adjacency_matrix:
        edge_data = csv.reader(open('MVG Code/adjacency_matrix.csv', 'r'))
        v_ids = next(edge_data)
        for u_pairs in edge_data:
            u_id = u_pairs[0]
            adjacency_dict[u_id] = {}
            for v_index in range(1, len(u_pairs)):
                v_id = v_ids[v_index]
                adjacency_dict[u_id][v_id] = float(u_pairs[v_index])

    data = csv.reader(open('Problem Instances/' + graph_name + '.csv', "r"))

    data_drop = False
    node_data = {}
    count, row_count, total_supply = 0, 0, 0

    next(data)
    for row in data:
        row_count += 1
        if 0 < truncate_after < row_count:
            break
        _, bin_id, supply_str, lat_str, long_str = row
        supply, x, y = int(supply_str), float(long_str), float(lat_str)
        if munich_long[0] < x < munich_long[1] and munich_lat[0] < y < munich_lat[1] and bin_id in adjacency_dict:
            total_supply += supply
            graph.add_node(str(count), sup=supply)
            for node, data in node_data.items():
                if use_adjacency_matrix:
                    dist = int(round(adjacency_dict[bin_id][data['bin_id']]))
                else:
                    dist = int(round(haversine((x, y), data['pos'], unit=Unit.METERS))) #Great circle distance between two coordinates on a earth
                graph.add_edge(node, str(count), dist=dist)
            node_data[str(count)] = {'bin_id': bin_id, 'pos': (x, y)}
            count += 1
        else:
            data_drop = True

    if total_supply != 0:
        print(bcolors.WARNING + "Warning: mismatch in supply and demand values {}, some algorithms may not work".format(total_supply) + bcolors.ENDC)
    if data_drop:
        print(bcolors.WARNING + "Warning: some data points ignored because position is too far out of Munich" + bcolors.ENDC)

    return graph, node_data


def load_subset_from_ordered_nodes(nodes=100, centeredness=3):
    """
         Loads a subset of valid nodes as a NetworkX graph.

         :param nodes: number of nodes.
         :param centeredness: 0 - not centred, 20 - heavily centred. The centeredness determines if central nodes are
                              prefered over exterior nodes, this will also determine the graph density
         :return graph: a NetworkX graph representation of the data
         :return node_data: dictionary of additional node information, bin_id and position
    """

    graph = nx.Graph()
    adjacency_dict = {}
    node_data = {}

    #Load node data
    data = csv.reader(open('Problem Instances/ordered_nodes.csv', "r"))
    next(data) #skip header line
    depot_node = next(data)
    source_nodes, sink_nodes = [], []
    for row in data:
        bin_id, supply_str, std_dev_str, lat_str, long_str = row
        new_row = [bin_id, int(supply_str), int(std_dev_str), float(long_str), float(lat_str)]
        if int(supply_str) > 0:
            source_nodes.append(new_row)
        else:
            sink_nodes.append(new_row)
    ordered_nodes = source_nodes + sink_nodes
    total_source = len(source_nodes)
    total_nodes = len(ordered_nodes)


    #Load edge data
    edge_data = csv.reader(open('MVG Code/adjacency_matrix.csv', 'r'))
    v_ids = next(edge_data)
    for u_pairs in edge_data:
        u_id = u_pairs[0]
        adjacency_dict[u_id] = {}
        for v_index in range(1, len(u_pairs)):
            v_id = v_ids[v_index]
            adjacency_dict[u_id][v_id] = float(u_pairs[v_index])

    #Generate Random Graph
    count, total_supply = 0, 0
    base = abs(centeredness)/32 + 1
    base = 2 if base > 2 else base #ignore large bases to avoid disappearing values
    sources = nodes//2
    sinks = nodes - sources

    indexes = np.arange(total_source)
    weights = np.array([base ** (-i) for i in range(total_source)])
    weights = weights/weights.sum() #normalise weights
    random_sources = np.random.choice(indexes, sources, replace=False, p=weights)

    indexes = np.arange(total_source, total_nodes)
    weights = np.array([base ** (-i) for i in range(total_source, total_nodes)])
    weights = weights/weights.sum() #normalise weights
    random_sinks = np.random.choice(indexes, sinks, replace=False, p=weights)

    random_indexes = np.concatenate((random_sources, random_sinks))

    bin_id, _, _, lat_str, long_str = depot_node
    graph.add_node(str(0), sup=0)  # add depot node with zero load
    node_data[str(0)] = {'bin_id': bin_id, 'pos': (float(long_str), float(lat_str))}

    for i in random_indexes:
        count += 1
        bin_id, mean_supply, std_dev, x, y = ordered_nodes[i]
        supply = int(round(np.random.normal(mean_supply, std_dev)))
        supply = -1 if supply == 0 else supply
        total_supply += supply
        graph.add_node(str(count), sup=supply)
        for node, data in node_data.items():
            avg_dist = (adjacency_dict[bin_id][data['bin_id']] + adjacency_dict[data['bin_id']][bin_id])/2
            graph.add_edge(node, str(count), dist=avg_dist)
        node_data[str(count)] = {'bin_id': bin_id, 'pos': (x, y)}

    #Fix balancing
    imbalance = total_supply
    fill = 1 if imbalance < 0 else -1
    while imbalance != 0:
        node_idx = str(np.random.randint(1, nodes - 1))
        graph.nodes[node_idx]['sup'] += fill
        imbalance += fill
        if graph.nodes[node_idx]['sup'] == 0 and imbalance != 0:
            graph.nodes[node_idx]['sup'] += fill
            imbalance += fill

    return graph, node_data
