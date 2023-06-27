from utils import *
import csv
from haversine import haversine, Unit


def load_graph(graph_name, location='muc', use_adjacency_matrix=True, truncate_after=-1):
    """
         Loads data from a csv file into a NetworkX graph.

         :param graph_name: name of csv file containing graph data
         :param location: 'muc' for munich data 'nyc' for new york data
         :param use_adjacency_matrix: set True for travelling times, false for straight line distances
         :param truncate_after: set number of rows to load, or -1 for all.
         :return graph: a NetworkX graph representation of the data
         :return node_data: dictionary of additional node information, bin_id and position
         :return depot: the depot node, usually 0
         """

    munich_lat = [47.975, 48.33]  # Use to filter out outliers in the data
    munich_long = [11.279, 11.825]

    # Adapt the following code if file location changes or used on a different system
    root = os.path.dirname(os.path.abspath(os.getcwd()))
    folder_mvg = os.path.join(root, 'MVG Code/')
    folder_p_instances = os.path.join(root, 'Problem Instances/')

    adjacency_dict = {}
    if use_adjacency_matrix:
        graph = nx.DiGraph()
        edge_data = csv.reader(open(folder_mvg + 'adjacency_matrix_' + location + '.csv', 'r'))
        adjacency_dict = _load_edge_data_into_dict(edge_data, adjacency_dict)
    else:
        graph = nx.Graph()

    data = csv.reader(open(folder_p_instances + graph_name + '.csv', "r"))
    data_drop = False
    node_data = {}
    count, row_count, total_supply = 0, 0, 0
    depot = 0  # We define node 0 as the depot

    next(data)
    for row in data:
        row_count += 1
        if 0 < truncate_after < row_count:
            break
        _, bin_id, supply_str, lat_str, long_str = row
        if bin_id == 'depot':
            depot = str(count)
        supply, x, y = int(round(float(supply_str))), float(long_str), float(lat_str)

        if munich_long[0] < x < munich_long[1] and munich_lat[0] < y < munich_lat[1] \
                and not use_adjacency_matrix or bin_id in adjacency_dict:
            total_supply += supply
            graph.add_node(str(count), sup=supply)
            graph.add_edge(str(count), str(count), dist=0)
            node_data[str(count)] = {'bin_id': bin_id, 'pos': (x, y)}
            for node, attr in node_data.items():
                if use_adjacency_matrix:
                    graph.add_edge(node, str(count), dist=int(round(adjacency_dict[attr['bin_id']][bin_id])))
                    graph.add_edge(str(count), node, dist=int(round(adjacency_dict[bin_id][attr['bin_id']])))
                else:  # Otherwise use great circle distance between two coordinates on a earth
                    dist = int(round(haversine((x, y), attr['pos'], unit=Unit.METERS)))
                    graph.add_edge(node, str(count), dist=dist)
            count += 1
        else:
            data_drop = True

    if total_supply != 0:
        print(bcolors.WARNING
              + "Warning: mismatch in supply and demand values {}, some algorithms may not work".format(total_supply)
              + bcolors.ENDC)
    if data_drop:
        print(bcolors.WARNING
              + "Warning: some data points ignored because position is too far out of Munich"
              + bcolors.ENDC)

    return graph, node_data, depot


def load_subset_from_ordered_nodes(nodes, centeredness=5, directed=True, randomness=True):
    """
         Loads a subset of valid nodes from the Munich dataset as a NetworkX graph.

         :param nodes: number of nodes.
         :param centeredness: 0 - not centred, 20 - heavily centred. The centeredness determines if central nodes are
                              preferred over exterior nodes. 0 = nodes chosen uniformly at random.
         :param directed: set if the graph is directed or not
         :param randomness: setting to false will set a fixed seed for reproducibility.
         :return graph: a NetworkX graph representation of the data
         :return node_data: dictionary of additional node information, bin_id and position
    """
    if not randomness:
        np.random.seed(1)

    if directed:
        graph = nx.DiGraph()
    else:
        graph = nx.Graph()

    adjacency_dict = {}
    node_data = {}

    # Load node data
    root = os.path.dirname(os.path.abspath(os.getcwd()))
    folder = os.path.join(root, 'Problem Instances')
    data = csv.reader(open(folder + '/ordered_nodes_with_ff_ratio.csv', "r"))
    next(data)  # skip header line
    depot_node = next(data)
    source_nodes, sink_nodes = [], []
    for row in data:
        bin_id, supply_str, std_dev_str, lat_str, long_str, ff_ratio = row
        new_row = [bin_id, int(supply_str), int(std_dev_str), float(long_str), float(lat_str), float(ff_ratio)]
        if int(supply_str) > 0:
            source_nodes.append(new_row)
        else:
            sink_nodes.append(new_row)
    ordered_nodes = source_nodes + sink_nodes
    total_source = len(source_nodes)
    total_nodes = len(ordered_nodes)

    # Load edge data
    folder = os.path.join(root, 'MVG Code')
    edge_data = csv.reader(open(folder + '/adjacency_matrix_muc.csv', 'r'))
    adjacency_dict = _load_edge_data_into_dict(edge_data, adjacency_dict)

    # Generate Random Graph
    count, total_supply = 0, 0
    base = abs(centeredness)/128 + 1
    base = 2 if base > 2 else base  # ignore large bases to avoid disappearing values
    sources = nodes//2
    sinks = nodes - sources

    indexes = np.arange(total_source)
    weights = np.array([base ** (-i) for i in range(total_source)])
    weights = weights/weights.sum()  # normalise weights
    random_sources = np.random.choice(indexes, sources, replace=False, p=weights)

    indexes = np.arange(total_source, total_nodes)
    weights = np.array([base ** (-i) for i in range(total_source, total_nodes)])
    weights = weights/weights.sum()  # normalise weights
    random_sinks = np.random.choice(indexes, sinks, replace=False, p=weights)

    random_indexes = np.concatenate((random_sources, random_sinks))

    bin_id, _, _, lat_str, long_str, ff_ratio = depot_node
    graph.add_node(str(0), sup=0)  # add depot node with zero load
    graph.add_edge(str(0), str(0), dist=0)  # add self edge
    node_data[str(0)] = {'bin_id': bin_id, 'pos': (float(long_str), float(lat_str)), 'ff_ratio': ff_ratio}

    for i in random_indexes:
        count += 1
        bin_id, mean_supply, std_dev, x, y, ff_ratio = ordered_nodes[i]
        supply = int(round(np.random.normal(mean_supply, std_dev)))
        supply = -1 if supply == 0 else supply
        total_supply += supply
        graph.add_node(str(count), sup=supply)
        node_data[str(count)] = {'bin_id': bin_id, 'pos': (x, y), 'ff_ratio': ff_ratio}
        for node, data in node_data.items():
            if directed:
                graph.add_edge(node, str(count), dist=adjacency_dict[bin_id][data['bin_id']])
                graph.add_edge(str(count), node, dist=adjacency_dict[data['bin_id']][bin_id])
            else:  # if undirected graph, take average distance
                avg_dist = (adjacency_dict[bin_id][data['bin_id']] + adjacency_dict[data['bin_id']][bin_id])/2
                graph.add_edge(node, str(count), dist=avg_dist)

    # Fix balancing
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


def _load_edge_data_into_dict(edge_data, adjacency_dict):
    v_ids = next(edge_data)
    for u_pairs in edge_data:
        u_id = u_pairs[0]
        adjacency_dict[u_id] = {}
        for v_index in range(1, len(u_pairs)):
            v_id = v_ids[v_index]
            adjacency_dict[u_id][v_id] = float(u_pairs[v_index])
    return adjacency_dict


def get_instances_names(filename='instances.pkl', path='../Problem Instances/Benchmark Instances/'):
    with open(path + filename, 'rb') as f:
        data = pickle.load(f)
    instances_names = data.keys()

    return instances_names


def load_from_pickle(instance_name='10Parma30.txt', filename='instances.pkl',
                     path='../Problem Instances/Benchmark Instances/', force_balance='none'):
    graph = nx.DiGraph()

    with open(path + filename, 'rb') as f:
        data = pickle.load(f)
    instance = data[instance_name]
    disbalances = instance['disbalances']
    adjacency_matrix = instance['adjacency']
    vehicle_capacity = int(instance['veh_capa'])
    vehicle_number = int(np.ceil(np.abs(sum(disbalances)) / vehicle_capacity))
    if vehicle_number == 0:
        vehicle_number = 1

    num_of_stations = len(disbalances)
    stations = []

    if force_balance == 'random':
        overall_disbalance = sum(disbalances)
        delta_disbalances = [0] * num_of_stations
        for _ in range(abs(overall_disbalance)):
            delta_disbalances[np.random.randint(num_of_stations)] -= np.sign(overall_disbalance)
        disbalances = [x + y for x, y in zip(disbalances, delta_disbalances)]

        for key, disbalance in enumerate(disbalances):
            station = str(key)
            graph.add_node(station, sup=disbalance)
            stations.append(station)

        for key_i, station_i in enumerate(stations):
            for key_j, station_j in enumerate(stations):
                graph.add_edge(station_i, station_j, dist=adjacency_matrix[key_i][key_j])

    elif force_balance == 'dummy':
        for key, disbalance in enumerate(disbalances):
            station = str(key)
            graph.add_node(station, sup=disbalance)
            stations.append(station)

        overall_disbalance = sum(disbalances)
        disbalances.append(-overall_disbalance)

        for key_i, station_i in enumerate(stations):
            for key_j, station_j in enumerate(stations):
                graph.add_edge(station_i, station_j, dist=adjacency_matrix[key_i][key_j])

        dummy_node = str(num_of_stations)
        stations.append(dummy_node)
        graph.add_node(dummy_node, sup=-overall_disbalance)

        for key in range(num_of_stations):
            graph.add_edge(stations[key], dummy_node, dist=adjacency_matrix[0][key])
            graph.add_edge(dummy_node, stations[key], dist=adjacency_matrix[key][0])

        graph.add_edge(stations[0], dummy_node, dist=0.0001)
        graph.add_edge(dummy_node, stations[0], dist=0.0001)
        graph.add_edge(dummy_node, dummy_node, dist=0)

    return graph, vehicle_capacity, vehicle_number
