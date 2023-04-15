"""
This test adds the functionality to create random gaussian noise to the cost function to check for numerical stability
"""
import matplotlib.pyplot as plt
import numpy as np

from loaders import load_subset_from_ordered_nodes
from structure import ProblemInstance, Vehicle
from copy import deepcopy
import solvers
import operators as ops
import time
import pandas as pd
import seaborn as sb
kwargs = {
    'num_vehicles':      5,
    'capacity':          15,
    'graph_size':        25,
    'trials_per_graph':  5,
    'timeout':           60,
    'ordered_nbhs': [ops.inter_segment_swap, ops.intra_two_opt, ops.intra_segment_swap, ops.inter_two_opt, ],
    'nbh_change': solvers.change_nbh_cyclic,
    'large_nbhs': [0.1, 0.15, 0.2, 0.25, 0.3, 0.4],
    'large_timeout': 300,
    'max_noise': 1
}

graph_size = kwargs.get('graph_size', 100)
trials_per_graph = kwargs.get('trials_per_graph', 10)
num_vehicles = kwargs.get('num_vehicles', 5)
capacity = kwargs.get('capacity', 15)
nbh_change = kwargs.get('nbh_change', solvers.change_nbh_cyclic)
ordered_nbhs = kwargs.get('ordered_nbhs',
                          [ops.intra_two_opt, ops.intra_segment_swap, ops.inter_two_opt, ops.inter_segment_swap])


# Load Problem Instance
graph, node_info = load_subset_from_ordered_nodes(nodes=graph_size, centeredness=10)
vehicles = [Vehicle(capacity=capacity, vehicle_id=str(i), distance_limit=0) for i in range(num_vehicles)]
problem = ProblemInstance(input_graph=graph, vehicles=vehicles, node_data=node_info, verbose=0)
solvers.greedy_routing(problem, dist_weight=2, randomness=True)
greedy_distance = problem.calculate_distances()
saved_problem = deepcopy(problem)

# Create dataframe
noise_trial = pd.DataFrame(columns=["noise_mean", "noise_std",  "greedy_dist", "vns_dist", "time", "improvement", "mean_cost"])
mean_costs = []

# Run VNS with greedy initialization
for noise_std in np.linspace(0, kwargs["max_noise"], num=30):
    for n in range(trials_per_graph):
        # Get maximum distance/cost
        mean_cost = np.mean([dist for _,_, dist in problem.model.edges.data("dist")])

        solvers.greedy_routing(problem, dist_weight=2, randomness=True)
        greedy_distance = problem.calculate_distances()
        start1 = time.time()
        solvers.general_variable_nbh_search(problem, ordered_nbhs, change_nbh=nbh_change, timeout=300, verbose=0)
        end1 = time.time()
        distance = problem.calculate_distances()

        vd = round(distance)
        vt = round(end1 - start1, 3)
        im = round((1 - distance / greedy_distance) * 100, 1)

        row_dict = {
            "noise_std": noise_std,
            "greedy_dist": greedy_distance,
            "vns_dist": vd,
            "time": vt,
            "improvement": im,
            "mean_cost": mean_cost
        }

        noise_trial.loc[len(noise_trial)] = row_dict
        noise_trial.reset_index()

        problem = deepcopy(saved_problem)
        problem.add_noise(distr_kwargs={'loc': 0, 'scale': noise_std * mean_cost})

grouped_noise_trial = noise_trial.groupby("noise_std", as_index=True).agg(['mean', "std"])

# Some basic of the distances
mean_1 = grouped_noise_trial[("vns_dist", "mean")]/1000
std_1 = grouped_noise_trial[("vns_dist", "std")]/1000

mean_2 = grouped_noise_trial[("greedy_dist", "mean")]/1000
std_2 = grouped_noise_trial[("greedy_dist", "std")]/1000

x_keys = noise_trial.groupby("noise_std", as_index=True).groups.keys()
x = [key*100 for key in x_keys]
plt.plot(x, mean_1, 'b-', label='VNS distance')
plt.fill_between(x, mean_1 - std_1, mean_1 + std_1, color='b', alpha=0.2)
plt.plot(x, mean_2, 'r-', label='Greedy distance')
plt.fill_between(x, mean_2 - std_2, mean_2 + std_2, color='r', alpha=0.2)
# plt.plot(x, grouped_noise_trial[('mean_cost', 'mean')]/1000)
plt.title("Greedy and VNS")
plt.xlabel("Noise level (\% of max distance)")
plt.ylabel("Distance [km]")
plt.legend()
plt.show()