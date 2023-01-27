from matplotlib import pyplot as plt
from matplotlib import rc
import numpy as np
import csv
import pandas as pd
import matplotlib.patches as patches

rc('font', **{'family': 'lmodern', 'serif': ['Latin  Modern'], 'size': 22})
rc('text', usetex=True)

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "lmodern",
    "font.sans-serif": "Latin  Modern",
    "font.size": 24
})


def vns_data_from_csv(path):
    # graphs = ['sample_graph_02.csv', 'sample_graph_03.csv', 'sample_graph_04_edited.csv']
    # vns_methods = ['change_nbh_sequential', 'change_nbh_cyclic', 'change_nbh_pipe', 'change_nbh_skewed_sequential']
    # with open(path, newline='') as csvfile:
    #     spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
    #     for key, row in enumerate(spamreader):
    #         if key == 0:
    #             continue
    #         graph_index = graphs.index(row[0])
    #         vns_method_index = vns_methods.index(row[1])
    #         print(row)
    #         print(graph_index, vns_method_index)

    df = pd.read_csv(path)
    groupby_sum1 = df.groupby(['Instance', 'Change Operator']).std()
    print(groupby_sum1.to_string())

    pass


def plot_vns_analysis():
    group_labels = ["G1", "G2", "G3"]
    num_groups = len(group_labels)
    labels = ["Sequential", "Cyclic", "Pipe", "Skewed sequential"]
    labels_colours = ['#F5A623', '#9013FE', '#7ED321', '#4A90E2']
    # F5A623
    # 9013FE
    # 7ED321
    # 4A90E2
    # F8E71C
    # D0021B

    num_labels = len(labels)

    x = np.arange(len(group_labels))  # the label locations
    width = 0.2  # the width of the bars

    # initial_values_by_group = []
    # improvement_values_by_group = []
    # std_improvement_values_by_group = []

    final_values_by_group = [
        [130.741, 495.8596, 569.1652],
        [134.919, 542.0024, 687.351],
        [130.741, 510.1478, 612.1944],
        [130.741, 484.7166, 590.5686]
    ]

    improvement_values_by_group = [
        [4.178, 98.0444, 225.7188],
        [0, 51.9016, 107.533],
        [4.178, 83.7562, 182.6896],
        [4.178, 109.1874, 204.3154]
    ]

    std_improvement_values_by_group = [[0, 1.116619, 25.344209], [0, 9.515737, 31.628669], [0, 1.017865, 32.716444],
                                       [0, 25.675967, 31.051708]]

    fig, ax = plt.subplots(figsize=(19, 13))
    bars_bottom = []
    bars_top = []

    for label_index in range(num_labels):
        bar_position_x = x - width / 2 * (num_labels / 2 + 1) + label_index * width
        bars_bottom.append(
            ax.bar(bar_position_x, final_values_by_group[label_index], width, label=labels[label_index],
                   yerr=std_improvement_values_by_group[label_index],
                   ecolor='black',
                   capsize=10,
                   color=labels_colours[label_index], alpha=0.6))
        bars_top.append(ax.bar(bar_position_x, improvement_values_by_group[label_index], width,
                               bottom=final_values_by_group[label_index],
                               color=labels_colours[label_index], alpha=1))

    # ax.set_xlabel('Graph instance')
    ax.set_ylabel('Distance [km]')
    # ax.set_title('Distances by problem instances and VNS schemes')
    ax.set_xticks(x, group_labels)
    ax.minorticks_on()
    # ax.set_axisbelow(False)
    # ax.yaxis.grid(which='major', linestyle='-', color='b', alpha=0.5)
    # ax.yaxis.grid(which='minor', linestyle='--', color='grey', alpha=0.3)

    ax.tick_params(which='major', axis='x', width=0, length=0)
    ax.tick_params(which='minor', axis='x', width=0, length=0)
    ax.tick_params(which='minor', axis='y', width=0, length=0)

    # ax.legend()

    plt.grid(False)

    # for bar in bars_bottom:
    #     ax.bar_label(bar, padding=3)

    fig.tight_layout()
    ax.set_position([0.1, 0.1, 0.8, 0.8])
    plt.savefig('vns_analysis')
    plt.show()


def plot_optimum_analysis(path):
    df = pd.read_csv(path)
    node_num = 320
    df_node_num = df.loc[df['graph_size'].isin([node_num])]
    # groupby_graph_instance = df_node_num.groupby(['graph_instance']).mean()
    groupby_graph_size = df.groupby(['graph_size']).mean()
    groupby_graph_size_std = df.groupby(['graph_size']).std()

    print(groupby_graph_size_std.to_string())
    # print(df_node_num.to_string())
    # print(groupby_graph_instance.to_string())
    # print(groupby_graph_size['greedy_distance'].to_string())
    # print(groupby_graph_size.index)

    greedy_distance_by_graph_size = groupby_graph_size['greedy_distance']
    vns_distance_by_graph_size = groupby_graph_size['vns_distance']

    greedy_distance_std_by_graph_size = groupby_graph_size_std['greedy_distance']
    vns_distance_std_by_graph_size = groupby_graph_size_std['vns_distance']

    improvement_distance_by_graph_size = greedy_distance_by_graph_size - vns_distance_by_graph_size
    improvement_percentage_by_graph_size = improvement_distance_by_graph_size / greedy_distance_by_graph_size * 100
    improvement_percentage_limit = improvement_percentage_by_graph_size.loc[80:].mean()

    graph_size = groupby_graph_size.index

    # plt.plot(graph_size, improvement_percentage_by_graph_size)
    # plot_name = 'graph_size_dependence'
    fig, ax = plt.subplots(figsize=(19, 13))
    bar_width = 3
    cap_size = 2

    ax.bar(graph_size - bar_width / 2, greedy_distance_by_graph_size, yerr=greedy_distance_std_by_graph_size,
           width=bar_width, capsize=cap_size, color='#F5A623')
    ax.bar(graph_size + bar_width / 2, vns_distance_by_graph_size, yerr=vns_distance_std_by_graph_size,
           width=bar_width, capsize=cap_size, color='#9013FE')

    ax2 = ax.twinx()
    ax2.text(21, improvement_percentage_limit + 0.5, '{:.2f}'.format(improvement_percentage_limit) + '\%')
    ax2.plot([17, 21], [improvement_percentage_limit, improvement_percentage_limit + 0.5], color='black', lw=1)

    ax.set_ylabel("Distance [km]")
    ax.set_xlabel("Number of stations")
    ax2.set_ylabel("Relative improvement", labelpad=20)
    ax2.plot(graph_size, improvement_percentage_by_graph_size, color='#7ED321', lw=3)
    ax2.axhline(y=improvement_percentage_limit, color='#4A90E2', linestyle='--', lw=2)
    ax.set_xticks(graph_size)

    ax2.set_yticks(np.arange(0, 25, 2.5))
    # print(['{:,.1f}%'.format(x) for x in np.arange(0, 25, 2.5)])
    # ax2.set_yticklabels(['{:,.1f}%'.format(x) for x in np.arange(0, 25, 2.5)])
    ax2.yaxis.set_ticklabels(['{:,.1f}\%'.format(x) for x in np.arange(0, 25, 2.5)])
    ax.tick_params(axis='x', labelrotation=45)
    plt.xlim(left=5, right=335)

    ax.set_position([0.1, 0.1, 0.8, 0.8])
    plt.savefig('improvement_analysis')
    plt.show()


def plot_discrepancy_analysis(path):
    df = pd.read_csv(path)
    node_num = 200
    print(df.to_string())
    greedy_distance = df['greedy_distance']
    greedy_distance_mean = df['greedy_distance'].mean()
    greedy_distance_std = df['greedy_distance'].std()

    vns_distance = df['vns_distance']
    vns_distance_mean = df['vns_distance'].mean()
    vns_distance_std = df['vns_distance'].std()
    colors = ['#F5A623', '#9013FE', '#7ED321', '#4A90E2']
    print(greedy_distance_std, vns_distance_std)

    index = df.index
    alpha = 0.05
    fig, ax = plt.subplots(figsize=(19, 13))
    plt.plot(index, greedy_distance, color=colors[0], marker='o')
    ax.axhline(y=greedy_distance_mean, color=colors[0], linestyle='--', lw=2)
    rect_greedy = patches.Rectangle((-1, greedy_distance_mean - greedy_distance_std / 2), index[-1] + 2,
                                    greedy_distance_std,
                                    linewidth=1, edgecolor=None, facecolor=colors[0], alpha=alpha)

    ax.add_patch(rect_greedy)

    plt.plot(index, vns_distance, color=colors[1], marker='o')
    ax.axhline(y=vns_distance_mean, color=colors[1], linestyle='--', lw=2)
    rect_vns = patches.Rectangle((-1, vns_distance_mean - vns_distance_std / 2), index[-1] + 2,
                                 vns_distance_std,
                                 linewidth=1, edgecolor=None, facecolor=colors[1], alpha=alpha)

    ax.add_patch(rect_vns)

    plt.xlim(left=index[0] - 1, right=index[-1] + 1)

    ax.set_ylabel("Distance [km]")
    ax.set_xlabel("Trial index")

    ax.set_position([0.1, 0.1, 0.8, 0.8])

    plt.savefig('discrepancy_analysis')
    plt.show()


# plot_vns_analysis()

# vns_data_from_csv('change_nbh_intermediate_output.csv')

# plot_optimum_analysis('stats_test_01.csv')

plot_discrepancy_analysis('stats_test_02.csv')
