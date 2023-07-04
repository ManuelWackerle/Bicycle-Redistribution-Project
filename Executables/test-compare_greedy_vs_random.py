"""
This test can be used to compare the performance of VNS when initialized with Greedy vs Random.
"""

import Tests.loops_for_testing as test
import operators as ops
import solvers as vns
import time


kwargs = {
    'num_vehicles':      5,
    'num_vehicles_max':  5,
    'num_vehicles_step': 1,
    'capacity':          15,
    'capacity_max':      15,
    'capacity_step':     1,
    'graph_size':        250,
    'graph_size_max':    250,
    'graph_size_step':   50,
    'graph_variations':  1,
    'trials_per_graph':  50,
    'stop_duration': 40,
    'load_duration': 15,
    'timeout':           60,
    'ordered_nbhs': [ops.inter_segment_swap, ops.intra_two_opt, ops.intra_segment_swap, ops.inter_two_opt, ],
    'nbh_change_set': [vns.change_nbh_cyclic],
    'large_nbhs': [0.1, 0.15, 0.2, 0.25, 0.3, 0.4],
    'large_timeout': 300,
}


print("Collecting Stats for VNS performance when initialized with Greedy vs Random")
start = time.time()
test.test_loop_vns_multi_init(kwargs)
stop = time.time()
print("TEST COMPLETE. (Runtime: {} minutes)\n".format(round((stop - start)/60, 1)))
