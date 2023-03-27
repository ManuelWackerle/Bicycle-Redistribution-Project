"""
This test can be used to compare results when using different number of vehicles capacities.
The test can be performed for a range of different problem instances and graph sizes.
"""

import Tests.loops_for_testing as test
import operators as ops
import solvers as vns
import time


kwargs = {
    'num_vehicles':      5,
    'num_vehicles_max':  5,
    'num_vehicles_step': 1,
    'capacity':          5,
    'capacity_max':      60,
    'capacity_step':     5,
    'graph_size':        50,
    'graph_size_max':    200,
    'graph_size_step':   50,
    'graph_variations':  1,
    'trials_per_graph':  10,
    'timeout':           60,
    'ordered_nbhs': [ops.intra_two_opt, ops.intra_segment_swap, ops.inter_two_opt, ops.inter_segment_swap,],
    'nbh_change_set': [vns.change_nbh_cyclic],
    'large_nbhs': [0.1, 0.15, 0.2, 0.3],
    'large_timeout': 200,
}


print("Collecting Stats for varying capacities - VNS only")
start = time.time()
test.test_loop_vns(kwargs)
stop = time.time()
print("TEST COMPLETE. (Runtime: {} minutes)\n".format(round((stop - start)/60, 1)))
