# Rerun fig of map again 20 times but with duration instead of distance
"""
This test can be used to compare run many instances with the same solver to collect statistics on it
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
    'graph_size':        50,
    'graph_size_max':    500,
    'graph_size_step':   50,
    'graph_variations':  3,
    'trials_per_graph':  10,
    'stop_duration':     40,
    'load_duration':     15,
    'timeout':           120,
    'ordered_nbhs': [ops.intra_two_opt, ops.intra_segment_swap, ops.inter_segment_swap, ops.inter_two_opt, ],
    'nbh_change_set': [vns.change_nbh_cyclic],
    'large_nbhs': [0.1, 0.15, 0.20, 0.30],
    'large_timeout': 300,
}


print("Collecting Stats")
start = time.time()
test.test_loop_vns_lns(kwargs)
stop = time.time()
print("TEST COMPLETE. (Runtime: {} minutes)\n".format(round((stop - start)/60, 1)))
