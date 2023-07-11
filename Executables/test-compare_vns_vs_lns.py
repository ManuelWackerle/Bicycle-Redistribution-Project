"""
This test can be used to compare the performance of LNS after applying it to the VNS results.
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
    'capacity_step':     2,
    'graph_size':        500,
    'graph_size_max':    500,
    'graph_size_step':   100,
    'graph_variations':  1,
    'trials_per_graph':  30,
    'stop_duration':     40,
    'load_duration':     15,
    'timeout':           60,
    'ordered_nbhs': [ops.intra_two_opt, ops.intra_segment_swap, ops.inter_two_opt, ops.inter_segment_swap, ],
    'nbh_change_set': [vns.change_nbh_cyclic],
    'large_nbhs': [0.1, 0.15, 0.20, 0.30],
    'large_timeout': 300,
}


print("Collecting Stats for VNS vs LNS")
start = time.time()
test.test_loop_vns_lns(kwargs)
stop = time.time()
print("TEST COMPLETE. (Runtime: {} minutes)\n".format(round((stop - start)/60, 1)))