"""
This test can be used to compare all the types of neighbourhood change methods with different problem instances
to establish convergence capabilities and speed.
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
    'graph_size_max':    300,
    'graph_size_step':   50,
    'graph_variations':  5,
    'trials_per_graph':  5,
    'timeout':           60,
    'ordered_nbhs': [ops.intra_two_opt, ops.intra_segment_swap, ops.inter_two_opt, ops.inter_segment_swap, ops.multi_remove_and_insert_station],
    'nbh_change_set': [vns.change_nbh_cyclic, vns.change_nbh_pipe, vns.change_nbh_sequential, vns.change_nbh_check_all],
    'large_nbhs': [0.1, 0.15, 0.20, 0.30],
    'large_timeout': 200,
}


print("Collecting Stats for different neighbourhood change methods - VNS only")
start = time.time()
test.test_loop_vns(kwargs)
stop = time.time()
print("TEST COMPLETE. (Runtime: {} minutes)\n".format(round((stop - start)/60, 1)))
