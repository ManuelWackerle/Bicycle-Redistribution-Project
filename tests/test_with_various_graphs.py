from load_csv import *
from structure import ProblemInstance, Vehicle
import vns
import csv

    # #Copy this into main
    # num_vehicles = 5
    # capacity = 20
    # min_graph_size = 10
    # max_graph_size = 400
    # graph_size_step = 10
    # graph_variations = 10
    # trials_per_graph = 10
    # run_test(num_vehicles, capacity, min_graph_size, max_graph_size, graph_size_step, graph_variations, trials_per_graph)


def run_test(num_vehicles=5,
             capacity=15,
             min_graph_size=10,
             max_graph_size=400,
             graph_size_step=10,
             graph_variations=10,
             trials_per_graph=10):

    writer = csv.writer(open('Saved/stats_test_01.csv', 'w', newline=''), delimiter=',')
    writer.writerow(["graph_size", "graph_instance", "trial", "greedy_distance", "vns_distance", "improvement", "vns_time"])
    np.random.seed(42)
    count = 0
    for n in range(min_graph_size, max_graph_size + 1, graph_size_step):
        gdt, vdt, imt, tmt = 0, 0, 0, 0
        print("\nTEST {}. Problem has {} nodes and {} vehicles with capacity {}"
              .format(count, n, num_vehicles, capacity))
        print("       | greedy dist  | vns distance |  vns time  | improved |")
        count += 1

        for m in range(graph_variations): #try different graph variations
            #Load Problem Instance
            graph, node_info = load_subset_from_ordered_nodes(nodes=n, centeredness=1)
            vehicles = [Vehicle(capacity=capacity, vehicle_id=str(i)) for i in range(num_vehicles)]
            problem = ProblemInstance(input_graph=graph, vehicles=vehicles, node_data=node_info, verbose=0)
            print("--------- new graph ----------")


            for trial in range(trials_per_graph):
                start1 = time.time()
                vns.greedy_routing_v1(problem, dist_weight=2, randomness=True)
                greedy_distance = problem.calculate_distances()


                ordered_nbhs = [vns.intra_two_opt_v2, vns.inter_two_opt_v2, vns.intra_or_opt, vns.remove_and_insert_station]
                step1 = time.time()
                vns.general_variable_nbh_search(problem, ordered_nbhs, change_nbh=vns.change_nbh_sequential, timeout=300)
                end1 = time.time()
                vns.calculate_loading_MF(problem)
                vns.remove_unused_stops(problem)
                distance = problem.calculate_distances()
                # visualize_routes(problem.get_all_routes(), node_info)


                # problem.display_results(False)
                problem.reset()

                gd = round(greedy_distance)/1000
                vd = round(distance)/1000
                im = round((1 - distance / greedy_distance) * 100, 1)
                tm = round(end1 - step1, 3)
                writer.writerow([n, m, trial, gd, vd, im, tm])
                gdt += gd
                vdt += vd
                imt += im
                tmt += tm
                print("{:5}: |{:11}km |{:11}km |{:10}s |{:8}% |".format(m*10 + trial, gd, vd, tm, im))

        gdt = round(gdt/trials_per_graph/graph_variations, 3)
        vdt = round(vdt/trials_per_graph/graph_variations, 3)
        imt = round(imt/trials_per_graph/graph_variations, 3)
        tmt = round(tmt/trials_per_graph/graph_variations, 3)
        print("AVG -  |{:11}km |{:11}km |{:10}s |{:8}% |".format(gdt, vdt, tmt, imt))
