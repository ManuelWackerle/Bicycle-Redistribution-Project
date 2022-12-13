"""
Implementations of the variable neighbourhood search based on the article "Variable Neighbourhood Search: basics and
variants, by P Hansen et al.
"""

from routes import compute_route_cost
from time import time
from random import randint

"""
Neighbourhood operators. Given an initial route, each operator returns an array of new routes obtained by some 
modification of the initial route. We generate as many routes as possible in the given time.
The general type of these functions is:

def neighbourhood_operator ( initial_routes [[]], timeout ) -> list_of_modified_routes: [[[]]]
"""


def do_two_opt(initial_routes: [[]], cost_matrix: [[]], timeout=10) -> [[]]:
    """
    Applies the 2-OPT swap to each route.
    WARNING: This is a trial and not meant to substitute the other implementations.
    :param: initial_routes: array of routes
    :param: cost_matrix: cost of all edges in the graph
    :param: timeout: maximum execution time per route.
    :return: Array of triples (capacity needed for route, route)
    """

    best_routes = [None] * len(initial_routes)
    current_route = 0
    improvement_made = True
    time_start = time()

    for route in initial_routes:
        while improvement_made and time() < time_start + timeout:
            improvement_made = False
            for i in range(1, len(route) - 2):
                for j in range(i+1, len(route)):
                    if j-i == 1:
                        continue
                    new_route = route[:]
                    new_route[i:j] = route[j-1:i-1:-1]
                    if compute_route_cost(new_route, cost_matrix) < \
                            compute_route_cost(best_routes[current_route], cost_matrix):
                        best_routes.append(new_route)
                        improvement_made = True
        current_route += 1
    return best_routes


"""
General VNS.
"""

"""
** Summary of Paper explanation. **
The goal of the local search is to find local optimizers within neighbourhoods. In a first improvement strategy, as soon 
as a route in the neighbourhood with lower cost is found, it is set as the current route.
"""


def local_search_first_improvement(current_routes, nbh_routes: [[[]]], cost_matrix: [[]]) -> [[]]:
    """
    As soon as an improving solution is detected in a neighbourhood, it is set as the new current solution
    :param current_routes: routes to improve.
    :param nbh_routes: array of routes in the neighbourhood.
    :param cost_matrix: to compute total cost of routes.
    :return:
    """
    while True:
        index = 0
        best_routes = current_routes

        while index in range(len(nbh_routes)):
            if compute_route_cost(nbh_routes[index], cost_matrix) < compute_route_cost(current_routes, cost_matrix):
                current_routes = nbh_routes[index]
                break
        if compute_route_cost(best_routes, cost_matrix) <= compute_route_cost(current_routes, cost_matrix):
            break

    return best_routes


def shake_solution(current_routes: [], nbh_operator) -> [[]]:
    """
    Return a random solution from the given neighbourhood.
    :param current_routes: array of routes to shake.
    :param nbh_operator: neighbourhood operator that takes a set of routes and returns a neighbouring route.
    :return: modified routes in the neighbourhood.
    """

    list_of_candidate_routes = nbh_operator(current_routes)
    index = randint(0, len(list_of_candidate_routes))

    return list_of_candidate_routes[index]


def sequential_variable_nbh_descent(current_routes: [[]], ordered_nbhs: [], cost_matrix: [[]], timeout=10) -> [[]]:
    """
    ** The idea behind Variable Neighbourhood descent, VND. **
    If a solution is a local optimizer with respect to several neighbourhood structures, then it is more likely to be a
    global optimizer. A sequential VND explores neighbourhoods in a sequence. Works as follows:
    1.  Several neighbourhood structures are ordered in a list, N = {N1,..., Nl_max}.
    2.  Starting on a given solution, x, the sequential VND explores Nl for 1<=l<=lmax one after the other in the established
        order. The exploration of neighbourhoods is done in first improvement fashion.
    3.  As soon as an improvement in a given neighbourhood occurs, the process is restarted from the first neighbourhood
        (and in the same order) with the new solution.
    4.  The process stops when the current solution can not be improved with respect to any neighbourhood.
    """
    while True:
        stop = False
        current_nbh = 0  # Start at the first neighbourhood in the list.
        best_routes = current_routes
        time_start = time()
        while current_nbh < len(ordered_nbhs) and time() < time_start + timeout:
            nbh_routes = ordered_nbhs[current_nbh](best_routes)
            modified_routes = local_search_first_improvement(best_routes, nbh_routes, cost_matrix)
            change_nbh_sequential(current_routes, modified_routes, cost_matrix, current_nbh)
        if compute_route_cost(current_routes, cost_matrix) < compute_route_cost(best_routes, cost_matrix):
            stop = True
            break

    return best_routes


def change_nbh_sequential(current_routes: [[]], modified_routes: [[]], cost_matrix: [[]], nbh) -> [int, [[]]]:
    """
    Guides the vns heuristic when exploring a solution space. It decides which nbh will be explored next
    :param current_routes: current best routes
    :param modified_routes: candidate for better routes
    :param cost_matrix: matrix cost of the solution
    :param nbh: neighbourhood to explore
    :return:
    """
    if compute_route_cost(current_routes, cost_matrix) > compute_route_cost(modified_routes, cost_matrix):
        current_routes = modified_routes
        nbh = 0
    else:
        nbh = nbh + 1

    return nbh, current_routes


def general_variable_nbh_search (current_routes: [[]], ordered_nbhs: [], cost_matrix: [[]], timeout = 10):
    """
    General VNS with VND
    :param current_routes: current array of routes for all vehicles
    :param ordered_nbhs: list of ordered neighbourhood operators.
    :param cost_matrix: to compute route costs.
    :param timeout: maximum execution time.
    :return: best route found.
    """
    start_time = time()
    while time() < start_time + timeout:
        nbh_index = 0
        while nbh_index < len(ordered_nbhs):
            new_routes = shake_solution(current_routes, nbh_index)
            best_route = sequential_variable_nbh_descent(new_routes, ordered_nbhs, cost_matrix)
            change_nbh_sequential(current_routes, best_route, cost_matrix, nbh_index)

    return current_routes
