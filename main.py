import json
import math
import random as rd
import traceback
import xml.etree.ElementTree as ElementTree
from enum import Enum
from itertools import combinations
from time import perf_counter

INSTANCES = [
    'res/augerat-1995-set-b/B-n31-k05.xml',
    'res/christofides-et-al-1979-cmt/CMT01.xml',
    'res/christofides-et-al-1979-set-m/M-n101-k10.xml',
    'res/golden-et-al-1998-set-1/Golden_01.xml',
    'res/augerat-1995-set-b/B-n78-k10.xml',
    'res/christofides-et-al-1979-cmt/CMT14.xml',
    'res/christofides-et-al-1979-set-m/M-n200-k17.xml',
    'res/golden-et-al-1998-set-1/Golden_20.xml',
]

MAX_ITER_MS = [1, 2, 3, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50]
MAX_ITER_ILS = None
REPORT_PATH = 'reports/report_#.json'
RUN_MILS_RVND = True
RUN_YACOS = True


class Struct(object):
    pass


def remove_if_exists(arr, value):
    if value in arr:
        arr.remove(value)


def build_edge_and_adjacent_dicts(edges):
    edge_dict = {}
    adjacent_dict = {}
    for edge in edges:
        source = edge[0]
        destination = edge[1]
        distance = edge[2]
        edge_dict[(edge[0], edge[1])] = distance
        if source not in adjacent_dict:
            adjacent_dict[source] = {}
        adjacent_dict[source][destination] = distance
    return edge_dict, adjacent_dict


class VRP_TYPE(Enum):
    VRP = 0  # Vehicle routing problem
    # Vehicle Routing Problem with Profits
    VRPP_TOP = 1  # The Team Orienteering Problem
    VRPP_CTOP = 2  # The Capacitated Team Orienteering Problem
    VRPP_TOPTW = 3  # The Team Orienteering Problem with Time Windows
    VRPPD = 4  # Vehicle Routing Problem with Pickup and Delivery
    VRP_LIFO = 5  # Vehicle Routing Problem with LIFO
    VRPTW = 6  # Vehicle Routing Problem with Time Windows
    CVRP = 7  # Capacitated Vehicle Routing Problem
    VRPMT = 8  # Vehicle Routing Problem with Multiple Trips
    OVRP = 9  # Open Vehicle Routing Problem
    IRP = 10  # Inventory Routing Problem
    MDVRP = 11  # Multi-Depot Vehicle Routing Problem
    HVRP = 12  # Heterogeneous Vehicle Routing Problem
    HVRPTW = 13  # Heterogeneous Vehicle Routing Problem with Time Windows

    def __str__(self):
        return self.name

    @staticmethod
    def get_all_types():
        return list(map(lambda c: c, VRP_TYPE))


def load_vrp_file(filepath):
    tree = ElementTree.parse(filepath)
    root = tree.getroot()

    departure = int(root.findall('.//departure_node')[0].text)
    arrival = int(root.findall('.//arrival_node')[0].text)
    capacity = float(root.findall('.//capacity')[0].text)
    vehicle_profiles = len(root.findall('.//vehicle_profile'))
    if vehicle_profiles > 1:
        raise NotImplementedError('Currently we cannot handle heterogeneous')
    cities = []
    demands = {}

    for request in root.iter('request'):
        city = int(request.get('node'))
        demand = float(request.find('quantity').text)
        demands[city] = demand

    for node in root.iter('node'):
        city = int(node.get('id'))
        x = float(node.find('cx').text)
        y = float(node.find('cy').text)
        demand = demands.get(city, 0)
        cities.append((city, demand, (x, y)))

    edges = []
    for src, dst in combinations(cities, 2):
        dist = math.dist(src[2], dst[2])
        edges.append((src[0], dst[0], dist))
        edges.append((dst[0], src[0], dist))
    for city in cities:
        edges.append((city[0], city[0], 0))

    edge_dict, adjacent_dict = build_edge_and_adjacent_dicts(edges)

    vrp_instance = Struct()
    vrp_instance.variant = VRP_TYPE.VRP
    vrp_instance.cities = {}
    for city in cities:
        c = Struct()
        c.demand = city[1]
        c.coords = city[2]
        vrp_instance.cities[city[0]] = c
    vrp_instance.edges = edges
    vrp_instance.edge_dict = edge_dict
    vrp_instance.adjacent_dict = adjacent_dict
    vrp_instance.fleet = Struct()
    vrp_instance.fleet.size = None
    vrp_instance.fleet.departure_city = departure
    vrp_instance.fleet.arrival_city = arrival
    vrp_instance.fleet.capacity = capacity

    return vrp_instance


def compute_fleet_size(vrp_instance):
    if vrp_instance.fleet.size is not None:
        return
    if vrp_instance.variant in (VRP_TYPE.HVRP, VRP_TYPE.HVRPTW):
        raise NotImplementedError('Sum of the amount of available vehicles of each profile')
    elif vrp_instance.variant == VRP_TYPE.MDVRP:
        raise NotImplementedError('Sum of the amount of available vehicles in each depot')
    elif vrp_instance.variant == VRP_TYPE.VRP:
        vrp_instance.fleet.size = int(
            math.floor(sum([el.demand for el in vrp_instance.cities.values()]) / vrp_instance.fleet.capacity))
    else:
        raise NotImplementedError('Unable to estimate fleet size')


def get_gamma_set():
    return list(range(0, 75, 5))  # TODO ?? paper is very messy here, just like the first argentina game on world cup 22


def copy_sol(sol):
    sol_copy = [el + [] for el in sol]
    return sol_copy


def compute_insertion_cost_and_pos(vrp_instance, route, city, gamma, insertion_criteria):
    if insertion_criteria == 'cimbvm':
        gs_and_pos = []
        for i in range(len(route) + 1):
            nxt = route[i] if i < len(route) else vrp_instance.fleet.arrival_city
            prev = route[i - 1] if i > 0 else vrp_instance.fleet.departure_city
            from_to_deposit_dist = vrp_instance.adjacent_dict[city][vrp_instance.fleet.arrival_city] + \
                                   vrp_instance.adjacent_dict[vrp_instance.fleet.departure_city][city]
            g = vrp_instance.adjacent_dict[city][nxt] + vrp_instance.adjacent_dict[prev][city]
            g -= vrp_instance.adjacent_dict[prev][nxt]
            g -= gamma * from_to_deposit_dist
            gs_and_pos.append((g, i))
        gs_and_pos.sort(key=lambda x: x[0])
        g_and_pos = gs_and_pos[0]
    elif insertion_criteria == 'cimpv':
        g = vrp_instance.adjacent_dict[city][route[-1]]
        g_and_pos = (g, len(route) - 1)
    else:
        adj_distance_sum = 0
        adj_distance_diff = 0
        prev = None
        for r in route:
            adj_distance_sum += vrp_instance.adjacent_dict[city][r]
            if prev is not None:
                adj_distance_diff += vrp_instance.adjacent_dict[r][prev]
            prev = r
        from_to_deposit_dist = vrp_instance.adjacent_dict[city][vrp_instance.fleet.arrival_city] + \
                               vrp_instance.adjacent_dict[vrp_instance.fleet.departure_city][city]
        g = adj_distance_sum - adj_distance_diff - (gamma * from_to_deposit_dist)
        g_and_pos = (g, len(route) - 1)
    return g_and_pos


def sequential_insertion(vrp_instance, routes, remaining_cities, insertion_criteria):
    # added_city = False
    gamma = 0
    if insertion_criteria == 'cimbvm':
        gamma = rd.choice(get_gamma_set())
    # r_0 = 1
    while len(remaining_cities) > 0:
        for route in routes:
            costs = []
            for rc in remaining_cities:
                cost, pos = compute_insertion_cost_and_pos(vrp_instance, route, rc, gamma, insertion_criteria)
                costs.append((cost, rc, pos))
            costs.sort(key=lambda x: x[0])
            cost, rc, pos = costs[0]
            route.insert(pos, rc)
            remaining_cities.remove(rc)
            # added_city = True
            # r_0 += 1
            if len(remaining_cities) == 0:
                break
    # TODO: implement for multiple deposits, route duration restriction and other variants of VRP...


def parallel_insertion(vrp_instance, routes, remaining_cities, insertion_criteria):
    # added_city = False
    gamma = 0
    if insertion_criteria == 'cimbvm':
        gamma = rd.choice(get_gamma_set())
    # r_0 = 1
    while len(remaining_cities) > 0:
        costs = []
        for r, route in enumerate(routes):
            for rc in remaining_cities:
                cost, pos = compute_insertion_cost_and_pos(vrp_instance, route, rc, gamma, insertion_criteria)
                costs.append((cost, rc, pos, r))
        costs.sort(key=lambda x: x[0])
        cost, rc, pos, r = costs[0]
        routes[r].insert(pos, rc)
        remaining_cities.remove(rc)
        # added_city = True
        # r_0 += 1
    # TODO: implement for multiple deposits, route duration restriction and other variants of VRP...


def is_valid_solution(vrp_instance, solution):
    fixed_demand = vrp_instance.cities[vrp_instance.fleet.departure_city].demand
    if vrp_instance.fleet.departure_city != vrp_instance.fleet.arrival_city:
        fixed_demand += vrp_instance.cities[vrp_instance.fleet.arrival_city].demand
    cities_seen = set()
    for route in solution:
        r_demand = fixed_demand
        for city in route:
            cities_seen.add(city)
            r_demand += vrp_instance.cities[city].demand
        if r_demand > vrp_instance.fleet.capacity:
            return False
        if len(route) != len(set(route)):
            print('ERROR, duplicated ciies')
            return False
    cities_seen.add(vrp_instance.fleet.departure_city)
    cities_seen.add(vrp_instance.fleet.arrival_city)
    return cities_seen == vrp_instance.cities.keys()


def mils_rvnd_gen_initial_solution(vrp_instance, cur_attempt=0, max_attempts=3):
    if cur_attempt >= max_attempts:
        vrp_instance.fleet.size += 1
        cur_attempt = 0
    # routes = [[] for _ in range(vrp_instance.fleet.size - 1)] # TODO: subtract 1 does not make sense
    routes = [[] for _ in range(vrp_instance.fleet.size)]
    remaining_cities = [city for city in vrp_instance.cities.keys()]  # lc
    remove_if_exists(remaining_cities, vrp_instance.fleet.departure_city)  # always the start
    remove_if_exists(remaining_cities, vrp_instance.fleet.arrival_city)  # always the end
    for route in routes:
        random_city = rd.choice(remaining_cities)
        route.append(random_city)
        remaining_cities.remove(random_city)
    insertion_method = rd.choice(['eis', 'eip'])
    insertion_criteria = rd.choice(['cimbvm', 'cimpv'])
    if insertion_method == 'eis':
        sequential_insertion(vrp_instance, routes, remaining_cities, insertion_criteria)
    else:
        parallel_insertion(vrp_instance, routes, remaining_cities, insertion_criteria)

    if not is_valid_solution(vrp_instance, routes):
        return mils_rvnd_gen_initial_solution(vrp_instance, cur_attempt=cur_attempt + 1, max_attempts=max_attempts)

    # TODO: implement for multiple deposits, route duration restriction and other variants of VRP...
    return routes


def compute_route_cost(start, end, route, adj_dict):
    tour_cost = 0
    for i in range(-1, len(route), 1):
        current_city = start if i < 0 else route[i]
        next_city = end if i >= len(route) - 1 else route[i + 1]
        tour_cost += adj_dict[current_city][next_city]
    return tour_cost


def get_solution_cost(vrp_instance, solution):
    if solution is None:
        return float('inf')
    sol_cost = 0
    for route in solution:
        sol_cost += compute_route_cost(vrp_instance.fleet.departure_city, vrp_instance.fleet.arrival_city, route,
                                       vrp_instance.adjacent_dict)
    return sol_cost


def get_n_random_indexes(array, n):
    get_random = lambda: int(rd.random() * len(array))
    if n == 1:
        return get_random()
    indexes = set()
    if n > len(array):
        raise Exception()
    while n != len(indexes):
        indexes.add(get_random())
    indexes = list(indexes)
    indexes.sort()
    return indexes


def insert_at_closest(vrp_instance, route, element):
    _, pos = compute_insertion_cost_and_pos(vrp_instance, route, element, 0, 'cimbvm')
    route.insert(pos, element)


def swap_1_1(routes, r1=None, r2=None, i=None, j=None):
    if r1 is None or r2 is None:
        r1, r2 = get_n_random_indexes(routes, 2)
    if len(routes[r1]) < 1 or len(routes[r2]) < 1:
        return
    if i is None:
        i = get_n_random_indexes(routes[r1], 1)
    if j is None:
        j = get_n_random_indexes(routes[r2], 1)
    routes[r1][i], routes[r2][j] = routes[r2][j], routes[r1][i]
    return routes


def best_swap_1_1(vrp_instance, routes):
    best_cos = float('inf')
    best_sol = None
    for r1 in range(len(routes)):
        for r2 in range(r1 + 1, len(routes), 1):
            for i in range(len(routes[r1])):
                for j in range(len(routes[r2])):
                    cand = swap_1_1(copy_sol(routes), r1, r2, i, j)
                    cost = get_solution_cost(vrp_instance, cand)
                    if is_valid_solution(vrp_instance, cand) and cost < best_cos:
                        best_cos = cost
                        best_sol = cand
    return best_sol


def shift_1_1(vrp_instance, routes, r1=None, r2=None, i=None, j=None):
    if r1 is None or r2 is None:
        r1, r2 = get_n_random_indexes(routes, 2)
    if len(routes[r1]) < 1 or len(routes[r2]) < 1:
        return
    if i is None:
        i = get_n_random_indexes(routes[r1], 1)
    i = routes[r1][i]
    if j is None:
        j = get_n_random_indexes(routes[r2], 1)
    j = routes[r2][j]
    routes[r1].remove(i)
    routes[r2].remove(j)
    insert_at_closest(vrp_instance, routes[r2], i)
    insert_at_closest(vrp_instance, routes[r1], j)
    return routes


def best_shift_1_1(vrp_instance, routes):
    best_cos = float('inf')
    best_sol = None
    for r1 in range(len(routes)):
        for r2 in range(r1 + 1, len(routes), 1):
            for i in range(len(routes[r1])):
                for j in range(len(routes[r2])):
                    cand = shift_1_1(vrp_instance, copy_sol(routes), r1, r2, i, j)
                    cost = get_solution_cost(vrp_instance, cand)
                    if is_valid_solution(vrp_instance, cand) and cost < best_cos:
                        best_cos = cost
                        best_sol = cand
    return best_sol


def shift_1_0(vrp_instance, routes, r1=None, r2=None, i=None):
    if r1 is None or r2 is None:
        r1, r2 = get_n_random_indexes(routes, 2)
    if len(routes[r1]) < 1:
        return
    if i is None:
        i = get_n_random_indexes(routes[r1], 1)
    i = routes[r1][i]
    routes[r1].remove(i)
    insert_at_closest(vrp_instance, routes[r2], i)
    return routes


def best_shift_1_0(vrp_instance, routes):
    best_cos = float('inf')
    best_sol = None
    for r1 in range(len(routes)):
        for r2 in range(r1 + 1, len(routes), 1):
            for i in range(len(routes[r1])):
                cand = shift_1_0(vrp_instance, copy_sol(routes), r1, r2, i)
                cost = get_solution_cost(vrp_instance, cand)
                if is_valid_solution(vrp_instance, cand) and cost < best_cos:
                    best_cos = cost
                    best_sol = cand
    return best_sol


def shift_2_0(vrp_instance, routes, r1=None, r2=None, i=None):
    if r1 is None or r2 is None:
        r1, r2 = get_n_random_indexes(routes, 2)
    if len(routes[r1]) < 2:
        return
    if i is None:
        i = get_n_random_indexes(routes[r1][:-1], 1)
    j = i + 1
    i = routes[r1][i]
    j = routes[r1][j]
    routes[r1].remove(i)
    routes[r1].remove(j)
    insert_at_closest(vrp_instance, routes[r2], i)
    insert_at_closest(vrp_instance, routes[r2], j)
    return routes


def best_shift_2_0(vrp_instance, routes):
    best_cos = float('inf')
    best_sol = None
    for r1 in range(len(routes)):
        for r2 in range(r1 + 1, len(routes), 1):
            for i in range(len(routes[r1]) - 1):
                cand = shift_2_0(vrp_instance, copy_sol(routes), r1, r2, i)
                cost = get_solution_cost(vrp_instance, cand)
                if is_valid_solution(vrp_instance, cand) and cost < best_cos:
                    best_cos = cost
                    best_sol = cand
    return best_sol


def swap_2_1(vrp_instance, routes, r1=None, r2=None, i=None, k=None):
    if r1 is None or r2 is None:
        r1, r2 = get_n_random_indexes(routes, 2)
    if len(routes[r1]) < 2 or len(routes[r2]) < 1:
        return
    if i is None:
        i = get_n_random_indexes(routes[r1][:-1], 1)
    j = i + 1
    i = routes[r1][i]
    j = routes[r1][j]
    if k is None:
        k = get_n_random_indexes(routes[r2], 1)
    k = routes[r2][k]
    routes[r1].remove(i)
    routes[r1].remove(j)
    routes[r2].remove(k)
    insert_at_closest(vrp_instance, routes[r2], i)
    insert_at_closest(vrp_instance, routes[r2], j)
    insert_at_closest(vrp_instance, routes[r1], k)
    return routes


def best_swap_2_1(vrp_instance, routes):
    best_cos = float('inf')
    best_sol = None
    for r1 in range(len(routes)):
        for r2 in range(r1 + 1, len(routes), 1):
            for i in range(len(routes[r1]) - 1):
                for k in range(len(routes[r2])):
                    cand = swap_2_1(vrp_instance, copy_sol(routes), r1, r2, i, k)
                    cost = get_solution_cost(vrp_instance, cand)
                    if is_valid_solution(vrp_instance, cand) and cost < best_cos:
                        best_cos = cost
                        best_sol = cand
    return best_sol


def swap_2_2(vrp_instance, routes, r1=None, r2=None, i=None, k=None):
    if r1 is None or r2 is None:
        r1, r2 = get_n_random_indexes(routes, 2)
    if len(routes[r1]) < 2 or len(routes[r2]) < 2:
        return
    if i is None:
        i = get_n_random_indexes(routes[r1][:-1], 1)
    j = i + 1
    i = routes[r1][i]
    j = routes[r1][j]
    if k is None:
        k = get_n_random_indexes(routes[r2][:-1], 1)
    l = k + 1
    k = routes[r2][k]
    l = routes[r2][l]
    routes[r1].remove(i)
    routes[r1].remove(j)
    routes[r2].remove(k)
    routes[r2].remove(l)
    insert_at_closest(vrp_instance, routes[r2], i)
    insert_at_closest(vrp_instance, routes[r2], j)
    insert_at_closest(vrp_instance, routes[r1], k)
    insert_at_closest(vrp_instance, routes[r1], l)
    return routes


def best_swap_2_2(vrp_instance, routes):
    best_cos = float('inf')
    best_sol = None
    for r1 in range(len(routes)):
        for r2 in range(r1 + 1, len(routes), 1):
            for i in range(len(routes[r1]) - 1):
                for k in range(len(routes[r2]) - 1):
                    cand = swap_2_2(vrp_instance, copy_sol(routes), r1, r2, i, k)
                    cost = get_solution_cost(vrp_instance, cand)
                    if is_valid_solution(vrp_instance, cand) and cost < best_cos:
                        best_cos = cost
                        best_sol = cand
    return best_sol


def cross(vrp_instance, routes, r1=None, r2=None, s1=None, e1=None, s2=None, e2=None):
    if r1 is None or r2 is None:
        r1, r2 = get_n_random_indexes(routes, 2)
    if len(routes[r1]) < 2 or len(routes[r2]) < 2:
        return
    if s1 is None or e1 is None:
        s, e = get_n_random_indexes(routes[r1][:-1], 2)
    else:
        s, e = s1, e1
    to_add_on_2 = []
    for _ in range(s, e, 1):
        i = routes[r1][s]
        routes[r1].remove(i)
        to_add_on_2.append(i)
    if s2 is None or e2 is None:
        s, e = get_n_random_indexes(routes[r2][:-1], 2)
    else:
        s, e = s2, e2
    to_add_on_1 = []
    for _ in range(s, e, 1):
        j = routes[r2][s]
        routes[r2].remove(j)
        to_add_on_1.append(j)
    for j in to_add_on_1:
        insert_at_closest(vrp_instance, routes[r1], j)
    for i in to_add_on_2:
        insert_at_closest(vrp_instance, routes[r2], i)
    return routes


def best_cross(vrp_instance, routes):
    best_cos = float('inf')
    best_sol = None
    for r1 in range(len(routes)):
        for r2 in range(r1 + 1, len(routes), 1):
            for s1 in range(len(routes[r1])):
                for e1 in range(s1 + 1, len(routes[r1]) - 1, 1):
                    for s2 in range(len(routes[r2])):
                        for e2 in range(s2 + 1, len(routes[r2]) - 1, 1):
                            cand = cross(vrp_instance, copy_sol(routes), r1, r2, s1, e1, s2, e2)
                            cost = get_solution_cost(vrp_instance, cand)
                            if is_valid_solution(vrp_instance, cand) and cost < best_cos:
                                best_cos = cost
                                best_sol = cand
    return best_sol


def t_shift(vrp_instance, routes, r1=None, r2=None, s1=None, s2=None):
    if r1 is None or r2 is None:
        r1, r2 = get_n_random_indexes(routes, 2)
    if len(routes[r1]) < 2 or len(routes[r2]) < 2:
        return
    if s1 is None:
        s = get_n_random_indexes(routes[r1][:-1], 1)
    else:
        s = s1
    to_add_on_2 = []
    for i in range(s, len(routes[r1]), 1):
        i = routes[r1][i]
        routes[r1].remove(i)
        to_add_on_2.append(i)
    if s2 is None:
        s = get_n_random_indexes(routes[r2][:-1], 1)
    else:
        s = s2
    to_add_on_1 = []
    for j in range(s, len(routes[r2]), 1):
        j = routes[r2][j]
        routes[r2].remove(j)
        to_add_on_1.append(j)
    for j in to_add_on_1:
        insert_at_closest(vrp_instance, routes[r1], j)
    for i in to_add_on_2:
        insert_at_closest(vrp_instance, routes[r2], i)
    raise NotImplementedError('Currently we cannot handle heterogeneous')
    return routes


def shift_depot(vrp_instance, routes):
    raise NotImplementedError('Currently we cannot handle multi-depots')


def swap_depot(vrp_instance, routes):
    raise NotImplementedError('Currently we cannot handle multi-depots')


def reinsertion(routes, r1=None, a=None, b=None):
    if r1 is None:
        r1 = get_n_random_indexes(routes, 1)
    if len(routes[r1]) < 2:
        return
    if a is None or b is None:
        a, b = get_n_random_indexes(routes[r1], 2)
    a = routes[r1][a]
    routes[r1].remove(a)
    routes[r1].insert(b, a)
    return routes


def best_reinsertion(vrp_instance, routes, r=None):
    best_cos = float('inf')
    best_sol = None
    for r1 in range(len(routes)):
        if r is not None:
            r1 = r
        for a in range(len(routes[r1])):
            for b in range(a + 1, len(routes[r1]), 1):
                cand = reinsertion(copy_sol(routes), r1, a, b)
                cost = get_solution_cost(vrp_instance, cand)
                if is_valid_solution(vrp_instance, cand) and cost < best_cos:
                    best_cos = cost
                    best_sol = cand
        if r is not None:
            break
    return best_sol


def or_2opt(routes, r1=None, a=None, c=None):
    # TODO work with indexes directly, would be faster
    if r1 is None:
        r1 = get_n_random_indexes(routes, 1)
    if len(routes[r1]) < 3:
        return
    if a is None or c is None:
        a, _, c = get_n_random_indexes(routes[r1], 3)
    b = a + 1
    a = routes[r1][a]
    b = routes[r1][b]
    routes[r1].remove(a)
    routes[r1].remove(b)
    routes[r1].insert(c, a)
    routes[r1].insert(c + 1, b)
    return routes


def best_or_2opt(vrp_instance, routes, r=None):
    best_cos = float('inf')
    best_sol = None
    for r1 in range(len(routes)):
        if r is not None:
            r1 = r
        for a in range(len(routes[r1]) - 1):
            for c in range(a + 2, len(routes[r1]), 1):
                cand = or_2opt(copy_sol(routes), r1, a, c)
                cost = get_solution_cost(vrp_instance, cand)
                if is_valid_solution(vrp_instance, cand) and cost < best_cos:
                    best_cos = cost
                    best_sol = cand
        if r is not None:
            break
    return best_sol


def or_3opt(routes, r1=None, a=None, d=None):
    # TODO work with indexes directly, would be faster
    if r1 is None:
        r1 = get_n_random_indexes(routes, 1)
    if len(routes[r1]) < 4:
        return
    if a is None or d is None:
        a, _, _, d = get_n_random_indexes(routes[r1], 4)
    b = a + 1
    c = b + 1
    a = routes[r1][a]
    b = routes[r1][b]
    c = routes[r1][c]
    routes[r1].remove(a)
    routes[r1].remove(b)
    routes[r1].remove(c)
    routes[r1].insert(d, a)
    routes[r1].insert(d + 1, b)
    routes[r1].insert(d + 2, c)
    return routes


def best_or_3opt(vrp_instance, routes, r=None):
    best_cos = float('inf')
    best_sol = None
    for r1 in range(len(routes)):
        if r is not None:
            r1 = r
        for a in range(len(routes[r1]) - 2):
            for d in range(a + 3, len(routes[r1]), 1):
                cand = or_3opt(copy_sol(routes), r1, a, d)
                cost = get_solution_cost(vrp_instance, cand)
                if is_valid_solution(vrp_instance, cand) and cost < best_cos:
                    best_cos = cost
                    best_sol = cand
        if r is not None:
            break
    return best_sol


def two_opt(routes, r1=None, i=None, j=None):
    if r1 is None:
        r1 = get_n_random_indexes(routes, 1)
    if len(routes[r1]) < 3:
        return
    if i is None or j is None:
        i, j = get_n_random_indexes(routes[r1], 2)
    routes[r1][i:j + 1] = reversed(routes[r1][i:j + 1])
    return routes


def best_two_opt(vrp_instance, routes, r=None):
    best_cos = float('inf')
    best_sol = None
    for r1 in range(len(routes)):
        if r is not None:
            r1 = r
        for i in range(len(routes[r1]) - 2):
            for j in range(i + 1, len(routes[r1]), 1):
                cand = two_opt(copy_sol(routes), r1, i, j)
                cost = get_solution_cost(vrp_instance, cand)
                if is_valid_solution(vrp_instance, cand) and cost < best_cos:
                    best_cos = cost
                    best_sol = cand
        if r is not None:
            break
    return best_sol


def swap(routes, r1=None, a=None, b=None):
    if r1 is None:
        r1 = get_n_random_indexes(routes, 1)
    if len(routes[r1]) < 2:
        return
    if a is None or b is None:
        a, b = get_n_random_indexes(routes[r1], 2)
    routes[r1][a], routes[r1][b] = routes[r1][b], routes[r1][a]
    return routes


def best_swap(vrp_instance, routes, r=None):
    best_cos = float('inf')
    best_sol = None
    for r1 in range(len(routes)):
        if r is not None:
            r1 = r
        for a in range(len(routes[r1]) - 2):
            for b in range(a + 1, len(routes[r1]), 1):
                cand = swap(copy_sol(routes), r1, a, b)
                cost = get_solution_cost(vrp_instance, cand)
                if is_valid_solution(vrp_instance, cand) and cost < best_cos:
                    best_cos = cost
                    best_sol = cand
        if r is not None:
            break
    return best_sol


def reverse(routes, r1=None):
    if r1 is None:
        r1 = get_n_random_indexes(routes, 1)
    if len(routes[r1]) < 2:
        return
    routes[r1].reverse()
    return routes


def best_reverse(vrp_instance, routes, r=None):
    best_cos = float('inf')
    best_sol = None
    for r1 in range(len(routes)):
        if r is not None:
            r1 = r
        cand = reverse(copy_sol(routes), r1)
        cost = get_solution_cost(vrp_instance, cand)
        if is_valid_solution(vrp_instance, cand) and cost < best_cos:
            best_cos = cost
            best_sol = cand
        if r is not None:
            break
    return best_sol


def multi_swap_1_1(routes, max_swap=2):
    n = 1 + int(rd.random() * max_swap)
    for _ in range(n):
        swap_1_1(routes)


def multi_shift_1_1(vrp_instance, routes, max_shift=2):
    n = 1 + int(rd.random() * max_shift)
    for _ in range(n):
        shift_1_1(vrp_instance, routes)


def try_empty_route(vrp_instance, routes):
    raise NotImplementedError('Currently we do not support OVRP')


def shake(vrp_instance, current_solution):
    solution = copy_sol(current_solution)
    met = rd.choice(['multi_swap(1,1)', 'multi_shift(1,1)'])
    if met == 'multi_swap(1,1)':
        multi_swap_1_1(current_solution)
    elif met == 'multi_shift(1,1)':
        multi_shift_1_1(vrp_instance, current_solution)
    else:
        raise NotImplementedError()
    # TODO: implement split for heterogeneous fleet
    return solution


def get_inter_route_best_methods():
    inter = [
        'best_shift_1_0',
        'best_swap_1_1',
        'best_shift_2_0',
        'best_swap_2_1',
        'best_swap_2_2',
        'best_cross',
    ]
    return inter


def get_intra_route_best_methods():
    intra = [
        'best_reinsertion',
        'best_or_2opt',
        'best_or_3opt',
        'best_two_opt',
        'best_swap',
    ]
    return intra


def intra_sol_search(vrp_instance, candidate_solution):
    solution = copy_sol(candidate_solution)
    best_cost = get_solution_cost(vrp_instance, solution)
    methods = get_intra_route_best_methods()
    while len(methods) > 0:
        method = rd.choice(methods)
        if method == 'best_reinsertion':
            candidate_solution = best_reinsertion(vrp_instance, solution)
        elif method == 'best_or_2opt':
            candidate_solution = best_or_2opt(vrp_instance, solution)
        elif method == 'best_or_3opt':
            candidate_solution = best_or_3opt(vrp_instance, solution)
        elif method == 'best_two_opt':
            candidate_solution = best_two_opt(vrp_instance, solution)
        elif method == 'best_swap':
            candidate_solution = best_swap(vrp_instance, solution)
        else:
            raise NotImplementedError('Unknown method')
        cand_cost = get_solution_cost(vrp_instance, candidate_solution)
        if cand_cost < best_cost:
            solution = candidate_solution
            best_cost = cand_cost
        else:
            methods.remove(method)
    return list(filter(lambda x: len(x) > 0, solution))


def rvnd(vrp_instance, candidate_solution):
    solution = copy_sol(candidate_solution)
    best_cost = get_solution_cost(vrp_instance, solution)
    methods = get_inter_route_best_methods()
    while len(methods) > 0:
        method = rd.choice(methods)
        if method == 'best_shift_1_0':
            candidate_solution = best_shift_1_0(vrp_instance, solution)
        elif method == 'best_swap_1_1':
            candidate_solution = best_swap_1_1(vrp_instance, solution)
        elif method == 'best_shift_2_0':
            candidate_solution = best_shift_2_0(vrp_instance, solution)
        elif method == 'best_swap_2_1':
            candidate_solution = best_swap_2_1(vrp_instance, solution)
        elif method == 'best_swap_2_2':
            candidate_solution = best_swap_2_2(vrp_instance, solution)
        elif method == 'best_cross':
            candidate_solution = best_cross(vrp_instance, solution)
        else:
            raise NotImplementedError('Unknown method')
        cand_cost = get_solution_cost(vrp_instance, candidate_solution)
        if cand_cost < best_cost:
            solution = candidate_solution
            solution = intra_sol_search(vrp_instance, solution)
            best_cost = get_solution_cost(vrp_instance, solution)
        else:
            methods.remove(method)
    return list(filter(lambda x: len(x) > 0, solution))


def mils_rvnd(vrp_instance, max_iter_ms, max_iter_ils=None, vehicle_reduction=False):
    compute_fleet_size(vrp_instance)
    best_cost = float('inf')
    best_solution = None
    if max_iter_ils is None:
        beta = 5
        n = len(vrp_instance.cities)
        v = vrp_instance.fleet.size
        max_iter_ils = n + beta * v
    for i in range(max_iter_ms):
        initial_solution = mils_rvnd_gen_initial_solution(vrp_instance)
        candidate_solution = initial_solution
        current_solution = initial_solution
        current_cost = get_solution_cost(vrp_instance, current_solution)
        candidate_cost = current_cost
        j = 0
        while j < max_iter_ils:
            candidate_solution = rvnd(vrp_instance, candidate_solution)
            candidate_cost = get_solution_cost(vrp_instance, candidate_solution)
            if vehicle_reduction or candidate_cost < current_cost:
                current_solution = candidate_solution
                current_cost = candidate_cost
                j = 0
            candidate_solution = shake(vrp_instance, current_solution)
            candidate_cost = get_solution_cost(vrp_instance, candidate_solution)
            j += 1
        if candidate_cost < best_cost:
            best_solution = candidate_solution
            best_cost = candidate_cost
    return best_cost, best_solution


def build_graph_from_edges(edges_list):
    nodes_set = set()
    graph_dict = {}
    total_cost = 0
    for edge in edges_list:
        if edge is None:
            continue
        cost, src, dst = edge
        total_cost += cost
        if src not in graph_dict:
            graph_dict[src] = set()
        if dst not in graph_dict:
            graph_dict[dst] = set()
        graph_dict[src].add(dst)
        graph_dict[dst].add(src)
        nodes_set.add(src)
        nodes_set.add(dst)
    return graph_dict, nodes_set, total_cost


def circle_finder(graph_edges):
    def circle_finder_recursive(edges, visited_set, node, parent):
        visited_set.add(node)
        for neighbour in edges.get(node, []):
            if neighbour not in visited_set:
                if circle_finder_recursive(edges, visited_set, neighbour, node):
                    return True
            elif parent != neighbour:
                return True
        return False

    has_circle = False
    if len(graph_edges) == 1:
        return has_circle
    graph_dict, nodes_set, _ = build_graph_from_edges(graph_edges)
    visited = set()
    for n in nodes_set:
        if n not in visited:
            if circle_finder_recursive(graph_dict, visited, n, None):
                has_circle = True
                break

    return has_circle


def get_canonical_edge(cost, src, dst):
    if src < dst:
        edge = f'{src}|{dst}|{cost}'
    else:
        edge = f'{dst}|{src}|{cost}'
    return edge


def build_minimum_spamming_tree(nodes, edge_list, degree_restriction=float('inf')):
    edge_list.sort(key=lambda x: x[0])  # sort by cheapest
    degrees = {node: 0 for node in nodes}
    canonical_edges_added = set()  # avoid inserting the same edge reversed

    minimum_spamming_tree = []
    edge_index = 0  # to iterate from near to far edges
    for _ in range(len(nodes) - 1):
        minimum_spamming_tree.append(None)
        added = False
        while not added and edge_index < len(edge_list):
            edge = edge_list[edge_index]
            cost, src, dst = edge
            canonical_edge = get_canonical_edge(cost, src, dst)
            if degrees[src] < degree_restriction and degrees[dst] < degree_restriction and \
                    canonical_edge not in canonical_edges_added:  # no city can be connected to more than 2 others
                minimum_spamming_tree[-1] = edge
                has_circle = circle_finder(
                    minimum_spamming_tree)  # if the path don't make a cycle we can add
                if not has_circle:
                    added = True
                    degrees[src] += 1
                    degrees[dst] += 1
                    canonical_edges_added.add(canonical_edge)
            edge_index += 1

    return minimum_spamming_tree, degrees


def get_eulerian_path(graph, starting=None):
    graph_deep_deep_copy = {k: set(list(v)) for k, v in
                            graph.items()}  # copy.copy / copy.deepcopy not working as expected, copy makes a shallow copy and deep copy causes the sets to be empty

    if starting is None:
        starting = next(iter(graph_deep_deep_copy.keys()))
    euler_path = [starting]

    while any([len(v) > 0 for v in graph_deep_deep_copy.values()]):
        src_node = None
        current_path_idx = None
        for idx, node in enumerate(euler_path):
            if len(graph_deep_deep_copy[node]) > 0:
                src_node = node
                current_path_idx = idx
                break
        while src_node is not None and len(graph_deep_deep_copy[src_node]) > 0:
            dst_node = next(iter(graph_deep_deep_copy[src_node]))
            euler_path.insert(current_path_idx, dst_node)
            current_path_idx += 1
            graph_deep_deep_copy[src_node].remove(dst_node)
            graph_deep_deep_copy[dst_node].remove(src_node)
    euler_path.reverse()
    return euler_path


def christofides(vrp_instance, route):
    nodes_set = set(route + [vrp_instance.fleet.departure_city, vrp_instance.fleet.arrival_city])
    nodes = list(nodes_set)
    edge_dict = vrp_instance.edge_dict

    edge_list = [[cost, src, dst] for (src, dst), cost in edge_dict.items() if
                 src != dst and src in nodes_set and dst in nodes_set]  # list only non self connected inside route
    mst, degrees = build_minimum_spamming_tree(nodes, edge_list)

    odd_vertices = [x[0] for x in filter(lambda x: x[1] % 2 == 1, degrees.items())]

    minimum_weight_matching = []
    rd.shuffle(odd_vertices)
    while len(odd_vertices) > 0:
        from_vertice = odd_vertices[-1]
        odd_vertices = odd_vertices[:-1]
        best_edge = [[float('inf'), from_vertice, from_vertice], -1]
        for idx, to_vertice in enumerate(odd_vertices):
            distance = edge_dict.get((from_vertice, to_vertice,), float('inf'))
            if distance < best_edge[0][0]:
                best_edge = [[distance, from_vertice, to_vertice], idx]
        minimum_weight_matching.append(best_edge[0])
        odd_vertices.pop(best_edge[1])

    eulerian_graph_edges = mst + minimum_weight_matching
    eulerian_graph = build_graph_from_edges(eulerian_graph_edges)[0]
    eulerian_path = get_eulerian_path(eulerian_graph, vrp_instance.fleet.departure_city)

    seen_nodes = {vrp_instance.fleet.departure_city, vrp_instance.fleet.arrival_city}  # prevent start and end on route
    # hamiltonian_tour = [el for el in eulerian_path if el not in seen_nodes and not seen_nodes.add(el)] + [
    #     eulerian_path[0]]
    # cost = compute_cost_ed(hamiltonian_tour, edge_dict)
    route = [el for el in eulerian_path if el not in seen_nodes and not seen_nodes.add(el)]
    return route


def christofides_on_solution(vrp_instance, routes):
    for r, route in enumerate(routes):
        routes[r] = christofides(vrp_instance, route)
    return routes


def yacos_gen_initial_solution(vrp_instance, cur_attempt=0, max_attempts=3):
    if cur_attempt >= max_attempts:
        vrp_instance.fleet.size += 1
        cur_attempt = 0
    routes = [[] for _ in range(vrp_instance.fleet.size)]
    remaining_cities = [city for city in vrp_instance.cities.keys()]
    remove_if_exists(remaining_cities, vrp_instance.fleet.departure_city)  # always the start
    remove_if_exists(remaining_cities, vrp_instance.fleet.arrival_city)  # always the end
    while len(remaining_cities) > 0:
        for route in routes:
            random_city = rd.choice(remaining_cities)
            route.append(random_city)
            remaining_cities.remove(random_city)
            if len(remaining_cities) == 0:
                break
    if not is_valid_solution(vrp_instance, routes):
        return yacos_gen_initial_solution(vrp_instance, cur_attempt=cur_attempt + 1, max_attempts=max_attempts)
    routes = christofides_on_solution(vrp_instance, routes)
    return routes


def yacos_refine_solution(vrp_instance, solution):
    #  TODO check cost route by route instead of aggregated sum
    cost = get_solution_cost(vrp_instance, solution)
    chris_sol = christofides_on_solution(vrp_instance, copy_sol(solution))
    chris_cost = get_solution_cost(vrp_instance, chris_sol)
    if chris_cost < cost:
        return chris_cost, chris_sol
    else:
        return cost, solution


def yacos(vrp_instance, max_iter_ms, max_iter_ils=None):
    compute_fleet_size(vrp_instance)
    best_cost = float('inf')
    best_solution = None
    if max_iter_ils is None:
        beta = 5
        n = len(vrp_instance.cities)
        v = vrp_instance.fleet.size
        max_iter_ils = n + beta * v
    for i in range(max_iter_ms):
        initial_solution = yacos_gen_initial_solution(vrp_instance)
        candidate_solution = initial_solution
        current_solution = initial_solution
        current_cost = get_solution_cost(vrp_instance, current_solution)
        candidate_cost = current_cost
        j = 0
        while j < max_iter_ils:
            candidate_solution = rvnd(vrp_instance, candidate_solution)
            candidate_cost = get_solution_cost(vrp_instance, candidate_solution)
            if candidate_cost < current_cost:
                current_solution = candidate_solution
                current_cost = candidate_cost
                j = 0
            candidate_solution = shake(vrp_instance, current_solution)
            candidate_cost = get_solution_cost(vrp_instance, candidate_solution)
            j += 1
        if candidate_cost < best_cost:
            best_cost, best_solution = yacos_refine_solution(vrp_instance, candidate_solution)
    return best_cost, best_solution


def save_results_report(rep, dst_path):
    try:
        with open(dst_path, 'w') as f:
            json.dump(rep, f, indent=2)
    except Exception as e:
        print(e)


report = {}

for m_iter in MAX_ITER_MS:
    for instance in INSTANCES:
        if RUN_MILS_RVND:
            try:
                vrp_instance = load_vrp_file(instance)
                runtime_s = -perf_counter()
                cost, sol = mils_rvnd(vrp_instance, m_iter, MAX_ITER_ILS)
                runtime_s += perf_counter()
                if instance not in report:
                    report[instance] = {}
                report[instance]['mils_rvnd'] = {
                    'cost': cost,
                    'fleet_size': len(sol),
                    'runtime_s': runtime_s,
                    'sol': str(sol),
                }
                save_results_report(report, REPORT_PATH.replace('#', f'm_iter-{m_iter}'))
            except Exception as e:
                print('ERORRRRRR---')
                print(e)
                traceback.print_exc()
                print('ERORRRRRR---')

        if RUN_YACOS:
            try:
                vrp_instance = load_vrp_file(instance)
                runtime_s = -perf_counter()
                cost, sol = yacos(vrp_instance, m_iter, MAX_ITER_ILS)
                runtime_s += perf_counter()
                if instance not in report:
                    report[instance] = {}
                report[instance]['YaCos'] = {
                    'cost': cost,
                    'fleet_size': len(sol),
                    'runtime_s': runtime_s,
                    'sol': str(sol),
                }
                save_results_report(report, REPORT_PATH.replace('#', f'm_iter-{m_iter}'))
            except Exception as e:
                print('ERORRRRRR---')
                print(e)
                traceback.print_exc()
                print('ERORRRRRR---')

print()
