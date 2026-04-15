import math
import numpy as np
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

# -------------------------------
# Distance Function
# -------------------------------
def haversine(coord1, coord2):
    R = 6371
    lat1, lon1 = coord1
    lat2, lon2 = coord2

    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return R * (2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)))

# -------------------------------
# Simulated Traffic Zones
# -------------------------------
traffic_zones = [
    {"center": (12.9352, 77.6245), "radius": 1.5}
]

def traffic_penalty(coord):
    for zone in traffic_zones:
        if haversine(coord, zone["center"]) < zone["radius"]:
            return 5
    return 1

# -------------------------------
# Cost Matrix (AI Core)
# -------------------------------
def create_distance_matrix(locations, carbon_weight):
    size = len(locations)
    matrix = np.zeros((size, size))

    for i in range(size):
        for j in range(size):
            if i != j:
                dist = haversine(locations[i], locations[j])
                penalty = traffic_penalty(locations[j])
                matrix[i][j] = dist * penalty * carbon_weight

    return matrix

# -------------------------------
# Optimization
# -------------------------------
def solve_route(distance_matrix):
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    def callback(from_index, to_index):
        return int(distance_matrix[
            manager.IndexToNode(from_index)
        ][
            manager.IndexToNode(to_index)
        ] * 1000)

    transit_callback_index = routing.RegisterTransitCallback(callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    solution = routing.SolveWithParameters(search_params)

    route = []
    index = routing.Start(0)

    while not routing.IsEnd(index):
        route.append(manager.IndexToNode(index))
        index = solution.Value(routing.NextVar(index))

    route.append(0)
    return route

# -------------------------------
# Metrics
# -------------------------------
def calculate_metrics(route, locations):
    total = 0
    for i in range(len(route)-1):
        total += haversine(locations[route[i]], locations[route[i+1]])
    co2 = total * 0.12
    return total, co2

# -------------------------------
# Final Function (IMPORTANT)
# -------------------------------
def get_optimized_route(locations, carbon_weight):
    matrix = create_distance_matrix(locations, carbon_weight)
    route = solve_route(matrix)
    distance, co2 = calculate_metrics(route, locations)

    return {
        "route": route,
        "distance": distance,
        "co2": co2
    }