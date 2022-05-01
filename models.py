from math import ceil
from itertools import chain, combinations
import gurobipy as gp
from distance_matrix import generate_distance_matrix

INF = gp.GRB.INFINITY

# def non_empty_powerset(l):
#     return list(chain.from_iterable(combinations(l, r) for r in range(1, len(l)+1)))

# def min_vehicles(s, d, C):
#     return ceil(sum(d[i] for i in s)/C)


def get_model_status(model):
    status = model.getAttr(gp.GRB.Attr.Status)
    status_str = 'optimal'
    if status == gp.GRB.INFEASIBLE:
        status_str = 'infeasible'
    elif status == gp.GRB.UNBOUNDED:
        status_str = 'unbounded'

    return status, status_str

def create_route_matrix(x, n, model=None):
    route_matrix = [[] for _ in range(n)]
    if model == 'vrp4':
        for i in range(n):
            for j in range(i+1, n+1):
                if x[i, j].X == 1 or x[j, i].X == 1:
                    route_matrix[i].append(j%n)
                    route_matrix[j%n].append(i)
        for j in range(n):
            if x[n, j].X == 1 or x[j, n].X == 1:
                route_matrix[0].append(j)

    else:
        for i in range(n):
            for j in range(n):
                if x[i, j].X == 1:
                    route_matrix[i].append(j)
        

    print(route_matrix)
    return route_matrix

def get_routes(x, locations, model=None):
    n = len(locations)
    route_matrix = create_route_matrix(x, n, model)
    routes = {j: [0, j] for j in route_matrix[0]}
    processed = [False]*n
    for j in range(n):
        if j not in routes: continue
        if processed[j]:
            routes.pop(j)
            continue
        processed[j] = True
        route = routes[j]
        while route[-1] != 0:
            for x in route_matrix[route[-1]]:
                if x != route[-2]:
                    route.append(x)
                    processed[x] = True
                    break

    routes = [routes[j] for j in routes]         
    for route in routes:
        for j in range(len(route)):
            route[j] = locations[route[j]]
    return routes

def modify_distance_matrix(distance_matrix, model=None):
    if model == 'vrp4':
        # Force a symmetric distance matrix
        for i in range(1, len(distance_matrix)):
            for j in range(i+1):
                distance_matrix[i][j] = distance_matrix[j][i]

        for row in distance_matrix:
            row.append(row[0])
        distance_matrix.append(distance_matrix[0])

    for i in range(len(distance_matrix)):
        distance_matrix[i][i] = INF

def vrp1(locations, d, C):
    distance_matrix, _ = generate_distance_matrix(locations)
    modify_distance_matrix(distance_matrix)

    n = len(distance_matrix)
    K = ceil(sum(d)/C)

    model = gp.Model('Vehicle Flow Model - ACVRP')
    # Add variables
    x = model.addVars(range(n), range(n), vtype=gp.GRB.BINARY, name='x')
    u = model.addVars(range(n), vtype=gp.GRB.CONTINUOUS)
    # Add constraints
    model.addConstrs(x.sum('*', j) == 1 for j in range(1, n))  # indegree constraints
    model.addConstrs(x.sum(i, '*') == 1 for i in range(1, n))  # outdegree constraints
    model.addConstr(x.sum('*', 0) == K)  # indegree depot
    model.addConstr(x.sum(0, '*') == K)  # outdegree depot
    model.addConstrs(u[i] - u[j] + C*x[i, j] <= C - d[j]
                     for i in range(1, n) for j in range(1, n)
                     if i != j and d[i] + d[j] <= C)
    model.addConstrs(d[i] <= u[i] for i in range(1, n))
    model.addConstrs(u[i] <= C for i in range(1, n))

    model.setObjective(gp.quicksum(
        distance_matrix[i][j]*x[i, j] for i in range(n) for j in range(n)), gp.GRB.MINIMIZE)
    model.optimize()

    status, status_str = get_model_status(model)
    if status != gp.GRB.OPTIMAL:
        return {'status': status_str}

    routes = get_routes(x, locations)
    print(routes)
    return {
        'status': status_str,
        'routes': routes
    }

def vrp3(locations, d, C):
    if type(C) == int:
        C = [C]*ceil(sum(d)/C)

def vrp4(locations, d, C):
    distance_matrix, _ = generate_distance_matrix(locations)
    modify_distance_matrix(distance_matrix, 'vrp4')

    n = len(distance_matrix)
    K = ceil(sum(d)/C)

    model = gp.Model('Commodity Flow Model - SCVRP')
    # Add variables
    x = model.addVars(range(n), range(n), vtype=gp.GRB.BINARY, name='x')
    y = model.addVars(range(n), range(n), vtype=gp.GRB.CONTINUOUS)
    # Add constraints
    model.addConstrs(y.sum('*', i) - y.sum(i, '*') == 2*d[i] for i in range(1, n-1))
    model.addConstr(gp.quicksum(y[0, j] for j in range(1, n-1)) == sum(d))
    model.addConstr(gp.quicksum(y[j, 0] for j in range(1, n-1)) == K*C - sum(d))
    model.addConstr(gp.quicksum(y[n-1, j] for j in range(1, n-1)) == K*C)
    model.addConstrs(y[i, j] + y[j, i] == C*x[i, j] for i in range(n) for j in range(i+1, n))
    model.addConstrs(x.sum(i, '*') + x.sum('*', i) == 2 for i in range(1, n-1))

    model.setObjective(gp.quicksum(
        distance_matrix[i][j]*x[i, j] for i in range(n) for j in range(n)), gp.GRB.MINIMIZE)
    model.optimize()

    status, status_str = get_model_status(model)
    if status != gp.GRB.OPTIMAL:
        return {'status': status_str}

    for i in range(n):
        for j in range(n):
            if (x[i, j].X == 1):
                print((i, j), x[i, j].X)

    routes = get_routes(x, locations, 'vrp4')
    print(routes)
    return {
        'status': status_str,
        'routes': routes
    }


if __name__ == '__main__':
    distance_matrix = [
        [
            0, 548, 776, 696, 582, 274, 502, 194, 308, 194, 536, 502, 388, 354,
            468, 776, 662
        ],
        [
            548, 0, 684, 308, 194, 502, 730, 354, 696, 742, 1084, 594, 480, 674,
            1016, 868, 1210
        ],
        [
            776, 684, 0, 992, 878, 502, 274, 810, 468, 742, 400, 1278, 1164,
            1130, 788, 1552, 754
        ],
        [
            696, 308, 992, 0, 114, 650, 878, 502, 844, 890, 1232, 514, 628, 822,
            1164, 560, 1358
        ],
        [
            582, 194, 878, 114, 0, 536, 764, 388, 730, 776, 1118, 400, 514, 708,
            1050, 674, 1244
        ],
        [
            274, 502, 502, 650, 536, 0, 228, 308, 194, 240, 582, 776, 662, 628,
            514, 1050, 708
        ],
        [
            502, 730, 274, 878, 764, 228, 0, 536, 194, 468, 354, 1004, 890, 856,
            514, 1278, 480
        ],
        [
            194, 354, 810, 502, 388, 308, 536, 0, 342, 388, 730, 468, 354, 320,
            662, 742, 856
        ],
        [
            308, 696, 468, 844, 730, 194, 194, 342, 0, 274, 388, 810, 696, 662,
            320, 1084, 514
        ],
        [
            194, 742, 742, 890, 776, 240, 468, 388, 274, 0, 342, 536, 422, 388,
            274, 810, 468
        ],
        [
            536, 1084, 400, 1232, 1118, 582, 354, 730, 388, 342, 0, 878, 764,
            730, 388, 1152, 354
        ],
        [
            502, 594, 1278, 514, 400, 776, 1004, 468, 810, 536, 878, 0, 114,
            308, 650, 274, 844
        ],
        [
            388, 480, 1164, 628, 514, 662, 890, 354, 696, 422, 764, 114, 0, 194,
            536, 388, 730
        ],
        [
            354, 674, 1130, 822, 708, 628, 856, 320, 662, 388, 730, 308, 194, 0,
            342, 422, 536
        ],
        [
            468, 1016, 788, 1164, 1050, 514, 514, 662, 320, 274, 388, 650, 536,
            342, 0, 764, 194
        ],
        [
            776, 868, 1552, 560, 674, 1050, 1278, 742, 1084, 810, 1152, 274,
            388, 422, 764, 0, 798
        ],
        [
            662, 1210, 754, 1358, 1244, 708, 480, 856, 514, 468, 354, 844, 730,
            536, 194, 798, 0
        ],
    ]
    d = [0, 1, 1, 2, 4, 2, 4, 8, 8, 1, 2, 1, 2, 4, 4, 8, 8]
    vrp1(distance_matrix, d, 15)
    pass