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

def get_routes(x, locations):
    n = len(locations)
    routes = [[0, j] for j in range(n) if x[0, j].X == 1]
    for route in routes:
        while route[-1] != 0:
            for j in range(n):
                if x[route[-1], j].X == 1:
                    route.append(j)
                    break

    for route in routes:
        for j in range(len(route)):
            route[j] = locations[route[j]]
    return routes

def vrp1(locations, d, C):
    distance_matrix, _ = generate_distance_matrix(locations)
    for i in range(len(distance_matrix)):
        distance_matrix[i][i] = INF

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

# def scvrp():
#     distance_matrix = get_distance_matrix([])
#     INF = gp.GRB.INFINITY
#     for i in range(len(distance_matrix)):
#         distance_matrix[i][i] = INF

#     d = [0, 1, 1, 2, 4, 2, 4, 8, 8, 1, 2, 1, 2, 4, 4, 8, 8]
#     capacities = [15]*4
#     n = len(distance_matrix)
#     K = len(capacities)
#     C = capacities[0] # Fixed capacities for now

#     model = gp.Model('Vehicle Flow Model - SCVRP')
#     # Add variables

#     x = model.addVars([(i, j) for i in range(1, n) for j in range(i+1, n)], vtype=gp.GRB.BINARY, name='x')
#     for j in range(1, n):
#         x[0, j] = model.addVar(ub=2, vtype=gp.GRB.INTEGER)

#     u = model.addVars(range(n), vtype=gp.GRB.CONTINUOUS)
#     # Add constraints
#     model.addConstrs(x.sum('*', i) + x.sum(i, '*') == 2 for i in range(1, n))
#     model.addConstr(x.sum(0, '*') == 2*K)

#     sets = non_empty_powerset(range(1, n))
#     model.addConstrs()

#     model.setObjective(gp.quicksum(distance_matrix[i][j]*x[i, j] for i in range(n) for j in range(i+1, n)), gp.GRB.MINIMIZE)
#     model.optimize()

#     for i in range(n):
#         for j in range(i+1, n):
#             if (x[i, j].X > 0):
#                 print((i, j), x[i,j].X)

#     for i in [11, 12, 13, 15]:
#         print(i, u[i])


if __name__ == '__main__':
    pass