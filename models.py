from math import ceil
import gurobipy as gp
from itertools import chain, combinations

def get_distance_matrix(locations):
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
    return distance_matrix

# def non_empty_powerset(l):
#     return list(chain.from_iterable(combinations(l, r) for r in range(1, len(l)+1)))

# def min_vehicles(s, d, C):
#     return ceil(sum(d[i] for i in s)/C)

def test():
    distance_matrix = get_distance_matrix([])
    INF = gp.GRB.INFINITY
    for i in range(len(distance_matrix)):
        distance_matrix[i][i] = INF

    d = [0, 1, 1, 2, 4, 2, 4, 8, 8, 1, 2, 1, 2, 4, 4, 8, 8]
    capacities = [15]*4
    n = len(distance_matrix)
    K = len(capacities)
    C = capacities[0] # Fixed capacities for now

    model = gp.Model('Vehicle Flow Model - ACVRP')
    # Add variables
    x = model.addVars(range(n), range(n), vtype=gp.GRB.BINARY, name='x')
    u = model.addVars(range(n), vtype=gp.GRB.CONTINUOUS)
    # Add constraints
    model.addConstrs(x.sum('*', j) == 1 for j in range(1, n)) # indegree constraints
    model.addConstrs(x.sum(i, '*') == 1 for i in range(1, n)) # outdegree constraints
    model.addConstr(x.sum('*', 0) == K) # indegree depot
    model.addConstr(x.sum(0, '*') == K) # outdegree depot
    model.addConstrs(u[i] - u[j] + C*x[i, j] <= C - d[j] 
        for i in range(1, n) for j in range(1, n) 
        if i != j and d[i] + d[j] <= C)
    model.addConstrs(d[i] <= u[i] for i in range(1, n))
    model.addConstrs(u[i] <= C for i in range(1, n))

    model.setObjective(gp.quicksum(distance_matrix[i][j]*x[i, j] for i in range(n) for j in range(n)), gp.GRB.MINIMIZE)
    model.optimize()

    for i in range(n):
        for j in range(n):
            if (x[i, j].X == 1):
                print((i, j), x[i,j].X)

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
    test()
    # scvrp()