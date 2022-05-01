from math import ceil
from itertools import chain, combinations
import gurobipy as gp
from distance_matrix import generate_distance_matrix

INF = gp.GRB.INFINITY

def get_model_status(model):
    status = model.getAttr(gp.GRB.Attr.Status)
    status_str = 'optimal'
    if status == gp.GRB.INFEASIBLE or model.getAttr(gp.GRB.Attr.ObjVal) >= INF:
        status_str = 'infeasible'
    elif status == gp.GRB.UNBOUNDED:
        status_str = 'unbounded'

    return status_str

def create_route_matrix(x, n, K=None, model=None):
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
    elif model == 'vrp3':
        for k in range(K):
            for i in range(n):
                for j in range(n):
                    if x[i, j, k].X == 1:
                        route_matrix[i].append(j)
    else:
        for i in range(n):
            for j in range(n):
                if x[i, j].X == 1:
                    route_matrix[i].append(j)
        
    for i in range(len(route_matrix)):
        route_matrix[i] = list(set(route_matrix[i]))
    print(route_matrix)
    return route_matrix

def get_routes(x, locations, K=None, model=None):
    n = len(locations)
    route_matrix = create_route_matrix(x, n, K, model)
    routes = {j: [0, j] for j in route_matrix[0]}
    processed = [False]*n
    for j in range(n):
        print(routes)

        if j not in routes: continue
        if processed[j]:
            routes.pop(j)
            continue
        processed[j] = True
        route = routes[j]
        while route[-1] != 0:
            if len(route_matrix[route[-1]]) == 1:
                route.append(route_matrix[route[-1]][0])
                continue
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


def process_result(model, locations, x, K=None, type=None):
    status_str = get_model_status(model)
    if status_str != 'optimal':
        return {'status': status_str}

    routes = get_routes(x, locations, K, type)
    return {'status': status_str, 'routes': routes}

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
    return process_result(model, x)
    

def vrp3(locations, d, C):
    if type(C) == int:
        C = [C]*ceil(sum(d)/C)

    distance_matrix, _ = generate_distance_matrix(locations)
    modify_distance_matrix(distance_matrix)
    n = len(distance_matrix)
    K = len(C)

    model = gp.Model('3-index Vehicle Flow Model - ACVRP')
    # Add variables
    x = model.addVars(range(n), range(n), range(K),
                      vtype=gp.GRB.BINARY, name='x')
    y = model.addVars(range(n), range(K), vtype=gp.GRB.BINARY)
    u = model.addVars(range(n), range(K), vtype=gp.GRB.CONTINUOUS)
    # Add constraints
    model.addConstrs(y.sum(i, '*') == 1 for i in range(1, n))
    model.addConstr(y.sum(0, '*') == K)
    model.addConstrs(x.sum(i, '*', k) == x.sum('*', i, k) for i in range(n) for k in range(K))
    model.addConstrs(x.sum(i, '*', k) == y[i, k] for i in range(n) for k in range(K))
    model.addConstrs(u[i, k] - u[j, k] + C[k]*x[i, j, k] <= C[k] - d[j]
                     for i in range(1, n) for j in range(1, n) for k in range(K)
                     if i != j and d[i] + d[j] <= C[k])
    model.addConstrs(d[i] <= u[i, k] for i in range(1, n) for k in range(K))
    model.addConstrs(u[i, k] <= C[k] for i in range(1, n) for k in range(K))


    model.setObjective(gp.quicksum(distance_matrix[i][j]*sum(
        x[i, j, k] for k in range(K)) for i in range(n) for j in range(n)), gp.GRB.MINIMIZE)
    model.optimize()

    # for i in range(n):
    #     for j in range(n):
    #         for k in range(K):
    #             if (x[i, j, k].X == 1):
    #                 print((i, j, k), x[i, j, k].X)
    # print()
    # for i in range(n):
    #     for k in range(K):
    #         print((i, k), u[i, k].X)

    return process_result(model, locations, x, K, 'vrp3')


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

    return process_result(model, locations, x, K, 'vrp4')

if __name__ == '__main__':
    locations = [[40.013077638324305, -105.26256374597169], [40.017942042003895, -105.28642467736817],
                 [40.00755546262642, -105.279729883667], [40.003479284476946, -105.25535396813966],
                 [40.02517227155021, -105.24883083581544]]
    d = [0, 4, 2, 6, 5]
    vrp1(locations, d, 15)
