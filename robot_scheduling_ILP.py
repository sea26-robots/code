import gurobipy  as gp
from gurobipy import GRB
from functools import *
import ast

# n is the number of vertices
# tasks is the list of pairs (position, duration)
# sv is the list of starting positions of robots
def Optimize_Robot_Scheduling(n, tasks, robots, license, max_time=None, verbose=0):
    
    k = len(robots)
    m = len(tasks)
    # if the max lifetime is predicted by other methods we can use it to accelerate scheduling
    LIFETIME = n + min(robots[0], n - 1 - robots[-1]) + sum([t[1] for t in tasks]) if max_time == None else max_time

    # Create a new model
    options = license
    env = gp.Env(params=options)
    model = gp.Model("RobotScheduling", env=env)
    # Create variables
    x = model.addVars(k, n, LIFETIME, vtype=GRB.BINARY, name="x") # x[r,v,t] = true if robot i at vertex v at the end of timestep t
    mo = model.addVars(k, n, LIFETIME, vtype=GRB.BINARY, name="mo") # mo[r,v,t] = 1 if robot r just moved to vertex v during timestep t
    TCR = model.addVars(k, len(tasks), LIFETIME, vtype=GRB.BINARY, name="TCR") #TCR[r,i,t] = task i complete by robot r at time t
    TC = model.addVars(len(tasks), LIFETIME, vtype=GRB.BINARY, name="TC") # TC[i,t] = task i complete at timestep t
    AC = model.addVars(LIFETIME, vtype=GRB.BINARY, name="AC") #ALL_COMPLETE at time t
    TS = model.addVar(vtype=GRB.INTEGER, name="TS") #ALL_COMPLETE but this time its an integer

    for i in range(k):
        model.addConstr(x[i, robots[i], 0] == 1, name="initialise x sv_r")
        model.addConstr(mo[i, robots[i], 0] == 1, name="initialise mo sv_r")
    
    for r in range(k):
        for i in range(m):
            for t in range(tasks[i][1]):
                model.addConstr(TCR[r, i, t] == 0, name="TCR_Initial_Values") #there's no way a task can be completed before its duration
    
    for r in range(k):
        model.addConstrs((TCR[r, i, t] <= TCR[r, i, t-1] + (gp.quicksum((1 - mo[r, tasks[i][0], j]) * x[r, tasks[i][0], j] 
                                                                for j in range(t - tasks[i][1], t))/(tasks[i][1])) 
                                                                for i in range(m) for t in range(tasks[i][1], LIFETIME)),
                                                                name=f"task_complete_R{r}")    
    
    model.addConstrs((TC[i, t] >= TC[i, t-1] for i in range(m) for t in range(1, LIFETIME)), name="TC_stays_complete")
    model.addConstrs((TCR[r, i, t] >= TCR[r, i, t-1] for r in range(k) for i in range(m) for t in range(1, LIFETIME)),
                 name="TCR_stays_complete")
    
    model.addConstrs((TC[i, t] == gp.quicksum(TCR[r, i, t]  for r in range(k)) for i in range(m) for t in range(LIFETIME)),
                 name="TC iff TCR")
    
    for r in range(k):
        model.addConstrs((gp.quicksum(x[r, v, t] for v in range(n)) == 1 for t in range(LIFETIME)), name=f"R_{r} only_one_place")
        model.addConstrs((gp.quicksum(mo[r, v, t] for v in range(n)) <= 1 for t in range(LIFETIME)))
        
    model.addConstrs((mo[r, v, t] == gp.quicksum(x[r, v_prime, t-1] * x[r, v, t] for v_prime in [v-1, v+1]) \
                  for r in range(k) \
                  for v in range(1, n-1) \
                  for t in range(1, LIFETIME)),
                  name="just_moved")
    
    model.addConstrs((mo[r, 0, t] == x[r, 1, t-1] * x[r, 0, t] 
                  for r in range(k) 
                  for t in range(1, LIFETIME)), 
                  name="just_moved")
    
    model.addConstrs((mo[r, n-1, t] == x[r, n-2, t-1] * x[r, n-1, t] 
                  for r in range(k) 
                  for t in range(1, LIFETIME)), 
                  name="just_moved")
    
    model.addConstrs((gp.quicksum(x[r, v, t] 
                              for r in range(k)) <= 1 
                              for v in range(n) 
                              for t in range(LIFETIME)), 
                              name="collision_free")
    
    # We removed the constraints on traversing to accelerate the algorith, the traversing check and fix afterwards is advised
    # m.addConstrs(x[r,v,t] + x[r,v+1,t+1] + x[r_prime,v+1,t]+x[r_prime,v,t+1] <= 3 for (r,r_prime) in itertools.combinations(range(len(sv)), 2) for v in range(n-1) for t in range(LIFETIME-1))
    # m.addConstrs(x[r_prime,v,t] + x[r_prime,v+1,t+1] + x[r,v+1,t]+x[r,v,t+1] <= 3 for (r,r_prime) in itertools.combinations(range(len(sv)), 2) for v in range(n-1) for t in range(LIFETIME-1))
    
    model.addConstrs((x[r, v, t] <= (x[r, v-1, t-1] + x[r, v, t-1] + x[r, v+1, t-1]) 
                  for v in range(1, n-1) 
                  for t in range(1, LIFETIME) 
                  for r in range(k)), 
                  name="Adjacency") #Adjacency conditions
    
    model.addConstrs((x[r, 0, t] <= (x[r, 0, t-1] + x[r, 1, t-1]) 
                  for t in range(1, LIFETIME) 
                  for r in range(k)), 
                  name="adjacency_first")
    
    model.addConstrs((x[r, n-1, t] <= (x[r, n-2, t-1] + x[r, n-1, t-1]) 
                  for t in range(1, LIFETIME) 
                  for r in range(k)), 
                  name="adjacency_last")
    
    model.addConstrs((AC[t] <= gp.quicksum(TC[i, t] 
                                       for i in range(m)) / m 
                                       for t in range(LIFETIME)), 
                                       name="All_tasks_Complete?")
    
    model.addConstr(gp.quicksum(AC[t] for t in range(LIFETIME)) == 1)
    model.addConstr(TS == gp.quicksum(t*(AC[t]) for t in range(LIFETIME)))
    
    model.setObjective(TS - 1, GRB.MINIMIZE)
        
    model.setParam('OutputFlag', verbose)
    model.optimize()

    if verbose:

        try:
            schedule = "["
            for r in range(k):
                schedule += "["
                for t in range(int(TS.X)):
                    for v in range(n):
                        if x[r, v, t].X == 1:
                            schedule+=f"{v}, "
                
                schedule = schedule[:-2] + "], "
            schedule = schedule[:-2] + "]"
        
        except AttributeError as e:
            print("No solution found. Error:", e)
            print("Input parameters:")
            print(f"n: {n}, tasks: {tasks}, robots: {robots}, max_time: {max_time}, verbose: {verbose}")

        schedule = ast.literal_eval(schedule)

    return(int(TS.X - 1), schedule)
