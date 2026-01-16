import numpy as np
import itertools


# constructs the schedule for one robot on the path graph
# if return_schedule==False calculates the schedule's length only
def C_1(graph, location_of_robot, return_schedule=False):

    task_locations = list(np.nonzero(graph)[0])
    if len(task_locations) == 0:
        return 0
    
    leftmost_task = task_locations[0]
    rightmost_task = task_locations[-1]
    left_first = sum([graph[i] for i in task_locations]) + (rightmost_task - leftmost_task + abs(location_of_robot - leftmost_task ))
    right_first = sum([graph[i] for i in task_locations]) + (rightmost_task - leftmost_task + abs(location_of_robot - rightmost_task))
    
    if not return_schedule:
        return min(left_first, right_first), None

    sorted_task_locations = task_locations.copy() if left_first < right_first else task_locations[::-1]
    schedule = [location_of_robot]

    list_of_lists = [schedule]

    for task in sorted_task_locations:
        
        last = list_of_lists[-1][-1]
        if np.sign(task - last) > 0:
            list_of_lists.append(range(last + 1, task + 1))
        else:
            list_of_lists.append(range(last - 1, task - 1, -1))
        
        list_of_lists.append([int(task)]*int(graph[task]))  # add the task duration to the schedule

    schedule = list(itertools.chain.from_iterable(list_of_lists))

    if int(min(left_first, right_first)) != len(schedule) - 1:
        print(f"Error in C_1: {min(left_first, right_first)} != {len(schedule)} for graph {graph} and robot at {location_of_robot}")

    return len(schedule) - 1, schedule

# calculates the near-optimal schedule on the path graph
# <graph> is an array of integer numbers, i.e. [0 1 3 0 0 0 0],
# containing either the task duration 
# or 0 for vertices with no tasks
# <robots> is the list of robots starting positions
def Partition_Algorithm(graph, robots):

    k = len(robots)
    task_locations = list(np.nonzero(graph)[0])
    m = len(task_locations)
    S = np.zeros(shape=(k, m+1))
    split = np.zeros(shape=(k, m+1))
    split_point = 0

    all_schedules = {'(0, 0)': [[robots[0]]]}

    # fill in <all tasks for one> schedules
    for l in range(m):
        S[0][l+1], schedule = C_1(graph[:task_locations[l]+1], robots[0], return_schedule=True)
        all_schedules[str((0, l+1))] = [schedule]

    # fill in <empty> schedules
    for c in range(1, k):
        all_schedules[str((c, 0))] = [[robots[c]]] 

    # the main loop with the auxiliary S table filled in
    for c in range(1, k): # for each robot
        for l in range(m): # for each task
            current_min = float('inf')
            r_min = 0

            for r in range(l+1):

                # first c-1 vertices are not available for the partition for c as we have c-1 robots on the left
                if task_locations[r] <= c-1:
                    continue

                # last k - (c+1) vertices are not available for the partition as well
                if task_locations[r] > len(graph) - 1 - (k - c - 1):
                    break

                temp_graph = np.asarray(graph.copy())
                temp_graph[0:task_locations[r]] = 0
                temp_graph[task_locations[l]+1:len(graph)] = 0

                s_len, schedule = C_1(temp_graph, robots[c], return_schedule=False)
                current_val = max(S[c-1][r], s_len)
            
                # if the current partition gives a shorter set of schedules we choose it
                if current_val < current_min:

                    r_min = r
                    current_min = current_val

                elif current_val == current_min:

                    # if the robot (c) is between (c-1) and task (r) it is better to assign r to c 
                    # to avoid collision in the case if c would become stationary robot
                    if abs(robots[c] - (task_locations[r])) < \
                       abs(robots[c-1] - task_locations[r]):
                            
                            r_min = r

            # if robot (c) stays stationary
            if S[c-1][l+1] < current_min or \
                (S[c-1][l+1] == current_min and abs(robots[c] - task_locations[r_min]) > \
                                                abs(robots[c-1] - task_locations[r_min])):

                r_min = l+1
                current_min = S[c-1][r_min]
                schedule = [robots[c]]
                
            # if robot (c) has a non-empty schedule
            else:

                temp_graph = np.asarray(graph.copy())
                temp_graph[0:task_locations[r_min]] = 0
                temp_graph[task_locations[l]+1:len(graph)] = 0

                s_len, schedule = C_1(temp_graph, robots[c], return_schedule=True)

            split_point = r_min
            all_schedules[str((c, l+1))] = all_schedules[str((c-1, r_min))] + [schedule]

            split[c][l+1] = split_point
            S[c][l+1] = current_min

    res_schedule = all_schedules[str((k-1, m))]

    return int(S[k-1][m]), res_schedule