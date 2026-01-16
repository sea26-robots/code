import numpy as np
import random
from Greedy_Algorithm import try_update_schedule


# calculates a schedule on the path graph
# <graph> is an array of integer numbers, i.e. [0 1 3 0 0 0 0],
# containing either the task duration 
# or 0 for vertices with no tasks
# <robots> is the list of robots starting positions
# def Random_Algorithm(graph, robots, attempt=0):
def Random_Algorithm(graph, robots):

    task_locations = list(np.nonzero(graph)[0])
    robot_task_pairs = [(a, b) for a in robots for b in task_locations]
    # randomize pairs
    random.shuffle(robot_task_pairs)

    robot_schedules = {robot: [robot] for robot in robots}
    assigned_tasks = []

    while robot_task_pairs:

        # updated = False
        for robot, task in robot_task_pairs:

            if task in assigned_tasks: #check if the robot is already done or the task is already assigned
                continue

            if try_update_schedule(graph, robot_schedules, robot, task): #check if the schedule is collision-free and update it if so
                
                # updated = True
                assigned_tasks.append(task)
                available_task_locations = [t for t in task_locations if t not in assigned_tasks]
                random.shuffle(available_task_locations)

                if available_task_locations is None:
                    robot_task_pairs = None
                else:
                    robot_task_pairs = [(a, b) for a in robots for b in available_task_locations]
                    robot_task_pairs.sort(key=lambda x: len(robot_schedules[x[0]]))
                
                break

        #if the algorithm stuck and cannot assign any more tasks, we restart it
        # if not updated:
        #     return Random_Algorithm(graph, robots, attempt=attempt+1)

    return max([len(schedule) for schedule in robot_schedules.values()]) - 1, list(robot_schedules.values())