import numpy as np


def try_update_schedule(graph, robot_schedules, robot, task):

    start_timestep = len(robot_schedules[robot])  # current timestep of the robot
    robot_loc = robot_schedules[robot][-1] # robot location at the end of the last step
    
    subschedule = [robot_loc]
    while len(subschedule) <= abs(task - robot_loc):  # move to the task
        subschedule.append(int(subschedule[-1] + np.sign(task - robot_loc)))  # move to the task
    
    subschedule.extend([int(task)]*int(graph[task])) #add the task duration to the schedule
    del subschedule[0] # remove the first element, which is the robot's current location

    for other_robot in robot_schedules: # check if any other robot is at the same location at the same timestep
        
        if other_robot == robot:
            continue

        # if the other robot's schedule is not long enough, we check if its latest location is between the robot and the task
        if len(robot_schedules[other_robot]) < start_timestep + len(subschedule):
            if robot_schedules[other_robot][-1] in subschedule[max(0, len(robot_schedules[other_robot]) - start_timestep):-1]:
                return False
            
        # if the other robot's schedule is longer
        if len(robot_schedules[other_robot]) > start_timestep + len(subschedule): 
            #  # check if the final vertex is in the other robot's schedule aftewards
            if subschedule[-1] in robot_schedules[other_robot][start_timestep + len(subschedule):]:
                return False            

        for i in range(len(subschedule)): # for each step in subschedule

            # if the schedule of the other robot is long enough
            if len(robot_schedules[other_robot]) > start_timestep + i:
                # check the collision
                if robot_schedules[other_robot][start_timestep + i] == subschedule[i]:
                    return False

    robot_schedules[robot].extend(subschedule) #finally, we add the subschedule to the robot's schedule
    return True  # if no collisions, return True


def Greedy_Algorithm(graph, robots):

    task_locations = list(np.nonzero(graph)[0])
    robot_task_pairs = [(a, b) for a in robots for b in task_locations]
    robot_task_pairs.sort(key=lambda x: abs(x[0]-x[1]))  # sort by distance to the task

    robot_schedules = {robot: [robot] for robot in robots}
    assigned_tasks = []

    while robot_task_pairs:

        for robot, task in robot_task_pairs:

            if task in assigned_tasks: #check if the task is already assigned
                continue

            if try_update_schedule(graph, robot_schedules, robot, task): #check if the schedule is collision-free and update it if so
                
                assigned_tasks.append(task)
            
                robot_task_pairs = [(a, b) for a in robots for b in task_locations if b not in assigned_tasks]
                robot_task_pairs.sort(key=lambda x: abs(robot_schedules[x[0]][-1]-x[1]) + len(robot_schedules[x[0]]))
                break

    return max([len(schedule) for schedule in robot_schedules.values()]) - 1, list(robot_schedules.values())