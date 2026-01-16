from pathlib import Path
import numpy as np
import time
import Partition_Algorithm
import Greedy_Algorithm
import Random_Algorithm
import robot_scheduling_ILP
import argparse
from collections import Counter
import os
import json
import pandas as pd
import ast
from generate_instances import generate_random_instance, generate_tasks_durations, generate_positions

def collision_free_check(schedule):

    # Find maximum length
    max_len = max(len(lst) for lst in schedule)

    # Pad each list with its last element
    padded = [
        lst + [lst[-1]] * (max_len - len(lst)) if lst else []  # handles empty lists
        for lst in schedule
    ]

    duplicates = [
    (idx, col) for idx, col in enumerate(zip(*padded))
    if any(count > 1 for count in Counter(map(int, col)).values())
    ]

    if len(duplicates) > 0:
        
        print(f"Collisions at timesteps: {duplicates}")
        for idx, col in enumerate(zip(*padded)):
            print(Counter(map(int, col)).values())

        for s in schedule:
            print(s)

    return len(duplicates) == 0


def run_algos(algos, instance, robots, IP_licence=None):

    task_locations = list(np.nonzero(instance)[0])
    tasks = [(task, int(instance[task])) for task in task_locations]

    def run_algorithm(a, max_length):

        if a == "p":
            return Partition_Algorithm.Partition_Algorithm(instance, robots)
        elif a == "i":
            return robot_scheduling_ILP.Optimize_Robot_Scheduling(len(instance), 
                                                                    tasks, 
                                                                    robots,
                                                                    license=IP_licence,
                                                                    max_time=max_length + 2,
                                                                    verbose=1)
        elif a == "g":
            return Greedy_Algorithm.Greedy_Algorithm(instance, robots)
        elif a == "r":
            return Random_Algorithm.Random_Algorithm(instance, robots)
        else:
            raise AssertionError(f"Unknown scheduling algorithm {a}")

    schedules = {}
    s_lengths = {}
    times = {}

    max_length = len(instance)*2 + sum(instance)

    for a in ["p", "i", "g", "r"]:

        s_lengths[a] = 0
        schedules[a] = None
        times[a] = 0

        start_time = time.time()
        
        if a in algos:
            s_lengths[a], schedules[a] = run_algorithm(a, max_length)
            max_length = min(max_length, s_lengths[a])
        
        times[a] = time.time() - start_time

    return s_lengths, schedules, times

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Optional app description')
    parser.add_argument('--config', type=str, default="config.json",
                        help='The config file for the experiment')
    parser.add_argument("--input_file", type=str, default=None,
                        help="Input file with instances and robots")
    parser.add_argument("--output_dir", type=str, default=None,
                        help="Output dir for results")
    parser.add_argument("--algos", type=str, default=None,
                        help="The algorithms to run: p - Partition, i - IP, g - Greedy, r - Random")
    
    args = parser.parse_args()

    with open(args.config, 'r') as f:
        config = json.load(f)

    algos = args.algos if args.algos is not None else config.get("algos", "pigr")
    
    licence = None
    if "i" in algos:
        licence = { "WLSACCESSID": config.get("WLSACCESSID", None),
                    "WLSSECRET": config.get("WLSSECRET", None),
                    "LICENSEID": config.get("LICENSEID", None)}

    input_file = args.input_file if args.input_file is not None else config.get("input_file", None)
    output_dir = args.output_dir if args.output_dir is not None else config.get("output_dir", "output")
    
    os.makedirs(output_dir, exist_ok=True)

    if input_file:

        input_df = pd.read_csv(input_file, names=['n_vertices', 'tasks', 'robots'])
        input_file_name = Path(input_file).stem

        for idx, row in input_df.iterrows():

            tasks = ast.literal_eval(row["tasks"])
            instance = np.zeros(row["n_vertices"])
            for t in tasks:
                instance[t[0]] = t[1]
            robots = ast.literal_eval(row["robots"])
            s_lengths, schedules, times = run_algos(algos, instance, robots, IP_licence=licence)

            f = open(f"{output_dir}/{input_file_name}.csv","a")

            output_str = f"{len(instance)},{len(robots)},\"{row['tasks']}\",\"{robots}\","
            for a in algos:
                lengths_str = f"{s_lengths[a]},"
                times_str = f"{times[a]},"
                schedules_str = f"\"{schedules[a]}\","
            
            output_str += lengths_str + times_str + schedules_str + "\n"

            f.write(output_str)

    else:

        # path length
        for n_vertices in range(config.get("path_vertices_min", 3), 
                                 config.get("path_vertices_max", 10) + 1, 
                                 config.get("path_vertices_step", 1)):

            f = open(f"{output_dir}/{n_vertices}.csv","a") 

            robots_n_max = n_vertices-1 if config.get("robots_num_max", n_vertices-1) is None else config.get["robots_num_max"]
            robots_n_max = min(robots_n_max, n_vertices-1)

            tasks_n_max = n_vertices if config.get("tasks_num_max", n_vertices) is None else config.get["tasks_num_max"]
            tasks_n_max = min(tasks_n_max, n_vertices)
            
            # number of tasks
            for n_tasks in range(config.get("tasks_num_min", 1), 
                                 tasks_n_max + 1, 
                                 config.get("tasks_num_step", 1)):
                
                # tasks durations
                for dur in range(config.get("dur_param_min", 1), 
                                 config.get("dur_param_max", 10) + 1, 
                                 config.get("dur_param_step", 1)):
                    
                    #for each instance 
                    for instance_id in range(config.get("max_instances_num", 10)):

                        tasks_durations = generate_tasks_durations(dur, n_tasks, config.get("tasks_dur_distr", "equal"))
                        instance = generate_random_instance(n_vertices, tasks_durations, config.get("tasks_pos_distr", "uniform"))

                        # number of robots
                        for n_robots in range(config.get("robots_n_min", 2),
                                              robots_n_max + 1, 
                                              config.get("robots_n_step", 1)): #for the number of robots
                                
                            robots = generate_positions(n_vertices, n_robots, config.get("robots_distr", "uniform"))
                            task_locations = list(np.nonzero(instance)[0])
                            tasks = [(task, int(instance[task])) for task in task_locations]

                            s_lengths, schedules, times = run_algos(algos, instance, robots, IP_licence=licence)

                            output_str = f"{n_vertices},{n_robots},\"{tasks}\",\"{robots}\","
                            
                            for a in "pigr":
                                lengths_str = f"{s_lengths[a]},"
                                times_str = f"{times[a]},"
                                schedules_str = f"\"{schedules[a]}\","
                            
                            output_str += lengths_str + times_str + f"{dur}," + schedules_str + "\n"

                            f.write(output_str)

                            with open(f"{output_dir}/collisions.txt", "a") as collisions_file:

                                for a in algos:
                                    if not collision_free_check(schedules[a]):
                                        collisions_file.write(f"Collisions are detected in the schedule generated by algorithm {a} \
                                                            for instance {instance} and robots locations {robots}.\n")
                                        collisions_file.write("Schedule with collisions: {}\n".format(schedules[a]))

            f.close()

                    
