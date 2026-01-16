import numpy as np
import random

UNIFORM_DISTR = "uniform"
UNEVEN_UNIFORM_DISTR = "uneven_uniform"
UNEVEN_NORMAL_DISTR = "uneven_normal"
NORMAL_DISTR = "normal"
EQUAL_DURATIONS = "equal"

possible_task_permutations = {}


# generates <num_tasks> task durations following the distribution <distribution> with parameter <dur_param>
def generate_tasks_durations(dur_param, num_tasks, distribution=UNIFORM_DISTR):

    if distribution == UNIFORM_DISTR:
        return random.choices(range(1, dur_param+1), k=num_tasks)
    
    if distribution == UNEVEN_UNIFORM_DISTR:

        half = int(num_tasks / 2)
        half_1 = random.choices(range(1, int(dur_param/2)), k=half)
        half_2 = random.choices(range(int(dur_param/2), dur_param+1), k=num_tasks-half)

        if random.random() < 0.5:
            return half_1 + half_2
        else:
            return half_2 + half_1
    
    elif distribution == NORMAL_DISTR:
        tasks_durations = np.random.normal(loc=dur_param, scale=dur_param/3, size=num_tasks).astype(int)
        tasks_durations = np.clip(tasks_durations, 1, dur_param*3)
        return tasks_durations
    
    else:  # EQUAL_DURATIONS
        return [dur_param] * num_tasks


# generates a random instance on the path with <n_vertices> vertices with tasks durations <tasks_durations> 
# following the distribution <tasks_pos_distr>
def generate_random_instance(n_vertices, tasks_durations, tasks_pos_distr=UNIFORM_DISTR):

    instance = np.zeros(n_vertices)

    tasks_positions = generate_positions(n_vertices, len(tasks_durations), distr=tasks_pos_distr)
    instance[tasks_positions] = tasks_durations

    return instance


# generates <n_positions> unique positions on the path of length <path_length> following the distribution <distr>
def generate_positions(n_vertices, n_positions, distr=UNIFORM_DISTR):

    if n_positions > n_vertices:
        raise ValueError("The number of positions must be less than or equal to the path length.")

    if distr == UNIFORM_DISTR:

        probs = None

    else:  

        mean = np.random.randint(0, n_vertices+1)

        xs = np.arange(0, n_vertices)
        std = n_vertices / 8
        weights = np.exp(-0.5 * ((xs - mean) / std) ** 2)
        probs = weights / weights.sum()

    return sorted(np.random.choice(n_vertices, n_positions, replace=False, p=probs))
