# Fast and Near-Optimal Collision-Free Robot Scheduling on Path Graphs

This repository is the official implementation of the "Fast and Near-Optimal Collision-Free Robot Scheduling
on Path Graphs" manuscript.

## Table of Contents

- [Requirements](#requirements)
- [Scheduling](#scheduling)
- [Config file](#config-file)
- [Input file](#input-file)
- [Output file](#output-file)
- [License](#license)

## Requirements

Python3.9 or Python3.10 is required.
Then you can install requirements:

```setup
pip install -r requirements.txt
```

If you would like to run the IP algorithm, you also need to obtain a Gurobi license, which can be done for academic purposes for free here: https://www.gurobi.com/academia/academic-program-and-licenses/. The Gurobi license parameters should be put in the config file.

## Scheduling

To find a collision-free task-completing schedule for the given problem run

```run
run.py
```

Optional parameters:

- `config`: the path to the .json file containing the configuration of the experiment, the file content is described below.
- `input_file`: the path to the .csv file containing the instances of the scheduling problems, the file format is described below. If `null` and the same parameter in the config file is `null` as well, the experiment with randomly generated instances is running with the parameters taken from the config file.
- `output_dir`: the path to the output folder to save the results to, "output" by default.
- `algos`: the algorithms string, "p" for PA, "i" for IP, "g" for GA, "r" for RA; "pigr" by default, i.e. all algorithms are chosen.

The config file may have all the optional parameters above, but the input parameters have a higher priority over the config file if they differ.

There are two options on how to run the program. The first option is called <MODE 1> and is used when the input file with the generated paths/tasks/robots instances is provided. If the input file is not provided, then the second option <MODE 2> is used: the instances of the problem are generated on-the-fly with the parameters in the config file.

## Config file

- `input_file`: <MODE 1> the path to the .csv file containing the instances of the scheduling problems.
- `output_dir`: the path to the output folder to save the results to, "output" by default.
- `algos`: the algorithms string, "p" for PA, "i" for IP, "G" for GA, R for "RA", "pigr" by default, i.e. all algorithms are selected.
- `path_vertices_min`: <MODE 2> the minimum number of vertices in a graph.
- `path_vertices_max`: <MODE 2> the maximum number of vertices in a graph. 
- `path_vertices_step`: <MODE 2> the step size for the number of vertices in a graph.
- `robots_n_min`: <MODE 2> the minimum number of robots.
- `robots_n_max`: <MODE 2> the maximum number of robots. If None, then it is equal to the number of vertices - 1.
- `robots_n_step`: <MODE 2> the step size for the number of robots.
- `tasks_n_min`: <MODE 2> the minimum number of tasks.
- `tasks_n_max`: <MODE 2> the maximum number of tasks. If None, then it is equal to the number of vertices.
- `tasks_n_step`: <MODE 2> the step size for the number of tasks.
- `dur_param_min`: <MODE 2> the minimum value for the d_{max} parameter.
- `dur_param_max`: <MODE 2> the maximum value for the d_{max} parameter.
- `dur_param_step`: <MODE 2> the step size for the value for the d_{max} parameter.
- `tasks_dur_distr`: <MODE 2> the tasks duration distribution, can be 'uniform', 'uneven_uniform' (used in DS2 in the manuscript), or 'equal'
- `max_instances_num`: <MODE 2> the number of instances for each tuple of parameters' values
- `robots_distr`: <MODE 2> the distribution of robots on the path, "uniform" or "normal"
- `tasks_pos_distr`: <MODE 2> the distribution of tasks on the path, "uniform" or "normal"
- `WLSACCESSID`: the parameter from the Gurobi license
- `WLSSECRET`: the parameter from the Gurobi license
- `LICENSEID`: the parameter from Gurobi license

The config files for the datasets $DS1-DS5$ of the manuscript are provided. To reproduce an experiment from the manuscript the program should be run with the corresponding config file.

## Input file

The input file should be in the .csv format with the following columns (without a header):
<vertices number>,<tasks>,<robots>

where <tasks> is a list of tuples as a string, where the first element is the position, and the second is the durations of the tasks, and <robots> is a list as a string, where each element is the position of the robot.
For example, the string "[(5, 10), (14, 10)]" represents two tasks on positions 5 and 14, both of duration 10, and the string "[17, 18]" represents two robots on positions 17 and 18.

## Output file

The output file is a file in the .csv format with the following columns:
<vertices number>,<robots number>,<tasks>,<robots>,<schedule_length>,<schedule_time>,<schedule>

where the schedule length, schedule time, and the schedule are presented for each algorithm that was used.

## License

The code is distributed under The Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public (CC BY-NC-SA 4.0) License.
