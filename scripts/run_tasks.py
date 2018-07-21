#!/usr/bin/python
import utils
import pickup_tasks
import argparse
import yaml

import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict


def do_task(task):

    # plan grasps
    task_with_grasp_results = utils.get_grasps_from_graspit(task)

    # check grasps for reachability
    task_with_reachability_result = utils.get_reachability_of_grasps(task_with_grasp_results)

    return task_with_reachability_result


def plot_reachability_result(finished_tasks):

    all_results = {}
    for task in finished_tasks:
        for result in task.results:
            if result.search_energy not in all_results:
                all_results[result.search_energy] = defaultdict(list)

            num_planning_steps = int(round(result.num_planning_steps, -1))

            stats = result.is_reachable
            all_results[result.search_energy][num_planning_steps].extend(stats)

    collated_results = defaultdict(list)
    for energy_key in all_results:
        for num_planning_steps_key in sorted(all_results[energy_key].keys()):
            collated_results[energy_key].append([num_planning_steps_key, np.mean(all_results[energy_key][num_planning_steps_key])])

    colors = ['r', 'b', 'g']
    for i, energy_key in enumerate(sorted(collated_results.keys())):
        result = collated_results[energy_key]
        xy = np.array(result).T
        plt.scatter(xy[0]-30000, xy[1], label=energy_key, color=colors[i])
    
    plt.ylim(0, 1.)
    plt.legend(loc='best')
    plt.xlabel('Number of planning steps')
    plt.ylabel('Fraction of planned grasps that are reachable')
    plt.title('Fraction of planned grasps that are reachable versus planning time')
    plt.show()
    import IPython
    IPython.embed()

if __name__ == "__main__":

    args = pickup_tasks.get_args()

    tasks = pickup_tasks.get_tasks(args)
    print "Generated {} Tasks".format(len(tasks))

    # finished_task = do_task(tasks[0])
    # plot_reachability_result([finished_task])
    
    finished_tasks = []
    for task in tasks:
        finished_task = do_task(task)
        finished_tasks.append(finished_task)
    plot_reachability_result(finished_tasks)



