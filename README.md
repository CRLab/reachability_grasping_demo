# reachability_grasping_demo

This is a demo package that shows the reachability-aware grasping approach in simulation.

## Generate tasks
Specify pickup tasks in a yaml according to `pickup_tasks.yaml`. This contains pickup configuration (meshes and pose(s) of the target object and obstacles) and the search_energy_types field defines the grasp planner type e.g. the reachability-aware grasp planner (`GUIDED_POTENTIAL_QUALITY_ENERGY`) and the naive grasp planner (`REACHABLE_FIRST_HYBRID_GRASP_ENERGY`).

## Run Graspit!/Reachability check on tasks
- Launch the robot and planning scene manager in simulation
```
roslaunch reachability_grasping_demo everything.launch
```

- Run the `run_tasks.py` script to iterate over pickup tasks one at a time and solve it (get grasps and reachability checks).
```
python run_tasks.py --config_file pickup_tasks.yaml
```

At the end, it plots the percentage reachability of the reachability-aware grasp planner relative to the naive grasp planner.