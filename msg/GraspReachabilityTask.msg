int32 task_id
string status                          # can be "Incomplete", "In Progress" or "Finished"
SceneObject target_object
SceneObject[] obstacles
string search_energy_type
string hand_name
int32 max_steps
string robot_reachability_package
string graspit_link_name
string moveit_end_effector_name
int8[] approach_direction

int32 planning_step_interval
GraspReachabilityResult[] results            # list of grasp results for each logged planning step