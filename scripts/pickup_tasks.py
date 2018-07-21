import yaml
import argparse
import os

import rospkg
import tf_conversions
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

from reachability_grasping_demo.msg import GraspReachabilityTask, SceneObject


def get_args():

    parser = argparse.ArgumentParser(description='Create Simmulated Reachability Tasks')
    
    parser.add_argument('-c', '--config_file', type=str,
                        help="The name of the yaml file that holds the configs for this script. Ex: pickup_tasks.yaml")
    args = parser.parse_args()

    config = yaml.load(open(args.config_file))
    for k, v in config.items():
        args.__dict__[k] = v

    args.__dict__['mesh_root'] = os.path.join(rospkg.RosPack().get_path('reachability_grasping_demo'), 'meshes')

    return args


def get_object_pose_list(args):
    pose_list = []
    for x_y in args.x_y:
        for z in args.z_range:
            for roll in args.roll_range:
                for pitch in args.pitch_range:
                    for yaw in args.yaw_range:
                        r = tf_conversions.Rotation.RPY(roll, pitch, yaw)
                        pose = Pose(Point(x_y[0],x_y[1],z), Quaternion(*r.GetQuaternion()))
                        pose_list.append(pose)

    return pose_list


def get_obstacle_list(args):
    obstacle_list = []
    for each_object in args.obstacle_info:

        object_name = each_object['file_name'].split('.')[0]
        mesh_filepath = os.path.join(args.mesh_root, each_object['file_name'])
        pose = Pose(Point(*each_object['pose'][:3]), Quaternion(*each_object['pose'][3:]))

        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
        pose_stamped.header.frame_id = args.frame_id
        obstacle = SceneObject(object_name, mesh_filepath, pose_stamped)
        obstacle_list.append(obstacle)
    return obstacle_list


def get_tasks(args):

    tasks = []

    for m, mesh_name in enumerate(args.mesh_names):
        mesh_filepath = os.path.join(args.mesh_root, mesh_name)
        object_pose_list = get_object_pose_list(args)
        obstacle_list = get_obstacle_list(args)

        for i, search_energy in enumerate(args.search_energy_types):
            for j, object_pose in enumerate(object_pose_list):
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = args.frame_id
                pose_stamped.pose = object_pose
                target_object = SceneObject('object_0', mesh_filepath, pose_stamped)

                task = GraspReachabilityTask(
                    target_object=target_object,
                    obstacles=obstacle_list,
                    search_energy_type=search_energy,
                    hand_name=args.hand_name,
                    max_steps=args.max_steps,
                    planning_step_interval=args.planning_step_interval,
                    robot_reachability_package=args.robot_reachability_package,
                    graspit_link_name=args.graspit_link_name,
                    moveit_end_effector_name=args.moveit_end_effector_name,
                    approach_direction=args.approach_direction)
                    
                tasks.append(task)
    return tasks


if __name__ == "__main__":
    args = get_args()
    print("Using Args:")
    for k, v in args.__dict__.items():
        print k + ": " + str(v)
    
    tasks = get_tasks(args)

    print "Generated {} Tasks".format(len(tasks))
    print "0th task:"
    print tasks[0]