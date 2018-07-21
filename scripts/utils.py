import copy

import actionlib
import rospy
import graspit_commander
import subprocess
import time
import tf
import tf_conversions
import tf2_ros
import tf2_kdl

from geometry_msgs.msg import Vector3, PoseStamped
from reachability_grasping_demo.msg import GraspReachabilityTask, SceneObject, GraspReachabilityResult
import world_manager.world_manager_client as wm_client
import reachability_analyzer.msg
from reachability_analyzer.message_utils import get_graspit_grasp_pose_in_new_reference_frame, change_end_effector_link


def start_graspit(cmd_str):
    # cmd_str = "roslaunch reachability_energy_plugin reachability_energy_plugin.launch"
    p = subprocess.Popen(cmd_str.split())
    time.sleep(3.0)  # Wait for graspit to start


def kill_graspit():
    cmd_str = "rosnode kill /graspit_interface_node"
    p = subprocess.Popen(cmd_str.split())
    time.sleep(3.0)  # Wait for graspit to start


def get_grasps_from_graspit(task=GraspReachabilityTask()):
    """
    This takes in the task and returns list of planned grasps
    :param task: type(GraspReachabilityTask)
    :return:
    """
    feedbacks = []

    def feedback_cb(fb):
        feedbacks.append(fb)

    cmd_str = "roslaunch {} reachability_energy_plugin.launch".format(task.robot_reachability_package)
    start_graspit(cmd_str)

    gc = graspit_commander.GraspitCommander()
    gc.clearWorld()

    gc.importRobot(task.hand_name)
    gc.importGraspableBody(task.target_object.mesh_filepath, task.target_object.pose_stamped.pose)

    for obstacle in task.obstacles:
        gc.importObstacle(obstacle.mesh_filepath, obstacle.pose_stamped.pose)

    gc.planGrasps(search_energy=task.search_energy_type,
                  max_steps=task.max_steps,
                  feedback_cb=feedback_cb,
                  feedback_num_steps=task.planning_step_interval)

    kill_graspit()

    # collate results into tasks
    for fb in feedbacks:
        grasp_reachability_results = GraspReachabilityResult()
        grasp_reachability_results.grasps = fb.grasps
        grasp_reachability_results.energies = fb.energies
        grasp_reachability_results.num_planning_steps = fb.current_step
        grasp_reachability_results.search_energy = fb.search_energy
        task.results.append(grasp_reachability_results)

    return task


def fix_grasps_for_moveit(grasp_results,
                          source_in_target_translation_rotation,
                          old_ee_to_new_ee_translation_rotation,
                          approach_direction):
    grasps_for_moveit = []
    for g_original in grasp_results.grasps:
        g = copy.deepcopy(g_original)
        # ensure grasp is in object frame
        g.pose = get_graspit_grasp_pose_in_new_reference_frame(g.pose, source_in_target_translation_rotation)
        # change from graspit end-effector to moveit end effector
        g.pose = change_end_effector_link(g.pose, old_ee_to_new_ee_translation_rotation)

        g.approach_direction.vector = Vector3(*approach_direction)
        grasps_for_moveit.append(g)

    return grasps_for_moveit


def get_transfrom(reference_frame, target_frame):
    listener = tf.TransformListener()
    try:
        listener.waitForTransform(reference_frame, target_frame,
                                  rospy.Time(0), timeout=rospy.Duration(1))
        translation_rotation = listener.lookupTransform(reference_frame, target_frame,
                                                        rospy.Time())
    except Exception as e1:
        try:
            tf_buffer = tf2_ros.Buffer()
            tf2_listener = tf2_ros.TransformListener(tf_buffer)
            transform_stamped = tf_buffer.lookup_transform(reference_frame, target_frame,
                                                           rospy.Time(0), timeout=rospy.Duration(1))
            translation_rotation = tf_conversions.toTf(tf2_kdl.transform_to_kdl(transform_stamped))
        except Exception as e2:
            rospy.logerr("get_transfrom::\n " +
                         "Failed to find transform from %s to %s" % (
                             reference_frame, target_frame,))
    return translation_rotation


def get_reachability_of_grasps(task):

    wm_client.clear_objects()
    # add objects and obstacles to planning scene
    # wm_client.add_mesh(task.target_object.object_name,
    #                    task.target_object.mesh_filepath,
    #                    task.target_object.pose_stamped)
    wm_client.add_tf(task.target_object.object_name,
                     task.target_object.pose_stamped)

    for obstacle in task.obstacles:
        wm_client.add_mesh(obstacle.object_name, obstacle.mesh_filepath, obstacle.pose_stamped)

    # get transform from grasp reference frame to end-effector frame
    current_grasp_frame = task.target_object.pose_stamped.header.frame_id
    target_grasp_frame = task.target_object.object_name
    source_in_target_translation_rotation = get_transfrom(target_grasp_frame, current_grasp_frame)
    # get transform from graspit end-effector to moveit end effector
    old_ee_to_new_ee_translation_rotation = get_transfrom(task.graspit_link_name, task.moveit_end_effector_name)

    # check grasps for reachability
    for grasp_reachability_results in task.results:
        # fix grasps for moveit
        grasps_for_moveit = fix_grasps_for_moveit(grasp_reachability_results,
                                                  source_in_target_translation_rotation,
                                                  old_ee_to_new_ee_translation_rotation,
                                                  task.approach_direction)
        reachability_result, reachability_check_time = check_for_reachability(grasps_for_moveit,
                                                                              task.target_object.object_name)
        # store result
        grasp_reachability_results.moveit_grasps = grasps_for_moveit
        grasp_reachability_results.is_reachable = reachability_result

    return task


def display_grasp_as_tf(grasp, frame_id):
    ps = PoseStamped()
    ps.pose = grasp.pose
    ps.header.frame_id = frame_id
    wm_client.add_tf('grasp', ps)


def display_grasp_in_graspit(grasp, robot, scene_object):
    gc = graspit_commander.GraspitCommander()
    gc.clearWorld()

    gc.importRobot(robot)
    gc.importGraspableBody(scene_object.mesh_filepath, scene_object.pose_stamped.pose)
    gc.setRobotPose(grasp.pose)


def check_for_reachability(unchecked_for_reachability_grasps, object_name):
    rospy.loginfo("checking grasps for reachability")
    reachability_client = actionlib.SimpleActionClient('analyze_grasp_action',
                                                       reachability_analyzer.msg.CheckGraspReachabilityAction)
    reachability_client.wait_for_server()
    reachability_result = []
    reachability_check_time = []

    for i, unchecked_grasp in enumerate(unchecked_for_reachability_grasps):
        rospy.loginfo("checking grasps for reachability " + str(i))

        # this is the message we are sending to reachability analyzer to check for reachability
        goal = reachability_analyzer.msg.CheckGraspReachabilityGoal()
        goal.grasp = unchecked_grasp
        goal.object_name = object_name

        start_time = time.time()
        reachability_client.send_goal(goal)
        reachability_client.wait_for_result()
        end_time = time.time()

        reachability_check_result = reachability_client.get_result()

        reachability_result.append(reachability_check_result.isReachable)
        reachability_check_time.append(end_time - start_time)

    return reachability_result, reachability_check_time
