#!/usr/bin/env python

import rospy
from std_msgs.msg import ColorRGBA
from mujoco_msgs.msg import ObjectStatus, ObjectInfo, ObjectState
from mujoco_msgs.srv import SpawnObject, SpawnObjectRequest

import rospy

from giskardpy.python_interface import GiskardWrapper
from geometry_msgs.msg import PoseStamped, Point, Quaternion

import tf
import math

import tf2_ros
from tf2_ros import TransformException

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from control_msgs.msg import GripperCommandAction, GripperCommandGoal

from typing import Dict

gripper = "panda_hand"

N_tries = 2

def set_objects_unreal():
    objects = []
    object = ObjectStatus()
    object.info.name = "Box"
    object.info.type = ObjectInfo.CUBE
    object.info.movable = True
    object.info.size.x = 0.4
    object.info.size.y = 0.6
    object.info.size.z = 0.6
    object.info.rgba = ColorRGBA(0.0, 0.0, 1.0, 1.0)
    object.info.inertial.ixx = 1.0
    object.info.inertial.iyy = 1.0
    object.info.inertial.izz = 1.0
    object.info.inertial.m = 1.0

    object.pose.position.x = -1.10
    object.pose.position.y = 0.0
    object.pose.position.z = 0.49
    object.pose.orientation.x = 0.0
    object.pose.orientation.y = 0.0
    object.pose.orientation.z = 0.0
    object.pose.orientation.w = 1.0
    objects.append(object)

    for i in range(3):
        object = ObjectStatus()
        object.info.name = "ProductWithAN036946_" + str(i)
        object.info.type = ObjectInfo.MESH
        object.info.movable = True
        object.info.size.x = 1.0
        object.info.size.y = 1.0
        object.info.size.z = 1.0
        object.info.rgba = ColorRGBA(0.0, 0.0, 0.0, 1)
        object.info.inertial.m = 0.01
        object.info.mesh = "/Game/Private/Models/DM/DMCatalog/ProductWithAN036946/SM_ProductWithAN036946.SM_ProductWithAN036946"

        object.pose.position.x = -1.20
        object.pose.position.y = 0.10 - i * 0.10
        object.pose.position.z = 0.87
        object.pose.orientation.x = 0.0
        object.pose.orientation.y = 0.707
        object.pose.orientation.z = 0.0
        object.pose.orientation.w = 0.707
        objects.append(object)

    spawn_objects = SpawnObjectRequest()
    spawn_objects.objects = objects

    gen_objects = rospy.ServiceProxy("/unreal/spawn_objects", SpawnObject)
    gen_objects(spawn_objects)
    return None


def set_objects_mujoco():
    objects = []
    object = ObjectStatus()
    object.info.name = "Box"
    object.info.type = ObjectInfo.CUBE
    object.info.movable = True
    object.info.size.x = 0.2
    object.info.size.y = 0.3
    object.info.size.z = 0.3
    object.info.rgba = ColorRGBA(0.0, 0.0, 1.0, 1.0)
    object.info.inertial.ixx = 1.0
    object.info.inertial.iyy = 1.0
    object.info.inertial.izz = 1.0
    object.info.inertial.m = 1.0

    object.pose.position.x = -1.10
    object.pose.position.y = 0.0
    object.pose.position.z = 0.49
    object.pose.orientation.x = 0.0
    object.pose.orientation.y = 0.0
    object.pose.orientation.z = 0.0
    object.pose.orientation.w = 1.0
    objects.append(object)

    for i in range(3):
        object = ObjectStatus()
        object.info.name = "ProductWithAN036946_" + str(i)
        object.info.type = ObjectInfo.MESH
        object.info.movable = True
        object.info.size.x = 0.026531
        object.info.size.y = 0.020535
        object.info.size.z = 0.072022
        object.info.rgba = ColorRGBA(0.0, 0.0, 0.0, 1)
        object.info.inertial.m = 0.01
        object.info.mesh = "ProductWithAN036946.xml"

        object.pose.position.x = -1.20
        object.pose.position.y = 0.10 - i * 0.10
        object.pose.position.z = 0.87
        object.pose.orientation.x = 0.0
        object.pose.orientation.y = 0.707
        object.pose.orientation.z = 0.0
        object.pose.orientation.w = 0.707
        objects.append(object)

    spawn_objects = SpawnObjectRequest()
    spawn_objects.objects = objects

    gen_objects = rospy.ServiceProxy("/mujoco/spawn_objects", SpawnObject)
    gen_objects(spawn_objects)
    return None


def move_to_target(
    client: actionlib.SimpleActionClient, x: float, y: float, yaw: float
) -> None:
    rospy.loginfo("Move to [" + str(x) + ", " + str(y) + "]")
    client.wait_for_server()
    move_goal = MoveBaseGoal()
    move_goal.target_pose.header.stamp = rospy.get_rostime()
    move_goal.target_pose.header.stamp = rospy.get_rostime()
    move_goal.target_pose.header.frame_id = "map"
    move_goal.target_pose.pose.position.x = x
    move_goal.target_pose.pose.position.y = y
    quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
    move_goal.target_pose.pose.orientation.x = quat[0]
    move_goal.target_pose.pose.orientation.y = quat[1]
    move_goal.target_pose.pose.orientation.z = quat[2]
    move_goal.target_pose.pose.orientation.w = quat[3]
    client.send_goal(move_goal)
    return None


def control_gripper(client: actionlib.SimpleActionClient, open: bool):
    if open:
        rospy.loginfo("Open gripper")
    else:
        rospy.loginfo("Close gripper")
    client.wait_for_server()
    gripper_cmd_goal = GripperCommandGoal()
    gripper_cmd_goal.command.position = open * 0.4
    gripper_cmd_goal.command.max_effort = 1000.0
    client.send_goal(gripper_cmd_goal)
    client.wait_for_result()
    return None


def set_joint_goal(joint_goals: Dict):
    giskard_wrapper = GiskardWrapper()
    giskard_wrapper.allow_self_collision()
    giskard_wrapper.set_joint_goal(joint_goals)
    giskard_wrapper.plan_and_execute()
    return None


def move_arm(goal: PoseStamped):
    giskard_wrapper = GiskardWrapper()
    giskard_wrapper.allow_self_collision()
    giskard_wrapper.set_cart_goal(root_link="odom", tip_link=gripper, goal_pose=goal)
    giskard_wrapper.plan_and_execute()
    return None


def get_pose_gripper_T_object(target_object):
    try:
        tf_listener.waitForTransform(
            gripper, target_object, rospy.Time(), rospy.Duration(5)
        )
    except TransformException as e:
        rospy.logwarn(gripper + " or " + target_object + " not found")
    else:
        t = tf_listener.getLatestCommonTime(gripper, target_object)
        return tf_listener.lookupTransform(gripper, target_object, t)

def get_pose_gripper_T_shelf_layer(shelf_layer):
    try:
        trans = tfBuffer.lookup_transform(target_frame=gripper, source_frame=shelf_layer, time=rospy.Time(), timeout=rospy.Duration(10))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn(e)
    else:
        pos = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
        quat = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
        return pos, tf.transformations.quaternion_multiply(quat, [0.5, 0.5, 0.5, -0.5])


def get_quat_gripper_T_gripper_goal():
    try:
        tf_listener.waitForTransform("map", gripper, rospy.Time(), rospy.Duration(5))
    except TransformException as e:
        rospy.logwarn(gripper + " not found")
    else:
        t = tf_listener.getLatestCommonTime("map", gripper)
        _, quat = tf_listener.lookupTransform("map", gripper, t)
        quat_inv = quat
        quat_inv[3] *= -1
        quat_goal = [1.0, 0.0, 0.0, 0.0]

        return tf.transformations.quaternion_multiply(quat_inv, quat_goal)


def move_arm_to_pre_pick(target_object):
    N = 0
    while True:
        pos, _ = get_pose_gripper_T_object(target_object)
        quat = get_quat_gripper_T_gripper_goal()
        r, p, y = tf.transformations.euler_from_quaternion(quat)
        pos[2] -= 0.2
        rospy.loginfo("Delta Position: [" + str(pos[0]) + ", " + str(pos[1]) + "]")
        rospy.loginfo("Delta RPY: [" + str(r) + ", " + str(p) + ", " + str(y) + "]")
        goal = PoseStamped()
        goal.header.frame_id = gripper
        goal.pose.position = Point(pos[0], pos[1], pos[2])
        goal.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        move_arm(goal)
        pos, _ = get_pose_gripper_T_object(target_object)
        quat = get_quat_gripper_T_gripper_goal()
        r, p, y = tf.transformations.euler_from_quaternion(quat)
        if (
            abs(pos[0]) < 0.01
            and abs(pos[1]) < 0.01
            and abs(r) < 0.01
            and abs(p) < 0.01
            and abs(y) < 0.01
        ) or N == N_tries:
            break
        N = N + 1
    return None


def move_arm_to_pick(target_object):
    N = 0
    while True:
        pos, _ = get_pose_gripper_T_object(target_object)
        quat = get_quat_gripper_T_gripper_goal()
        r, p, y = tf.transformations.euler_from_quaternion(quat)
        pos[2] -= 0.1
        rospy.loginfo(
            "Delta Position: ["
            + str(pos[0])
            + ", "
            + str(pos[1])
            + ", "
            + str(pos[2])
            + "]"
        )
        goal = PoseStamped()
        goal.header.frame_id = gripper
        goal.pose.position = Point(pos[0], pos[1], pos[2])
        goal.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        move_arm(goal)
        pos, _ = get_pose_gripper_T_object(target_object)
        quat = get_quat_gripper_T_gripper_goal()
        r, p, y = tf.transformations.euler_from_quaternion(quat)
        if (
            abs(pos[0]) < 0.01
            and abs(pos[1]) < 0.01
            and abs(r) < 0.01
            and abs(p) < 0.01
            and abs(y) < 0.01
        ) or N == N_tries:
            break
        N = N + 1
    return None


def move_arm_to_post_pick():
    goal = PoseStamped()
    goal.header.frame_id = gripper
    goal.pose.position = Point(0.0, 0.0, -0.3)
    move_arm(goal)
    goal = PoseStamped()
    goal.header.frame_id = gripper
    goal.pose.position = Point(-0.4, 0.0, 0.0)
    move_arm(goal)
    return None

def move_arm_to_pre_place(target_shelf_layer):
    N = 0
    while True:
        _, quat = get_pose_gripper_T_shelf_layer(target_shelf_layer)
        goal = PoseStamped()
        goal.header.frame_id = gripper
        goal.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        move_arm(goal)
        _, quat = get_pose_gripper_T_shelf_layer(target_shelf_layer)
        r, p, y = tf.transformations.euler_from_quaternion(quat)
        if (abs(r) < 0.01
            and abs(p) < 0.01
            and abs(y) < 0.01
        ) or N == N_tries:
            break
        N = N + 1
    N = 0
    while True:
        pos, quat = get_pose_gripper_T_shelf_layer(target_shelf_layer)
        pos[0] += 0.25
        pos[2] -= 0.5
        r, p, y = tf.transformations.euler_from_quaternion(quat)
        rospy.loginfo(
            "Delta Position: ["
            + str(pos[0])
            + ", "
            + str(pos[1])
            + ", "
            + str(pos[2])
            + "]"
        )
        goal = PoseStamped()
        goal.header.frame_id = gripper
        goal.pose.position = Point(pos[0], pos[1], pos[2])
        goal.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        move_arm(goal)
        pos, quat = get_pose_gripper_T_shelf_layer(target_shelf_layer)
        r, p, y = tf.transformations.euler_from_quaternion(quat)
        if (
            abs(pos[0] - 0.25) < 0.01
            and abs(pos[1]) < 0.01
            and abs(r) < 0.01
            and abs(p) < 0.01
            and abs(y) < 0.01
        ) or N == N_tries:
            break
        N = N + 1
    return None

def move_arm_to_place(target_shelf_layer):
    N = 0
    while True:
        pos, quat = get_pose_gripper_T_shelf_layer(target_shelf_layer)
        pos[0] += 0.25
        pos[2] -= 0.4
        r, p, y = tf.transformations.euler_from_quaternion(quat)
        rospy.loginfo(
            "Delta Position: ["
            + str(pos[0])
            + ", "
            + str(pos[1])
            + ", "
            + str(pos[2])
            + "]"
        )
        goal = PoseStamped()
        goal.header.frame_id = gripper
        goal.pose.position = Point(pos[0], pos[1], pos[2])
        goal.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        move_arm(goal)
        pos, quat = get_pose_gripper_T_shelf_layer(target_shelf_layer)
        r, p, y = tf.transformations.euler_from_quaternion(quat)
        if (
            abs(pos[0] - 0.25) < 0.01
            and abs(pos[1]) < 0.01
            and abs(r) < 0.01
            and abs(p) < 0.01
            and abs(y) < 0.01
        ) or N == N_tries:
            break
        N = N + 1
    return None

def move_arm_to_post_place():
    goal = PoseStamped()
    goal.header.frame_id = gripper
    goal.pose.position = Point(0, 0, -0.3)
    move_arm(goal)
    return None

if __name__ == "__main__":
    rospy.init_node("replenishment")

    tf_listener = tf.TransformListener()

    tfBuffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tfBuffer)

    rospy.sleep(1)
    # set_objects_unreal()
    set_objects_mujoco()
    rospy.sleep(2)

    ms_move_base_client = actionlib.SimpleActionClient(
        "/mir_system/move_base", MoveBaseAction
    )
    rp_move_base_client = actionlib.SimpleActionClient(
        "/ridgeback_panda/move_base", MoveBaseAction
    )
    gripper_client = actionlib.SimpleActionClient(
        "/ridgeback_panda/gripper_controller/gripper_cmd", GripperCommandAction
    )
    control_gripper(gripper_client, open=True)

    move_to_target(ms_move_base_client, 10.5, -3.6, 0.0)

    rospy.sleep(5)
    move_to_target(rp_move_base_client, 7.0, -3.6, 0.0)

    rp_move_base_client.wait_for_result()

    goal_js = {
        "panda_joint1": 0.0,
        "panda_joint2": 1.0,
        "panda_joint3": 0.0,
        "panda_joint4": -0.6,
        "panda_joint5": 0.0,
        "panda_joint6": 1.6,
        "panda_joint7": 0.7,
    }
    set_joint_goal(goal_js)

    target_object = "ProductWithAN036946_1"
    move_arm_to_pre_pick(target_object)

    move_arm_to_pick(target_object)

    control_gripper(gripper_client, open=False)

    move_arm_to_post_pick()

    target_shelf_layer = "ShelfLayer6TilesL38"
    move_arm_to_pre_place(target_shelf_layer)

    move_arm_to_place(target_shelf_layer)

    control_gripper(gripper_client, open=True)

    move_arm_to_post_place()

    goal_js = {
        "panda_joint1": 0.0,
        "panda_joint2": 1.0,
        "panda_joint3": 0.0,
        "panda_joint4": -0.6,
        "panda_joint5": 0.0,
        "panda_joint6": 1.6,
        "panda_joint7": 0.7,
    }
    set_joint_goal(goal_js)

    # # Create a goal for the right hand
    # giskard_wrapper.allow_self_collision()
    # goal = PoseStamped()
    # goal.header.frame_id = "panda_hand"
    # goal.pose.position = Point(0.1, 0, 0)
    # goal.pose.orientation = Quaternion(0, 0, 0, 1)
    # giskard_wrapper.set_cart_goal(root_link="odom", tip_link="panda_hand", goal_pose=goal)

    # giskard_wrapper.plan_and_execute()
