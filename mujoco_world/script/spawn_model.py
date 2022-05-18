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

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def set_objects():
    objects = []
    object = ObjectStatus()
    object.info.name = "Box"
    object.info.type = ObjectInfo.CUBE
    object.info.movable = True
    object.info.size.x = 1.0
    object.info.size.y = 1.0
    object.info.size.z = 1.0
    object.info.rgba = ColorRGBA(1.0, 0.0, 0.0, 1)
    object.info.inertial.ixx = 1.0
    object.info.inertial.iyy = 1.0
    object.info.inertial.izz = 1.0
    object.info.inertial.m = 1.0

    object.pose.position.x = -1.20
    object.pose.position.y = 0.0
    object.pose.position.z = 0.26
    object.pose.orientation.x = 0.0
    object.pose.orientation.y = 0.0
    object.pose.orientation.z = 0.0
    object.pose.orientation.w = 1.0
    objects.append(object)

    # for i in range(3):
    #     object = ObjectStatus()
    #     object.info.name = "ProductWithAN036942_" + str(i)
    #     object.info.type = ObjectInfo.MESH
    #     object.info.movable = True
    #     object.info.size.x = 1.0
    #     object.info.size.y = 1.0
    #     object.info.size.z = 1.0
    #     object.info.rgba = ColorRGBA(0.0, 0.0, 0.0, 1)
    #     object.info.inertial.m = 1.0
    #     object.info.mesh = "/Game/Private/Models/DM/DMCatalog/ProductWithAN036946/SM_ProductWithAN036946.SM_ProductWithAN036946"

    #     object.pose.position.x = -1.20
    #     object.pose.position.y = 0.20 - i * 0.10
    #     object.pose.position.z = 0.26
    #     object.pose.orientation.x = 0.0
    #     object.pose.orientation.y = 0.7071068
    #     object.pose.orientation.z = 0.0
    #     object.pose.orientation.w = 0.7071068
    #     objects.append(object)

    spawn_objects = SpawnObjectRequest()
    spawn_objects.objects = objects

    gen_objects = rospy.ServiceProxy("/mujoco/spawn_objects", SpawnObject)
    gen_objects(spawn_objects)


def move_to_target(
    client: actionlib.ActionClient, x: float, y: float, yaw: float
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


if __name__ == "__main__":
    rospy.init_node("replenishment")

    rospy.sleep(1)
    # set_objects()
    # ms_move_base_client = actionlib.SimpleActionClient(
    #     "mir_system/move_base", MoveBaseAction
    # )
    # rp_move_base_client = actionlib.SimpleActionClient(
    #     "ridgeback_panda/move_base", MoveBaseAction
    # )
    # move_to_target(ms_move_base_client, 10.5, -3.6, 0.0)

    # move_to_target(rp_move_base_client, 7.0, -3.6, 0.0)

    goal_js = {
        "panda_joint1": 0.0,
        "panda_joint2": 0.0,
        "panda_joint3": 0.0,
        "panda_joint4": 0.0,
        "panda_joint5": 0.0,
        "panda_joint6": 0.0,
        "panda_joint7": 0.0,
    }
    # create a GiskardWrapper object and execute the joint goal
    giskard_wrapper = GiskardWrapper()
    giskard_wrapper.set_joint_goal(goal_js)
    giskard_wrapper.plan_and_execute()

    # Create a goal for the right hand
    # giskard_wrapper = GiskardWrapper()
    # goal = PoseStamped()
    # goal.header.frame_id = "panda_hand"
    # goal.header.stamp = rospy.Time.now()
    # goal.pose.position = Point(0.1, 0, 0)
    # goal.pose.orientation = Quaternion(0, 0, 0, 1)
    # giskard_wrapper.set_cart_goal(goal, "odom", "panda_hand", WEIGHT_BELOW_CA)

    # giskard_wrapper.plan_and_execute()
