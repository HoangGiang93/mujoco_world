#!/usr/bin/env python3

import rospy
from std_msgs.msg import ColorRGBA
from mujoco_msgs.msg import ObjectStatus, ObjectInfo, ObjectState
from mujoco_msgs.srv import SpawnObject, SpawnObjectRequest
from geometry_msgs.msg import Twist

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

move_base_client = actionlib.SimpleActionClient(
    "/move_base", MoveBaseAction)

gripper_clients = [actionlib.SimpleActionClient(
    "/gripper_left_right_finger/gripper_cmd", GripperCommandAction
),
    actionlib.SimpleActionClient(
        "/gripper_left_left_finger/gripper_cmd", GripperCommandAction
),
    actionlib.SimpleActionClient(
        "/gripper_right_left_finger/gripper_cmd", GripperCommandAction
),
    actionlib.SimpleActionClient(
        "/gripper_right_right_finger/gripper_cmd", GripperCommandAction
)]

gripper = "gripper_right_grasping_frame"

target_object = "Cup"

N_tries = 5

target_place = "iai_apartment/bedside_table"


def set_objects_unreal():
    objects = []
    names = ["Box", "Cup"]
    meshes = ['/Game/Models/Box/box.box', '/Game/Models/Cup/cup.cup']
    for i in [0, 1]:
        object = ObjectStatus()
        object.info.name = names[i]
        object.info.type = ObjectInfo.MESH
        object.info.movable = True
        object.info.size.x = 1
        object.info.size.y = 1
        object.info.size.z = 1

        object.pose.position.x = 16.25
        object.pose.position.y = 2.81
        object.pose.position.z = 0.57
        object.pose.orientation.x = 0.0
        object.pose.orientation.y = 0.0
        object.pose.orientation.z = 0.0
        object.pose.orientation.w = 1.0
        object.info.mesh = meshes[i]
        objects.append(object)

    spawn_objects = SpawnObjectRequest()
    spawn_objects.objects = objects

    gen_objects = rospy.ServiceProxy("/unreal/spawn_objects", SpawnObject)
    gen_objects(spawn_objects)
    return None


def set_objects_mujoco():
    objects = []
    names = ["Box", "Cup"]
    meshes = ['../box/box/box.xml', '../cup/cup.xml']
    for i in [0, 1]:
        object = ObjectStatus()
        object.info.name = names[i]
        object.info.type = ObjectInfo.MESH
        object.info.movable = True
        object.info.size.x = 1
        object.info.size.y = 1
        object.info.size.z = 1

        object.pose.position.x = 16.25
        object.pose.position.y = 2.81
        object.pose.position.z = 0.57
        object.pose.orientation.x = 0.0
        object.pose.orientation.y = 0.0
        object.pose.orientation.z = 0.707
        object.pose.orientation.w = 0.707
        object.info.mesh = meshes[i]
        objects.append(object)

    spawn_objects = SpawnObjectRequest()
    spawn_objects.objects = objects

    gen_objects = rospy.ServiceProxy("/mujoco/spawn_objects", SpawnObject)
    gen_objects(spawn_objects)
    return None


def set_joint_goal(joint_goals):
    giskard_wrapper = GiskardWrapper()
    giskard_wrapper.allow_all_collisions()
    giskard_wrapper.set_joint_goal(joint_goals)
    giskard_wrapper.plan_and_execute()
    return None


def move_to_target(x: float, y: float, yaw: float) -> None:
    rospy.loginfo("Move to [" + str(x) + ", " + str(y) + "]")
    move_base_client.wait_for_server()
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
    move_base_client.send_goal(move_goal)
    return None


def control_gripper(open: bool):
    if open:
        rospy.loginfo("Open gripper")
    else:
        rospy.loginfo("Close gripper")

    for gripper_client in gripper_clients:
        gripper_client.wait_for_server()
    gripper_cmd_goal = GripperCommandGoal()
    gripper_cmd_goal.command.position = open * 0.4
    gripper_cmd_goal.command.max_effort = 100.0

    for gripper_client in gripper_clients:
        gripper_client.send_goal(gripper_cmd_goal)
    return None


def get_pose_gripper_T_object(target):
    try:
        tf_listener.waitForTransform(
            gripper, target, rospy.Time(), rospy.Duration(5)
        )
    except TransformException as e:
        rospy.logwarn(gripper + " or " + target + " not found")
    else:
        t = tf_listener.getLatestCommonTime(gripper, target)
        return tf_listener.lookupTransform(gripper, target, t)


def get_quat_gripper_T_gripper_goal():
    try:
        tf_listener.waitForTransform(
            "map", gripper, rospy.Time(), rospy.Duration(5))
    except TransformException as e:
        rospy.logwarn(gripper + " not found")
    else:
        t = tf_listener.getLatestCommonTime("map", gripper)
        _, quat = tf_listener.lookupTransform("map", gripper, t)
        quat_inv = quat
        quat_inv[3] *= -1
        quat_goal = [-0.5, -0.5, 0.5, -0.5]

        return tf.transformations.quaternion_multiply(quat_inv, quat_goal)


def move_arm(goal: PoseStamped, root_link="odom", tip_link=gripper):
    giskard_wrapper = GiskardWrapper()
    giskard_wrapper.allow_all_collisions()
    giskard_wrapper.set_cart_goal(
        root_link=root_link, tip_link=tip_link, goal_pose=goal
    )
    giskard_wrapper.plan_and_execute()
    return None


def move_arm_to_pre_pick():
    N = 0
    while True:
        pos, _ = get_pose_gripper_T_object(target_object)
        pos[0] = 0
        quat = get_quat_gripper_T_gripper_goal()
        if (
            abs(pos[0]) < 0.01 and abs(pos[1]) < 0.01 and abs(pos[2]) < 0.01
        ) or N == N_tries:
            break
        goal = PoseStamped()
        goal.header.frame_id = gripper
        goal.pose.position = Point(pos[0], pos[1], pos[2])
        goal.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        rospy.loginfo(goal.pose.position)
        rospy.loginfo(goal.pose.orientation)
        move_arm(goal)
        N = N + 1
    return None


def move_arm_to_post_pick():
    goal = PoseStamped()
    goal.header.frame_id = gripper
    goal.pose.position = Point(-0.1, 0.0, 0.0)
    move_arm(goal)
    return None


def move_arm_to_pre_place():
    N = 0
    while True:
        quat = get_quat_gripper_T_gripper_goal()
        if (
            abs(quat[0]) < 0.01 and abs(quat[1]) < 0.01 and abs(quat[2]) < 0.01
        ) or N == N_tries:
            break
        goal = PoseStamped()
        goal.header.frame_id = gripper
        goal.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        move_arm(goal)
        N = N + 1
    N = 0
    while True:
        pos, _ = get_pose_gripper_T_object(target_place)
        pos[0] = 0
        quat = get_quat_gripper_T_gripper_goal()
        if (
            abs(pos[0]) < 0.01 and abs(pos[1]) < 0.01 and abs(pos[2]) < 0.01
        ) or N == N_tries:
            break
        goal = PoseStamped()
        goal.header.frame_id = gripper
        goal.pose.position = Point(pos[0], pos[1], pos[2])
        goal.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        move_arm(goal)
        N = N + 1
    return None


if __name__ == "__main__":
    rospy.init_node("box_opning")

    tf_listener = tf.TransformListener()

    tfBuffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tfBuffer)

    rospy.sleep(1)
    set_objects_unreal()
    set_objects_mujoco()

    joint_goals = {
        "torso_lift_joint": 0.0,
        "arm_left_1_joint": 0.92,
        "arm_left_2_joint": 0.0,
        "arm_left_3_joint": 1.9,
        "arm_left_4_joint": 1.2,
        "arm_left_5_joint": -0.2,
        "arm_left_6_joint": 0.2,
        "arm_left_7_joint": 1.8,
        "arm_right_1_joint": 0.92,
        "arm_right_2_joint": 0.0,
        "arm_right_3_joint": 1.9,
        "arm_right_4_joint": 1.2,
        "arm_right_5_joint": -0.2,
        "arm_right_6_joint": 0.2,
        "arm_right_7_joint": 1.8
    }

    control_gripper(open=True)

    move_to_target(15.6, 2.8, 0.0)

    move_base_client.wait_for_result()

    set_joint_goal(joint_goals)

    joint_goals = {}
    joint_goals["arm_right_4_joint"] = 1.25
    joint_goals["arm_left_2_joint"] = -0.25
    joint_goals["arm_left_6_joint"] = 0.4

    set_joint_goal(joint_goals)

    joint_goals = {}
    joint_goals["arm_right_4_joint"] = 1.27
    joint_goals["arm_left_3_joint"] = 1.8
    joint_goals["arm_left_6_joint"] = 0.2

    set_joint_goal(joint_goals)

    joint_goals = {}
    joint_goals["arm_right_4_joint"] = 1.3
    joint_goals["arm_left_3_joint"] = 1.55
    joint_goals["arm_left_6_joint"] = 0.0

    set_joint_goal(joint_goals)

    joint_goals = {}
    joint_goals["arm_left_3_joint"] = 1.4
    joint_goals["arm_left_4_joint"] = 1.4
    joint_goals["arm_left_6_joint"] = 0.1

    set_joint_goal(joint_goals)

    joint_goals = {}
    joint_goals["arm_left_7_joint"] = 2.5

    set_joint_goal(joint_goals)

    joint_goals = {}
    joint_goals["arm_left_1_joint"] = 1.5
    joint_goals["arm_left_2_joint"] = -0.35
    joint_goals["arm_left_6_joint"] = 0.4
    joint_goals["arm_right_4_joint"] = -0.3

    set_joint_goal(joint_goals)

    joint_goals = {}
    joint_goals["arm_left_1_joint"] = 0.4
    joint_goals["arm_right_4_joint"] = 1.2

    set_joint_goal(joint_goals)

    ####

    joint_goals = {
        "torso_lift_joint": 0.0,
        "arm_left_1_joint": 0.92,
        "arm_left_2_joint": 0.0,
        "arm_left_3_joint": 1.9,
        "arm_left_4_joint": 1.2,
        "arm_left_5_joint": -0.2,
        "arm_left_6_joint": 0.2,
        "arm_left_7_joint": 1.8,
        "arm_right_1_joint": 0.92,
        "arm_right_2_joint": 0.0,
        "arm_right_3_joint": 1.9,
        "arm_right_4_joint": 1.2,
        "arm_right_5_joint": -0.2,
        "arm_right_6_joint": 0.2,
        "arm_right_7_joint": 1.8
    }

    set_joint_goal(joint_goals)

    joint_goals = {}
    joint_goals["arm_left_4_joint"] = 1.25
    joint_goals["arm_right_2_joint"] = -0.25
    joint_goals["arm_right_6_joint"] = 0.4

    set_joint_goal(joint_goals)

    joint_goals = {}
    joint_goals["arm_left_4_joint"] = 1.27
    joint_goals["arm_right_3_joint"] = 1.8
    joint_goals["arm_right_6_joint"] = 0.2

    set_joint_goal(joint_goals)

    joint_goals = {}
    joint_goals["arm_left_4_joint"] = 1.3
    joint_goals["arm_right_3_joint"] = 1.55
    joint_goals["arm_right_6_joint"] = 0.0

    set_joint_goal(joint_goals)

    joint_goals = {}
    joint_goals["arm_right_3_joint"] = 1.4
    joint_goals["arm_right_4_joint"] = 1.4
    joint_goals["arm_right_6_joint"] = 0.1

    set_joint_goal(joint_goals)

    joint_goals = {}
    joint_goals["arm_right_7_joint"] = 2.5

    set_joint_goal(joint_goals)

    joint_goals = {}
    joint_goals["arm_right_1_joint"] = 1.5
    joint_goals["arm_right_2_joint"] = -0.35
    joint_goals["arm_right_6_joint"] = 0.4
    joint_goals["arm_left_4_joint"] = -0.3

    set_joint_goal(joint_goals)

    joint_goals = {}
    joint_goals["arm_right_1_joint"] = 0.4
    joint_goals["arm_left_4_joint"] = 1.2

    set_joint_goal(joint_goals)

    ###

    joint_goals = {}
    joint_goals["torso_lift_joint"] = 3.5
    joint_goals["arm_left_1_joint"] = 0.0
    joint_goals["arm_right_1_joint"] = 0.276318
    joint_goals["arm_right_2_joint"] = -0.994569
    joint_goals["arm_right_3_joint"] = 2.038428
    joint_goals["arm_right_4_joint"] = 1.949348
    joint_goals["arm_right_5_joint"] = 0.950820
    joint_goals["arm_right_6_joint"] = -0.983220
    joint_goals["arm_right_7_joint"] = -0.524971
    set_joint_goal(joint_goals)

    move_arm_to_pre_pick()

    joint_goals = {}
    joint_goals["torso_lift_joint"] = 0.05
    set_joint_goal(joint_goals)

    rospy.sleep(5)

    control_gripper(open=False)

    joint_goals = {}
    joint_goals["torso_lift_joint"] = 0.35
    set_joint_goal(joint_goals)

    joint_goals = {}
    joint_goals["arm_right_2_joint"] = -0.8
    set_joint_goal(joint_goals)

    pub = rospy.Publisher('/tiago_dual/cmd_vel', Twist, queue_size=10)
    twist = Twist()
    twist.linear.x = -0.1

    time = 0
    r = rospy.Rate(10) # 10hz
    while(True):
        pub.publish(twist)
        if time > 50:
            break
        time += 1
        r.sleep()

    twist = Twist()
    pub.publish(twist)

    move_to_target(15.27, 1.612, 0.388)
    move_base_client.wait_for_result()

    joint_goals = {}
    joint_goals["torso_lift_joint"] = 0.0
    joint_goals["arm_right_2_joint"] = -1
    set_joint_goal(joint_goals)

    control_gripper(open=True)
