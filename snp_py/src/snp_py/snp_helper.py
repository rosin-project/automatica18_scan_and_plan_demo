#!/usr/bin/env python

import sys
import rospy
import time
import moveit_commander
import actionlib
from moveit_msgs.msg import MoveGroupAction
from std_srvs.srv import Empty, EmptyRequest


def unpause_gazebo(delay):
    # Give Gazebo enough time. Sim_time not running yet
    time.sleep(delay)

    # Pause is required to span model with defined joint_values
    unpause = init_srv("/gazebo/unpause_physics", Empty)
    unpause(EmptyRequest())


def init_move_group(group_name):
    # This is to make sure that MoveIt! has been started up.
    init_action("/move_group", MoveGroupAction, timeout=60)

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    return moveit_commander.MoveGroupCommander(group_name)


def init_srv(srv_ns, srv_type):
    rospy.loginfo("Wait for service '%s' ..." % srv_ns)
    rospy.wait_for_service(srv_ns)
    return rospy.ServiceProxy(srv_ns, srv_type)


def init_action(act_ns, act_type, timeout=30):
    client = actionlib.SimpleActionClient(act_ns, act_type)
    rospy.loginfo("Wait for action '%s' ..." % act_ns)
    server_up = client.wait_for_server(timeout=rospy.Duration(timeout))
    if not server_up:
        raise rospy.ROSInterruptException("Action timed out.")
    return client


def call_action(client, request, timeout):
    client.send_goal(request)
    action_returned = client.wait_for_result(timeout=rospy.Duration(timeout))
    if not action_returned:
        raise rospy.ROSInterruptException("Action timed out.")
    return client.get_result()
