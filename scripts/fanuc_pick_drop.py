#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class FanucArm(object):

  def __init__(self, group_name):
    super(FanucArm, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('fanuc_arm_with_gripper', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = group_name
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = move_group.get_planning_frame()
    eef_link = move_group.get_end_effector_link()
    group_names = robot.get_group_names()
    move_group.set_planning_time(10)
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def go_to_joint_state(self, goal):

    move_group = self.move_group

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = goal[0]
    joint_goal[1] = goal[1]
    joint_goal[2] = goal[2]
    joint_goal[3] = goal[3]
    joint_goal[4] = goal[4]
    joint_goal[5] = goal[5]
    joint_goal[6] = goal[6]

    move_group.go(joint_goal, wait=True)
    move_group.stop()

    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self, goal):

    move_group = self.move_group

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = goal[0]
    pose_goal.orientation.y = goal[1]
    pose_goal.orientation.z = goal[2]
    pose_goal.orientation.w = goal[3]
    pose_goal.position.x = goal[4]
    pose_goal.position.y = goal[5]
    pose_goal.position.z = goal[6]

    move_group.set_pose_target(pose_goal)


    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def display_trajectory(self, plan):

    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)

    display_trajectory_publisher.publish(display_trajectory);

  def gripper_on(self):

    # Wait till the srv is available
    rospy.wait_for_service('/gripper/on')
    try:
        turn_on = rospy.ServiceProxy('/gripper/on', Empty)
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

  def gripper_off(self):
    rospy.wait_for_service('/gripper/off')
    try:
        turn_off = rospy.ServiceProxy('/gripper/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def main():

  goal1 = [0.0,0.707,0.0,0.707,0.40,0.0,0.46]
  goal2 = [0.0,0.707,0.0,0.707,0.40,0.0,0.06]
  goal3 = [0.0,0.707,0.0,0.707,0.40,0.0,0.36]
  goal4 = [0.0,0.707,0.0,0.707,0.00,-0.50,0.00]

  try:

    print "============ Press `Enter` to begin the fanuc by setting up the moveit_commander ..."
    raw_input()
    fanuc = FanucArm("fanuc")

    print "============ Press `Enter` to execute ready ..."
    raw_input()
    fanuc.go_to_pose_goal(goal1)

    print "============ Press `Enter` to execute grip position ..."
    raw_input()
    fanuc.go_to_pose_goal(goal2)

    print "============ Press `Enter` to execute grip ..."
    raw_input()
    fanuc.gripper_on()

    print "============ Press `Enter` to execute carry ..."
    raw_input()
    fanuc.go_to_pose_goal(goal3)

    print "============ Press `Enter` to execute drop position ..."
    raw_input()
    fanuc.go_to_pose_goal(goal4)

    print "============ Press `Enter` to execute drop ..."
    raw_input()
    fanuc.gripper_off()

    print "============ Python fanuc demo complete!"

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

