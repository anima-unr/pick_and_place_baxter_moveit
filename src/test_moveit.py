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
# Author: Acorn Pooley

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from ar_track_alvar_msgs.msg import AlvarMarkers
## END_SUB_TUTORIAL

import baxter_interface

from std_msgs.msg import String



class Pick_and_Place(object):


  def __init__(self, limb, side):

  ## BEGIN_TUTORIAL
  ##
  ## Setup
  ## ^^^^^
  ## CALL_SUB_TUTORIAL imports
  ##
  ## First initialize moveit_commander and rospy.


    self.robot = moveit_commander.RobotCommander()

    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    self.scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a MoveGroupCommander object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the left
    ## arm.  This interface can be used to plan and execute motions on the left
    ## arm.
    self.group = moveit_commander.MoveGroupCommander("right_arm")
    self.group.set_max_velocity_scaling_factor(0.4);

    self.gripper = baxter_interface.Gripper("right")
    self.gripper.calibrate()
    self.gripper.set_holding_force(100.0)


    ## We create this DisplayTrajectory publisher which is used below to publish
    ## trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory,queue_size=10)

    ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
    print "============ Waiting for RVIZ..."
    print "============ Starting tutorial "

    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    ##
    ## We can get the name of the reference frame for this robot
    print "============ Reference frame: %s" % self.group.get_planning_frame()
    self.group.set_pose_reference_frame("pedestal")

    ## We can also print the name of the end-effector link for this group
    print "============ Reference frame: %s" % self.group.get_end_effector_link()

    ## We can get a list of all the groups in the robot
    print "============ Robot Groups:"
    print self.robot.get_group_names()




  
  def pick_place_implementation(self,pose_target):
    print "============ picking 1"
    # pose_target = geometry_msgs.msg.Pose()
    # #pose_target.orientation.w = 1.0
    # pose_target.position.x = 0.742312980224
    # pose_target.position.y = 0.0484443011482
    # pose_target.position.z = -0.0723735271964

    # pose_target.orientation.x = 0.211411915325
    # pose_target.orientation.y = 0.977291845123
    # pose_target.orientation.z = 0.0130963043807
    # pose_target.orientation.w = 0.00584280116247

    pick_pose_offset = copy.deepcopy(pose_target)
  
    pick_pose_offset.position.z = pick_pose_offset.position.z + 0.2
    print pick_pose_offset
    rospy.sleep(3)

    self.moveToPose(pick_pose_offset)

    print "============ grasping open"

    self.gripper.command_position(100.0)

    rospy.sleep(3)

    print "============ picking 2"

    self.moveToPose(pose_target)

    print "============ grasping close"

    self.gripper.command_position(0.0)

    rospy.sleep(3)

    print pick_pose_offset
    print "++++++++++++ going back"



    self.moveToPose(pick_pose_offset)


    print "============ placing 1"
    place_pose_target = geometry_msgs.msg.Pose()
    #pose_target.orientation.w = 1.0
    place_pose_target.position.x = 0.505527106173
    place_pose_target.position.y = -0.512224242973
    place_pose_target.position.z = -0.162671165001

    place_pose_target.orientation.x = 0.0162278031886
    place_pose_target.orientation.y = 0.999276339747
    place_pose_target.orientation.z = -0.00852550066179
    place_pose_target.orientation.w = -0.0333282322412

    place_pose_offset = copy.deepcopy(place_pose_target)
  
    place_pose_offset.position.z = place_pose_offset.position.z + 0.2
    print place_pose_offset
    rospy.sleep(3)



    self.moveToPose(place_pose_offset)

    #print "============ grasping open"

    #self.gripper.command_position(100.0)

    #rospy.sleep(2)

    print "============ placing 2"

    self.moveToPose(place_pose_target)

    # print "============ grasping close"

    # self.gripper.command_position(0.0)

    # rospy.sleep(2)

    print pick_pose_offset
    print "++++++++++++ going back"



    self.moveToPose(place_pose_offset)




    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()

    ## END_TUTORIAL

    print "============ STOPPING"

  def moveToPose(self,pose):

        # define temp pose
    pose_target = geometry_msgs.msg.Pose()

        # format the pose correctly
    print "HELLOOOOOOOOOOO"
    print pose
    pose_target.orientation.x = pose.orientation.x
    pose_target.orientation.y = pose.orientation.y
    pose_target.orientation.z = pose.orientation.z
    pose_target.orientation.w = pose.orientation.w
    pose_target.position.x = pose.position.x
    pose_target.position.y = pose.position.y
    pose_target.position.z = pose.position.z

        # set things
    self.group.set_pose_target(pose_target)
    self.group.set_num_planning_attempts(5);
    self.group.set_planning_time(10.0);
    self.group.set_goal_position_tolerance(0.0075)
    self.group.set_goal_orientation_tolerance(0.0075)

    print("\tPlanning...")
    plan1 = self.group.plan()
        # rospy.sleep(5)
    print("\tExecuting...")
    self.group.go(wait=True)


  def ar_tag_callback(self,data):

    pose_target = geometry_msgs.msg.Pose()
    #pose_target.orientation.w = 1.0

    print data.markers[0].id 

    for x in range(12):
      if data.markers[x].id == 1:
        print data.markers[x].pose.pose.position.z
        pose_target.position.x = data.markers[x].pose.pose.position.x
        pose_target.position.y = data.markers[x].pose.pose.position.y
        pose_target.position.z =  data.markers[x].pose.pose.position.z

        pose_target.orientation.x = data.markers[x].pose.pose.orientation.x
        pose_target.orientation.y = data.markers[x].pose.pose.orientation.y
        pose_target.orientation.z = data.markers[x].pose.pose.orientation.z
        pose_target.orientation.w = data.markers[x].pose.pose.orientation.w

    rospy.sleep(2)
    print "adding box"
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = self.robot.get_planning_frame()
    box_pose.pose.position.x = pose_target.position.x
    box_pose.pose.position.y = pose_target.position.y
    box_pose.pose.position.z = pose_target.position.z
    # box_pose.pose.position.x = 0.137449324269
    # box_pose.pose.position.y = 0.183503351278
    # box_pose.pose.position.z = 1.16385190575
    box_name = "box"
    self.scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))
    box_pose.header.frame_id = self.robot.get_planning_frame()
    box_pose.pose.position.x = 0.505527106173
    box_pose.pose.position.y = -0.512224242973
    box_pose.pose.position.z = -0.162671165001
    box_name = "boxx"
    self.scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))
    self.pick_place_implementation(pose_target)
    # print data.markers[1].id
    # print data.markers[0].pose.pose.position.x
    # print data.markers[1].pose.pose.position.x

def main():
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  ## BAnima - enabling robot
  
  rs = baxter_interface.RobotEnable()
  rs.enable()
  pickplace = Pick_and_Place('left_arm','left')
  sub_img = rospy.Subscriber("ar_pose_marker", AlvarMarkers, pickplace.ar_tag_callback)
  #pickplace.pick_place_implementation()
  rospy.spin()


if __name__=='__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass

