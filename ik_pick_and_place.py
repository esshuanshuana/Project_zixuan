#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Inverse Kinematics Pick and Place Demo
"""
import argparse
import struct
import sys
import copy

import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface

class PickAndPlace(object):
    def __init__(self, limb, hover_distance = 0.15, verbose=True):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def Waypoints_Path(self, pose):
        approach = copy.deepcopy(pose)
        approach.position.z = approach.position.z +  self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)

    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    def pick(self, pose, Waypoints):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()
	print("done retract")
	print("-------------------")
        for i in Waypoints:
            print("Moving")
            self.Waypoints_Path(i)
        pass

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

def load_gazebo_models(block_pose=Pose(position=Point(x=0.75, y=0.0265, z=0.7825)),block_reference_frame="world", table_pose1 = Pose(position=Point(x=0.24, y= 1.19, z = 0.0)), table_reference_frame1 = "world") :
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
    # Load Block URDF
    block_xml = ''
    with open (model_path + "block/model.urdf", "r") as block_file:
        block_xml=block_file.read().replace('\n', '')
 
#    # Load obstacle Block URDF
#    block_xml1 = ''
#    with open (model_path + "block/model1.urdf", "r") as block_file:
#        block_xml1=block_file.read().replace('\n', '')
#
#    # Load obstacle Block 2 URDF
#    block_xml2 = ''
#    with open (model_path + "block/model2.urdf", "r") as block_file:
#        block_xml2=block_file.read().replace('\n', '')
#
#	block_xml3 = ''
#    with open (model_path + "block/model3.urdf", "r") as block_file:
#        block_xml3=block_file.read().replace('\n', '')

#   # Spawn Table SDF
#    rospy.wait_for_service('/gazebo/spawn_sdf_model')
#    try:
#        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
#        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
#                             table_pose, table_reference_frame)
#    except rospy.ServiceException, e:
#        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    # Spawn Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))
 
   # Spawn side table
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table1", table_xml, "/",
                             table_pose1, table_reference_frame1)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

#    # Spawn Obstacle Block URDF
#    rospy.wait_for_service('/gazebo/spawn_urdf_model')
#    try:
#        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
#        resp_urdf = spawn_urdf("block1", block_xml1, "/",
#                               block_pose1, block_reference_frame1)
#    except rospy.ServiceException, e:
#        rospy.logerr("Spawn URDF service call failed: {0}".format(e))
#
#    # Spawn Obstacle Block 2 URDF
#    rospy.wait_for_service('/gazebo/spawn_urdf_model')
#    try:
#        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
#        resp_urdf = spawn_urdf("block2", block_xml2, "/",
#                               block_pose2, block_reference_frame2)
#    except rospy.ServiceException, e:
#        rospy.logerr("Spawn URDF service call failed: {0}".format(e))
#
#    # Spawn Obstacle Block 3 URDF
#    rospy.wait_for_service('/gazebo/spawn_urdf_model')
#    try:
#        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
#        resp_urdf = spawn_urdf("block3", block_xml3, "/",
#                               block_pose3, block_reference_frame3)
#    except rospy.ServiceException, e:
#        rospy.logerr("Spawn URDF service call failed: {0}".format(e))


#def delete_gazebo_models():
#    # This will be called on ROS Exit, deleting Gazebo models
#    # Do not wait for the Gazebo Delete Model service, since
#    # Gazebo should already be running. If the service is not
#    # available since Gazebo has been killed, it is fine to error out
#    try:
#        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
#        resp_delete = delete_model("cafe_table")
#	resp_delete = delete_model("cafe_table1")
#        resp_delete = delete_model("block")
#        resp_delete = delete_model("block1")
#        resp_delete = delete_model("block2")
#        resp_delete = delete_model("block3")
#    except rospy.ServiceException, e:
#        rospy.loginfo("Delete Model service call failed: {0}".format(e))

def main():
    """RSDK Inverse Kinematics Pick and Place Example
    A Pick and Place example using the Rethink Inverse Kinematics
    Service which returns the joint angles a requested Cartesian Pose.
    This ROS Service client is used to request both pick and place
    poses in the /base frame of the robot.
    Note: This is a highly scripted and tuned demo. The object location
    is "known" and movement is done completely open loop. It is expected
    behavior that Baxter will eventually mis-pick or drop the block. You
    can improve on this demo by adding perception and feedback to close
    the loop.
    """
    rospy.init_node("ik_pick_and_place_demo")
    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame
    #load_gazebo_models()
    # Remove models from the scene on shutdown
    #rospy.on_shutdown(delete_gazebo_models)

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'left'
    hover_distance = 0.15 # meters
    # Starting Joint angles for left arm
    starting_joint_angles = {'left_w0': 0.6699952259595108,
                             'left_w1': 1.030009435085784,
                             'left_w2': -0.4999997247485215,
                             'left_e0': -1.189968899785275,
                             'left_e1': 1.9400238130755056,
                             'left_s0': -0.08000397926829805,
                             'left_s1': -0.9999781166910306}
    pnp = PickAndPlace(limb, hover_distance)
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)
    block_poses1 = list()
    block_poses2 = list()
    waypoints_red = []
    waypoints_white = []
 
   # way points from Rviz recorded for Red Block movement
    waypoints_red.append(Pose(
        position=Point(x=0.772243728412, y=0.049878392412, z=0.237445637829),
        orientation=overhead_orientation))
    waypoints_red.append(Pose(
        position=Point(x=0.442334362732, y=0.588345362253, z=0.252167453272),
        orientation=overhead_orientation))
    waypoints_red.append(Pose(
        position=Point(x=0.488997664471, y=0.799654343053, z=0.232236784345),
        orientation=overhead_orientation))
    waypoints_red.append(Pose(
        position=Point(x=0.228884087043, y=0.704006806216, z=0.0508355368998),
        orientation=overhead_orientation))
    waypoints_red.append(Pose(
        position=Point(x=0.176512084352,y=0.9398524315222,z=0.21987734644744),
        orientation=overhead_orientation))
  
  # way points from Rviz recorded for White Block movement
    waypoints_white.append(Pose(
        position=Point(x=0.35156773222, y=0.756424456567, z=0.06500676986513),
        orientation=overhead_orientation))
    waypoints_white.append(Pose(
        position=Point(x=0.413156773222, y=0.622078801232, z=0.1200391823423),
        orientation=overhead_orientation))
    waypoints_white.append(Pose(
        position=Point(x=0.573093167106, y=0.353891577179, z=0.12655786157),
        orientation=overhead_orientation))
   
    # The Pose of the block in its initial location.
    # You may wish to replace these poses with estimates
    # from a perception node.
    block_poses1.append(Pose(
        position=Point(x=0.7752790859786, y=0.05158832059135, z=-0.13806896344),
        orientation=overhead_orientation))
    # Feel free to add additional desired poses for the object.
    # Each additional pose will get its own pick and place.
    block_poses1.append(Pose(
        position=Point(x=0.176599153165, y=0.944884152042, z=-0.1251730968286),
        orientation=overhead_orientation))
    # The Pose of the block in its initial location.
    # You may wish to replace these poses with estimates
    # from a perception node.
    block_poses2.append(Pose(
        position=Point(x=0.325322322232, y=0.824979557941, z=-0.150692393622),
        orientation=overhead_orientation))
    # Feel free to add additional desired poses for the object.
    # Each additional pose will get its own pick and place.
    block_poses2.append(Pose(
        position=Point(x=0.775444322831, y=0.0514932762513, z=-0.124422125162),
        orientation=overhead_orientation))
    # Move to the desired starting angles
    pnp.move_to_start(starting_joint_angles)
    idx_1 = 0
    idx_2 = 0
    while not rospy.is_shutdown():
        print("\nPicking...Red Block")
        pnp.pick(block_poses1[idx_1], waypoints_red)
        print("\nPlacing...Red Block")
        idx_1 = (idx_1+1) % len(block_poses1)
        pnp.place(block_poses1[idx_1])

	break
    return 0

if __name__ == '__main__':
	sys.exit(main())
