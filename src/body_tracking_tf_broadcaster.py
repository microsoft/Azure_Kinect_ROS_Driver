#!/usr/bin/env python
#Author: Aswin K Ramasubramanian
#Affiliation: University College Dublin
# /*********************************************************************
# * Software License Agreement (BSD License)
# *
# * Copyright (c) 2020, Aswin K Ramasubramanian 
# * All rights reserved.
# *
# * Redistribution and use in source and binary forms, with or without
# * modification, are permitted provided that the following conditions
# * are met:
# *
# Redistributions of source code must retain the above copyright
# * notice, this list of conditions and the following disclaimer.
# Redistributions in binary form must reproduce the above
# * copyright notice, this list of conditions and the following
# * disclaimer in the documentation and/or other materials provided
# * with the distribution.
# Neither the name of the Aswin K Ramasubramanian nor the names of its
# * contributors may be used to endorse or promote products derived
# * from this software without specific prior written permission.
# *
# * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# * POSSIBILITY OF SUCH DAMAGE.
# *********************************************************************/
import math
from time import sleep
import rospy
from geometry_msgs.msg import Quaternion, Vector3, Pose, Point, TransformStamped
from std_msgs.msg import Float64, Float32MultiArray, Float32
import rospy
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray
from ros_openpose.msg import Frame
import tf
import tf2_ros
import tf_conversions
import geometry_msgs.msg
     
class Transform_publisher:
	def __init__(self):
		self.pub = rospy.Publisher('joint_states', JointState, queue_size=100)		
		self.body_marker_kinect = rospy.Subscriber("/body_tracking_data", MarkerArray, self.callback, queue_size=10)

		self.pelvis_marker = Float32()

		self.pelvis_pose = Pose()
		self.spine_naval_pose = Pose()
		self.spine_chest_pose = Pose()
		self.neck_pose = Pose()
		self.clavicle_left_pose = Pose()
		self.shoulder_left_pose = Pose()
		self.wrist_left_pose = Pose()
		self.elbow_left_pose = Pose()
		self.hand_left_pose = Pose()
		self.handTip_left_pose = Pose()
		self.thumb_left_pose = Pose()

		self.clavicle_right_pose = Pose()
		self.shoulder_right_pose = Pose()
		self.elbow_right_pose = Pose()
		self.wrist_right_pose = Pose()
		self.hand_right_pose = Pose()
		self.handTip_right_pose = Pose()
		self.thumb_right_pose = Pose()

		self.hip_left_pose = Pose()
		self.knee_left_pose = Pose()
		self.ankle_left_pose = Pose()
		self.foot_left_pose = Pose()

		self.hip_right_pose = Pose()
		self.knee_right_pose = Pose()
		self.ankle_right_pose = Pose()
		self.foot_right_pose = Pose()

		self.head_pose = Pose()
		self.nose_pose = Pose()





	def transform_publisher(self,pose, parent_frame, child_frame):
		br = tf2_ros.TransformBroadcaster()
		t = TransformStamped()
		t.header.frame_id = parent_frame
		t.child_frame_id = child_frame
		
		t.transform.translation = pose.position
		t.transform.rotation = pose.orientation

		br.sendTransform(t)

		return	t

	def callback(self,msg):
		for i in range(32):		
			position_id = msg.markers[i].id
			joint_information = [msg.markers[i].id,msg.markers[i].pose.position,msg.markers[i].pose.orientation]
			joint_orientation = [msg.markers[i].id,joint_information[2]]
			if msg.markers[i].id == 100: #Pelvis
				self.pelvis_marker = msg.markers[i]
				self.pelvis_pose = msg.markers[i].pose
				pelvis_transform = self.transform_publisher(self.pelvis_pose,"depth_camera_link","Pelvis")
			if msg.markers[i].id == 101: #Spine_Naval
				self.spine_naval_pose = msg.markers[i].pose
				spine_transform = self.transform_publisher(self.spine_naval_pose,"depth_camera_link","Spine_Naval")	 
			if msg.markers[i].id == 102: #Spine_Chest
				self.spine_chest_pose = msg.markers[i].pose
				chest_transform = self.transform_publisher(self.spine_chest_pose,"depth_camera_link","Spine_Chest")
			if msg.markers[i].id == 103: # Neck
				self.neck_pose = msg.markers[i].pose
				neck_transform = self.transform_publisher(self.neck_pose,"depth_camera_link","Neck")

			if msg.markers[i].id == 104: # Clavicle_left
				self.clavicle_left_pose = msg.markers[i].pose
				Clavicle_left_transform = self.transform_publisher(self.clavicle_left_pose,"depth_camera_link","Clavicle_left")
			if msg.markers[i].id == 105: # Shoulder_left
				self.shoulder_left_pose = msg.markers[i].pose
				shoulder_left_transform = self.transform_publisher(self.shoulder_left_pose,"depth_camera_link","Shoulder_left")
			if msg.markers[i].id == 106: # Elbow_left
				self.elbow_left_pose = msg.markers[i].pose
				elbow_left_transform = self.transform_publisher(self.elbow_left_pose,"depth_camera_link","Elbow_left")
			if msg.markers[i].id == 107: # Wrist_left
				self.wrist_left_pose = msg.markers[i].pose
				wrist_left_transform = self.transform_publisher(self.wrist_left_pose,"depth_camera_link","Wrist_left")
			if msg.markers[i].id == 108: #Hand_left
				self.hand_left_pose = msg.markers[i].pose
				hand_left_transform = self.transform_publisher(self.hand_left_pose,"depth_camera_link","Hand_left")
			if msg.markers[i].id == 109: #Handtip_left
				self.handTip_left_pose = msg.markers[i].pose
				handtip_left_transform = self.transform_publisher(self.handTip_left_pose,"depth_camera_link","Handtip_left")
			if msg.markers[i].id == 110: #Thumb_left
				self.thumb_left_pose = msg.markers[i].pose
				thumb_left_transform = self.transform_publisher(self.thumb_left_pose,"depth_camera_link", "thumb_left")

			if msg.markers[i].id == 111: #Clavicle_right
				self.clavicle_right_pose = msg.markers[i].pose
				clavicle_right_transform = self.transform_publisher(self.clavicle_right_pose,"depth_camera_link","Clavicle_right")
			if msg.markers[i].id == 112: #Shoulder_right
				self.shoulder_right_pose = msg.markers[i].pose
				shoulder_right_transform = self.transform_publisher(self.shoulder_right_pose,"depth_camera_link","Shoulder_right")
			if msg.markers[i].id == 113: #Elbow_right
				self.elbow_right_pose = msg.markers[i].pose
				elbow_right_transform = self.transform_publisher(self.elbow_right_pose,"depth_camera_link","Elbow_right")
			if msg.markers[i].id == 114: #Wrist_right
				self.wrist_right_pose = msg.markers[i].pose
				wrist_right_transform = self.transform_publisher(self.wrist_right_pose,"depth_camera_link","Wrist_right")
			if msg.markers[i].id == 115: #Hand_right
				self.hand_right_pose = msg.markers[i].pose
				hand_right_transform = self.transform_publisher(self.hand_right_pose,"depth_camera_link","Hand_right")
			if msg.markers[i].id == 116: #Handtip_right
				self.handTip_right_pose = msg.markers[i].pose
				handTip_right_transform = self.transform_publisher(self.handTip_right_pose,"depth_camera_link","Handtip_right")
			if msg.markers[i].id == 117: #Thumb_right
				self.thumb_right_pose = msg.markers[i].pose
				thumb_right_transform = self.transform_publisher(self.thumb_right_pose, "depth_camera_link", "Thumb_right")
			if msg.markers[i].id == 118: #Hip_left
				self.hip_left_pose = msg.markers[i].pose
				hip_left_transform = self.transform_publisher(self.hip_left_pose,"depth_camera_link","Hip_left")
			if msg.markers[i].id == 119:#Knee_left
				self.knee_left_pose = msg.markers[i].pose
				knee_left_transform = self.transform_publisher(self.knee_left_pose,"depth_camera_link","Knee_left")
			if msg.markers[i].id == 120:#Ankle_left
				self.ankle_left_pose = msg.markers[i].pose
				ankle_left_transform = self.transform_publisher(self.ankle_left_pose,"depth_camera_link","Ankle_left")
			if msg.markers[i].id == 121: #Foot_left
				self.foot_left_pose = msg.markers[i].pose
				foot_left_transform = self.transform_publisher(self.foot_left_pose,"depth_camera_link","Foot_left")

			if msg.markers[i].id == 122: #Hip_right
				self.hip_right_pose = msg.markers[i].pose
				hip_right_transform = self.transform_publisher(self.hip_right_pose,"depth_camera_link","Hip_right")
			if msg.markers[i].id == 123:#Knee_right
				self.knee_right_pose = msg.markers[i].pose
				knee_right_transform = self.transform_publisher(self.knee_right_pose,"depth_camera_link","Knee_right")
			if msg.markers[i].id == 124:#Ankle_right
				self.ankle_right_pose = msg.markers[i].pose
				ankle_right_transform = self.transform_publisher(self.ankle_right_pose,"depth_camera_link","Ankle_right")
			if msg.markers[i].id == 125: #Foot_right
				self.foot_right_pose = msg.markers[i].pose
				foot_right_transform = self.transform_publisher(self.foot_right_pose,"depth_camera_link","Foot_right")

			if msg.markers[i].id == 126: #Head
				self.head_pose = msg.markers[i].pose
				head_transform = self.transform_publisher(self.head_pose,"depth_camera_link","Head")
			if msg.markers[i].id == 127: #Nose
				self.nose_pose = msg.markers[i].pose
				nose_transform = self.transform_publisher(self.nose_pose,"depth_camera_link","Nose")


if __name__ == '__main__':
	rospy.init_node('tf_body_tracking_publisher')
	rospy.loginfo_once("Node Initialized")
	rospy.loginfo_once("Started")

	jRE = Transform_publisher()
	r = rospy.Rate(30)
	while not rospy.is_shutdown():
		r.sleep()