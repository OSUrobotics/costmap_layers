#!/usr/bin/env python

# ROS data types
from simple_nav.srv import ChangeState
from nav_msgs.srv import GetMap
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
# ROS Libraries
import rospy
import tf
import rosbag
import rospkg
# External Libraries
import time




class PathRecorder():
	"""Service to record a robot's paths to a bag"""
	def __init__(self):
 
		# Start Node
		rospy.init_node('path_recorder')

		# Initialize state machine
		self._init_state_machine()

		# Start Service
		self.server				= rospy.Service('record_path', ChangeState, self.change_state)

		# Listen for transforms between /odom and map_frame
		self.transformer 		= tf.TransformListener()

		# Subscribe to robot position via /odom
		self.position_sub 		= rospy.Subscriber("odom", Odometry, self.position_callback)

		# User guidance
		rospy.loginfo("Run 'rosservice call /record_path \"\" ' to start/stop recording a path.")
		rospy.loginfo("Run 'rosservice call /record_path \"close\" ' to safely turn path recording off.")
		rospy.loginfo("Run 'rosservice call /record_path \"open\" ' to safely turn path recording back on.")

		# Ready to record a path
		self.state 				= 'IDLE'
		rospy.spin()


	def _init_state_machine(self):
		## Get state machine ready for use
		# Initialialize state
		self.state 				= 'INIT'

		# Get map metadata
		rospy.wait_for_service('static_map')
		get_map 				= rospy.ServiceProxy('static_map', GetMap)
		static_map 				= get_map().map

		self.map_header			= static_map.header
		self.map_metadata 		= static_map.info

		# For use in message generation
		self.path_counter 		= 0
		## Start a PoseArray to record path in
		self.path 				= PoseArray()
		# Header
		self.path.header.seq	= self.path_counter
		self.path_counter 		+= 1
		self.path.header.stamp 	= rospy.Time.now()
		self.path.header.frame_id = self.map_header.frame_id
		# Poses
		self.path.poses			= []

		# ROS Bag for saving map and path data
		package 				= rospkg.RosPack()
		bags_folder 			= package.get_path('simple_nav') + '/bags/paths/'
		bag_name				= time.strftime("%Y-%m-%d_%H-%M-%S_path_data.bag")
		self.bag 				= rosbag.Bag(bags_folder + bag_name, 'w')
		self.bag.write("map_header", self.map_header)
		self.bag.write("map_metadata", self.map_metadata)


	def change_state(self, action):
		"""Control Path Recorder in a thread-safe manner"""

		# convert action to a usable format
		action = action.action
		# Was service call successful? Assume so.
		success = True
		## State Machine
		## Only toggle recording/idle if it's ready
		if self.state 		== 'INIT' :
			# Do nothing if it's still initializing.
			# Failed because it was still initializing
			success 			= False

		elif self.state 	== 'OFF':
			if action.lower() 	== "open":
				self._init_state_machine()
				self.state 			= 'IDLE'
			else:
				success 			= False

		elif self.state 	== 'IDLE' :
			if action.lower()	== "close":
				self.shut_off()
			else:
				# Toggle to recording
				self.state 			= 'RECORDING'
		
		elif self.state 	== 'RECORDING':
			# Done recording, so save the path
			self.state 			= 'SAVING'

			self.save()

			if action.lower() 	== "close":
				self.shut_off()
			else:
				self.state 			= 'IDLE'

		elif self.state 	== 'SAVING':
			# Don't let things happen during saving
			# Clearly things did not go as planned
			success 			= False

		else:
			err = "State " + str(self.state) + " not recognized."
			raise ValueError(err)

		info = "Path Recorder is " + self.state + "."
		rospy.loginfo(info)
		return success


	def save(self):
		with open('workfile', 'w') as outfile:
			self.bag.write("path", self.path)
			print "Path saved to bag"

		self.path.header.seq	= self.path_counter
		self.path_counter 		+= 1
		self.path.header.stamp 	= rospy.Time.now()
		self.path.poses 		= []


	def shut_off(self):
		self.bag.close()
		self.state = 'OFF'


	def position_callback(self, odom):
		if self.state 		== 'INIT' :
			pass

		if self.state 		== 'OFF':
			pass

		elif self.state 	== 'IDLE' :
			pass

		elif self.state 	== 'RECORDING':
			# create a stamped pose to transform
			pose				= PoseStamped()
			pose.header 		= odom.header
			pose.pose 			= odom.pose.pose
			# transform pose
			self.transformer.waitForTransform(pose.header.frame_id, 
				self.map_header.frame_id, 
				rospy.Time.now(), 
				rospy.Duration(2.0)
				)
			pose 				= self.transformer.transformPose(self.map_header.frame_id, pose)
			# Append pose to path
			self.path.poses.append(pose.pose)

		elif self.state == 'SAVING':
			pass



if __name__ == "__main__":
	PathRecorder()