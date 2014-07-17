#!/usr/bin/env python

# ROS data types
from std_srvs.srv import Empty
from nav_msgs.srv import GetMap
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
# ROS Libraries
import rospy
import tf
# External Libraries
from time import sleep



class PathRecorder():
	"""Service to record the path a robot takes."""
	def __init__(self):
		
		# Init pseudo State Machine
		self.state = 'INIT'
		# Start Service
		rospy.init_node('path_recorder')
		s = rospy.Service('record_path', Empty, self.handle_service)

		# Get map metadata
		rospy.wait_for_service('static_map')
		try:
			get_map = rospy.ServiceProxy('static_map', GetMap)
			static_map = get_map().map
			self.map_frame = static_map.header.frame_id
			self.map_metadata = static_map.info
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

		# Listen for transforms between /odom and map_frame
		self.transformer = tf.TransformListener()

		# Subscribe to robot position via /odom
		self.position_sub = rospy.Subscriber("odom", Odometry, self.position_callback)

		# User guidance
		rospy.loginfo("Send an Empty srv to start or stop recording data.")

		self.path = []
		self.state = 'IDLE'
		rospy.spin()


	def handle_service(self, req):

		if self.state == 'INIT' :
			pass

		elif self.state == 'IDLE' :
			self.state = 'RECORDING'
		
		elif self.state == 'RECORDING':
			self.state = 'SAVING'

			self.save()

			print "Path length:", len(self.path)
			self.path = []
			self.state = 'IDLE'

		elif self.state == 'SAVING':
			pass

		info = "State: " + self.state
		rospy.loginfo(info)
		return []


	def save(self):
		print self.path
		print self.map_metadata


	def position_callback(self, odom):
		if self.state == 'INIT' :
			pass

		elif self.state == 'IDLE' :
			pass

		elif self.state == 'RECORDING':
			# create a stamped point to transform
			pt = PointStamped()
			pt.header = odom.header
			pt.point = odom.pose.pose.position
			# transform point
			self.transformer.waitForTransform(pt.header.frame_id, self.map_frame, rospy.Time.now(), rospy.Duration(2.0))
			pt = self.transformer.transformPoint(self.map_frame, pt)

			self.path.append(pt.point)

		elif self.state == 'SAVING':
			pass



if __name__ == "__main__":
	PathRecorder()