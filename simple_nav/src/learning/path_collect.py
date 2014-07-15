#!/usr/bin/env python

# ROS Libraries
from std_srvs.srv import Empty
from nav_msgs.srv import GetMap
import rospy

from time import sleep



class PathRecorder():
	"""Service to record the path a robot takes."""
	def __init__(self):
		
		# Init pseudo State Machine
		self.state = 'INIT'
		# Start Service
		rospy.init_node('path_recorder')
		s = rospy.Service('record_path', Empty, self.handle_record)

		# Get map metadata
		rospy.wait_for_service('static_map')
		try:
			get_map = rospy.ServiceProxy('static_map', GetMap)
			static_map = static_map()
			self.map_frame = static_map.header.frame_id
			self.map_metadata = static_map.info
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

		# Subscribe to robot position via /odom

		# User guidance
		rospy.loginfo("Send an Empty srv to start or stop recording data.")
		self.state = 'IDLE'
		rospy.spin()


	def handle_record(self, req):

		if self.state == 'INIT' :
			pass

		elif self.state == 'IDLE' :
			self.state = 'RECORDING'
		
		elif self.state == 'RECORDING':
			self.state = 'SAVING'

		elif self.state == 'SAVING':
			pass

		info = "State: " + self.state
		rospy.loginfo(info)
		return []

	def switch(self):
		pass



if __name__ == "__main__":
	PathRecorder()