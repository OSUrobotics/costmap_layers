#!/usr/bin/env python

# ROS data types
from simple_nav.srv import ChangeState
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
# ROS Libraries
import rospy
# External Libraries
import sys
# Local Libraries
from pc2_to_numpy import numpy_msg



class PeoplePath(object):
	"""Publish PoseStamped of person positions from PointCloud2"""
	def __init__(self, topic):
		super(PeoplePath, self).__init__()
		self.topic = topic

		print "Starting Node people_paths"
		rospy.init_node("people_paths")

		# self.topic = "/torso/cluster_centers"
		print "Subscribing to", self.topic
		self.sub = rospy.Subscriber(self.topic, numpy_msg(PointCloud2), self.people_callback)

		print "Publishing to /person_poses"
		self.pub = rospy.Publisher("person_poses", PoseStamped)


	def path_collect_client(self):
		rospy.wait_for_service('record_path')
		try:
			path = rospy.ServiceProxy('record_path', ChangeState)
			response = path()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e


	def people_callback(self, people):
		x, y, z = people.data[0, 0]
		pose  = PoseStamped()
		pose.pose.position.x = x
		pose.pose.position.y = y
		pose.pose.orientation.w = 1.0
		pose.header = people.header

		self.pub.publish(pose)



if __name__ == "__main__":
	
	p = PeoplePath("/torso/cluster_centers")

	rospy.spin()