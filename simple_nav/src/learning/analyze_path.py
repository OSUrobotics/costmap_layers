#!/usr/bin/env python

# ROS data types
from simple_nav.srv 		import ChangeState
from nav_msgs.srv 			import GetMap
from nav_msgs.msg 			import OccupancyGrid
from nav_msgs.msg 			import Odometry
from geometry_msgs.msg 		import PoseArray
from geometry_msgs.msg 		import PoseStamped
# ROS Libraries
import rospy
import tf
import rosbag
import rospkg
# External Libraries
import numpy as np
import matplotlib.pyplot as plt
import cv2



class PathAnalyzer():
	"""Analyzes a robot's paths and puts them into a costmap-style format"""
	def __init__(self):
		rospy.init_node('analyze_path')


	def load(self):
		"""Load saved data from bag file
		args: none
		Pre: 
			- Bag File must exist
		Post:
			- self.map_metadata describes map
			- self.paths is a list containing one PoseArray
				for each path recorded
		"""
		package 			= rospkg.RosPack()
		bag_path  			= package.get_path('simple_nav') + "/bags/" + "2014-07-24_14-53-24_path_data.bag"
		bag 				= rosbag.Bag(bag_path)

		for topic, msg, t in bag.read_messages(topics=['map_metadata']):
			# print topic
			# print msg
			self.map_metadata 	= msg
			# print "\n\n"

		for topic, msg, t in bag.read_messages(topics=['map_header']):
			self.map_header 	= msg

		self.paths 			= []

		for topic, msg, t in bag.read_messages(topics=['path']):
			# print topic
			# print msg.header
			self.paths.append(msg)
			# print "------\n"

		bag.close()


	def convert(self):
		"""Store loaded data in an image representing a costmap
		args: none
		Pre:
			- must be called after load()
		Post:
			- all points visited by robot have been marked with a 255 in cost_img
		"""
		origin 			= self.map_metadata.origin.position
		resolution		= self.map_metadata.resolution
		width 			= self.map_metadata.width
		height 			= self.map_metadata.height
		self.cost_img	= np.zeros((height, width), dtype=np.uint8)

		for path in self.paths:
			for ind in range(len(path.poses) - 1):
				i1 = int((path.poses[ind].position.x - origin.x) / resolution)
				j1 = int((path.poses[ind].position.y - origin.y) / resolution)
				i2 = int((path.poses[ind + 1].position.x - origin.x) / resolution)
				j2 = int((path.poses[ind + 1].position.y - origin.y) / resolution)

				cv2.line(self.cost_img, (i1, j1), (i2, j2), 255)


		# show_image(cv2.resize(self.cost_img, (800, 800)))


	def connect(self):
		pass


	def process(self):
		el = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
		self.cost_img = cv2.dilate(self.cost_img, el)
		self.cost_img = cv2.GaussianBlur(self.cost_img, (15, 15), 0)
		# show_image(cv2.resize(self.cost_img, (800, 800)))
		# show_image(self.cost_img[1500:2500, 2000:3000])


	def costmap_serve(self, req):
		costmap = OccupancyGrid()
		costmap.header = self.map_header
		costmap.info = self.map_metadata
		data = list(self.cost_img.flatten())
		costmap.data = data

		return costmap



def show_image(img):
	cv2.imshow("cost_img", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()



if __name__ == '__main__':
	p = PathAnalyzer()
	p.load()
	p.convert()
	p.process()

	s = rospy.Service('costmap_serve', GetMap, p.costmap_serve)
	rospy.loginfo("Service /costmap_serve ready.")
	rospy.spin()
