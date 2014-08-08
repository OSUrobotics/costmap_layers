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
from shapely.geometry import LineString, Point as sPoint
import cv2
import sys, time
import argparse



class PathAnalyzer():
	"""Analyzes a robot's paths and puts them into a costmap-style format"""
	def __init__(self):
		rospy.init_node('analyze_path')


	def load(self, bag_path):
		"""Load saved data from bag file
		args: bag_path
		Pre: 
			- Bag File must exist
		Post:
			- self.map_metadata describes map
			- self.map_header is the old map header
			- self.paths is a list containing one PoseArray
				for each path recorded
		"""
		bag 				= rosbag.Bag(bag_path)

		for topic, msg, t in bag.read_messages(topics=['map_metadata']):
			self.map_metadata 	= msg

		for topic, msg, t in bag.read_messages(topics=['map_header']):
			self.map_header 	= msg

		self.paths 			= []

		for topic, msg, t in bag.read_messages(topics=['path']):
			self.paths.append(msg)

		bag.close()


	def _msg_to_paths(self):
		line_strings = []
		for path in self.paths:
			line_list = []
			for pose in path.poses:
				line_list.append((pose.position.x, pose.position.y))

			line_strings.append(LineString(line_list))

		self.paths = line_strings

		self.origin			= self.map_metadata.origin.position
		self.resolution		= self.map_metadata.resolution
		self.width 			= self.map_metadata.width
		self.height			= self.map_metadata.height
		

	def _paths_to_cost(self):
		self.cost_grid = np.ones((self.height, self.width))
		for path in self.paths:
			distances = np.empty((self.height, self.width))
			distances.fill(np.inf)

			min_x, min_y, max_x, max_y = path.bounds
			faraway = 5
			min_x -= faraway; min_y -= faraway
			max_x += faraway; max_y += faraway
			print min_x, min_y, max_x, max_y
			for x in np.arange(min_x, max_x, self.resolution):
				for y in np.arange(min_y, max_y, self.resolution):
					i, j = self._world_to_map(x, y)
					pt = sPoint(x, y)
					distances[i, j] = path.distance(pt)

			# self._distances_to_cost(distances)
			self.cost_grid *= np.tanh(distances * (3.0 / faraway))


	def _world_to_map(self, x, y):
		j = int((x - self.origin.x) / self.resolution)
		i = int((y - self.origin.y) / self.resolution)
		return i, j


	def process(self):
		self._msg_to_paths()
		self._paths_to_cost()

		show_image(self.cost_grid)

		self.cost_grid *= 50
		self.cost_grid = self.cost_grid.astype(np.uint8)
		
		# show_image(self.cost_grid)


	def costmap_serve(self):
		self.costmap = OccupancyGrid()
		self.costmap.header = self.map_header
		self.costmap.info = self.map_metadata
		data = self.cost_grid.flatten()
		data = list(data)
		self.costmap.data = data
		
		s = rospy.Service('costmap_server', GetMap, self.costmap_request)
		rospy.loginfo("Service /costmap_serve ready.")
		rospy.spin()


	def save(self):
		costmap = OccupancyGrid()
		costmap.header = self.map_header
		costmap.info = self.map_metadata
		data = self.cost_grid.flatten()
		data = list(data)
		costmap.data = data

		package 				= rospkg.RosPack()
		bags_folder 			= package.get_path('simple_nav') + '/bags/costmaps/'
		bag_name				= time.strftime("%Y-%m-%d_%H-%M-%S_costmap.bag")
		bag 					= rosbag.Bag(bags_folder + bag_name, 'w')
		bag.write("costmap", costmap)
		bag.close()


	def costmap_request(self, req):
		rospy.loginfo("Sending Costmap...")
		return self.costmap



def show_image(img):
	im = plt.imshow(img)
	im.set_cmap('binary')
	plt.colorbar()
	plt.show()


def parse(argv):
	parser = argparse.ArgumentParser(description="A node that analyzes a bag file of robot paths into a costmap-readable format.")
	parser.add_argument('bagfile')
	parser.add_argument('launch_crap', nargs=argparse.REMAINDER)
	parsed_args = parser.parse_args(argv)

	if '/' not in parsed_args.bagfile:
		package = rospkg.RosPack()
		bag_path  = package.get_path('simple_nav') + "/bags/paths/" + parsed_args.bagfile
	else:
		bag_path = parsed_args.bagfile

	return bag_path


def main(argv):
	
	p = PathAnalyzer()
	p.load(parse(argv))
	p.process()
	p.save()


if __name__ == '__main__':
	main(sys.argv[1:])
