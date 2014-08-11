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
	def __init__(self, parsed_args):
		package = rospkg.RosPack()
		package_path = package.get_path('simple_nav')


		if '/' not in parsed_args.paths:
			self.paths_bag  = package_path + "/bags/paths/" + parsed_args.paths
		else:
			self.paths_bag = parsed_args.paths
		

		bags_folder 			= package_path + '/bags/costmaps/'

		self.new_bag = False
		if parsed_args.costmap == None:
			self.new_bag 			= True
			bag_name				= time.strftime("%Y-%m-%d_%H-%M-%S_costmap.bag")
		else:
			if '/' in parsed_args.costmap:
				bags_folder = ''
			bag_name 				= parsed_args.costmap

		self.costmap_bag				= bags_folder + bag_name

		print "Analyzing data from paths:"
		print "\t", self.paths_bag
		print "into costmap:"
		print "\t", self.costmap_bag


	def _load_paths(self):
		"""Load saved data from bag file
		args: None
		Pre: 
			- self.paths_bag is full filepath to bagfile
			- Bag File must exist
		Post:
			- self.map_metadata describes map
			- self.map_header is the old map header
			- self.paths is a list containing one PoseArray
				for each path recorded
		"""
		bag 				= rosbag.Bag(self.paths_bag)

		for topic, msg, t in bag.read_messages(topics=['map_metadata']):
			self.map_metadata 	= msg

		for topic, msg, t in bag.read_messages(topics=['map_header']):
			self.map_header 	= msg

		self.paths 			= []

		for topic, msg, t in bag.read_messages(topics=['path']):
			self.paths.append(msg)

		bag.close()

		self._msg_to_paths()


	def _load_costmap(self):
		self.cost_grid = np.ones((self.height, self.width))

		if self.new_bag: return

		bag 			= rosbag.Bag(self.costmap_bag)

		for topic, msg, t in bag.read_messages(topics=['costmap']):
			cost_grid 	= msg

		bag.close()

		height = cost_grid.info.height
		width = cost_grid.info.width
		self.cost_grid = np.array(cost_grid.data).reshape((height, width)).astype(np.float)
		self.cost_grid /= 50


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
		
		print "Analyzing paths..."
		count = 0
		for path in self.paths:
			count += 1
			print "\t%d of %d" % (count, len(self.paths))
			distances = np.empty((self.height, self.width))
			distances.fill(np.inf)

			min_x, min_y, max_x, max_y = path.bounds
			faraway = 5
			min_x -= faraway; min_y -= faraway
			max_x += faraway; max_y += faraway
			print "\tPath bounding  box area: %fm by %fm" % ( abs(max_x - min_x), abs(max_y - min_y) )
			for x in np.arange(min_x, max_x, self.resolution):
				for y in np.arange(min_y, max_y, self.resolution):
					i, j = self._world_to_map(x, y)
					pt = sPoint(x, y)
					distances[i, j] = path.distance(pt)

			self.cost_grid *= np.tanh(distances * (3.0 / faraway))


	def _world_to_map(self, x, y):
		j = int((x - self.origin.x) / self.resolution)
		i = int((y - self.origin.y) / self.resolution)
		return i, j


	def process(self):
		self._load_paths()
		self._load_costmap()
		# show_image(self.cost_grid)

		self._paths_to_cost()

		# show_image(self.cost_grid)

		self.cost_grid *= 50
		self.cost_grid = self.cost_grid.astype(np.uint8)
		
		# show_image(self.cost_grid)


	def save(self):
		costmap = OccupancyGrid()
		costmap.header = self.map_header
		costmap.info = self.map_metadata
		data = self.cost_grid.flatten()
		data = list(data)
		costmap.data = data

		bag = rosbag.Bag(self.costmap_bag, 'w')
		bag.write("costmap", costmap)
		bag.close()

		print "Costmap saved to " + self.costmap_bag



def show_image(img):
	im = plt.imshow(img)
	im.set_cmap('binary')
	plt.colorbar()
	plt.show()


def parse(argv):
	parser = argparse.ArgumentParser(description="A node that analyzes a bag file of robot paths into a costmap-readable format.")
	parser.add_argument('-c', '--costmap')
	parser.add_argument('paths')
	# parser.add_argument('launch_crap', nargs=argparse.REMAINDER)
	parsed_args = parser.parse_args(argv)

	return parsed_args


def main(argv):
	
	p = PathAnalyzer(parse(argv))
	p.process()
	p.save()


if __name__ == '__main__':
	main(sys.argv[1:])
