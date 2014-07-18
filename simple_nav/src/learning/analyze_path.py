#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import cv2
import json

class PathAnalyzer():
	"""Analyzes a robot's paths and puts them into a costmap-style format"""
	def __init__(self):
		self.loaded = False


	def load(self, filename):
		with open(filename, 'r') as f:
			path_data = json.load(f)

		if not self.loaded:
			width = path_data['width']
			height  = path_data['height']
			self.costmap = np.zeros((width, height), dtype=np.uint8)
			self.loaded = True
		
		self.path = path_data['path']


	def analyze(self):
		self.min_i, self.min_j = self.path[0]
		self.max_i, self.max_j = self.path[0]
		for i, j in self.path:
			self.min_i = min(self.min_i, i)
			self.min_j = min(self.min_j, j)
			self.max_i = max(self.max_i, i)
			self.max_j = max(self.max_j, j)
			self.costmap[i, j] = 255

		self.min_i -= 100
		self.max_i += 100
		self.min_j -= 100
		self.max_j += 100

		print "Showing range [%d:%d, %d:%d]" % (self.min_i, self.max_i, self.min_j, self.max_j)

		self.costmap = cv2.dilate(self.costmap, np.ones((7, 7),np.uint8))
		self.costmap = cv2.GaussianBlur(self.costmap, (11,11), 0)

		# cv2.imshow('image', cv2.resize(self.costmap, (800, 800)))
		cv2.imshow('image', self.costmap[self.min_i:self.max_i, self.min_j:self.max_j])
		cv2.waitKey(0)
		cv2.destroyAllWindows()


	def save(self):
		cropped_costmap = self.costmap[self.min_i : self.max_i, self.min_j : self.max_j]
		costmap_header = "%d %d %d %d \n" % (self.min_i, self.max_i, self.min_j, self.max_j)
		with open('costmap.txt', 'w') as outfile:
			outfile.write(costmap_header)
			np.savetxt(outfile, cropped_costmap, '%u')






if __name__ == '__main__':
	p = PathAnalyzer()
	p.load('workfile')
	p.analyze()
	p.save()
