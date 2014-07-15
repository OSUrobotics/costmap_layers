#!/usr/bin/env python

import sys
import rospy
from std_srvs.srv import Empty

def path_collect_client():
	rospy.wait_for_service('record_path')
	try:
		path = rospy.ServiceProxy('record_path', Empty)
		path()
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def usage():
	print "USAGE: rosservice call path_collect"

if __name__ == "__main__":
	if len(sys.argv) != 1:
		usage()
		sys.exit(1)
	else:
		print "Starting/Stopping Path Recording"
		path_collect_client()