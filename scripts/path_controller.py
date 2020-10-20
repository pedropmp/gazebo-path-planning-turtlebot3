#! /usr/bin/env python
import rospy 
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
import random
import math
from gazebo_msgs.msg import *
import numpy as np
import csv
import rospkg
import matplotlib.pyplot as plt
from matplotlib import cm
import time
from tf.transformations import *

laser = LaserScan()
pose = Pose()
rospack = rospkg.RosPack()
_map = np.array(list(csv.reader(open(rospack.get_path('path_controller')+"/scripts/"+"map.csv", "rb"), delimiter=" "))).astype("int")
map_resolution = 4

def path_callback(data):
	global pose
	pose = data.pose[-1]
	# rospy.loginfo("pose x: %s", pose.position.x)
	# rospy.loginfo("pose y: %s", pose.position.y)

	index_x = int(pose.position.x*map_resolution)
	index_y = int(pose.position.y*map_resolution)

	if (index_x < 0): index_x = 0
	if (index_x > _map.shape[0]): index_x = _map.shape[0]
	if (index_x < 0): index_x = 0
	if (index_x > _map.shape[0]): index_x = _map.shape[0]

	if (_map[_map.shape[0] - index_y][index_x] == 1):
		_map[_map.shape[0] - index_y][index_x] = 2
	
		rospy.loginfo("Another part mowed ... percentage total aspirated.... %s ", 100*float(np.count_nonzero(_map == 2))/np.count_nonzero(_map == 1) )
		rospy.loginfo("Discrete Map")
		rospy.loginfo("%s", str(_map))
	
def laser_callback(data):
	global laser
	laser = data

def path_planning():
	index_x = int(pose.position.x * map_resolution)
	index_y = int(pose.position.y * map_resolution)
	where_is_the_closest = look_for_closer(index_x, index_y)
	rospy.loginfo("where if the closest: %s", where_is_the_closest)

def look_for_closer(index_x, index_y):
	rospy.loginfo("where i am:")
	rospy.loginfo("index x: %s", index_x)
	rospy.loginfo("index y: %s", index_y)
	for d in range(0, 3):
		rospy.loginfo("left: %s", 	_map[_map.shape[0] - (index_y - 1),			index_x - d])
		rospy.loginfo("right: %s", 	_map[_map.shape[0] - (index_y - 1),			index_x + d])
		rospy.loginfo("up: %s", 	_map[_map.shape[0] - (index_y - 1 + d),		index_x])
		rospy.loginfo("down: %s", 	_map[_map.shape[0] - (index_y - 1 - d),		index_x])

	d = 1
	while d < 15:
		if _map[_map.shape[0] - (index_y), index_x - d] == 1:
			return "left"
		if _map[_map.shape[0] - (index_y), index_x + d] == 1:
			return "right"
		if _map[_map.shape[0] - (index_y + d - 1), index_x] == 1:
			return "up"
		if _map[_map.shape[0] - (index_y + d - 1), index_x] == 1:
			return "down"
		d += 1

def navigation():
	if (min(laser.ranges[90:270]) > .25):
		velocity.linear.x = random.uniform(-.1, -.25)
		velocity.angular.z = .0
	else:
		velocity.linear.x = .0
		velocity.angular.z = .25
	pass

if __name__ == "__main__": 
	rospy.init_node("path_controller_node", anonymous=False)  

	rospy.Subscriber("/gazebo/model_states", ModelStates, path_callback)

	rospy.Subscriber("/scan", LaserScan, laser_callback)

	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)    
	r = rospy.Rate(5) # 10hz
	velocity = Twist()

	rospy.sleep(5)
	while not rospy.is_shutdown():
		where_to_go = path_planning()
		# pub.publish(velocity)	
		r.sleep()

# DISCRETE MAP 		
# 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
# 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
# 0 1 1 1 1 1 1 1 0 1 1 1 1 1 1 1 1 1 1 0
# 0 1 1 1 1 1 1 1 0 1 1 1 1 1 1 1 1 1 1 0
# 0 1 1 1 1 1 1 1 0 1 1 1 1 1 1 1 1 1 1 0
# 0 0 0 0 0 1 1 1 0 1 1 1 1 1 1 1 1 1 1 0
# 0 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0 1 1 1 0
# 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0
# 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0
# 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0
# 0 1 1 1 1 1 1 0 0 0 0 1 1 1 1 1 1 1 1 0
# 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0
# 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0
# 0 1 1 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0
# 0 1 1 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0
# 0 1 1 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0
# 0 1 1 0 1 1 1 0 0 0 0 1 1 1 1 1 1 1 1 0
# 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 1 1 0
# 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 1 1 0
# 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
 
 
