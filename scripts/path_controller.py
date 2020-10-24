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
	seconds = rospy.get_time()
	# if seconds % 5 < .01:
	# 	rospy.loginfo("pose x: %s", pose.position.x)
	# 	rospy.loginfo("pose y: %s", pose.position.y)

	index_x = int(pose.position.x*map_resolution)
	index_y = int(pose.position.y*map_resolution)
	if (index_y <= 0): index_y = 1
	if (index_y > _map.shape[0]): index_y = _map.shape[0]
	if (index_x <= 0): index_x = 1
	if (index_x > _map.shape[1]): index_x = _map.shape[1]

	# if seconds % .1 < .01:
	# 	rospy.loginfo("index x: %s", index_x)
	# 	rospy.loginfo("index y: %s", _map.shape[0] - index_y)

	if (_map[_map.shape[0] - index_y][index_x] == 1):	# numero de linhas - index_y
		_map[_map.shape[0] - index_y][index_x] = 2
	
		rospy.loginfo("Another part mowed ... percentage total aspirated.... %s ", 100*float(np.count_nonzero(_map == 2))/(np.count_nonzero(_map == 1) + np.count_nonzero(_map == 2)))
		rospy.loginfo("Discrete Map")
		rospy.loginfo("%s", str(_map))
	
def laser_callback(data):
	global laser
	laser = data
	seconds = rospy.get_time()
	# if seconds % 10 < .1:
	# 	rospy.loginfo("laser: %s", laser.ranges)

def path_planning():
	index_x = int(pose.position.x * map_resolution)
	index_y = int(pose.position.y * map_resolution)
	if (index_y <= 0): index_y = 1
	if (index_y > _map.shape[0]): index_y = _map.shape[0]
	if (index_x <= 0): index_x = 1
	if (index_x > _map.shape[1]): index_x = _map.shape[1]

	where_is_the_closest = look_for_closer(index_x, index_y)
	seconds = rospy.get_time()
	# if seconds % 1 < .01:
	# rospy.loginfo("where is the closest: %s", where_is_the_closest)
	return where_is_the_closest

def look_for_closer(index_x, index_y):
	left_wall = False
	down_wall = False
	right_wall = False
	up_wall = False
	d = 1
	while d < 20:
		row = _map.shape[0] - (index_y)
		column = index_x
		row_max = _map.shape[0]
		column_max = _map.shape[1]
		if (column - d) >= 0 and not left_wall:
			if _map[row, column - d] == 1:
				return "left"
			elif _map[row, column - d] == 0:
				left_wall = True

		if (row + d) >= 0 and (row + d) < row_max and not down_wall:
			if _map[_map.shape[0] - (index_y - d), index_x] == 1:
				return "down"
			elif _map[_map.shape[0] - (index_y - d), index_x] == 0:
				down_wall = True

		if (column + d) < column_max and not right_wall:
			if _map[_map.shape[0] - (index_y), index_x + d] == 1:
				return "right"
			elif _map[_map.shape[0] - (index_y), index_x + d] == 0:
				right_wall = True

		if (row - d) >= 0 and not up_wall:
			if _map[_map.shape[0] - (index_y + d), index_x] == 1:
				return "up"
			if _map[_map.shape[0] - (index_y + d), index_x] == 0:
				up_wall = True
		d += 1
		return "random"

def navigation(where_to_go):
	global velocity
	YAW_ERROR = random.uniform(.1, .2)
	YAW_VELOCITY = .8
	LINEAR_VELOCITY = .28
	OBJECT_NEARBY_DISTANCE = random.uniform(.35, .4)
	OBJECT_AHEAD_DISTANCE = .3
	OBJECT_CLOSE_CORNER = OBJECT_AHEAD_DISTANCE

	orientation_q = pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
	seconds = rospy.get_time()

	if min(laser.ranges) < OBJECT_NEARBY_DISTANCE:
		if (
				min(laser.ranges[0:30]) < OBJECT_AHEAD_DISTANCE or 
				min(laser.ranges[31:41]) < OBJECT_CLOSE_CORNER
			):
			velocity.linear.x = .0
			velocity.angular.z = -YAW_VELOCITY
		elif (
				min(laser.ranges[329:-1]) < OBJECT_AHEAD_DISTANCE or 
				min(laser.ranges[318:328]) < OBJECT_CLOSE_CORNER
			):
			velocity.linear.x = .0
			velocity.angular.z = YAW_VELOCITY
		else:
			velocity.linear.x = LINEAR_VELOCITY * .8
			velocity.angular.z = .0

	else:
		if where_to_go == "left":
			# turn CCW
			if math.pi - yaw > YAW_ERROR and yaw > 0:
				velocity.linear.x = .0
				velocity.angular.z = YAW_VELOCITY
			#turn CW
			elif math.pi + yaw > YAW_ERROR and yaw < 0:
				velocity.linear.x = .0
				velocity.angular.z = -YAW_VELOCITY
			else:
				velocity.linear.x = LINEAR_VELOCITY
				velocity.angular.z = .0


		elif where_to_go == "down":
			# turn CCW
			if abs(yaw + math.pi / 2) > YAW_ERROR and abs(yaw) > math.pi / 2: 
				velocity.linear.x = .0
				velocity.angular.z = YAW_VELOCITY
			# turn CW
			elif abs(yaw + math.pi / 2) > YAW_ERROR and abs(yaw) < math.pi / 2:
				velocity.linear.x = .0
				velocity.angular.z = -YAW_VELOCITY
			else:
				velocity.linear.x = LINEAR_VELOCITY
				velocity.angular.z = .0
				
		elif where_to_go == "right":
			# turn CCW
			if abs(yaw) > YAW_ERROR and yaw < 0:
				velocity.linear.x = .0
				velocity.angular.z = YAW_VELOCITY
			# turn CW
			elif abs(yaw) > YAW_ERROR and yaw > 0:
				velocity.linear.x = .0
				velocity.angular.z = -YAW_VELOCITY
			else:
				velocity.linear.x = LINEAR_VELOCITY
				velocity.angular.z = .0
				
		elif where_to_go == "up":
			if abs(yaw - math.pi / 2) > YAW_ERROR and abs(yaw) < math.pi / 2:
				velocity.linear.x = .0
				velocity.angular.z = YAW_VELOCITY
			elif abs(yaw - math.pi / 2) > YAW_ERROR and abs(yaw) > math.pi / 2:
				velocity.linear.x = .0
				velocity.angular.z = -YAW_VELOCITY
			else:
				velocity.linear.x = LINEAR_VELOCITY
				velocity.angular.z = .0
		
		elif where_to_go == "random":
			velocity.linear.x = random.uniform(LINEAR_VELOCITY*.5, LINEAR_VELOCITY)
			velocity.angular.z = random.uniform(-YAW_VELOCITY, YAW_VELOCITY)

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
		navigation(where_to_go)
		pub.publish(velocity)	
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
 
 
