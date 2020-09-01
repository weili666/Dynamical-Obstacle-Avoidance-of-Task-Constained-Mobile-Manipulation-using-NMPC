#!/usr/bin/python3
import sys
import time
import numpy as np 
import math
from Config import Config
import rospy
from std_msgs.msg import String
from mecanum_sim.srv import *
import sys
import roslaunch 


class RLMoMaBot(object):
	
	def __init__(self):
	
		self.Num_step=0
		sys.path.append('/home/jeffrey/catkin_ws/src/mecanum_sim/src')
		self.reset()


	def reset(self):

		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(uuid)
		tracking_launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/jeffrey/catkin_ws/src/mecanum_sim/launch/mecanum.launch"])
		##tracking_launch.shutdown()
		##tracking_launch.start()
		##tracking_launch2 = roslaunch.parent.ROSLaunchParent(uuid, ["/home/jeffrey/catkin_ws/src/mecanum_moveit_config/launch/mecanum_moveit_planning_execution.launch sim:=true"])
		##tracking_launch2.shutdown()
		##tracking_launch2.start()
		##tracking_launch3 = roslaunch.parent.ROSLaunchParent(uuid, ["/home/jeffrey/catkin_ws/src/mecanum_moveit_config/launch/moveit_rviz.launch config:=true"])
		##tracking_launch3.shutdown()
		##tracking_launch3.start()


	def step(self, action):
		##collisionHandle = vrep.simxGetCollisionHandle(self.client_id,'Omnirob',vrep.simx_opmode_blocking)
		##cH = collisionHandle[0]
		##print("cH is :",cH)

		rospy.wait_for_service('step_drl')
		state = []
		o = []
		p = []
		reward = 0
		
		step_clint = rospy.ServiceProxy('step_drl', DRLStep)
		resp = step_clint(action)
		state = resp.state
		##print("state :",state)
		o = resp.object
		##print("object :",o)
		p = resp.position
		##print("position :",p)
		reward = resp.reward
		##print("reward :",reward)
		
		return state, o, p, reward
