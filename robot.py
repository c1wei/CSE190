#!/usr/bin/env python
#robot.py implementation goes here

import rospy
import math
import ast
from std_msgs.msg import String, Float32, Bool #, PolicyList, AStarPath
from read_config import read_config
from astar import a_star
from mdp import m_d_p
from cse_190_assi_3.msg import *

class robot:
	def __init__(self):
		rospy.init_node('robot', anonymous = True)
		print "hello world!"
		self.config = read_config()
		self.mdp_pub = rospy.Publisher('/results/policy_list', PolicyList, queue_size = 10)		
		self.a_pub = rospy.Publisher('/results/path_list', AStarPath, queue_size = 100)
		self.shut_down = rospy.Publisher('/map_node/sim_complete', Bool, queue_size = 10)
		rospy.sleep(10)
		#parameter setup
		self.move_list = self.config["move_list"]
		self.map_size = self.config["map_size"]
		self.start = self.config["start"]
		self.goal = self.config["person_goal"]
		self.walls = self.config["walls"]
		self.pits = self.config["pits"]
		self.max_iterations = self.config["max_iterations"]
		self.threshHoldDifference = self.config["threshold_difference"]
		self.exit = self.config["exit_goal"]
		self.exitVal = self.config["reward_for_reaching_exit"]
		#rewards
		self.rewardForHittingWall = self.config["reward_for_hitting_wall"]
		self.rewardForReachingGoal = self.config["reward_for_reaching_goal"]
		self.rewardForFalling = self.config["reward_for_falling_in_pit"]
		self.rewardForMove = self.config["reward_for_each_step"]
		self.discountFactor = self.config["discount_factor"]
		self.probForward = self.config["prob_move_forward"]
		self.probBackward = self.config["prob_move_backward"]
		self.probLeft = self.config["prob_move_left"]
		self.probRight = self.config["prob_move_right"]


		#calling A*
		self.callingAStar()
		self.callingMDP()
		rospy.sleep(10)
		#calling shutdown function
		self.shutDown()


	def callingAStar(self):
		a_star_path = a_star( self.move_list,
			self.map_size,
			self.start,
			self.goal,
			self.walls,
			self.pits)
		#print "a_star_path"
		toPub = []
		for entry in reversed(a_star_path):
			toPub.append(ast.literal_eval(entry))
			step = ast.literal_eval(entry)
			#print step
			self.a_pub.publish(step)


	def callingMDP(self):
		#for iteration in range(max_iterations):
			mdpPath = m_d_p( self.move_list,
				self.map_size,
				self.start,
				self.goal,
				self.walls,
				self.pits,
				self.max_iterations,
				self.threshHoldDifference,
				self.rewardForMove,
				self.rewardForFalling,
				self.rewardForReachingGoal,
				self.rewardForHittingWall,
				self.discountFactor,
				self.probRight,
				self.probLeft,
				self.probForward,
				self.probBackward,
				self.mdp_pub,
				self.exit,
				self.exitVal)

			toPub = []
			table_h = self.map_size[0]
			table_w = self.map_size[1]
			for h in range(table_h):
				for w in range(table_w):
					toPub.append(mdpPath[h,w])
			print toPub

			#self.mdp_pub.publish(toPub)
			rospy.sleep(2)


	#shuttind machine down, could change sleep if you want
	def shutDown(self):
		print "we are in shutdown"
		self.shut_down.publish(True)
		rospy.sleep(1)
		rospy.signal_shutdown(1)

if __name__ == '__main__':
	r = robot()
