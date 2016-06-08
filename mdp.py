# mdp implementation needs to go here

import rospy
import math
import ast
from std_msgs.msg import String, Float32, Bool #, PolicyList, AStarPath
from read_config import read_config
from astar import a_star
from cse_190_assi_3.msg import *

def m_d_p(move_list, map_size, start, goal, walls, pits, maxIterations, threshHoldDifference,
	      rewardForMove, rewardForFalling, rewardForReachingGoal, rewardForHittingWall, discountFactor,
	      probRight, probLeft, probForward, probBackward, publisher, exit, exitValue):
	#initialization
	table_h = map_size[0]
	table_w = map_size[1]
	refTable = {}
	updating = {}
	moves = {}

	rewardForFalling = rewardForFalling*10 #10 - 15

	for w in range(table_w):
		for h in range(table_h):
			refTable[h,w] = 0
			updating[h,w] = 0
			moves[h,w] = "N"
			#check if goal
			if (h == goal[0] and w == goal[1]):
				refTable[h,w] = rewardForReachingGoal
				moves[h,w] = "GOAL"
			if ( h == exit[0] and w == exit[1]):
				refTable[h,w] = exitValue
				moves[h,w] = "GOAL"
			for wall in walls:
				wall_h = wall[0]
				wall_w = wall[1]
				if (wall_h == h and wall_w == w):
					moves[h,w] = "WALL"
					refTable[h,w] = rewardForHittingWall
			for pit in pits:
				pit_h = pit[0]
				pit_w = pit[1]
				if (pit_h == h and pit_w == w):
					moves[h,w] = "PIT"
					refTable[h, w] = rewardForFalling

	updating = refTable

	w = 0
	h = 0
	prevSum = 0
	currSum = 0
	count = 0
	#the main part
	for i in range(maxIterations):
		for w in range(0,table_w):
			for h in range(0,table_h):
				if(isWPG(h, w, walls, pits, goal, exit)):
					continue
				#checking for up
				moveCostUp = rewardForMove
				moveCostDown = rewardForMove
				moveCostLeft = rewardForMove
				moveCostRight = rewardForMove
				if(h == 0):
					upMove = rewardForHittingWall
					moveCostUp = 0
				elif(isWall(h-1, w, walls)):
					upMove = rewardForHittingWall
					moveCostUp = 0
				elif(isPit(h-1, w, pits)):
					upMove = rewardForFalling
				elif(isGoal(h-1, w, goal, exit)):
					upMove = rewardForReachingGoal
				else:
					upMove = refTable[h-1, w]

				#checking for down
				if(h == table_h-1):
					downMove = rewardForHittingWall
					moveCostDown = 0
				elif(isWall(h+1, w, walls)):
					downMove = rewardForHittingWall
					moveCostDown = 0
				elif(isPit(h+1, w, pits)):
					downMove = rewardForFalling
				elif(isGoal(h+1, w, goal, exit)):
					downMove = rewardForReachingGoal
				else:
					downMove = refTable[h+1, w]

				#checking for right
				if(w == table_w-1):
					rightMove = rewardForHittingWall
					moveCostRight = 0
				elif(isWall(h, w+1, walls)):
					rightMove = rewardForHittingWall
					moveCostRight = 0
				elif(isPit(h, w+1, pits)):
					rightMove = rewardForFalling
				elif(isGoal(h, w+1, goal, exit)):
					rightMove = rewardForReachingGoal
				else:
					rightMove = refTable[h, w+1]

				#checking for left
				if(w == 0):
					leftMove = rewardForHittingWall
					moveCostLeft = 0
				elif(isWall(h, w-1, walls)):
					leftMove = rewardForHittingWall
					moveCostLeft = 0
				elif(isPit(h, w-1, pits)):
					leftMove = rewardForFalling
				elif(isGoal(h, w-1, goal, exit)):
					leftMove = rewardForReachingGoal
				else:
					leftMove = refTable[h, w-1]

				#CALCULATIONS
				
				#UP
				#TODO: WHAT IS DISCOUNT FACTOR WHEN YOU HIT WALL
				tempUp = (probForward*(moveCostUp + (discountFactor*upMove)))
				tempDown = probBackward*(moveCostDown + (discountFactor*downMove))
				tempLeft = probLeft * (moveCostLeft + (discountFactor*leftMove))
				tempRight = probRight * (moveCostRight + (discountFactor*rightMove))
				sumUp = tempUp + tempDown + tempLeft + tempRight

				#DOWN
				tempUp = (probBackward*(moveCostUp + (discountFactor*upMove)))
				tempDown = probForward*(moveCostDown + (discountFactor*downMove))
				tempLeft = probRight * (moveCostLeft + (discountFactor*leftMove))
				tempRight = probLeft * (moveCostRight + (discountFactor*rightMove))
				sumDown = tempUp + tempDown + tempLeft + tempRight

				#LEFT
				tempUp = (probRight*(moveCostUp + (discountFactor*upMove)))
				tempDown = probLeft*(moveCostDown + (discountFactor*downMove))
				tempLeft = probForward * (moveCostLeft + (discountFactor*leftMove))
				tempRight = probBackward * (moveCostRight + (discountFactor*rightMove))
				sumLeft = tempUp + tempDown + tempLeft + tempRight

				#RIGHT
				tempUp = (probLeft*(moveCostUp + (discountFactor*upMove)))
				tempDown = probRight*(moveCostDown + (discountFactor*downMove))
				tempLeft = probBackward * (moveCostLeft + (discountFactor*leftMove))
				tempRight = probForward * (moveCostRight + (discountFactor*rightMove))
				sumRight = tempUp + tempDown + tempLeft + tempRight

				maxSum = 0
				move = "N"
				
				if sumUp > sumDown and sumUp > sumLeft and sumUp > sumRight:
					maxSum = sumUp
					move = "N"
				elif sumDown > sumUp and sumDown > sumLeft and sumDown > sumRight:
					maxSum = sumDown
					move = "S"
				elif sumLeft > sumUp and sumLeft > sumDown and sumLeft > sumRight:
					maxSum = sumLeft
					move = "W"
				elif sumRight > sumUp and sumRight > sumDown and sumRight > sumLeft:
					maxSum = sumRight
					move = "E"
				

				'''
				if sumUp > maxSum:
					maxSum = sumUp
					move = "N"
				if sumDown > maxSum:
					move = "S"
					maxSum = sumDown
				if sumLeft > maxSum:
					move = "W"
					maxSum = sumLeft
				if sumRight > maxSum:
					maxSum = sumRight
					move = "E"
				'''

				updating[h,w] = maxSum
				moves[h,w] = move
				currSum += abs(maxSum)

		refTable = updating
		if ( abs(prevSum - currSum) < threshHoldDifference ):
			publishIter(publisher, moves, map_size)
			return moves
		else:

			count += 1
			prevSum = currSum
			currSum = 0
		#print moves
		publishIter( publisher, moves, map_size)
		rospy.sleep(2)
	return moves

def isWall(h, w, walls):
	for wall in walls:
		if (h == wall[0] and w == wall[1]):
			return True
	return False

def isPit(h, w, pits):
	for pit in pits:
		if (h == pit[0] and w == pit[1]):
			return True

	return False


def isGoal(h, w, goal, exit):
	if (h == goal[0] and w == goal[1]):
		return True
	if (h == exit[0] and w == exit[0]):
		return True
	return False

def isWPG(h, w, walls, pits, goal, exit):
	if isWall(h,w,walls) or isPit(h,w,pits) or isGoal(h,w,goal, exit):
		return True
	return False

def publishIter( publisher, mapToPub, map_size ):
	#print "hello we are in publish"
	toPub = []
	table_h = map_size[0]
	table_w = map_size[1]
	for h in range(table_h):
		for w in range(table_w):
			toPub.append(mapToPub[h,w])
	publisher.publish(toPub)
	#rospy.sleep(2)
