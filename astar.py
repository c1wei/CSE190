from heapq import heappush, heappop

# astar implementation needs to go here
def a_star(move_list, map_size, start, goal, walls, pits):
	#print "we are in a different file A star!!!"
	#print start

	closedSet = [] #this LIST does not need to be ordered

	openSet = [] #this will be a heapqueue, filled with points and values

	cameFrom = {} #storing where the nodes came from

	gScore = {} 
	fScore = {} 

	#setting up the initial table
	table_h = map_size[0]
	table_w = map_size[1]
	table = {}
	
	#table filled with manDist
	for w in range(table_w):
		for h in range(table_h):
			table[h,w] = manToGoal(h, w, goal)
			cameFrom[str([h,w])] = False
			gScore[h,w] = float("inf")
	#print table
	gScore[start[0],start[1]] = manToGoal( start[0], start[1], goal)
	#print gScore[start[0],start[1]] 

	heappush( openSet, ([start[0],start[1]], gScore[start[0],start[1]]) )
		
	#print heappop(openSet)
	moves = 0

	while openSet:
		current = heappop(openSet)
		moves += 1
		if current[0] == goal:
			#print "goal has been reached"
			#print moves
			toReturn = traceBack(current[0], cameFrom)
			return toReturn
		closedSet.append(current[0])
		for i in range(len(move_list)):
			#for right move
			if move_list[i] == [0,1]:
				#print "this is move right"
				r_neighbor  = getNeighbor(current, move_list[i], map_size, walls, pits)
				#print r_neighbor
				if( r_neighbor == [-1,-1] or r_neighbor == [-2,-2] or r_neighbor == [-3,-3]):
					#print "we are continuing because it is a bad thing" + str(r_neighbor)
					continue
				if( r_neighbor in  closedSet ):
					#print "in closedSet"
					continue 
				tentative_gScore = gScore[current[0][0],current[0][1]] + 1
				#print "this is tentative gscore "  + str(tentative_gScore)	
				if( tentative_gScore >= gScore[r_neighbor[0],r_neighbor[1]]):
					continue
				#calculations
				cameFrom[str(r_neighbor)] = str(current[0])	
				gScore[r_neighbor[0],r_neighbor[1]] = tentative_gScore
				fScore[r_neighbor[0],r_neighbor[1]] = tentative_gScore + manToGoal(r_neighbor[0], r_neighbor[1], goal)
				#print "this is man " + str(manToGoal(r_neighbor[0], r_neighbor[1], goal))
				#print "this is fscore " + str(fScore[r_neighbor[0],r_neighbor[1]] )
				#might cause problems later having it after everything
				if( r_neighbor not in openSet):
					heappush(openSet, ([r_neighbor[0],r_neighbor[1]], fScore[r_neighbor[0],r_neighbor[1]]))

			#for left move
			if move_list[i] == [0,-1]:
				#print "this is move left"
				l_neighbor  = getNeighbor(current, move_list[i], map_size, walls, pits)
				#print l_neighbor
				if( l_neighbor == [-1,-1] or l_neighbor == [-2,-2] or l_neighbor == [-3,-3]):
					#print "we are continuing because it is a bad thing" + str(r_neighbor)
					continue
				if( l_neighbor in  closedSet ):
					#print "in closedSet"
					continue 
				tentative_gScore = gScore[current[0][0],current[0][1]] + 1
				#print "this is tentative gscore "  + str(tentative_gScore)	
				if( tentative_gScore >= gScore[l_neighbor[0],l_neighbor[1]]):
					continue
				#calculations
				cameFrom[str(l_neighbor)] = str(current[0])		
				gScore[l_neighbor[0],l_neighbor[1]] = tentative_gScore
				fScore[l_neighbor[0],l_neighbor[1]] = tentative_gScore + manToGoal(l_neighbor[0], l_neighbor[1], goal)
				#print "this is man " + str(manToGoal(r_neighbor[0], r_neighbor[1], goal))
				#print "this is fscore " + str(fScore[r_neighbor[0],r_neighbor[1]] )
				#might cause problems later having it after everything
				if( l_neighbor not in openSet):
					heappush(openSet, ([l_neighbor[0],l_neighbor[1]], fScore[l_neighbor[0],l_neighbor[1]]))

			#for down move
			if move_list[i] == [1,0]:
				#print "this is move down"
				d_neighbor  = getNeighbor(current, move_list[i], map_size, walls, pits)
				#print d_neighbor
				if( d_neighbor == [-1,-1] or d_neighbor == [-2,-2] or d_neighbor == [-3,-3]):
					#print "we are continuing because it is a bad thing" + str(r_neighbor)
					continue
				if( d_neighbor in  closedSet ):
					#print "in closedSet"
					continue 
				tentative_gScore = gScore[current[0][0],current[0][1]] + 1
				#print "this is tentative gscore "  + str(tentative_gScore)	
				if( tentative_gScore >= gScore[d_neighbor[0],d_neighbor[1]]):
					continue
				#calculations
				cameFrom[str(d_neighbor)] = str(current[0])	
				gScore[d_neighbor[0],d_neighbor[1]] = tentative_gScore
				fScore[d_neighbor[0],d_neighbor[1]] = tentative_gScore + manToGoal(d_neighbor[0], d_neighbor[1], goal)
				#print "this is man " + str(manToGoal(r_neighbor[0], r_neighbor[1], goal))
				#print "this is fscore " + str(fScore[r_neighbor[0],r_neighbor[1]] )
				#might cause problems later having it after everything
				if( d_neighbor not in openSet):
					heappush(openSet, ([d_neighbor[0],d_neighbor[1]], fScore[d_neighbor[0],d_neighbor[1]]))

			#for up move
			if move_list[i] == [-1,0]:
				#print "this is move up"
				u_neighbor  = getNeighbor(current, move_list[i], map_size, walls, pits)
				#print u_neighbor
				if( u_neighbor == [-1,-1] or u_neighbor == [-2,-2] or u_neighbor == [-3,-3]):
					#print "we are continuing because it is a bad thing" + str(r_neighbor)
					continue
				if( u_neighbor in  closedSet ):
					#print "in closedSet"
					continue 
				tentative_gScore = gScore[current[0][0],current[0][1]] + 1
				#print "this is tentative gscore "  + str(tentative_gScore)	
				if( tentative_gScore >= gScore[u_neighbor[0],u_neighbor[1]]):
					continue
				#calculations
				cameFrom[str(u_neighbor)] = str(current[0])	
				gScore[u_neighbor[0],u_neighbor[1]] = tentative_gScore
				fScore[u_neighbor[0],u_neighbor[1]] = tentative_gScore + manToGoal(u_neighbor[0], u_neighbor[1], goal)
				#print "this is man " + str(manToGoal(r_neighbor[0], r_neighbor[1], goal))
				#print "this is fscore " + str(fScore[r_neighbor[0],r_neighbor[1]] )
				#might cause problems later having it after everything
				if( u_neighbor not in openSet):
					heappush(openSet, ([u_neighbor[0],u_neighbor[1]], fScore[u_neighbor[0],u_neighbor[1]]))

'''	for i in range(table_h):
		table.append([])

	for w in range(table_w):
		for h in range(table_h):
			table[i].append("(" + str(h) + ", " + str(w) + ")")
'''

def manToGoal( h, w, goal ):
	heur = abs(h-goal[0]) + abs(w-goal[1])
	return heur	

def getNeighbor( current, move, mapsize, walls, pits ):
	h = current[0][0] + move[0]
	w = current[0][1] + move[1]
	neighbor = [h,w]
	for i in range(len(walls)):
		wall_h = walls[i][0]
		wall_w = walls[i][1]
		if h == wall_h and w == wall_w:
			return [-1,-1]

	for i in range(len(pits)):
		pit_h = pits[i][0]
		pit_w = pits[i][1]
		if h == pit_h and w == pit_w:
			return [-3,-3] #going into pit is [-3,-3]


	if (h < 0 or w < 0) or (h >= mapsize[0] or w >= mapsize[1]):
		return [-2,-2] #out of bounds hitting wall is [-2, -2]
	else:
		return neighbor 


def traceBack( current , cameFrom):
	#print "this is traceback"
	path = []
	#print current
	#print cameFrom
	while str(current) in cameFrom.keys():
		#print current
		if( current != False ):
			path.append(str(current))
		current = cameFrom[str(current)]
		
	return path
