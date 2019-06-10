from mapmath import *
import map


# A basic Mission_list structure is [ (targer.x, targer.y), [(p1x,p1y),(p2x,p2y)...]]
# each UAV contain a few Mission_list
# and M_list store all of 15 UAVs' Mission_lists
#M_list = None
T_list = None # include original T_note and T_old
# Start with the same position as it in T_list and update when detect threat move
T_note = None
# The not-moving threat
T_old = None
# every Forbid[i] is the vertex set of the Forbid area
Forbid = None
# It'a
TarNew = None

tarlist = []

Risk = [0 for i in range(15)]


def update(pos, info, M_list , T_list, T_note, T_old, Forbid, TarNew):
	global tarlist

	# info is a list of INFOM type meaning that multiple cases could be happened in a second
	#INFOM type:
	# [0] UAV fail,    [1] index of fail UAV
	# [0] new target, [1] target coord (x, y)  , [2] detected by UAV i (index of UAV)
	# [0] detect threat, [1] threat coord (x, y) , [2] detected by UAV i (index of UAV)
	#allocate different case
	for i in range(len(pos) - 1):
		for j in range(len(M_list) - 1):
			if(len(M_list[j]) != 0):
				for k in range(len(M_list[j]) - 1):
					if(len(M_list[j][k]) != 0):
						if(M_list[j][k][0] in tarlist):
							M_list[j].pop(k)
					
					if(pos[i] == M_list[j][k][1][-1]):
						tarlist.append(M_list[j][k][0])
						print(tarlist, " -- tarlist")
					


	# for m in range(len(M_list) - 1):
	# 	for n in range(len(M_list[m]) - 1):
			
					

	for i in info:
		if(i[0] == 'new target'):
			contractnet_negotiation('new target', i, M_list, TarNew)
		elif(i[0] == 'fail'):
			contractnet_negotiation('fail', i, M_list, TarNew)
			TarNew.clear()
		elif(i[0] == 'detect threat'):
			path_replanning(pos, i,M_list, T_list)
		elif(i[0] == 'collision danger'):
			path_change(pos, i, M_list)
		elif(i[0] == 'forbid area'):
			forbid_handle(pos, i, M_list, Forbid)


def contractnet_negotiation(case,info,M_list, TarNew):
	# example1: If UAV i fail turn its mission to the leader
	if(case == 'fail'):
		return
#        leader = int(info[1]/5)*5
#        # make leader took over the mission when original mission had all done
#        mission_of_leader = [M_list[leader][i][0] for i in range(len(M_list[leader]))]
#        for T in TarNew:
#            if(T[0] not in mission_of_leader):
#                M_list[leader].append(T) # mission woulbe be put into TarNew when UAV fail
	# example2: new target
	# allocate new mission to UAV 0
	elif(case == 'new target'):
		if len(TarNew) == 0:
			return
		new_mission = TarNew[0]
		# append last end point
		#M_list[info[2]].insert(-1, new_mission)# new target would also be mlist_struct with [[xnewtar, xnewtar], [[[xnewtar, xnewtar]]],...]
		M_list[info[2]].insert(0, new_mission)
		TarNew.pop()



def path_replanning(pos ,info,M_list, T_list):
	# return
	
	thr = info[1]
	# the UAV(info[2]) now([0]) path[1]
	nowpath = M_list[info[2]][0][1]
	# It's based on calculation time
	# Since we allow at most 1 seconde calculation time in every step of update(relate to one second flying time)
	# This is a templete to check wheather we are still in the same situation when calculation has done.
	ifmuta = M_list[info[2]][0][1][0].copy() # first point of nowpath , or can check M_list[info[2]][0][0], if we still in the same mission
	# example 3 new threat
	# make a function call to the built function to test if still can afford the risk
	plus_risk = mapmath.path_risk(T_list, [pos[info[2]],  nowpath[0]])
	# if the path would not pass the threat range
	if (plus_risk == 0):
	   return
	# else if risk still in tollerance range
	elif(Risk[info[2]] + plus_risk < 60): # 60 is the half of the tollerance risk
	   Risk[info[2]]  += plus_risk
	   return

	# or to insert a new point to let UAV move around the threat

	else:
	   #orth = (nowpath[0][1] - pos[info[2]][1] , nowpath[0][0] - pos[info[2]][0])
	   #move_around = [thr[0] + thr[2]*orth[0], thr[1] + thr[2]*orth[1]]
	   move_around = [thr[1], thr[0]]
	   # to check if M_list has been revised
	   if ifmuta == M_list[info[2]][0][1][0]:
		   # change all in the team'
		   nowpath.insert(0, pos[info[2]]) # stop at the current pos
		   nowpath.insert(1, move_around) # move away from the  threat
		   return


def path_change(pos, info, M_list):

	if(len(M_list[info[1]]) != 0):
		if(len(M_list[info[2]])  != 0):
			if(M_list[info[1]][0] == M_list[info[2]][0]):
				if(distance(pos[info[1]], M_list[info[1]][0][1][0]) < distance(pos[info[2]], M_list[info[1]][0][1][0])):
					M_list[info[2]].pop(0)
				else:
					M_list[info[1]].pop(0)
			if(pos[info[1]][0] > pos[info[2]][0]):
				if(pos[info[1]][1] > pos[info[2]][1]):	
					M_list[info[1]][0][1].insert(0, [pos[info[1]][0] + 30, pos[info[1]][1] + 30])
				else:
					M_list[info[1]][0][1].insert(0, [pos[info[1]][0] + 30, pos[info[1]][1] - 30])
				return
			else:
				if(pos[info[1]][1] > pos[info[2]][1]):	
					M_list[info[1]][0][1].insert(0, [pos[info[1]][0] - 30, pos[info[1]][1] + 30])
				else:
					M_list[info[1]][0][1].insert(0, [pos[info[1]][0] - 30, pos[info[1]][1] - 30])
				return
		else:
			if(distance(pos[info[1]], pos[info[2]]) < 20):
				if(pos[info[1]][0] > pos[info[2]][0]):
					if(pos[info[1]][1] > pos[info[2]][1]):	
						M_list[info[1]][0][1].insert(0, [pos[info[1]][0], pos[info[2]][1] + 50])
						M_list[info[1]][0][1].insert(1, [pos[info[1]][0] - 50, pos[info[2]][1] + 50])
						M_list[info[1]][0][1].insert(2, [pos[info[1]][0] - 50, pos[info[2]][1]])
						M_list[info[1]][0][1].insert(3, [pos[info[1]][0] - 50, pos[info[2]][1] - 50])
						M_list[info[1]][0][1].insert(4, [pos[info[1]][0], pos[info[2]][1] - 50])

					else:
						M_list[info[1]][0][1].insert(0, [pos[info[1]][0], pos[info[2]][1] - 50])
						M_list[info[1]][0][1].insert(1, [pos[info[1]][0] - 50, pos[info[2]][1] - 50])
						M_list[info[1]][0][1].insert(2, [pos[info[1]][0] - 50, pos[info[2]][1]])
						M_list[info[1]][0][1].insert(3, [pos[info[1]][0] - 50, pos[info[2]][1] + 50])
						M_list[info[1]][0][1].insert(4, [pos[info[1]][0], pos[info[2]][1] + 50])
					return
				else:
					if(pos[info[1]][1] > pos[info[2]][1]):	
						M_list[info[1]][0][1].insert(0, [pos[info[1]][0], pos[info[2]][1] + 50])
						M_list[info[1]][0][1].insert(1, [pos[info[1]][0] + 50, pos[info[2]][1] + 50])
						M_list[info[1]][0][1].insert(2, [pos[info[1]][0] + 50, pos[info[2]][1]])
						M_list[info[1]][0][1].insert(3, [pos[info[1]][0] + 50, pos[info[2]][1] - 50])
						M_list[info[1]][0][1].insert(4, [pos[info[1]][0], pos[info[2]][1] - 50])
					else:
						M_list[info[1]][0][1].insert(0, [pos[info[1]][0], pos[info[2]][1] + 50])
						M_list[info[1]][0][1].insert(1, [pos[info[1]][0] - 50, pos[info[2]][1] + 50])
						M_list[info[1]][0][1].insert(2, [pos[info[1]][0] - 50, pos[info[2]][1]])
						M_list[info[1]][0][1].insert(3, [pos[info[1]][0] - 50, pos[info[2]][1] + 50])
						M_list[info[1]][0][1].insert(4, [pos[info[1]][0], pos[info[2]][1] + 50])
					return

	else:
		if(pos[info[2]][0] > pos[info[1]][0]):
			if(pos[info[2]][1] > pos[info[1]][1]):	
				M_list[info[2]][0][1].insert(0, [pos[info[1]][0], pos[info[1]][1] + 30])
			else:
				M_list[info[2]][0][1].insert(0, [pos[info[1]][0], pos[info[1]][1] - 30])
		else:
			if(pos[info[2]][1] > pos[info[1]][1]):	
				M_list[info[2]][0][1].insert(0, [pos[info[1]][0], pos[info[1]][1] + 30])
			else:
				M_list[info[2]][0][1].insert(0, [pos[info[1]][0], pos[info[1]][1] - 30])
		return

def forbid_handle(pos, info, M_list, Forbid):
	# info['collision danger', index UAV, index forbid, index forbid point]

	f1 = Forbid[info[2]][info[3]]
	f2 = Forbid[info[2]][info[3] - 1]
	f3 = 0

	if(len(Forbid[info[2]]) > info[3]):
		f3 = Forbid[info[2]][info[3] + 1]
	else:
		f3 = Forbid[info[2]][0]

	forbid_list_x = [f1[0], f2[0], f3[0]]
	forbid_list_y = [f1[1], f2[1], f3[1]]

	if(pos[info[1]][0] > f1[0]):
		if(pos[info[1]][1] > f1[1]):
			M_list[info[1]][0][1].insert(0, [f1[0] + 120, f1[1] + 120])
			M_list[info[1]][0][1].insert(1, [max(forbid_list_x) + 120, pos[info[1]][1]])
			M_list[info[1]][0][1].insert(2, [max(forbid_list_x) + 120, max(forbid_list_y) + 120])
		else:
			M_list[info[1]][0][1].insert(0, [f1[0] + 120, f1[1] - 120])
			M_list[info[1]][0][1].insert(1, [max(forbid_list_x) + 120, pos[info[1]][1]])
			M_list[info[1]][0][1].insert(2, [max(forbid_list_x) + 120, min(forbid_list_y) - 120])
	else:
		if(pos[info[1]][1] > f1[1]):
			M_list[info[1]][0][1].insert(0, [f1[0] - 120, f1[1] + 120])
			M_list[info[1]][0][1].insert(1, [min(forbid_list_x) - 120, pos[info[1]][1]])
			M_list[info[1]][0][1].insert(2, [min(forbid_list_x) - 120, max(forbid_list_y) + 120])
		else:
			M_list[info[1]][0][1].insert(0, [f1[0] - 120, f1[1] - 120])
			M_list[info[1]][0][1].insert(1, [min(forbid_list_x) - 120, pos[info[1]][1]])
			M_list[info[1]][0][1].insert(2, [min(forbid_list_x) - 120, min(forbid_list_y) - 120])






	# if(pos[info[1]][0] > f1[0]):
	# 	if(pos[info[1]][1] > Forbid[info[2]][info[3]]):
	# 		M_list[info[1]][0][1].insert(0, [Forbid[info[2][info[3] - 1]][0] + 20, Forbid[info[2]][info[3] - 1] + 20])
	# 	else:
	# 		M_list[info[1]][0][1].insert(0, [Forbid[info[2][info[3] - 1]][0] + 20, Forbid[info[2]][info[3] - 1] - 20])
	# else:
	# 	if(pos[info[1]][1] > Forbid[info[2]][info[3]]):
	# 		M_list[info[1]][0][1].insert(0, [Forbid[info[2][info[3] - 1]][0] - 20, Forbid[info[2]][info[3] - 1] + 20])
	# 	else:
	# 		M_list[info[1]][0][1].insert(0, [Forbid[info[2][info[3] - 1]][0] - 20, Forbid[info[2]][info[3] - 1] - 20])