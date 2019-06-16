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
list_target = []

Risk = [0 for i in range(15)]


def update(pos, info, M_list , T_list, T_note, T_old, Forbid, TarNew):
	global tarlist

	# info is a list of INFOM type meaning that multiple cases could be happened in a second
	#INFOM type:
	# [0] UAV fail,    [1] index of fail UAV
	# [0] new target, [1] target coord (x, y)  , [2] detected by UAV i (index of UAV)
	# [0] detect threat, [1] threat coord (x, y) , [2] detected by UAV i (index of UAV)
	#allocate different case
	
	for i in range(len(pos)):
		for j in range(len(M_list) - 1):
			if(len(M_list[j]) != 0):
				for k in range(len(M_list[j]) - 1):
					if(len(M_list[j][k]) != 0):
						if(M_list[j][k][0] in tarlist):
							M_list[j].pop(k)
					
					if(pos[i] == M_list[j][k][1][-1]):
						tarlist.append(M_list[j][k][0])
						print(tarlist, " -- tarlist")

	
	# ^^^^^^^^^^^^
	# nge-loop posisi UAV -> nge-loop UAV-nya -> cek UAV-nya udah mati belum -> nge-loop misi yang ada di dalam UAV -> cek masih punya misi atau ngga
	# kalo masih punya -> misinya udah pernah ada yang ambil atau belum (udah ada dalam array atau belum)
	# kalo udah ada dalam array, berarti gausah di ambil lagi, jadi misinya di pop dari mission list
	# cek posisi UAV == posisi misi
	# kalo sama, artinya misi udah diambil, jadi dimasukkin ke dalam array
	#----------------------------------------------------------------------------------------------------------------------------------------------------


	# for i in range(len(M_list)):
	# 	if(len(M_list[i]) != 0):
	# 		for j in range(M_list[i]):
	# 			if(len(M_list[i][j]) != 0):
	# 				if([M_list[i][j][0], M_list[i][j][1][-1]] not in list_target):
	# 					list_target.append([M_list[i][j][0], M_list[i][j][1][-1]])

	# for i in range(len(pos)):
	# 	for j in range(len(list_target)):
	# 		if(distance(pos[i], target[j]) < 30):


	# for i in range(len(pos) - 1):
	# 	for j in range(len(M_list) - 1):
	# 		if(len(M_list[j]) != 0):
	# 			for k in range(len(M_list[j]) - 1):
	# 				if(len(M_list[j][k]) != 0):
	# 					if(M_list[j][k][0] in tarlist):
	# 						M_list[j].pop(k)
					
	# 				if(pos[i] == M_list[j][k][1][-1]):
	# 					tarlist.append(M_list[j][k][0])
	# 					print(tarlist, " -- tarlist")
					


	# for m in range(len(M_list) - 1):
	#   for n in range(len(M_list[m]) - 1):
			
					

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
		# elif(i[0] == 'far'):
		# 	far_leader(pos, i, M_list)


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
	# Function to change UAV path in order to avoid collision
	# info isinya = ['collision danger', i = index UAV ke-i, s = index UAV ke-s]
	

	# if(len(M_list[info[1]]) != 0):
	# 	len_arr = len(M_list[info[1]][0][1])
	# 	mission = M_list[info[1]][0][1][-1]
	# 	if(len(M_list[info[2]])  != 0):
	# 		len_arr_2 = len(M_list[info[2]][0][1])
	# 		mission_2 = M_list[info[2]][0][1][-1]
	# 		if(mission == mission_2):
	# 			if(distance(pos[info[1]], M_list[info[1]][0][1][0]) < distance(pos[info[2]], M_list[info[1]][0][1][0])):
	# 				M_list[info[2]].pop(0)
	# 			else:
	# 				M_list[info[1]].pop(0)

	# 		if(pos[info[1]][0] > pos[info[2]][0]):
	# 			if(pos[info[1]][1] > pos[info[2]][1]):
	# 				M_list[info[1]][0][1] = [[pos[info[1]][0] + 30, pos[info[1]][1] + 30], mission]
	# 				# M_list[info[1]][0][1][-1] = mission
	# 			else:
	# 				M_list[info[1]][0][1] = [[pos[info[1]][0] + 30, pos[info[1]][1] - 30], mission]
	# 				# M_list[info[1]][0][1][-1] = mission
	# 		else:
	# 			if(pos[info[1]][1] > pos[info[2]][1]):
	# 				M_list[info[1]][0][1] = [[pos[info[1]][0] - 30, pos[info[1]][1] + 30], mission]
	# 				# M_list[info[1]][0][1][-1] = mission
	# 			else:
	# 				M_list[info[1]][0][1] = [[pos[info[1]][0] - 30, pos[info[1]][1] - 30], mission]
	# 				# M_list[info[1]][0][1][-1] = mission
	# 	else:

	# 		if(pos[info[1]][0] > pos[info[2]][0]):
	# 			if(pos[info[1]][1] > pos[info[2]][1]):
	# 				M_list[info[1]][0][1] = [[pos[info[2]][0] + 60, pos[info[2]][1] + 60], [pos[info[2]][0], pos[info[2]][1] - 60], [pos[info[2]][0] - 60, pos[info[2]][1] - 60], mission]
	# 				# M_list[info[1]][0][1][-1] = mission
	# 			else:
	# 				M_list[info[1]][0][1] = [[pos[info[2]][0] + 60, pos[info[2]][1] - 60], [pos[info[2]][0], pos[info[2]][1] + 60], [pos[info[2]][0] - 60, pos[info[2]][1] + 60], mission]
	# 				# M_list[info[1]][0][1][-1] = mission
	# 		else:
	# 			if(pos[info[1]][1] > pos[info[2]][1]):
	# 				M_list[info[1]][0][1] = [[pos[info[2]][0] - 60, pos[info[2]][1] + 60], [pos[info[2]][0], pos[info[2]][1] - 60], [pos[info[2]][0] + 60, pos[info[2]][1] - 60], mission]
	# 				# M_list[info[1]][0][1][-1] = mission
	# 			else:
	# 				M_list[info[1]][0][1] = [[pos[info[2]][0] - 60, pos[info[2]][1] - 60], [pos[info[2]][0], pos[info[2]][1] + 60], [pos[info[2]][0] + 60, pos[info[2]][1] + 60], mission]
	# 				# M_list[info[1]][0][1][-1] = mission
	# else:
	# 	if(pos[info[2]][0] > pos[info[1]][0]):
	# 		if(pos[info[2]][1] > pos[info[1]][1]):  
	# 			M_list[info[2]][0][1].insert(0, [pos[info[1]][0], pos[info[1]][1] + 30])
	# 		else:
	# 			M_list[info[2]][0][1].insert(0, [pos[info[1]][0], pos[info[1]][1] - 30])
	# 	else:
	# 		if(pos[info[2]][1] > pos[info[1]][1]):  
	# 			M_list[info[2]][0][1].insert(0, [pos[info[1]][0], pos[info[1]][1] + 30])
	# 		else:
	# 			M_list[info[2]][0][1].insert(0, [pos[info[1]][0], pos[info[1]][1] - 30])
	# 	return



	if(len(M_list[info[1]]) != 0): # cek UAV i udah mati atau belum
	  if(len(M_list[info[2]])  != 0): # cek UAV j udah mati atau belum
	      
	      if(M_list[info[1]][0] == M_list[info[2]][0]): # cek kalo i sama j misinya sama
	          if(distance(pos[info[1]], M_list[info[1]][0][1][0]) < distance(pos[info[2]], M_list[info[1]][0][1][0])): # yang ambil yang plaing deket
	              M_list[info[2]].pop(0)
	          else:
	              M_list[info[1]].pop(0)


	      if(pos[info[1]][0] > pos[info[2]][0]):  # cek koordinat X UAV i lebih besar dari UAV j
	          if(pos[info[1]][1] > pos[info[2]][1]):  # cek koordinat Y UAV i lebih besar dari UAV j
	              M_list[info[1]][0][1].insert(0, [pos[info[1]][0] + 80, pos[info[1]][1] + 80])
	          else: # i < j
	              M_list[info[1]][0][1].insert(0, [pos[info[1]][0] + 80, pos[info[1]][1] - 80])
	          return
	      else: # i > j
	          if(pos[info[1]][1] > pos[info[2]][1]):  
	              M_list[info[1]][0][1].insert(0, [pos[info[1]][0] - 80, pos[info[1]][1] + 80])
	          else:
	              M_list[info[1]][0][1].insert(0, [pos[info[1]][0] - 80, pos[info[1]][1] - 80])
	          return



	  else: # Kalo UAV j udah mati
	      if(distance(pos[info[1]], pos[info[2]]) < 20): # cek jarak UAV hidup dengan UAV mati
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

	else: # Kalo UAV i mati
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
	# Function to handle UAV that closer to forbid area (A)
	# info['collision danger', index UAV, index forbid, index forbid point] (A)

	f1 = Forbid[info[2]][info[3]]
	f2 = Forbid[info[2]][info[3] - 1]
	f3 = 0

	# f1 = the nearest forbid point, f2 & f3 = forbid points that in line with f1 (A)

	if(len(Forbid[info[2]]) > info[3]):
		f3 = Forbid[info[2]][info[3] + 1]
	else:
		f3 = Forbid[info[2]][0]

	forbid_list_x = [f1[0], f2[0], f3[0]]
	forbid_list_y = [f1[1], f2[1], f3[1]]

	if(pos[info[1]][0] > f1[0]): # Check if UAV's X coordinate is greater than the forbid point (A)
		if(pos[info[1]][1] > f1[1]): # Check if UAV's Y coordinate is greater than the forbid point (A)
			M_list[info[1]][0][1].insert(0, [f1[0] + 120, f1[1] + 80])
			M_list[info[1]][0][1].insert(1, [max(forbid_list_x) + 80, pos[info[1]][1]])
			M_list[info[1]][0][1].insert(2, [max(forbid_list_x) + 80, max(forbid_list_y) + 80])
		else:
			M_list[info[1]][0][1].insert(0, [f1[0] + 120, f1[1] - 80])
			M_list[info[1]][0][1].insert(1, [max(forbid_list_x) + 80, pos[info[1]][1]])
			M_list[info[1]][0][1].insert(2, [max(forbid_list_x) + 80, min(forbid_list_y) - 80])
	else:
		if(pos[info[1]][1] > f1[1]):
			M_list[info[1]][0][1].insert(0, [f1[0] - 120, f1[1] + 80])
			M_list[info[1]][0][1].insert(1, [min(forbid_list_x) - 80, pos[info[1]][1]])
			M_list[info[1]][0][1].insert(2, [min(forbid_list_x) - 80, max(forbid_list_y) + 80])
		else:
			M_list[info[1]][0][1].insert(0, [f1[0] - 120, f1[1] - 80])
			M_list[info[1]][0][1].insert(1, [min(forbid_list_x) - 80, pos[info[1]][1]])
			M_list[info[1]][0][1].insert(2, [min(forbid_list_x) - 80, min(forbid_list_y) - 80])

# def far_leader(pos, info, M_list):

# 	if(pos[info[1]][0] > pos[info[2]][0]):
# 		if(pos[info[1]][1] > pos[info[2]][1]):
# 			M_list[info[1]][0][1].insert(0, [pos[info[1]][0] - 100, pos[info[1]][1] - 100])
# 		else:
# 			M_list[info[1]][0][1].insert(0, [pos[info[1]][0] - 100, pos[info[1]][1] + 100])
# 	else:
# 		if(pos[info[1]][1] > pos[info[2]][1]):
# 			M_list[info[1]][0][1].insert(0, [pos[info[1]][0] + 100, pos[info[1]][1] - 100])
# 		else:
# 			M_list[info[1]][0][1].insert(0, [pos[info[1]][0] + 100, pos[info[1]][1] + 100])