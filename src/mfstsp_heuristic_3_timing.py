import sys
import time
import datetime
import math
from parseCSV import *
from gurobipy import *
from heapq import *
from collections import defaultdict
import operator
import itertools
import endurance_calculator
import copy

NODE_TYPE_DEPOT		= 0
NODE_TYPE_CUST		= 1

TYPE_TRUCK 			= 1
TYPE_UAV 			= 2

TRAVEL_UAV_PACKAGE		= 1
TRAVEL_UAV_EMPTY		= 2
TRAVEL_TRUCK_W_UAV		= 3
TRAVEL_TRUCK_EMPTY		= 4

VERTICAL_UAV_EMPTY		= 5
VERTICAL_UAV_PACKAGE	= 6

STATIONARY_UAV_EMPTY	= 7
STATIONARY_UAV_PACKAGE	= 8
STATIONARY_TRUCK_W_UAV	= 9
STATIONARY_TRUCK_EMPTY	= 10

GANTT_IDLE		= 1
GANTT_TRAVEL	= 2
GANTT_DELIVER	= 3
GANTT_RECOVER	= 4
GANTT_LAUNCH	= 5
GANTT_FINISHED	= 6

packageIcons		= ['box_yellow_centered.gltf', 'box_blue_centered.gltf', 'box_orange_centered.gltf', 'box_green_centered.gltf', 'box_gray_centered.gltf', 'box_brown_centered.gltf']

def make_dict():
	return defaultdict(make_dict)

def mfstsp_heuristic_3_timing(x, y, z, node, inieee, N, P, V, c, sigma, sigmaprime, tau, iniTauprimeE, iniTauprimeF, minDistance, sR, sL, vehicle, travel, optLowBnd, iniSpeedE, iniSpeedF, P3Type, UAVSpeedType):

	B = defaultdict(make_dict)
	A = defaultdict(make_dict)
	AB = defaultdict(make_dict)
	ABC = defaultdict(make_dict)
	for [v,i,j,k] in y:
		B[v][k] = j
		A[v][i] = j
		AB[v][k] = i
		ABC[v][i] = k

	C = []
	for [v,i,j,k] in y:
		if (j not in C):
			C.append(j)

	launchesfrom = {}
	landsat = {}
	for i in N:
		launchesfrom[i] = []
		landsat[i] = []
	for [v,i,j,k] in y:
		launchesfrom[i].append(v)
		landsat[k].append(v)

	UAVtau = defaultdict(make_dict)
	for [v,i,j,k] in y:
		UAVtau[v][i][j][k] = iniTauprimeF[v][i][j] + sigmaprime[j] + iniTauprimeE[v][j][k]

	availVehicles = {}
	TSPtour = []
	for [i,k] in x:
		TSPtour.append(i)
		availVehicles[i] = list(V)
		availVehicles[i].append(1)

	availVehicles[c+1] = list(V)
	availVehicles[c+1].append(1)

	TSPtour.append(c+1)

	for [v,i,j,k] in y:
		tempp = TSPtour.index(i)
		tempq = TSPtour.index(k)

		for tempindex in range(tempp+1, tempq+1):
			availVehicles[TSPtour[tempindex]].remove(v)

	decvarchecktprime = defaultdict(make_dict)
	decvarhattprime = defaultdict(make_dict)
	decvarbart = defaultdict(make_dict)
	decvarhatt = defaultdict(make_dict)
	decvarcheckt = defaultdict(make_dict)

	ArrTime = defaultdict(make_dict)
	EndTime = defaultdict(make_dict)

	serviceOrder = {}
	for i in TSPtour:
		serviceOrder[i] = []

	curTime = 0
	Status = 'Feasible'

	for i in TSPtour:

		decvarcheckt[i] = curTime

		if (i == 0) or (len(landsat[i]) == 0):
			Retrievals = []
		else:
			isFeasible = True

			Retrievals = sorted(ArrTime[i], key=ArrTime[i].get)
			checkTime = curTime

			for v in Retrievals:
				if checkTime < EndTime[i][v]:
					checkTime = max(checkTime, ArrTime[i][v])
					checkTime += sR[v][i]
				else:
					isFeasible = False
					break

			if isFeasible == False:

				bestTime = float('inf')
				Retrievals = []

				Rset = list(itertools.permutations(list(landsat[i])))

				for Rcand in Rset:
					candFeasible = True
					checkTime = curTime

					for v in Rcand:
						if checkTime < EndTime[i][v]:
							checkTime = max(checkTime, ArrTime[i][v])
							checkTime += sR[v][i]
						else:
							candFeasible = False
							break

					if candFeasible == True:
						if checkTime < bestTime:
							bestTime = checkTime
							Retrievals = list(Rcand)

				if len(Retrievals) == 0:

					Status = 'Infeasible'
					break

		Launches = [1]

		zzz = {}

		if P3Type == 1:
			for v in launchesfrom[i]:
				zzz[v] = UAVtau[v][i][A[v][i]][ABC[v][i]]

			Lzzz = sorted(zzz, key=zzz.get, reverse=True)
			for v in Lzzz:
				Launches.append(v)

		elif P3Type == 2:
			for v in launchesfrom[i]:
				zzz[v] = inieee[v][i][A[v][i]][ABC[v][i]]

			Lzzz = sorted(zzz, key=zzz.get, reverse=True)
			for v in Lzzz:
				Launches.append(v)

		else:
			for v in launchesfrom[i]:
				zzz[v] = inieee[v][i][A[v][i]][ABC[v][i]] - UAVtau[v][i][A[v][i]][ABC[v][i]]

			Lzzz = sorted(zzz, key=zzz.get, reverse=True)
			for v in Lzzz:
				Launches.append(v)

		while len(Retrievals) != 0:

			v = Retrievals[0]

			if curTime < ArrTime[i][v]:

				if len(set(availVehicles[i]).intersection(set(Launches))) != 0:

					for v2 in Launches:

						if v2 in availVehicles[i]:

							if v2 == 1:
								ServiceLaunch = sigma[i]
							else:
								ServiceLaunch = sL[v2][i]

							if curTime + ServiceLaunch < EndTime[i][v]:
								curTime += ServiceLaunch

								if v2 == 1:
									decvarbart[i] = curTime
									serviceOrder[i].append((v2, 's'))
								else:
									decvarhattprime[v2][i] = curTime
									serviceOrder[i].append((v2, 'l'))
									ArrTime[ABC[v2][i]][v2] = curTime + UAVtau[v2][i][A[v2][i]][ABC[v2][i]]
									EndTime[ABC[v2][i]][v2] = curTime + inieee[v2][i][A[v2][i]][ABC[v2][i]]

									decvarchecktprime[v2][A[v2][i]] = curTime + iniTauprimeF[v2][i][A[v2][i]]
									decvarhattprime[v2][A[v2][i]] = curTime + iniTauprimeF[v2][i][A[v2][i]] + sigmaprime[A[v2][i]]

								availVehicles[i].remove(v2)

								break

							else:
								curTime = float(ArrTime[i][v])
								curTime += sR[v][i]
								decvarchecktprime[v][i] = curTime
								availVehicles[i].append(v)
								Retrievals.remove(v)
								serviceOrder[i].append((v, 'r'))
								break

					continue

				else:
					curTime = float(ArrTime[i][v])
					curTime += sR[v][i]
					decvarchecktprime[v][i] = curTime
					availVehicles[i].append(v)
					Retrievals.remove(v)
					serviceOrder[i].append((v, 'r'))

			else:

				if curTime <= EndTime[i][v]:

					curTime += sR[v][i]
					decvarchecktprime[v][i] = curTime
					availVehicles[i].append(v)
					Retrievals.remove(v)
					serviceOrder[i].append((v, 'r'))

				else:
					dummyLaunch = []
					dummyTime = curTime
					dummyOrder = list(serviceOrder[i])
					dummyOrder.append((v, 'r'))

					for revIndex in range(len(serviceOrder[i])-1, -1, -1):
						dv = serviceOrder[i][revIndex][0]
						dt = serviceOrder[i][revIndex][1]

						if dt in ('l','s'):
							if dt == 'l':
								dummyTime -= sL[dv][i]
							elif dt == 's':
								dummyTime -= sigma[i]
							dummyOrder.remove((dv,dt))
							dummyLaunch.append((dv,dt))

							if dummyTime <= EndTime[i][v]:
								pivot = revIndex - 1
								break

					dummyLaunch.reverse()

					serviceOrder[i] = list(dummyOrder + dummyLaunch)
					availVehicles[i].append(v)
					Retrievals.remove(v)

					if serviceOrder[i][pivot][1] == 'l':
						curTime = decvarhattprime[serviceOrder[i][pivot][0]][i]
					elif serviceOrder[i][pivot][1] == 'r':
						curTime = decvarchecktprime[serviceOrder[i][pivot][0]][i]
					elif serviceOrder[i][pivot][1] == 's':
						curTime = decvarbart[i]

					for newIndex in range(pivot+1, len(serviceOrder[i])):
						dv = serviceOrder[i][newIndex][0]
						dt = serviceOrder[i][newIndex][1]

						if dt == 's':
							curTime += sigma[i]
							decvarbart[i] = curTime

						elif dt == 'l':
							curTime += sL[dv][i]
							decvarhattprime[dv][i] = curTime

							decvarchecktprime[dv][A[dv][i]] = curTime + iniTauprimeF[dv][i][A[dv][i]]
							decvarhattprime[dv][A[dv][i]] = curTime + iniTauprimeF[dv][i][A[dv][i]] + sigmaprime[A[dv][i]]

							ArrTime[ABC[dv][i]][dv] = curTime + UAVtau[dv][i][A[dv][i]][ABC[dv][i]]
							EndTime[ABC[dv][i]][dv] = curTime + inieee[dv][i][A[dv][i]][ABC[dv][i]]

						elif dt == 'r':
							curTime = max(curTime, ArrTime[i][dv])
							curTime += sR[dv][i]
							decvarchecktprime[dv][i] = curTime

		for v in Launches:
			if v in availVehicles[i]:

				if v == 1:
					curTime += sigma[i]
					decvarbart[i] = curTime
					serviceOrder[i].append((v, 's'))
					availVehicles[i].remove(v)
				else:
					curTime += sL[v][i]
					decvarhattprime[v][i] = curTime
					serviceOrder[i].append((v, 'l'))
					ArrTime[ABC[v][i]][v] = curTime + UAVtau[v][i][A[v][i]][ABC[v][i]]
					EndTime[ABC[v][i]][v] = curTime + inieee[v][i][A[v][i]][ABC[v][i]]
					availVehicles[i].remove(v)

					decvarchecktprime[v][A[v][i]] = curTime + iniTauprimeF[v][i][A[v][i]]
					decvarhattprime[v][A[v][i]] = curTime + iniTauprimeF[v][i][A[v][i]] + sigmaprime[A[v][i]]

		decvarhatt[i] = curTime

		if i != c+1:
			k = TSPtour[TSPtour.index(i)+1]
			curTime += tau[i][k]

	ls_hatt = {}
	ls_checkt = {}
	ls_checktprime = {}

	finTauprimeF = defaultdict(make_dict)
	finTauprimeE = defaultdict(make_dict)
	fineee 		 = defaultdict(make_dict)
	finSpeedF 	 = defaultdict(make_dict)
	finSpeedE 	 = defaultdict(make_dict)

	if (Status == 'Infeasible'):

		assignmentsArray 	= []
		packagesArray 		= []
		p3isFeasible 		= False
		p3OFV				= float('inf')
		waitingTruck		= -1
		waitingUAV			= -1
		waitingArray 		= {}
		oldWaitingArray 	= {}

	else:

		p3isFeasible 			= True
		p3OFV 					= decvarhatt[c+1]
		waitingTruck			= 0.0
		waitingUAV				= 0.0

		for [v,i,j,k] in y:
			finTauprimeF[v][i][j] = float(iniTauprimeF[v][i][j])
			finTauprimeE[v][j][k] = float(iniTauprimeE[v][j][k])
			finSpeedF[v][i][j] = float(iniSpeedF[v][i][j])
			finSpeedE[v][j][k] = float(iniSpeedE[v][j][k])
			fineee[v][i][j][k] = float(inieee[v][i][j][k])

		newcheckt 	= defaultdict(make_dict)
		newbart 	= defaultdict(make_dict)
		newhatt 	= defaultdict(make_dict)
		newchecktprime 	= defaultdict(make_dict)
		newhattprime 	= defaultdict(make_dict)

		if UAVSpeedType == 1:
			currentTime = 0

			for [i,k] in x:

				if i == 0:
					newcheckt[i] = currentTime
					newbart[i] = currentTime

					activities = []

					for v in launchesfrom[i]:
						heappush(activities, (decvarhattprime[v][i], ['hattprime',v,i]))

					while len(activities) > 0:
						actTime, act = heappop(activities)

						if act[0] == 'hattprime':
							currentTime += sL[act[1]][act[2]]
							newhattprime[act[1]][act[2]] = currentTime

					newhatt[i] = currentTime

				currentTime += tau[i][k]
				newcheckt[k] = currentTime

				activities = []

				for v in landsat[k]:
					heappush(activities, (decvarchecktprime[v][k], ['checktprime',v,k]))

				for v in launchesfrom[k]:
					heappush(activities, (decvarhattprime[v][k], ['hattprime',v,k]))

				heappush(activities, (decvarbart[k], ['bart',k]))

				while len(activities) > 0:
					actTime, act = heappop(activities)

					if act[0] == 'bart':
						currentTime += sigma[k]
						newbart[k] = currentTime

					elif act[0] == 'hattprime':
						currentTime += sL[act[1]][act[2]]
						newhattprime[act[1]][act[2]] = currentTime

					elif act[0] == 'checktprime':
						potCurrentTime = currentTime + sR[act[1]][act[2]]
						vcur = act[1]
						icur = AB[act[1]][act[2]]
						jcur = B[act[1]][act[2]]
						kcur = act[2]

						if ((decvarchecktprime[vcur][kcur] - decvarhattprime[vcur][icur]) - (potCurrentTime - newhattprime[vcur][icur]) > 0.0001) and ((newhattprime[vcur][icur] + iniTauprimeF[vcur][icur][jcur] + sigmaprime[jcur] + iniTauprimeE[vcur][jcur][kcur] + sR[vcur][kcur]) > potCurrentTime):
							if kcur == c+1:
								[newchecktprime[vcur][jcur], newhattprime[vcur][jcur], currentTime, finTauprimeF[vcur][icur][jcur], finTauprimeE[vcur][jcur][kcur], finSpeedF[vcur][icur][jcur], finSpeedE[vcur][jcur][kcur]] = endurance_calculator.modify_speed(decvarhattprime[vcur][icur], decvarchecktprime[vcur][kcur], newhattprime[vcur][icur], potCurrentTime, vcur, icur, jcur, 0, node, vehicle, travel, iniTauprimeF[vcur][icur][jcur], iniTauprimeE[vcur][jcur][kcur], iniSpeedF[vcur][icur][jcur], iniSpeedE[vcur][jcur][kcur])
							else:
								[newchecktprime[vcur][jcur], newhattprime[vcur][jcur], currentTime, finTauprimeF[vcur][icur][jcur], finTauprimeE[vcur][jcur][kcur], finSpeedF[vcur][icur][jcur], finSpeedE[vcur][jcur][kcur]] = endurance_calculator.modify_speed(decvarhattprime[vcur][icur], decvarchecktprime[vcur][kcur], newhattprime[vcur][icur], potCurrentTime, vcur, icur, jcur, kcur, node, vehicle, travel, iniTauprimeF[vcur][icur][jcur], iniTauprimeE[vcur][jcur][kcur], iniSpeedF[vcur][icur][jcur], iniSpeedE[vcur][jcur][kcur])

							if kcur == c+1:
								fineee[vcur][icur][jcur][kcur] = endurance_calculator.give_endurance(node, vehicle, travel, vcur, icur, jcur, 0, finTauprimeF[vcur][icur][jcur], finTauprimeE[vcur][jcur][kcur], finSpeedF[vcur][icur][jcur], finSpeedE[vcur][jcur][kcur])
							else:
								fineee[vcur][icur][jcur][kcur] = endurance_calculator.give_endurance(node, vehicle, travel, vcur, icur, jcur, kcur, finTauprimeF[vcur][icur][jcur], finTauprimeE[vcur][jcur][kcur], finSpeedF[vcur][icur][jcur], finSpeedE[vcur][jcur][kcur])

							newchecktprime[vcur][kcur] = currentTime
						else:
							currentTime = potCurrentTime
							newchecktprime[vcur][kcur] = currentTime
							newchecktprime[vcur][jcur] = newhattprime[vcur][icur] + iniTauprimeF[vcur][icur][jcur]
							newhattprime[vcur][jcur] = newchecktprime[vcur][jcur] + sigmaprime[jcur]

				newhatt[k] = max(currentTime, newbart[k])

		else:
			for i in decvarcheckt:
				newcheckt[i] = float(decvarcheckt[i])
			for i in decvarbart:
				newbart[i] = float(decvarbart[i])
			for i in decvarhatt:
				newhatt[i] = float(decvarhatt[i])
			for v in decvarchecktprime:
				for i in decvarchecktprime[v]:
					newchecktprime[v][i] = float(decvarchecktprime[v][i])
			for v in decvarhattprime:
				for i in decvarhattprime[v]:
					newhattprime[v][i] = float(decvarhattprime[v][i])

		p3OFV = newhatt[c+1]

		prevTime = {}
		assignmentsArray = {}
		packagesArray = {}
		waitingArray = {}
		oldWaitingArray = {}
		waitingArray[0] = 0
		oldWaitingArray[0] = 0

		prevTime[1] = 0.0
		assignmentsArray[1] = []
		for v in V:
			prevTime[v] = 0.0
			assignmentsArray[v] = []

		uavRiders = []
		for v in V:
			uavRiders.append(v)

		tmpIcon = 'ub_truck_%d.gltf' % (1)
		for [i,j] in x:

			waitingTruck += ((newcheckt[j] - newcheckt[i]) - (tau[i][j] + sigma[j]))

			dummy_1 = 0
			for v in launchesfrom[i]:
				dummy_1 += sL[v][i]

			dummy_2 = 0
			for v in landsat[i]:
				dummy_2 += sR[v][i]

			waitingArray[i] = (newcheckt[j] - newcheckt[i]) - (tau[i][j] + sigma[i] + dummy_1 + dummy_2)
			oldWaitingArray[i] = (decvarcheckt[j] - decvarcheckt[i]) - (tau[i][j] + sigma[i] + dummy_1 + dummy_2)

			for v in V:
				if ((v in landsat[i]) and (v not in uavRiders)):
					uavRiders.append(v)
				if ((v in launchesfrom[i]) and (v in uavRiders)):
					uavRiders.remove(v)

			tmpTimes = []

			if (i == 0):
				for v in launchesfrom[i]:
					if (len(uavRiders) > 0):
						A_statusID = STATIONARY_TRUCK_W_UAV
					else:
						A_statusID = STATIONARY_TRUCK_EMPTY
					A_vehicleType = TYPE_TRUCK
					A_startTime = newhattprime[v][i] - sL[v][i]
					A_startNodeID = i
					A_startLatDeg = node[i].latDeg
					A_startLonDeg = node[i].lonDeg
					A_startAltMeters = 0.0
					A_endTime = newhattprime[v][i]
					A_endNodeID = i
					A_endLatDeg = node[i].latDeg
					A_endLonDeg = node[i].lonDeg
					A_endAltMeters = 0.0
					A_icon = tmpIcon
					A_description = 'Launching UAV %d' % (v)
					A_UAVsOnBoard = uavRiders
					A_ganttStatus = GANTT_LAUNCH

					tmpTimes.append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])

			if (len(uavRiders) > 0):
				A_statusID = TRAVEL_TRUCK_W_UAV
			else:
				A_statusID = TRAVEL_TRUCK_EMPTY
			A_vehicleType = TYPE_TRUCK
			A_startTime = newhatt[i]
			A_startNodeID = i
			A_startLatDeg = node[i].latDeg
			A_startLonDeg = node[i].lonDeg
			A_startAltMeters = 0.0
			A_endTime = newhatt[i] + tau[i][j]
			A_endNodeID = j
			A_endLatDeg = node[j].latDeg
			A_endLonDeg = node[j].lonDeg
			A_endAltMeters = 0.0
			A_icon = tmpIcon
			A_description = 'Travel from node %d to node %d' % (i, j)
			A_UAVsOnBoard = uavRiders
			A_ganttStatus = GANTT_TRAVEL

			tmpTimes.append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])

			if (newcheckt[j] - newhatt[i] - tau[i][j] > 0.01):
				if (len(uavRiders) > 0):
					A_statusID = STATIONARY_TRUCK_W_UAV
				else:
					A_statusID = STATIONARY_TRUCK_EMPTY
				A_vehicleType = TYPE_TRUCK
				A_startTime = (newhatt[i] + tau[i][j])
				A_startNodeID = j
				A_startLatDeg = node[j].latDeg
				A_startLonDeg = node[j].lonDeg
				A_startAltMeters = 0.0
				A_endTime = newcheckt[j]
				A_endNodeID = j
				A_endLatDeg = node[j].latDeg
				A_endLonDeg = node[j].lonDeg
				A_endAltMeters = 0.0
				A_icon = tmpIcon
				A_description = 'Idle for %3.0f seconds at node %d' % (A_endTime - A_startTime, j)
				A_UAVsOnBoard = uavRiders
				A_ganttStatus = GANTT_IDLE

				tmpTimes.append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])

			if (j == c+1):
				myMin, mySec	= divmod(newhatt[j], 60)
				myHour, myMin 	= divmod(myMin, 60)
				A_description	= 'At the Depot.  Total Time = %d:%02d:%02d' % (myHour, myMin, mySec)
				A_endTime		= -1
				A_ganttStatus	= GANTT_FINISHED
			else:
				A_description		= 'Dropping off package to Customer %d' % (j)
				A_endTime 		= newbart[j]
				A_ganttStatus 	= GANTT_DELIVER
				packagesArray[j] = [TYPE_TRUCK, node[j].latDeg, node[j].lonDeg, newbart[j], packageIcons[1]]

			if (len(uavRiders) > 0):
				A_statusID = STATIONARY_TRUCK_W_UAV
			else:
				A_statusID = STATIONARY_TRUCK_EMPTY
			A_vehicleType = TYPE_TRUCK
			if (j == c+1):
				A_startTime = newhatt[j] - sigma[j]
			else:
				A_startTime = newbart[j] - sigma[j]
			A_startNodeID = j
			A_startLatDeg = node[j].latDeg
			A_startLonDeg = node[j].lonDeg
			A_startAltMeters = 0.0
			A_endNodeID = j
			A_endLatDeg = node[j].latDeg
			A_endLonDeg = node[j].lonDeg
			A_endAltMeters = 0.0
			A_icon = tmpIcon
			A_UAVsOnBoard = uavRiders

			tmpTimes.append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])

			if (j <= c+1):

				for v in landsat[j]:
					if (len(uavRiders) > 0):
						A_statusID = STATIONARY_TRUCK_W_UAV
					else:
						A_statusID = STATIONARY_TRUCK_EMPTY
					A_vehicleType = TYPE_TRUCK
					A_startTime = newchecktprime[v][j] - sR[v][j]
					A_startNodeID = j
					A_startLatDeg = node[j].latDeg
					A_startLonDeg = node[j].lonDeg
					A_startAltMeters = 0.0
					A_endTime = newchecktprime[v][j]
					A_endNodeID = j
					A_endLatDeg = node[j].latDeg
					A_endLonDeg = node[j].lonDeg
					A_endAltMeters = 0.0
					A_icon = tmpIcon
					A_description = 'Retrieving UAV %d' % (v)
					A_UAVsOnBoard = uavRiders
					A_ganttStatus = GANTT_RECOVER

					tmpTimes.append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])

			for v in launchesfrom[j]:
				if (len(uavRiders) > 0):
					A_statusID = STATIONARY_TRUCK_W_UAV
				else:
					A_statusID = STATIONARY_TRUCK_EMPTY
				A_vehicleType = TYPE_TRUCK
				A_startTime = newhattprime[v][j] - sL[v][j]
				A_startNodeID = j
				A_startLatDeg = node[j].latDeg
				A_startLonDeg = node[j].lonDeg
				A_startAltMeters = 0.0
				A_endTime = newhattprime[v][j]
				A_endNodeID = j
				A_endLatDeg = node[j].latDeg
				A_endLonDeg = node[j].lonDeg
				A_endAltMeters = 0.0
				A_icon = tmpIcon
				A_description = 'Launching UAV %d' % (v)
				A_UAVsOnBoard = uavRiders
				A_ganttStatus = GANTT_LAUNCH

				tmpTimes.append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])

			unasgnInd = list(range(0, len(tmpTimes)))
			while (len(unasgnInd) > 0):
				tmpMin = 2*newhatt[j]

				for tmpIndex in unasgnInd:
					if (tmpTimes[tmpIndex][2] < tmpMin):
						tmpMin = tmpTimes[tmpIndex][2]
						myIndex = tmpIndex

				if (tmpMin - prevTime[1] > 0.01):

					if (len(tmpTimes[myIndex][14]) > 0):
						A_statusID = STATIONARY_TRUCK_W_UAV
					else:
						A_statusID = STATIONARY_TRUCK_EMPTY
					A_vehicleType = TYPE_TRUCK
					A_startTime = prevTime[1]
					A_startNodeID = tmpTimes[myIndex][3]
					A_startLatDeg = node[tmpTimes[myIndex][3]].latDeg
					A_startLonDeg = node[tmpTimes[myIndex][3]].lonDeg
					A_startAltMeters = 0.0
					A_endTime = prevTime[1] + (tmpMin - prevTime[1])
					A_endNodeID = tmpTimes[myIndex][3]
					A_endLatDeg = node[tmpTimes[myIndex][3]].latDeg
					A_endLonDeg = node[tmpTimes[myIndex][3]].lonDeg
					A_endAltMeters = 0.0
					A_icon = tmpIcon
					A_description = 'Idle for %3.0f seconds' % (A_endTime - A_startTime)
					A_UAVsOnBoard = tmpTimes[myIndex][14]
					A_ganttStatus = GANTT_IDLE

					assignmentsArray[1].append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])

				assignmentsArray[1].append(tmpTimes[myIndex])

				prevTime[1] = tmpTimes[myIndex][7]
				unasgnInd.remove(myIndex)

			if ((j != c+1) and (prevTime[1] - newhatt[j] < -0.01)):

				if (len(tmpTimes[myIndex][14]) > 0):
					A_statusID = STATIONARY_TRUCK_W_UAV
				else:
					A_statusID = STATIONARY_TRUCK_EMPTY
				A_vehicleType = TYPE_TRUCK
				A_startTime = tmpMin
				A_startNodeID = tmpTimes[myIndex][3]
				A_startLatDeg = node[tmpTimes[myIndex][3]].latDeg
				A_startLonDeg = node[tmpTimes[myIndex][3]].lonDeg
				A_startAltMeters = 0.0
				A_endTime = newhatt[j]
				A_endNodeID = tmpTimes[myIndex][3]
				A_endLatDeg = node[tmpTimes[myIndex][3]].latDeg
				A_endLonDeg = node[tmpTimes[myIndex][3]].lonDeg
				A_endAltMeters = 0.0
				A_icon = tmpIcon
				A_description = 'Idle for %3.0f seconds before departing' % (A_endTime - A_startTime)
				A_UAVsOnBoard = tmpTimes[myIndex][14]
				A_ganttStatus = GANTT_IDLE

				assignmentsArray[1].append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])

			prevTime[1] = newhatt[j]

		for [v,i,j,k] in y:

			waitingUAV += ((newchecktprime[v][k] - newcheckt[i]) - (finTauprimeF[v][i][j] + finTauprimeE[v][j][k] + sigmaprime[j] + sL[v][i] + sR[v][k]))

			waitingArray[j] = ((newchecktprime[v][k] - newcheckt[i]) - (finTauprimeF[v][i][j] + finTauprimeE[v][j][k] + sigmaprime[j] + sL[v][i] + sR[v][k]))
			oldWaitingArray[j] = ((decvarchecktprime[v][k] - decvarcheckt[i]) - (iniTauprimeF[v][i][j] + iniTauprimeE[v][j][k] + sigmaprime[j] + sL[v][i] + sR[v][k]))

			A_statusID = STATIONARY_UAV_PACKAGE
			A_vehicleType = TYPE_UAV
			A_startTime = newhattprime[v][i] - sL[v][i]
			A_startNodeID = i
			A_startLatDeg = node[i].latDeg
			A_startLonDeg = node[i].lonDeg
			A_startAltMeters = 0.0
			A_endTime = newhattprime[v][i]
			A_endNodeID = i
			A_endLatDeg = node[i].latDeg
			A_endLonDeg = node[i].lonDeg
			A_endAltMeters = 0.0
			A_icon = 'iris_black_blue_plus_box_yellow.gltf'
			A_description = 'Prepare to launch from truck'
			A_UAVsOnBoard = []
			A_ganttStatus = GANTT_LAUNCH

			assignmentsArray[v].append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])

			A_statusID = VERTICAL_UAV_PACKAGE
			A_vehicleType = TYPE_UAV
			A_startTime = newhattprime[v][i]
			A_startNodeID = i
			A_startLatDeg = node[i].latDeg
			A_startLonDeg = node[i].lonDeg
			A_startAltMeters = 0.0
			A_endTime = newhattprime[v][i] + travel[v][i][j].takeoffTime
			A_endNodeID = i
			A_endLatDeg = node[i].latDeg
			A_endLonDeg = node[i].lonDeg
			A_endAltMeters = vehicle[v].cruiseAlt
			A_icon = 'iris_black_blue_plus_box_yellow.gltf'
			if (i == 0):
				A_description = 'Takeoff from Depot'
			else:
				A_description = 'Takeoff from truck at Customer %d' % (i)
			A_UAVsOnBoard = []
			A_ganttStatus = GANTT_TRAVEL

			assignmentsArray[v].append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])

			tmpStart = newhattprime[v][i] + travel[v][i][j].takeoffTime

			if (newchecktprime[v][j] - newhattprime[v][i] - finTauprimeF[v][i][j] > 0.01):
				tmpIdle = newchecktprime[v][j] - (newhattprime[v][i] + finTauprimeF[v][i][j])
				tmpEnd = tmpStart + tmpIdle
				A_statusID = STATIONARY_UAV_PACKAGE
				A_vehicleType = TYPE_UAV
				A_startTime = tmpStart
				A_startNodeID = i
				A_startLatDeg = node[i].latDeg
				A_startLonDeg = node[i].lonDeg
				A_startAltMeters = vehicle[v].cruiseAlt
				A_endTime = tmpEnd
				A_endNodeID = i
				A_endLatDeg = node[i].latDeg
				A_endLonDeg = node[i].lonDeg
				A_endAltMeters = vehicle[v].cruiseAlt
				A_icon = 'iris_black_blue_plus_box_yellow.gltf'
				A_description = 'Idle at initial takeoff (node %d)' % (i)
				A_UAVsOnBoard = []
				A_ganttStatus = GANTT_IDLE

				assignmentsArray[v].append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])

				tmpStart = tmpEnd

			A_statusID = TRAVEL_UAV_PACKAGE
			A_vehicleType = TYPE_UAV
			A_startTime = tmpStart
			A_startNodeID = i
			A_startLatDeg = node[i].latDeg
			A_startLonDeg = node[i].lonDeg
			A_startAltMeters = vehicle[v].cruiseAlt
			A_endTime = tmpStart + (finTauprimeF[v][i][j] - travel[v][i][j].takeoffTime - travel[v][i][j].landTime)
			A_endNodeID = j
			A_endLatDeg = node[j].latDeg
			A_endLonDeg = node[j].lonDeg
			A_endAltMeters = vehicle[v].cruiseAlt
			A_icon = 'iris_black_blue_plus_box_yellow.gltf'
			A_description = 'Fly to UAV customer %d' % (j)
			A_UAVsOnBoard = []
			A_ganttStatus = GANTT_TRAVEL

			assignmentsArray[v].append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])

			A_statusID = VERTICAL_UAV_PACKAGE
			A_vehicleType = TYPE_UAV
			A_startTime = newchecktprime[v][j] - travel[v][i][j].landTime
			A_startNodeID = j
			A_startLatDeg = node[j].latDeg
			A_startLonDeg = node[j].lonDeg
			A_startAltMeters = vehicle[v].cruiseAlt
			A_endTime = newchecktprime[v][j]
			A_endNodeID = j
			A_endLatDeg = node[j].latDeg
			A_endLonDeg = node[j].lonDeg
			A_endAltMeters = 0.0
			A_icon = 'iris_black_blue_plus_box_yellow.gltf'
			A_description = 'Land at UAV customer %d' % (j)
			A_UAVsOnBoard = []
			A_ganttStatus = GANTT_TRAVEL

			assignmentsArray[v].append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])

			A_statusID = STATIONARY_UAV_PACKAGE
			A_vehicleType = TYPE_UAV
			A_startTime = newchecktprime[v][j]
			A_startNodeID = j
			A_startLatDeg = node[j].latDeg
			A_startLonDeg = node[j].lonDeg
			A_startAltMeters = 0.0
			A_endTime = newhattprime[v][j]
			A_endNodeID = j
			A_endLatDeg = node[j].latDeg
			A_endLonDeg = node[j].lonDeg
			A_endAltMeters = 0.0
			A_icon = 'iris_black_blue_plus_box_yellow.gltf'
			A_description = 'Serving UAV customer %d' % (j)
			A_UAVsOnBoard = []
			A_ganttStatus = GANTT_DELIVER

			assignmentsArray[v].append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])

			packagesArray[j] = [TYPE_UAV, node[j].latDeg, node[j].lonDeg, newhattprime[v][j], packageIcons[0]]

			A_statusID = VERTICAL_UAV_EMPTY
			A_vehicleType = TYPE_UAV
			A_startTime = newhattprime[v][j]
			A_startNodeID = j
			A_startLatDeg = node[j].latDeg
			A_startLonDeg = node[j].lonDeg
			A_startAltMeters = 0.0
			if (k == c+1):

				A_endTime = newhattprime[v][j] + travel[v][j][0].takeoffTime
			else:
				A_endTime = newhattprime[v][j] + travel[v][j][k].takeoffTime
			A_endNodeID = j
			A_endLatDeg = node[j].latDeg
			A_endLonDeg = node[j].lonDeg
			A_endAltMeters = vehicle[v].cruiseAlt
			A_icon = 'iris_with_props_black_blue.gltf'
			A_description = 'Takeoff from UAV customer %d' % (j)
			A_UAVsOnBoard = []
			A_ganttStatus = GANTT_TRAVEL

			assignmentsArray[v].append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])

			A_statusID = TRAVEL_UAV_EMPTY
			A_vehicleType = TYPE_UAV
			if (k == c+1):

				A_startTime = newhattprime[v][j] + travel[v][j][0].takeoffTime
			else:
				A_startTime = newhattprime[v][j] + travel[v][j][k].takeoffTime
			A_startNodeID = j
			A_startLatDeg = node[j].latDeg
			A_startLonDeg = node[j].lonDeg
			A_startAltMeters = vehicle[v].cruiseAlt
			if (k == c+1):
				A_endTime = newhattprime[v][j] + (finTauprimeE[v][j][k] - travel[v][j][0].landTime)
			else:
				A_endTime = newhattprime[v][j] + (finTauprimeE[v][j][k] - travel[v][j][k].landTime)
			A_endNodeID = k
			A_endLatDeg = node[k].latDeg
			A_endLonDeg = node[k].lonDeg
			A_endAltMeters = vehicle[v].cruiseAlt
			A_icon = 'iris_with_props_black_blue.gltf'
			if (k == c+1):
				A_description = 'Fly to depot'
			else:
				A_description = 'Fly to truck at customer %d' % (k)
			A_UAVsOnBoard = []
			A_ganttStatus = GANTT_TRAVEL

			assignmentsArray[v].append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])

			if (k == c+1):
				tmpStart =  newhattprime[v][j] +  (finTauprimeE[v][j][k] - travel[v][j][0].landTime)
			else:
				tmpStart =  newhattprime[v][j] +  (finTauprimeE[v][j][k] - travel[v][j][k].landTime)
			tmpIdle = newchecktprime[v][k] - sR[v][k] - newhattprime[v][j] - finTauprimeE[v][j][k]

			if (tmpIdle > 0.01):
				tmpEnd = tmpStart + tmpIdle
				A_statusID = STATIONARY_UAV_EMPTY
				A_vehicleType = TYPE_UAV
				A_startTime = tmpStart
				A_startNodeID = k
				A_startLatDeg = node[k].latDeg
				A_startLonDeg = node[k].lonDeg
				A_startAltMeters = vehicle[v].cruiseAlt
				A_endTime = tmpEnd
				A_endNodeID = k
				A_endLatDeg = node[k].latDeg
				A_endLonDeg = node[k].lonDeg
				A_endAltMeters = vehicle[v].cruiseAlt
				A_icon = 'iris_with_props_black_blue.gltf'
				if (k == c+1):
					A_description = 'Idle above depot location'
				else:
					A_description = 'Idle above rendezvous location (customer %d)' % (k)
				A_UAVsOnBoard = []
				A_ganttStatus = GANTT_IDLE

				assignmentsArray[v].append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])

				tmpStart = tmpEnd

			A_statusID = VERTICAL_UAV_EMPTY
			A_vehicleType = TYPE_UAV
			A_startTime = tmpStart
			A_startNodeID = k
			A_startLatDeg = node[k].latDeg
			A_startLonDeg = node[k].lonDeg
			A_startAltMeters = vehicle[v].cruiseAlt
			if (k == c+1):

				A_endTime = tmpStart + travel[v][j][0].landTime
			else:
				A_endTime = tmpStart + travel[v][j][k].landTime
			A_endNodeID = k
			A_endLatDeg = node[k].latDeg
			A_endLonDeg = node[k].lonDeg
			A_endAltMeters = 0.0
			A_icon = 'iris_with_props_black_blue.gltf'
			if (k == c+1):
				A_description = 'Land at depot'
			else:
				A_description = 'Land at truck rendezvous location (customer %d)' % (k)
			A_UAVsOnBoard = []
			A_ganttStatus = GANTT_TRAVEL

			assignmentsArray[v].append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])

			A_statusID = STATIONARY_UAV_EMPTY
			A_vehicleType = TYPE_UAV
			A_startTime = newchecktprime[v][k] - sR[v][k]
			A_startNodeID = k
			A_startLatDeg = node[k].latDeg
			A_startLonDeg = node[k].lonDeg
			A_startAltMeters = 0.0
			A_endTime = newchecktprime[v][k]
			A_endNodeID = k
			A_endLatDeg = node[k].latDeg
			A_endLonDeg = node[k].lonDeg
			A_endAltMeters = 0.0
			A_icon = 'iris_with_props_black_blue.gltf'
			if (k == c+1):
				A_description = 'Recovered at depot'
			else:
				A_description = 'Recovered by truck at customer %d' % k
			A_UAVsOnBoard = []
			A_ganttStatus = GANTT_RECOVER

			assignmentsArray[v].append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])

		for k in N:
			if k not in C:
				ls_hatt[k] = decvarhatt[k]
				ls_checkt[k] = decvarcheckt[k]
			for v in V:
				if ((v in landsat[k]) or (v in launchesfrom[k])):
					if (k != 0):
						ls_checktprime[v,k] = decvarchecktprime[v][k]

	return (p3isFeasible, p3OFV, assignmentsArray, packagesArray, waitingTruck, waitingUAV, oldWaitingArray, landsat, launchesfrom, ls_checkt, ls_hatt, ls_checktprime, finTauprimeE, finTauprimeF, finSpeedE, finSpeedF, fineee)

