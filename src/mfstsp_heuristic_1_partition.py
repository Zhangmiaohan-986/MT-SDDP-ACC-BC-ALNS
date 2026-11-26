import sys
import time
import datetime
import math
from parseCSV import *
from gurobipy import *
from collections import defaultdict

from solve_tsp_callback import *
from checkP2Feasibility import *

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

packageIcons		= ['', 'box_blue_centered.gltf', 'box_orange_centered.gltf', 'box_green_centered.gltf', 'box_gray_centered.gltf', 'box_brown_centered.gltf']

def make_dict():
	return defaultdict(make_dict)

class make_node:
	def __init__(self, nodeType, latDeg, lonDeg, altMeters, parcelWtLbs, serviceTimeTruck, serviceTimeUAV, address):

		self.nodeType 			= nodeType
		self.latDeg 			= latDeg
		self.lonDeg				= lonDeg
		self.altMeters			= altMeters
		self.parcelWtLbs 		= parcelWtLbs
		self.serviceTimeTruck	= serviceTimeTruck
		self.serviceTimeUAV 	= serviceTimeUAV
		self.address 			= address

class make_assignments:
	def __init__(self, vehicleType, startTime, startNodeID, startLatDeg, startLonDeg, startAltMeters, endTime, endNodeID, endLatDeg, endLonDeg, endAltMeters, icon, description, UAVsOnBoard, ganttStatus):

		self.vehicleType 	= vehicleType
		self.startTime 		= startTime
		self.startNodeID	= startNodeID
		self.startLatDeg 	= startLatDeg
		self.startLonDeg 	= startLonDeg
		self.startAltMeters = startAltMeters
		self.endTime 		= endTime
		self.endNodeID 		= endNodeID
		self.endLatDeg		= endLatDeg
		self.endLonDeg		= endLonDeg
		self.endAltMeters 	= endAltMeters
		self.icon			= icon
		self.description 	= description
		self.UAVsOnBoard 	= UAVsOnBoard
		self.ganttStatus	= ganttStatus

class make_packages:
	def __init__(self, packageType, latDeg, lonDeg, deliveryTime, icon):

		self.packageType 	= packageType
		self.latDeg 		= latDeg
		self.lonDeg 		= lonDeg
		self.deliveryTime 	= deliveryTime
		self.icon 			= icon

def generateTSPinfo(myTour, c, C, node, tau, sigma):
	tmpAssignments = defaultdict(make_dict)

	vehicleType = TYPE_TRUCK
	UAVsOnBoard = []
	startAltMeters = 0.0
	endAltMeters = 0.0

	i = 0
	mayEnd = 0
	tmpDepart = 0.0
	icon = 'ub_truck_1.gltf'
	for myIndex in range(1,len(myTour)):
		j = myTour[myIndex]

		statusID 	= TRAVEL_TRUCK_EMPTY
		ganttStatus	= GANTT_TRAVEL
		startTime 	= tmpDepart
		startNodeID = i
		startLatDeg = node[i].latDeg
		startLonDeg	= node[i].lonDeg
		endTime		= startTime + tau[i][j]
		endNodeID	= j
		endLatDeg	= node[j].latDeg
		endLonDeg	= node[j].lonDeg
		if ((i in C) and (j in C)):
			description 	= 'Driving from Customer %d to Customer %d' % (i,j)
		elif ((i == 0) and (j in C)):
			description 	= 'Driving from Depot to Customer %d' % (j)
		elif ((i in C) and (j == c+1)):
			description 	= 'Returning to the Depot from Customer %d' % (i)
		elif ((i == 0) and (j == c+1)):
			description 	= 'Truck 1 was not used'
		else:
			print('WE HAVE A PROBLEM.  What is the proper description?')
			print('\t Quitting Now.')
			exit()

		if (0 in tmpAssignments[1][statusID]):
			statusIndex = len(tmpAssignments[1][statusID])
		else:
			statusIndex = 0

		tmpAssignments[1][statusID][statusIndex] = make_assignments(vehicleType, startTime, startNodeID, startLatDeg, startLonDeg, startAltMeters, endTime, endNodeID, endLatDeg, endLonDeg, endAltMeters, icon, description, UAVsOnBoard, ganttStatus)

		startTime 		= endTime
		startNodeID 	= j
		startLatDeg 	= node[j].latDeg
		startLonDeg		= node[j].lonDeg
		endTime			= startTime + sigma[j]
		endNodeID		= j
		endLatDeg		= node[j].latDeg
		endLonDeg		= node[j].lonDeg
		objVal 			= endTime
		if (j == c+1):
			statusID		= STATIONARY_TRUCK_EMPTY
			ganttStatus		= GANTT_FINISHED
			description		= 'Arrived at the Depot.'
		else:
			statusID		= STATIONARY_TRUCK_EMPTY
			ganttStatus		= GANTT_DELIVER
			description		= 'Dropping off package to Customer %d' % (j)

		if (0 in tmpAssignments[1][statusID]):
			statusIndex = len(tmpAssignments[1][statusID])
		else:
			statusIndex = 0

		tmpAssignments[1][statusID][statusIndex] = make_assignments(vehicleType, startTime, startNodeID, startLatDeg, startLonDeg, startAltMeters, endTime, endNodeID, endLatDeg, endLonDeg, endAltMeters, icon, description, UAVsOnBoard, ganttStatus)

		tmpDepart = endTime
		if (j != c+1):

			i = j

	return (objVal, tmpAssignments, myTour)

def make_TSP_package(myTour, c, C, node, tau, sigma):

	assignments	= defaultdict(make_dict)
	packages	= defaultdict(make_dict)

	vehicleType = TYPE_TRUCK
	UAVsOnBoard = []
	startAltMeters = 0.0
	endAltMeters = 0.0

	i = 0
	mayEnd = 0
	tmpDepart = 0.0
	icon = 'ub_truck_1.gltf'
	for myIndex in range(1,len(myTour)):
		j = myTour[myIndex]

		statusID 	= TRAVEL_TRUCK_EMPTY
		ganttStatus	= GANTT_TRAVEL
		startTime 	= tmpDepart
		startNodeID = i
		startLatDeg = node[i].latDeg
		startLonDeg	= node[i].lonDeg
		endTime		= startTime + tau[i][j]
		endNodeID	= j
		endLatDeg	= node[j].latDeg
		endLonDeg	= node[j].lonDeg
		if ((i in C) and (j in C)):
			description 	= 'Driving from Customer %d to Customer %d' % (i,j)
		elif ((i == 0) and (j in C)):
			description 	= 'Driving from Depot to Customer %d' % (j)
		elif ((i in C) and (j == c+1)):
			description 	= 'Returning to the Depot from Customer %d' % (i)
		elif ((i == 0) and (j == c+1)):
			description 	= 'Truck 1 was not used'
		else:
			print('WE HAVE A PROBLEM.  What is the proper description?')
			print('\t Quitting Now.')
			exit()

		if (0 in assignments[1][statusID]):
			statusIndex = len(assignments[1][statusID])
		else:
			statusIndex = 0

		assignments[1][statusID][statusIndex] = make_assignments(vehicleType, startTime, startNodeID, startLatDeg, startLonDeg, startAltMeters, endTime, endNodeID, endLatDeg, endLonDeg, endAltMeters, icon, description, UAVsOnBoard, ganttStatus)

		startTime 		= endTime
		startNodeID 	= j
		startLatDeg 	= node[j].latDeg
		startLonDeg		= node[j].lonDeg
		endTime			= startTime + sigma[j]
		endNodeID		= j
		endLatDeg		= node[j].latDeg
		endLonDeg		= node[j].lonDeg
		objVal 			= endTime
		if (j == c+1):
			statusID		= STATIONARY_TRUCK_EMPTY
			ganttStatus		= GANTT_FINISHED
			tmpMin, tmpSec	= divmod(endTime, 60)
			tmpHour, tmpMin = divmod(tmpMin, 60)
			description		= 'Arrived at the Depot.  Total Time = %d:%02d:%02d' % (tmpHour, tmpMin, tmpSec)
			endTime			= -1
		else:
			statusID		= STATIONARY_TRUCK_EMPTY
			ganttStatus		= GANTT_DELIVER
			description		= 'Dropping off package to Customer %d' % (j)
			packageType 	= TYPE_TRUCK
			pkgIcon 		= packageIcons[1]
			packages[j] 	= make_packages(packageType, endLatDeg, endLonDeg, endTime, pkgIcon)

		if (0 in assignments[1][statusID]):
			statusIndex = len(assignments[1][statusID])
		else:
			statusIndex = 0

		assignments[1][statusID][statusIndex] = make_assignments(vehicleType, startTime, startNodeID, startLatDeg, startLonDeg, startAltMeters, endTime, endNodeID, endLatDeg, endLonDeg, endAltMeters, icon, description, UAVsOnBoard, ganttStatus)

		tmpDepart = endTime
		if (j != c+1):

			i = j

	return (objVal, assignments, packages, myTour)

def getTSP(c, tmpTruckCustomers, node, vehicle, travel):

	newNode = {}
	newNode[0] = make_node(node[0].nodeType, node[0].latDeg, node[0].lonDeg, node[0].altMeters, node[0].parcelWtLbs, node[0].serviceTimeTruck, node[0].serviceTimeUAV, node[0].address)
	for j in tmpTruckCustomers:
		newNode[j] = make_node(node[j].nodeType, node[j].latDeg, node[j].lonDeg, node[j].altMeters, node[j].parcelWtLbs, node[j].serviceTimeTruck, node[j].serviceTimeUAV, node[j].address)

	[TSPobjVal, TSPassignments, TSPpackages, TSPtour] = solve_tsp_callback(newNode, vehicle, travel)

	fixedTSPtour = []
	for i in TSPtour[0:-1]:
		fixedTSPtour.append(i)
	fixedTSPtour.append(c+1)

	return (fixedTSPtour, TSPobjVal)

def getTotalCost(TSPtour, tau, sigma, V, C, sL, sR):

	totalCost = 0
	i = 0
	for j in TSPtour[1:len(TSPtour)]:
		totalCost += (tau[i][j] + sigma[j])
		i = j
	v = min(V)
	for j in C:
		if j not in TSPtour:
			totalCost += (sL[v][j] + sR[v][j])

	return totalCost

def getTruckMoves(V, C, TSPtour, tau, sigma, sL, sR):

	v = min(V)
	moreTruck = []
	moreTruckSavings = []
	for j in C:

		if j not in TSPtour:
			for iii in range(0, len(TSPtour)-1):
				i = TSPtour[iii]
				k = TSPtour[iii+1]
				tmpSavings = (sL[v][j] + sR[v][j]) - (tau[i][j] + tau[j][k] + sigma[j] - tau[i][k])
				if (tmpSavings > 0):

					if (j not in moreTruck):
						moreTruck.append(j)
						moreTruckSavings.append(tmpSavings)

	return (list(moreTruck), list(moreTruckSavings))

def getUAVmoves(TSPtour, xxxTruckOnly, sL, sR, tau, sigma, V, Pprime):

	v = min(V)
	moreUAV = []
	moreUAVsavings = []
	for iii in range(0, len(TSPtour)-2):
		i = TSPtour[iii]
		j = TSPtour[iii+1]
		k = TSPtour[iii+2]
		if (j not in xxxTruckOnly):
			if ([i,k] in Pprime[v][j]):
				tmpSavings = (sigma[j] + tau[i][j] + tau[j][k]) - (sL[v][j] + sR[v][j] + tau[i][k])
				if (tmpSavings > 0):

					if (j not in moreUAV):
						moreUAV.append(j)
						moreUAVsavings.append(tmpSavings)

	return (list(moreUAV), list(moreUAVsavings))

def mfstsp_heuristic_1_partition(node, vehicle, travel, N, N_zero, N_plus, C, P, tau, tauprimeE, tauprimeF, sigma, sigmaprime, sL, sR, lowerTruckLimit, requireUniqueTSP, prevTSPtours, bestOFV, p1_previousTSP, p1_FEASobjVal):

	V = []
	for vehicleID in vehicle:
		if (vehicle[vehicleID].vehicleType == TYPE_UAV):
			V.append(vehicleID)

	c = len(C)

	Cprime = []

	Pprime = defaultdict(make_dict)
	for v in V:
		for j in C:
			Pprime[v][j] = []

	for [v,i,j,k] in P:
		if (j not in Cprime):
			Cprime.append(j)

		Pprime[v][j].append([i,k])

	customersUAV = []
	customersTruck = []

	if len(p1_previousTSP) == 0:

		xxxTruckOnly = []
		for i in C:
			if (i not in Cprime):
				xxxTruckOnly.append(i)

		[TSPtour, TSPobjVal] = getTSP(c, xxxTruckOnly, node, vehicle, travel)

		totalCost = getTotalCost(TSPtour, tau, sigma, V, C, sL, sR)

		bestCost = totalCost
		currentTSP = list(TSPtour)

		for xyz in range(0,20):

			[moreTruck, moreTruckSavings] = getTruckMoves(V, C, TSPtour, tau, sigma, sL, sR)

			if (len(moreTruck) > 0):

				tmpTruckCustomers = list(TSPtour + moreTruck)
				tmpTruckCustomers = list( set(tmpTruckCustomers) - set([0, c+1]) )

				[TSPtour, TSPobjVal] = getTSP(c, tmpTruckCustomers, node, vehicle, travel)

				totalCost = getTotalCost(TSPtour, tau, sigma, V, C, sL, sR)

				if (totalCost < bestCost):
					bestCost = totalCost
					currentTSP = list(TSPtour)

			[moreUAV, moreUAVsavings] = getUAVmoves(TSPtour, xxxTruckOnly, sL, sR, tau, sigma, V, Pprime)

			if (len(moreUAV) > 0):

				tmpTruckCustomers = list(set(TSPtour) - set(moreUAV))
				tmpTruckCustomers = list( set(tmpTruckCustomers) - set([0, c+1]) )

				[TSPtour, TSPobjVal] = getTSP(c, tmpTruckCustomers, node, vehicle, travel)

				totalCost = getTotalCost(TSPtour, tau, sigma, V, C, sL, sR)

				if (totalCost < bestCost):
					bestCost = totalCost
					currentTSP = list(TSPtour)

	else:
		currentTSP = list(p1_previousTSP)
		bestCost = float(p1_FEASobjVal)

	isFailed = True

	while isFailed:

		failed2reach = []
		insertCost = defaultdict(make_dict)

		maxSupport = 0
		support = defaultdict(make_dict)
		mostHelpful = None

		cheapestCost = float('inf')
		cheapestCostInfo = None

		bestRatio = float('inf')
		bestRatioInfo = None

		droners = list( set(C) - set(currentTSP) )

		for j in droners:

			canServe = False
			for iii in range(0, len(currentTSP)-1):
				i = currentTSP[iii]
				for kkk in [iii+1]:
					k = currentTSP[kkk]
					for v in V:
						if ([i,k] in Pprime[v][j]):
							canServe = True
							break
			if not canServe:
				failed2reach.append(j)

		if (len(failed2reach) == 0):
			if len(V) >= 1:
				[p3_status, p3_infeas_cust] = checkP2Feasibility(droners, currentTSP, V, Pprime)
				if (p3_status == 1) and (len(currentTSP) - 2 >= lowerTruckLimit):
					isFailed = False
				else:
					isFailed = True
					if (p3_status == 0):
						failed2reach = list(p3_infeas_cust)
			else:
				if (len(currentTSP) - 2 >= lowerTruckLimit):
					isFailed = False
				else:
					isFailed = True
		else:
			isFailed = True

		if (isFailed):

			for j in droners:
				for iii in range(0, len(currentTSP)-1):
					i = currentTSP[iii]
					k = currentTSP[iii+1]

					v = min(V)

					tmpCost = (tau[i][j] + tau[j][k] + sigma[j] - tau[i][k]) - (sL[v][j] + sR[v][j])
					insertCost[j][iii+1] = tmpCost
					if (tmpCost < cheapestCost):
						cheapestCost = tmpCost
						cheapestCostInfo = [j, iii+1, tmpCost]

					dummyTSP = list(currentTSP)
					dummyTSP.insert(iii+1, j)

					if dummyTSP not in prevTSPtours:

						support[j][iii+1] = []

						if (j in failed2reach):
							support[j][iii+1].append(j)

						for l in list( set(failed2reach) - set([j]) ):
							foundIt = False
							for v in V:
								if ([i,j] in Pprime[v][l]):

									support[j][iii+1].append(l)
									foundIt = True

								elif ([j,k] in Pprime[v][l]):

									support[j][iii+1].append(l)
									foundIt = True

								if (foundIt):
									break

						if (len(support[j][iii+1]) > maxSupport):
							maxSupport = len(support[j][iii+1])
							mostHelpful = [j, iii+1, len(support[j][iii+1])]

						if (len(support[j][iii+1]) > 0):
							if (tmpCost < 0):

								tmpRatio = tmpCost * len(support[j][iii+1])
							else:

								tmpRatio = tmpCost / len(support[j][iii+1])

							if (tmpRatio < bestRatio):
								bestRatio = tmpRatio
								bestRatioInfo = [j, iii+1, bestRatio]

			if ( (len(currentTSP) - 2 < lowerTruckLimit) and (len(failed2reach) == 0) ):

				j = cheapestCostInfo[0]
				tmpTruckCustomers = list(currentTSP + [j])
				tmpTruckCustomers = list( set(tmpTruckCustomers) - set([0, c+1]) )

				[currentTSP, TSPobjVal] = getTSP(c, tmpTruckCustomers, node, vehicle, travel)

			elif ( (len(currentTSP) - 2 >= lowerTruckLimit) and (len(failed2reach) > 0) and (bestRatioInfo != None) ):

				currentTSP.insert(bestRatioInfo[1], bestRatioInfo[0])

			elif (bestRatioInfo != None):

				j = bestRatioInfo[0]
				tmpTruckCustomers = list(currentTSP + [j])
				tmpTruckCustomers = list( set(tmpTruckCustomers) - set([0, c+1]) )

				[currentTSP, TSPobjVal] = getTSP(c, tmpTruckCustomers, node, vehicle, travel)

			else:

				j = cheapestCostInfo[0]
				tmpTruckCustomers = list(currentTSP + [j])
				tmpTruckCustomers = list( set(tmpTruckCustomers) - set([0, c+1]) )

				[currentTSP, TSPobjVal] = getTSP(c, tmpTruckCustomers, node, vehicle, travel)

			totalCost = getTotalCost(currentTSP, tau, sigma, V, C, sL, sR)

	basicPreviousTSP = list(currentTSP)

	customersTruck = list(currentTSP[1:-1])
	customersUAV = list(set(C) - set(currentTSP))

	newNode = {}
	newNode[0] = make_node(node[0].nodeType, node[0].latDeg, node[0].lonDeg, node[0].altMeters, node[0].parcelWtLbs, node[0].serviceTimeTruck, node[0].serviceTimeUAV, node[0].address)
	for j in customersTruck:
		newNode[j] = make_node(node[j].nodeType, node[j].latDeg, node[j].lonDeg, node[j].altMeters, node[j].parcelWtLbs, node[j].serviceTimeTruck, node[j].serviceTimeUAV, node[j].address)

	newNode[c+1] = make_node(node[0].nodeType, node[0].latDeg, node[0].lonDeg, node[0].altMeters, node[0].parcelWtLbs, node[0].serviceTimeTruck, node[0].serviceTimeUAV, node[0].address)

	[TSPobjVal, TSPassignments, TSPpackages, TSPtour] = make_TSP_package(currentTSP, c, C, newNode, tau, sigma)

	totalCost = getTotalCost(TSPtour, tau, sigma, V, C, sL, sR)

	if (TSPtour not in prevTSPtours):

		prevTSPtours.append(TSPtour)
		foundTSP = True

	elif ((requireUniqueTSP) and (TSPtour in prevTSPtours)):

		foundTSP = False
		action = None

		if (not foundTSP):

			newTruckCust = None
			newUAVcust = None
			minCost = float('inf')
			for tmpIndex in range(0,len(TSPtour)-3):
				i = TSPtour[tmpIndex]
				j = TSPtour[tmpIndex+1]
				k = TSPtour[tmpIndex+2]
				foundEligible = False
				for v in V:
					if node[j].parcelWtLbs <= vehicle[v].capacityLbs:
						foundEligible = True
						break
				if (foundEligible):
					for l in customersUAV:

						tmpTSPtour = list(TSPtour)
						tmpTSPtour[tmpIndex + 1] = l
						if (tmpTSPtour not in prevTSPtours):
							tmpCost = (tau[i][l] + tau[l][k] - (tau[i][j] + tau[j][k]))
							if (tmpCost < minCost):

								failed2reach = []

								droners = list( set(C) - set(tmpTSPtour) )

								for temp_j in droners:

									canServe = False
									for iii in range(0, len(tmpTSPtour)-1):
										temp_i = tmpTSPtour[iii]
										for kkk in [iii+1]:
											temp_k = tmpTSPtour[kkk]
											for temp_v in V:
												if ([temp_i,temp_k] in Pprime[temp_v][temp_j]):
													canServe = True
													break
									if not canServe:
										failed2reach.append(temp_j)

								if (len(failed2reach) == 0):
									if len(V) >= 1:
										[p3_status, p3_infeas_cust] = checkP2Feasibility(droners, tmpTSPtour, V, Pprime)
										if (p3_status == 1) and (len(tmpTSPtour) - 2 >= lowerTruckLimit):

											minCost = tmpCost
											newTruckCust = l
											newUAVcust = j
											insertionIndex = tmpIndex + 1
											action = 'truckuavswap'
									else:
										if (len(tmpTSPtour) - 2 >= lowerTruckLimit):

											minCost = tmpCost
											newTruckCust = l
											newUAVcust = j
											insertionIndex = tmpIndex + 1
											action = 'truckuavswap'

			bestTour = []
			for tmpIndex in range(1,len(TSPtour)-2):
				i = TSPtour[tmpIndex-1]
				j = TSPtour[tmpIndex]
				k = TSPtour[tmpIndex+1]
				l = TSPtour[tmpIndex+2]

				tmpTSPtour = list(TSPtour)
				tmpTSPtour[tmpIndex] = k
				tmpTSPtour[tmpIndex+1] = j
				if (tmpTSPtour not in prevTSPtours):
					tmpCost = tau[i][k] + tau[k][j] + tau[j][l] - tau[i][j] - tau[j][k] - tau[k][l]

					if (tmpCost < minCost):

						failed2reach = []

						droners = list( set(C) - set(tmpTSPtour) )

						for temp_j in droners:

							canServe = False
							for iii in range(0, len(tmpTSPtour)-1):
								temp_i = tmpTSPtour[iii]
								for kkk in [iii+1]:
									temp_k = tmpTSPtour[kkk]
									for temp_v in V:
										if ([temp_i,temp_k] in Pprime[temp_v][temp_j]):
											canServe = True
											break
							if not canServe:
								failed2reach.append(temp_j)

						if (len(failed2reach) == 0):
							if len(V) >= 1:
								[p3_status, p3_infeas_cust] = checkP2Feasibility(droners, tmpTSPtour, V, Pprime)
								if (p3_status == 1) and (len(tmpTSPtour) - 2 >= lowerTruckLimit):

									bestTour = list(tmpTSPtour)
									minCost = tmpCost
									action = 'subtour'
							else:
								if (len(tmpTSPtour) - 2 >= lowerTruckLimit):

									bestTour = list(tmpTSPtour)
									minCost = tmpCost
									action = 'subtour'

			tmpTSPtour = list(reversed(TSPtour))
			tmpTSPtour[0] = 0
			tmpTSPtour[-1] = c+1
			if (tmpTSPtour not in prevTSPtours):

				costOld = TSPobjVal
				[costNew, JUNKassignments, JUNKtour] = generateTSPinfo(tmpTSPtour, c, C, node, tau, sigma)

				if (costNew - costOld < minCost):

					failed2reach = []

					droners = list( set(C) - set(tmpTSPtour) )

					for temp_j in droners:

						canServe = False
						for iii in range(0, len(tmpTSPtour)-1):
							temp_i = tmpTSPtour[iii]
							for kkk in [iii+1]:
								temp_k = tmpTSPtour[kkk]
								for temp_v in V:
									if ([temp_i,temp_k] in Pprime[temp_v][temp_j]):
										canServe = True
										break
						if not canServe:
							failed2reach.append(temp_j)

					if (len(failed2reach) == 0):
						if len(V) >= 1:
							[p3_status, p3_infeas_cust] = checkP2Feasibility(droners, tmpTSPtour, V, Pprime)
							if (p3_status == 1) and (len(tmpTSPtour) - 2 >= lowerTruckLimit):

								entireTSPtour = list(tmpTSPtour)
								minCost = costNew - costOld
								action = 'entire'
						else:
							if (len(tmpTSPtour) - 2 >= lowerTruckLimit):

								entireTSPtour = list(tmpTSPtour)
								minCost = costNew - costOld
								action = 'entire'

			if (action == 'truckuavswap'):
				foundTSP = True
				customersTruck.remove(newUAVcust)
				customersTruck.append(newTruckCust)
				customersUAV.remove(newTruckCust)
				customersUAV.append(newUAVcust)

				tmpTSPtour = list(TSPtour)
				tmpTSPtour[insertionIndex] = newTruckCust

				prevTSPtours.append(tmpTSPtour)
				[TSPobjVal, TSPassignments, TSPtour] = generateTSPinfo(tmpTSPtour, c, C, node, tau, sigma)

			elif (action == 'subtour'):
				prevTSPtours.append(bestTour)
				[TSPobjVal, TSPassignments, TSPtour] = generateTSPinfo(bestTour, c, C, node, tau, sigma)

				if (TSPobjVal < bestOFV):
					foundTSP = True

			elif (action == 'entire'):
				prevTSPtours.append(entireTSPtour)
				[TSPobjVal, TSPassignments, TSPtour] = generateTSPinfo(entireTSPtour, c, C, node, tau, sigma)

				if (TSPobjVal < bestOFV):
					foundTSP = True

			else:

				foundTSP = False

	else:

		foundTSP = False

	return (bestCost, customersTruck, customersUAV, TSPobjVal, TSPassignments, TSPpackages, TSPtour, foundTSP, prevTSPtours, basicPreviousTSP)
