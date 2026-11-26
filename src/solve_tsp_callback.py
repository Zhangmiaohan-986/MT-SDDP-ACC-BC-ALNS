import sys
import time
import datetime
import math
from gurobipy import *
from collections import defaultdict
from dfs import *

import os

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

def solve_tsp_callback(node, vehicle, travel):

	assignments	= defaultdict(make_dict)
	packages	= defaultdict(make_dict)

	C 			= []
	N			= []
	N_zero		= []
	N_plus		= []
	tau			= defaultdict(make_dict)
	sigma		= {}

	c = 0
	N.append(0)
	N_zero.append(0)
	for nodeID in node:
		if (node[nodeID].nodeType == NODE_TYPE_CUST):
			C.append(nodeID)
			N.append(nodeID)
			N_zero.append(nodeID)
			N_plus.append(nodeID)
			if (nodeID > c):
				c = nodeID

	N.append(c+1)
	N_plus.append(c+1)

	node[c+1] = make_node(node[0].nodeType, node[0].latDeg, node[0].lonDeg, node[0].altMeters, node[0].parcelWtLbs, node[0].serviceTimeTruck, node[0].serviceTimeUAV, node[0].address)

	for k in N_plus:
		if (k == c+1):
			sigma[k] = 0.0
		else:
			sigma[k] = node[k].serviceTimeTruck

	cost = {}

	for vehicleID in vehicle:
		for i in N_zero:
			for j in C:
				if (vehicle[vehicleID].vehicleType == TYPE_TRUCK):
					tau[i][j] = travel[vehicleID][i][j].totalTime
					cost[i,j] = travel[vehicleID][i][j].totalTime

			if (vehicle[vehicleID].vehicleType == TYPE_TRUCK):
				tau[i][c+1] = travel[vehicleID][i][0].totalTime
				cost[i,0] = travel[vehicleID][i][0].totalTime

	def subtourelim(model, where):

		if where == GRB.Callback.MIPSOL:

			delta_dfs = defaultdict(list)

			for i in N_zero:
				for j in N_zero:
					if i != j:
						x_sol = model.cbGetSolution(x[i,j])
						if x_sol > 0.5:
							delta_dfs[i].append(j)
							delta_dfs[j].append(i)

			component = dfs(N_zero, delta_dfs)

			if (len(component) > 1):

				for k in component:
					model.cbLazy(quicksum((quicksum(x[i,j] for j in component[k] if j != i)) for i in component[k]) <= len(component[k]) - 1)

	m = Model("dfj_callback")

	m.params.OutputFlag = 0

	x = {}

	for i in N_zero:
		for j in N_zero:
			if i != j:
				x[i,j] = m.addVar(lb=0, obj=float(cost[i,j]), vtype=GRB.BINARY, name="x.%f.%f" % (i,j))

	m.modelSense = GRB.MINIMIZE

	m.Params.timeLimit = 600

	m.update()

	for i in N_zero:
		m.addConstr(quicksum(x[i,j] for j in N_zero if j != i) == 1, "Constr.1.%f" % (i))

	for j in N_zero:
		m.addConstr(quicksum(x[i,j] for i in N_zero if i != j) == 1, "Constr.2.%f" % (j))

	m.Params.lazyConstraints = 1
	m.optimize(subtourelim)

	myTour = [0]

	tmp_i = 0

	for tmploop in range(1,len(N_zero)):
		for tmp_j in N_zero:
			if tmp_i != tmp_j:
				if x[tmp_i,tmp_j].x > 0.9:
					myTour.append(tmp_j)
					tmp_i = tmp_j
					break

	myTour.append(c+1)

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
