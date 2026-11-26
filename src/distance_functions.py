from __future__ import division
from math import *
from collections import defaultdict
import os
import numpy as np
import networkx as nx

radius_of_earth = 6378100.0
DIST_TOL = 1.0
ALT_TOL = 1.0

MODE_CAR 		= 1
MODE_BIKE 		= 2
MODE_WALK 		= 3
MODE_FLY 		= 4

METERS_PER_MILE = 1609.34

def calcMultirotorTravelTime(takeoffSpeed, cruiseSpeed, landSpeed, yawRateDeg, initAlt, flightAlt, goalAlt, initLatDeg, initLongDeg, goalLatDeg, goalLongDeg, initHeadingDeg, goalHeadingDeg, G_air, G_ground, init_node, goal_node):

	initx = initLatDeg
	inity = initLongDeg
	initz = initAlt
	goalx = goalLatDeg
	goaly = goalLongDeg
	goalz = goalAlt

	takeoffTime = 0.0
	flyTime = 0.0
	landTime = 0.0
	totalTime = 0.0

	takeoffDistance = 0.0
	flyDistance = 0.0
	landDistance = 0.0
	totalDistance = 0.0
	totalpower = 0.0

	distanceToGoal = AirDistance(initx, inity, initz, goalx, goaly, goalz)
	if (distanceToGoal <= DIST_TOL):

		initx = goalx
		inity = goaly
		initz = goalz

	if (([initx, inity, initz] == [goalx, goaly, goalz])):
		totalTime = 0.0
		myPath = [init_node.map_key, goal_node.map_key]
	else:

		if ([initx, inity] != [goalx, goaly]):
			init_deltaAlt = initz
			init_node_alt = init_node.map_position[2]
			init_deltaAlt = init_deltaAlt - init_node_alt
			totalTime += (abs(init_deltaAlt)/takeoffSpeed)
			takeoffTime += (abs(init_deltaAlt)/takeoffSpeed)
			takeoffDistance += abs(init_deltaAlt)
			totalDistance += abs(init_deltaAlt)

		init_key = init_node.index
		goal_key = goal_node.index
		myDistance = nx.shortest_path_length(G_air, source=init_key, target=goal_key, weight='weight')
		myPath = nx.shortest_path(G_air, source=init_key, target=goal_key, weight='weight')

		totalTime += myDistance/cruiseSpeed
		flyTime += myDistance/cruiseSpeed
		flyDistance += myDistance
		totalDistance += myDistance

		goal_node_alt = goal_node.map_position[2]
		goal_deltaAlt = goalz - goal_node_alt
		totalTime += (abs(goal_deltaAlt)/landSpeed)
		landTime += (abs(goal_deltaAlt)/landSpeed)
		landDistance += abs(goal_deltaAlt)
		totalDistance += abs(goal_deltaAlt)

		myPath.insert(0, init_node.map_key)
		myPath.append(goal_node.map_key)

	return [takeoffTime, flyTime, landTime, totalTime, takeoffDistance, flyDistance, landDistance, totalDistance, myPath]

def groundDistanceStraight(lat1, long1, lat2, long2):

	distance = 2*radius_of_earth*asin( sqrt( pow(sin((lat2 - lat1)/2),2) + cos(lat1)*cos(lat2)*pow(sin((long2-long1)/2),2) ))

	return (distance)

def AirDistance(initx, inity, initz, goalx, goaly, goalz):
	distance = np.linalg.norm(np.array([initx, inity, initz]) - np.array([goalx, goaly, goalz]))
	return distance

def GroundDistance(initx, inity, goalx, goaly):
	distance = np.linalg.norm(np.array([initx, inity]) - np.array([goalx, goaly]))
	return distance
