import sys
import datetime
import time
import math
from collections import defaultdict
import pandas as pd
from utils_shared import *
from parseCSV import *
from parseCSVstring import *

from gurobipy import *
import os
import os.path
from subprocess import call
from task_data import *
from cost_y import *
from call_function import *
from initialize import *
from cbs_plan import *

from down_data import *
from solve_mfstsp_heuristic import *

import distance_functions

from generate_test_problems import *

startTime 		= time.time()

METERS_PER_MILE = 1609.34

UAVSpeedTypeString = {1: 'variable', 2: 'maximum', 3: 'maximum-range'}

NODE_TYPE_DEPOT	= 0
NODE_TYPE_CUST	= 1

TYPE_TRUCK 		= 1
TYPE_UAV 		= 2

SEED = 6
Z_COORD = 0.05
UAV_DISTANCE = 25

def make_dict():
	return defaultdict(make_dict)

class make_node:
	def __init__(self, index, nodeType, latDeg, lonDeg, altMeters, parcelWtLbs, serviceTimeTruck, serviceTimeUAV, map_key, map_type, map_position, map_index):

		self.index				= index
		self.nodeType 			= nodeType
		self.latDeg 			= latDeg
		self.lonDeg				= lonDeg
		self.altMeters			= altMeters
		self.position			= (latDeg, lonDeg, altMeters)
		self.parcelWtLbs 		= parcelWtLbs
		self.serviceTimeTruck	= serviceTimeTruck
		self.serviceTimeUAV 	= serviceTimeUAV
		if nodeType == 'DEPOT':
			self.map_key 			= index
			self.map_type 			= 'DEPOT'
			self.map_position 		= (latDeg, lonDeg, altMeters)
			self.map_index 			= map_index
		else:
			self.map_key 			= map_key
			self.map_type 			= map_type
			self.map_position 		= map_position
			self.map_index 			= map_index

class make_vehicle:
	def __init__(self, id, vehicleType, takeoffSpeed, cruiseSpeed, landingSpeed, yawRateDeg, cruiseAlt, capacityLbs, launchTime, recoveryTime, serviceTime, batteryPower, flightRange, vehicleSpeed, maxDrones, per_cost,fix_cost=10):

		self.vehicleType	= vehicleType

		self.takeoffSpeed	= takeoffSpeed * 3.6

		self.cruiseSpeed	= vehicleSpeed
		self.landingSpeed	= landingSpeed * 3.6
		self.yawRateDeg		= yawRateDeg
		self.cruiseAlt		= cruiseAlt
		self.capacityLbs	= capacityLbs
		self.launchTime		= launchTime / 3600
		self.recoveryTime	= recoveryTime / 3600
		self.serviceTime	= serviceTime / 60
		self.batteryPower	= batteryPower
		self.flightRange	= flightRange

		self.vehicleSpeed	= vehicleSpeed
		self.per_cost = per_cost
		self.fix_cost = fix_cost
		self.time_cost = 2

		self.id = id
		self.route = defaultdict(make_dict)
		self.current_time = 0
		self.current_node = None
		self.entry_node_time = defaultdict(make_dict)
		self.exit_node_time = defaultdict(make_dict)
		self.dict_vehicle = {}
		if self.vehicleType == TYPE_TRUCK:
			self.drones = defaultdict(make_dict)
			self.launch_records = defaultdict(make_dict)
			self.recover_records = defaultdict(make_dict)
			self.available_drones = defaultdict(make_dict)
			self.maxDrones = maxDrones
		elif self.vehicleType == TYPE_UAV:
			self.is_available = True
			self.returnable_nodes = []
			self.violate = False

	def add_drone(self, drone):
		self.drones[drone.id] = drone
	def remove_drone(self, drone):
		del self.drones[drone.id]

class make_travel:
	def __init__(self, takeoffTime, flyTime, landTime, totalTime, takeoffDistance, flyDistance, landDistance, totalDistance, path):

		self.takeoffTime 	 = takeoffTime
		self.flyTime 		 = flyTime
		self.landTime 		 = landTime
		self.totalTime 		 = totalTime
		self.takeoffDistance = takeoffDistance
		self.flyDistance	 = flyDistance
		self.landDistance	 = landDistance
		self.totalDistance	 = totalDistance
		self.path			 = path

class missionControl():
	def __init__(self,config=None):

		timestamp = datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d %H:%M:%S')

		self.base_dir = os.path.dirname(os.path.abspath(__file__))

		if config:
			self.problemName = config.get('problem_name', 'case_001')
			numUAVs = config.get('num_uavs', 6)
			numTrucks = config.get('num_trucks', 3)
			per_uav_cost = config.get('per_uav_cost', 1)
			per_vehicle_cost = config.get('per_vehicle_cost', 2)
			early_arrival_cost = config.get('early_arrival_cost', [5, 0.083])
			late_arrival_cost = config.get('late_arrival_cost', [20, 0.333])
			max_Drones = config.get('max_drones', 10)
			num_points = config.get('num_points', 100)
			self.loop_iterations = config.get('loop_iterations', 300)
			self.max_iterations = config.get('iterations', 500)
			self.instance_name = config.get('save_name', 'default_experiment')
			self.air_node_num, self.ground_node_num, self.customer_node_num = config.get('split_ratio', (1/3, 1/3, 1/3))
			self.vehicleFileID = 1
			self.seed = int(config.get("seed", 6))
			self.Z_coord = float(config.get("Z_coord", 0.05))
			self.uav_distance = float(config.get("uav_distance", 15))
			self.algo_seed = config.get("algo_seed", None)
			self.Z_coord = config.get("Z_coord", 0.05)
			self.uav_distance = config.get("uav_distance", 15)

			SEED = self.seed
			Z_COORD = self.Z_coord
			UAV_DISTANCE = self.uav_distance

			random.seed(self.seed)
			np.random.seed(self.seed)

			self.run_tag = config.get("run_tag", "default_experiment")
			self.algorithm = config.get("algorithm_name", "HP_ALNS")

			UAVSpeedType = 1
		else:

			self.problemName = "case_001"
			numUAVs = 6
			numTrucks = 3
			per_uav_cost = 1
			per_vehicle_cost = 2
			max_Drones = 10
			num_points = 100
			early_arrival_cost = [5, 0.083]
			late_arrival_cost = [20, 0.333]
			self.max_iterations = 50
			self.instance_name = "case_001"
			self.vehicleFileID = 1
			UAVSpeedType = 1

		self.vehiclesFile = os.path.join(self.base_dir, 'Problems', f'tbl_vehicles_tits.csv')

		seed = 6
		Z_coord = 0.05
		uav_distance = 15

		self.early_arrival_cost = early_arrival_cost
		self.late_arrival_cost = late_arrival_cost

		self.node = {}
		self.vehicle = {}
		self.uav_travel = defaultdict(make_dict)
		self.veh_travel = defaultdict(make_dict)
		self.veh_distance = defaultdict(make_dict)
		self.num_points = num_points

		self.readData(numUAVs, numTrucks, per_uav_cost, per_vehicle_cost, max_Drones, num_points)

		_fast_travel_src = {}
		for vehicleID in self.vehicle:
			if (self.vehicle[vehicleID].vehicleType == TYPE_UAV):
				_fv = self.vehicle[vehicleID]
				_fkey = (_fv.takeoffSpeed, _fv.cruiseSpeed, _fv.landingSpeed, _fv.yawRateDeg, _fv.cruiseAlt)
				_fsrc = _fast_travel_src.get(_fkey)
				for i in self.node:
					if self.node[i].nodeType == 'VTP Takeoff/Landing Node' or self.node[i].nodeType == 'CUSTOMER Takeoff/Landing Node':
						for j in self.node:
							if self.node[j].nodeType == 'VTP Takeoff/Landing Node' or self.node[j].nodeType == 'CUSTOMER Takeoff/Landing Node':
								if (j == i):
									self.uav_travel[vehicleID][i][j] = main.make_travel(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, [])
								elif _fsrc is not None:
									_ft = self.uav_travel[_fsrc][i][j]
									self.uav_travel[vehicleID][i][j] = main.make_travel(_ft.takeoffTime, _ft.flyTime, _ft.landTime, _ft.totalTime, _ft.takeoffDistance, _ft.flyDistance, _ft.landDistance, _ft.totalDistance, list(_ft.path))
								else:
									init_node = self.node[i]
									target_node = self.node[j]
									[takeoffTime, flyTime, landTime, totalTime, takeoffDistance, flyDistance, landDistance, totalDistance, path] = distance_functions.calcMultirotorTravelTime(self.vehicle[vehicleID].takeoffSpeed, self.vehicle[vehicleID].cruiseSpeed, self.vehicle[vehicleID].landingSpeed, self.vehicle[vehicleID].yawRateDeg, self.node[i].altMeters, self.vehicle[vehicleID].cruiseAlt, self.node[j].altMeters, self.node[i].latDeg, self.node[i].lonDeg, self.node[j].latDeg, self.node[j].lonDeg, -361, -361, self.G_air, self.G_ground, init_node, target_node)
									self.uav_travel[vehicleID][i][j] = main.make_travel(takeoffTime, flyTime, landTime, totalTime, takeoffDistance, flyDistance, landDistance, totalDistance, path)
				if _fsrc is None:
					_fast_travel_src[_fkey] = vehicleID

		for vehicleID in self.vehicle:
			if (self.vehicle[vehicleID].vehicleType == TYPE_TRUCK):
				for i in self.node:
					if self.node[i].nodeType == 'VTP' or self.node[i].nodeType == 'DEPOT':
						for j in self.node:
							if self.node[j].nodeType == 'VTP' or self.node[j].nodeType == 'DEPOT':
								if (j == i):
									self.veh_distance[vehicleID][i][j] = 0
									self.veh_distance[vehicleID][j][i] = 0
									self.veh_travel[vehicleID][i][j] = 0
									self.veh_travel[vehicleID][j][i] = 0
								else:
									init_node =  self.node[i].map_index
									target_node = self.node[j].map_index
									distance = self.ground_matrix[init_node][target_node]
									self.veh_distance[vehicleID][i][j] = distance
									self.veh_travel[vehicleID][i][j] = distance / self.vehicle[vehicleID].vehicleSpeed

		print('Calling a Heuristic to solve VDCD-AC...')

		solve_mfstsp_heuristic(
			self.node, self.vehicle, self.air_matrix, self.ground_matrix,
			self.air_node_types, self.ground_node_types, self.num_points,
			numUAVs, numTrucks, self.uav_travel, self.veh_travel, self.veh_distance,
			self.G_air, self.G_ground, self.customer_time_windows_h,
			self.early_arrival_cost, self.late_arrival_cost,
			self.problemName, self.max_iterations, self.loop_iterations,
			run_tag=self.run_tag,
			algorithm=self.algorithm,
			seed=self.seed,
			algo_seed=self.algo_seed,
			instance_name=self.instance_name
		)
		print('所有任务完成')
		return

	def readData(self, numUAVs, numTrucks, per_uav_cost, per_vehicle_cost, max_Drones, num_points):

		if int(numTrucks) > 10:
			self.vehiclesFile = os.path.join(self.base_dir, 'Problems', 'tbl_vehicles_tits_xl.csv')
		set_max_dict_vehicle_id(numTrucks)

		tmpUAVs = 0
		tmpTrucks = 0
		rawData = parseCSVstring(self.vehiclesFile, returnJagged=False, fillerValue=-1, delimiter=',', commentChar='%')
		for i in range(0,len(rawData)):
			vehicleID 			= int(rawData[i][0])
			vehicleType			= int(rawData[i][1])
			takeoffSpeed		= float(rawData[i][2])
			cruiseSpeed			= float(rawData[i][3])
			landingSpeed		= float(rawData[i][4])
			yawRateDeg			= float(rawData[i][5])
			cruiseAlt			= float(rawData[i][6])
			capacityLbs			= float(rawData[i][7])
			launchTime			= float(rawData[i][8])
			recoveryTime		= float(rawData[i][9])
			serviceTime			= float(rawData[i][10])
			batteryPower		= float(rawData[i][11])
			flightRange			= str(rawData[i][12])
			maxDrones			= max_Drones
			if (vehicleType == TYPE_UAV):
				tmpUAVs += 1
				if (tmpUAVs <= numUAVs):

					vehicleSpeed = 55
					self.vehicle[vehicleID] = make_vehicle(vehicleID, vehicleType, takeoffSpeed, cruiseSpeed, landingSpeed, yawRateDeg, cruiseAlt, capacityLbs, launchTime, recoveryTime, serviceTime, batteryPower, flightRange, vehicleSpeed, maxDrones, per_uav_cost)
				else:
					break
			else:
				tmpTrucks += 1
				if (tmpTrucks <= numTrucks):
					vehicleSpeed = 30

					self.vehicle[vehicleID] = make_vehicle(vehicleID, vehicleType, takeoffSpeed, cruiseSpeed, landingSpeed, yawRateDeg, cruiseAlt, capacityLbs, launchTime, recoveryTime, serviceTime, batteryPower, flightRange, vehicleSpeed, maxDrones, per_vehicle_cost)

		if (tmpUAVs < numUAVs):
			print("WARNING: You requested %d UAVs, but we only have data on %d UAVs." % (numUAVs, tmpUAVs))
			print("\t We'll solve the problem with %d UAVs.  Sorry." % (tmpUAVs))

		G_air, G_ground, air_adj_matrix, air_positions, ground_adj_matrix, ground_positions, all_data, air_node_types, ground_node_types, customer_time_windows_h = generate_complex_network(num_points, SEED, Z_COORD, UAV_DISTANCE, self.air_node_num, self.ground_node_num, self.customer_node_num)

		air_ground_node_types =  merge_and_renumber_dicts(air_node_types, ground_node_types)
		air_ground_positions = merge_and_renumber_dicts(air_positions, ground_positions)
		len_air_ground_node_types = len(air_ground_node_types)
		len_air_node_types = len(air_node_types)
		len_ground_node_types = len(ground_node_types)
		serviceTimeTruck = 0
		for index, node_type in air_ground_node_types.items():
			nodeID = index
			nodeType = node_type
			latDeg = air_ground_positions[index][0]
			lonDeg = air_ground_positions[index][1]
			altMeters = air_ground_positions[index][2]

			if nodeType == 'CUSTOMER':
				parcelWtLbs = random.randint(1, 6)
				serviceTimeUAV = 5
			else:
				parcelWtLbs = 0
				serviceTimeUAV = 0
			address = ''

			map_result = find_same_xy_different_z(air_ground_positions, air_ground_positions[index])

			if map_result == None:
				map_key = None
				map_type = None
				map_position = None
				if index < len_air_node_types:
					map_index = index
				else:
					map_index = index - len_air_node_types
			else:
				map_key = map_result[0]

				map_position = map_result[2]
				map_type = air_ground_node_types[map_key]
				if index < len_air_node_types:
					map_index = index
				else:
					map_index = index - len_air_node_types

			self.node[nodeID] = make_node(index, nodeType, latDeg, lonDeg, altMeters, parcelWtLbs, serviceTimeTruck, serviceTimeUAV, map_key, map_type, map_position, map_index)

		self.air_matrix = air_adj_matrix
		self.ground_matrix = ground_adj_matrix
		self.air_node_types = air_node_types
		self.ground_node_types = ground_node_types
		self.G_air = G_air
		self.G_ground = G_ground

		self.customer_time_windows_h = customer_time_windows_h

if __name__ == '__main__':
	try:
		missionControl()
	except:
		print("There was a problem.  Sorry things didn't work out.  Bye.")
		raise
