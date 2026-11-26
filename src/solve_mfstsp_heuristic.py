import sys
import time
import datetime
import math
from parseCSV import *
from gurobipy import *
from collections import defaultdict
import copy
from initialize import init_agent, initialize_drone_vehicle_assignments
from create_vehicle_route import *

import os
import json
import numpy as np
import pandas as pd

from mfstsp_heuristic_1_partition import *
from mfstsp_heuristic_2_asgn_uavs import *
from mfstsp_heuristic_3_timing import *

from local_search import *
from rm_node_sort_node import rm_empty_node

import main
import endurance_calculator
import distance_functions

import random

NODE_TYPE_DEPOT	= 0
NODE_TYPE_CUST	= 1

TYPE_TRUCK 		= 1
TYPE_UAV 		= 2

METERS_PER_MILE = 1609.34

_HERE = os.path.dirname(os.path.abspath(__file__))
SAVE_DIR_TOTAL = os.path.join(_HERE, "saved_solutions", "data_total")

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

def solve_mfstsp_heuristic(
    node, vehicle, air_matrix, ground_matrix, air_node_types, ground_node_types, numPoints,
    numUAVs, numTrucks, uav_travel, veh_travel, veh_distance, G_air, G_ground,
    customer_time_windows_h, early_arrival_cost, late_arrival_cost,
    problemName, max_iterations, loop_iterations,
    run_tag=None, algorithm=None, seed=None, algo_seed=None, instance_name=None):

	C 			= []
	tau			= defaultdict(make_dict)
	xtauprimeF	= defaultdict(make_dict)
	xtauprimeE	= defaultdict(make_dict)
	xspeedF		= defaultdict(make_dict)
	xspeedE 		= defaultdict(make_dict)
	xeee 		= defaultdict(make_dict)
	xeeePrime 	= defaultdict(make_dict)
	V			= []
	T = []
	N = []
	sL			= defaultdict(make_dict)
	sR			= defaultdict(make_dict)

	sigma		= {}
	sigmaprime	= {}

	if seed is not None:
		try:
			import random
			import numpy as np
			random.seed(int(seed))
			np.random.seed(int(seed))
		except Exception:
			pass

	for nodeID in node:
		if (node[nodeID].nodeType == 'CUSTOMER'):
			C.append(nodeID)

	for vehicleID in vehicle:
		if (vehicle[vehicleID].vehicleType == TYPE_UAV):
			V.append(vehicleID)
		else:
			T.append(vehicleID)

	c = len(C)
	for key, nodeID in node.items():
		if nodeID.nodeType == 'DEPOT':
			DEPOT_nodeID = nodeID.index
			break

	for nodeID in node:
		if (node[nodeID].nodeType == 'VTP'):
			N.append(nodeID)
	N_zero = [DEPOT_nodeID] + N.copy()
	N_plus = N.copy() + [DEPOT_nodeID]
	N_total = [DEPOT_nodeID] + N.copy() + [DEPOT_nodeID]

	A_vtp = []
	A_cust = []
	A_aerial_relay_node = []

	for key, nodeID in node.items():
		if node[key].nodeType == 'DEPOT':
			DEPOT_nodeID = key
		if node[key].nodeType == 'Aerial Relay Node':
			A_aerial_relay_node.append(key)
		if node[key].nodeType == 'VTP Takeoff/Landing Node':
			A_vtp.append(key)
		if node[key].nodeType == 'CUSTOMER Takeoff/Landing Node':
			A_cust.append(key)

	A_total = A_aerial_relay_node+A_vtp+A_cust
	A_cvtp = A_vtp+A_cust
	A_c = A_cust

	xP = {}
	_fast_xeee_src = {}
	for v in V:
		xP[v] = []
		_fv = vehicle[v]
		_fkey = (_fv.takeoffSpeed, _fv.cruiseSpeed, _fv.landingSpeed, _fv.yawRateDeg, _fv.cruiseAlt, _fv.batteryPower, _fv.capacityLbs)
		_fsrc = _fast_xeee_src.get(_fkey)
		if _fsrc is not None:
			_fx = xeee[_fsrc]
			for i in A_vtp:
				for j in A_cust:
					if ((j != i) and (node[j].parcelWtLbs <= vehicle[v].capacityLbs)):
						for k in A_vtp:
							if (k != i) and (k != j):
								xeee[v][i][j][k] = _fx[i][j][k]
								if (xeee[v][i][j][k] >= 0):
									xP[v].append([v,i,j,k])
			continue
		_fast_xeee_src[_fkey] = v
		for i in A_vtp:
			for j in A_cust:
				if ((j != i) and (node[j].parcelWtLbs <= vehicle[v].capacityLbs)):
					for k in A_vtp:
						if (k != i) and (k != j):

							xeee[v][i][j][k] = endurance_calculator.give_endurance(node, vehicle, uav_travel, v, i, j, k, uav_travel[v][i][j].totalTime, uav_travel[v][j][k].totalTime, vehicle[v].cruiseSpeed, vehicle[v].cruiseSpeed)
							if (xeee[v][i][j][k] >= 0):
								xP[v].append([v,i,j,k])

	for v in V:
		for i in N:
			sL[v][i] = vehicle[v].launchTime

	for v in V:
		for k in N:
			sR[v][k] = vehicle[v].recoveryTime

	vehicle = initialize_drone_vehicle_assignments(vehicle, V, T)

	sigma[0] = 0.0
	for k in A_cust:
		sigmaprime[k] = node[k].serviceTimeUAV/60

	numTrucks = len(T)
	numUAVs = len(V)
	numCustomers = len(C)

	solveAlns = algorithm

	ROOT_BASE_DIR = os.path.join(_HERE, "saved_solutions")

	SUMMARY_PARENT = f"{solveAlns}_C{numCustomers}result"
	SUMMARY_FOLDER = f"{solveAlns}_T{numUAVs}_U{numCustomers}"

	os.makedirs(ROOT_BASE_DIR, exist_ok=True)

	parent_dir = os.path.join(ROOT_BASE_DIR, SUMMARY_PARENT)
	if not os.path.isdir(parent_dir):
		os.makedirs(parent_dir, exist_ok=True)

	summary_dir = os.path.join(parent_dir, SUMMARY_FOLDER)
	if not os.path.isdir(summary_dir):
		os.makedirs(summary_dir, exist_ok=True)

	results_all = init_results_framework([algorithm])
	iter_num = 1

	best_total_cost_list = np.array([])
	best_uav_plan_list = []
	best_customer_plan_list = []
	best_time_uav_task_dict_list = []
	best_vehicle_plan_time_list = []
	best_vehicle_task_data_list = []
	best_global_reservation_table_list = []

	allow_uav_congestion = False
	for iter in range(loop_iterations):

		init_total_cost, init_uav_plan, init_customer_plan, init_time_uav_task_dict, init_uav_cost, init_vehicle_route, init_vehicle_plan_time, init_vehicle_task_data, init_global_reservation_table=initial_route(node, DEPOT_nodeID,
		 V, T, vehicle, uav_travel, veh_distance, veh_travel,
		N, N_zero, N_plus, A_total, A_cvtp, A_vtp,
		A_aerial_relay_node, G_air, G_ground,air_matrix, ground_matrix, air_node_types, ground_node_types, A_c, xeee, customer_time_windows_h, early_arrival_cost, late_arrival_cost, numPoints, numTrucks, numUAVs)

		initial_state = create_fast_initial_state(
			init_total_cost, init_uav_plan, init_customer_plan, init_uav_cost,
			init_time_uav_task_dict, init_vehicle_route,
			init_vehicle_plan_time, init_vehicle_task_data,
			init_global_reservation_table, node, DEPOT_nodeID,
			V, T, vehicle, uav_travel, veh_distance, veh_travel, N, N_zero, N_plus, A_total, A_cvtp, A_vtp, A_aerial_relay_node, G_air, G_ground,
			air_matrix, ground_matrix, air_node_types, ground_node_types, A_c, xeee,
			customer_time_windows_h, early_arrival_cost, late_arrival_cost
		)
		H_alns_initial_state = initial_state.fast_copy()
		T_alns_initial_state = initial_state.fast_copy()

		if algorithm in ("HP_ALNS", "BC_ALNS"):
			from bc_alns import solve_with_bc_alns
			(H_alns_best_state, H_alns_best_final_state, H_alns_best_objective, H_alns_best_final_objective, H_alns_best_final_uav_cost,
			H_alns_best_final_win_cost, H_alns_best_total_win_cost, H_alns_best_final_global_max_time, H_alns_best_global_max_time, H_alns_best_window_total_cost,
			H_alns_best_total_uav_tw_violation_cost, H_alns_best_total_vehicle_cost, H_alns_elapsed_time, H_alns_win_cost, H_alns_uav_route_cost, H_alns_vehicle_route_cost,
			H_alns_final_uav_cost, H_alns_final_total_list, H_alns_final_win_cost, H_alns_final_total_objective, H_alns_y_cost, H_alns_y_best, H_alns_work_time,
			H_alns_final_work_time) = solve_with_bc_alns(
				H_alns_initial_state, node, DEPOT_nodeID, V, T, vehicle, uav_travel, veh_distance, veh_travel,
				N, N_zero, N_plus, A_total, A_cvtp, A_vtp,
				A_aerial_relay_node, G_air, G_ground, air_matrix, ground_matrix,
				air_node_types, ground_node_types, A_c, xeee,
				customer_time_windows_h, early_arrival_cost, late_arrival_cost, problemName,
				iter=iter, max_iterations=max_iterations, max_runtime=30, summary_dir=summary_dir, use_incremental=True, seed=seed, algo_seed=algo_seed
			)
			record_one_run(
			results_all[algorithm],

			best_state_list=H_alns_best_state,
			best_final_state_list=H_alns_best_final_state,

			best_objective_list=H_alns_best_objective,
			best_final_objective_list=H_alns_best_final_objective,
			best_final_uav_cost_list=H_alns_best_final_uav_cost,
			best_final_win_cost_list=H_alns_best_final_win_cost,
			best_total_win_cost_list=H_alns_best_total_win_cost,
			best_final_global_max_time_list=H_alns_best_final_global_max_time,
			best_global_max_time_list=H_alns_best_global_max_time,
			best_window_total_cost_list=H_alns_best_window_total_cost,
			best_total_uav_tw_violation_cost_list=H_alns_best_total_uav_tw_violation_cost,
			best_total_vehicle_cost_list=H_alns_best_total_vehicle_cost,
			elapsed_time_list=H_alns_elapsed_time,

			win_cost_curve_list=H_alns_win_cost,
			uav_route_cost_curve_list=H_alns_uav_route_cost,
			vehicle_route_cost_curve_list=H_alns_vehicle_route_cost,
			final_uav_cost_curve_list=H_alns_final_uav_cost,
			final_total_list_curve_list=H_alns_final_total_list,
			final_win_cost_curve_list=H_alns_final_win_cost,
			final_total_objective_curve_list=H_alns_final_total_objective,
			y_cost_curve_list=H_alns_y_cost,
			y_best_curve_list=H_alns_y_best,
			work_time_curve_list=H_alns_work_time,
			final_work_time_curve_list=H_alns_final_work_time,
			)
			print(f"HP-ALNS_finished_{iter}_iterations")
		else:
			raise ValueError(f"Unsupported algorithm: {algorithm}")
	base_dir_route = SAVE_DIR_TOTAL

	folder_name = f"{len(C)}customers{algorithm}"
	target_dir = os.path.join(base_dir_route, folder_name)

	os.makedirs(target_dir, exist_ok=True)

	export_name = f"{algorithm}_{problemName}_C_{len(C)}_{run_tag}"

	export_results_to_excel(results_all, export_name, save_dir=target_dir)
	print(f"H-ALNS求解完成，保存到文件中。")

def _to_py(obj):

    if obj is None:
        return None
    if isinstance(obj, (np.integer, np.int64, np.int32)):
        return int(obj)
    if isinstance(obj, (np.floating, np.float64, np.float32)):
        return float(obj)
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    return obj

def _jsonify(obj):

    obj = _to_py(obj)
    if isinstance(obj, (list, dict)):
        return json.dumps(obj, ensure_ascii=False)
    return obj

def init_results_framework(algo_names):

    results = {}
    for algo in algo_names:
        results[algo] = {

            "best_state_list": [],
            "best_final_state_list": [],

            "best_objective_list": [],
            "best_final_objective_list": [],
            "best_final_uav_cost_list": [],
            "best_final_win_cost_list": [],
            "best_total_win_cost_list": [],
            "best_final_global_max_time_list": [],
            "best_global_max_time_list": [],
            "best_window_total_cost_list": [],
            "best_total_uav_tw_violation_cost_list": [],
            "best_total_vehicle_cost_list": [],
            "elapsed_time_list": [],

            "win_cost_curve_list": [],
            "uav_route_cost_curve_list": [],
            "vehicle_route_cost_curve_list": [],
            "final_uav_cost_curve_list": [],
            "final_total_list_curve_list": [],
            "final_win_cost_curve_list": [],
            "final_total_objective_curve_list": [],
            "y_cost_curve_list": [],
            "y_best_curve_list": [],
            "work_time_curve_list": [],
            "final_work_time_curve_list": [],
        }
    return results

def record_one_run(results_for_algo, **kwargs):

    for k, v in kwargs.items():
        if k not in results_for_algo:

            results_for_algo[k] = []
        results_for_algo[k].append(_to_py(v))

def export_results_to_excel(results_by_algo, problemName, save_dir=None):
    if save_dir is None:
        save_dir = SAVE_DIR_TOTAL
    """
    把所有算法导出到同一个 Excel：
      - 每个算法两个 sheet：{algo}_summary 和 {algo}_curves
      - summary：每次 iter 一行（曲线字段写成 JSON 字符串，便于后续解析）
      - curves：长表(run_id, gen, 指标...) 方便画迭代曲线
    """
    os.makedirs(save_dir, exist_ok=True)
    xlsx_path = os.path.join(save_dir, f"{problemName}_data_total.xlsx")

    with pd.ExcelWriter(xlsx_path, engine="openpyxl") as writer:
        for algo, data in results_by_algo.items():

            n_runs = len(data.get("best_objective_list", []))
            summary_rows = []
            for i in range(n_runs):
                row = {"run_id": i, "algo": algo}
                for key, lst in data.items():
                    if key in ("best_state_list", "best_final_state_list"):
                        continue
                    if not isinstance(lst, list) or i >= len(lst):
                        continue
                    row[key] = _jsonify(lst[i])
                summary_rows.append(row)
            df_summary = pd.DataFrame(summary_rows)

            curve_keys = [
                "win_cost_curve_list",
                "uav_route_cost_curve_list",
                "vehicle_route_cost_curve_list",
                "final_uav_cost_curve_list",
                "final_total_list_curve_list",
                "final_win_cost_curve_list",
                "final_total_objective_curve_list",
                "y_cost_curve_list",
                "y_best_curve_list",
                "work_time_curve_list",
                "final_work_time_curve_list",
            ]

            curve_rows = []
            for run_id in range(n_runs):

                max_len = 0
                run_curves = {}
                for ck in curve_keys:
                    curves_all_runs = data.get(ck, [])
                    curve = curves_all_runs[run_id] if run_id < len(curves_all_runs) else None
                    curve = _to_py(curve)
                    if curve is None:
                        run_curves[ck] = None
                        continue
                    if not isinstance(curve, list):
                        curve = [curve]
                    run_curves[ck] = curve
                    max_len = max(max_len, len(curve))

                for gen in range(max_len):
                    row = {"run_id": run_id, "gen": gen, "algo": algo}
                    for ck, curve in run_curves.items():

                        col = ck.replace("_curve_list", "")
                        if curve is None or gen >= len(curve):
                            row[col] = np.nan
                        else:
                            row[col] = _to_py(curve[gen])
                    curve_rows.append(row)

            df_curves = pd.DataFrame(curve_rows)

            sheet_summary = f"{algo}_summary"
            sheet_curves = f"{algo}_curves"
            df_summary.to_excel(writer, sheet_name=sheet_summary[:31], index=False)
            df_curves.to_excel(writer, sheet_name=sheet_curves[:31], index=False)

    print(f"[OK] 导出完成：{xlsx_path}")
    return xlsx_path
