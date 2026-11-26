import numpy as np
import random
from collections import defaultdict
from sklearn.cluster import KMeans
import copy
import networkx as nx
from collections import defaultdict
import matplotlib.pyplot as plt
from scipy.spatial import distance
from scipy.spatial.distance import cdist

from task_data import *
from cost_y import *
from call_function import *
from initialize import *
from cbs_plan import *

from down_data import *

from utils_shared import *

TASK_IDLE = 10
TASK_LOADING = 11
TASK_UNLOADING = 12
TASK_DRONE_LAUNCH = 13
TASK_DRONE_RECOVERY = 14
TASK_SERVICE = 15
TASK_CHARGING = 16
TASK_MAINTENANCE = 17

NODE_TYPE_DEPOT	= 0
NODE_TYPE_CUST	= 1

TYPE_TRUCK 		= 1
TYPE_UAV 		= 2

TASK_IDLE = 10
TASK_LOADING = 11
TASK_UNLOADING = 12
TASK_DRONE_LAUNCH = 13
TASK_DRONE_RECOVERY = 14
TASK_SERVICE = 15
TASK_CHARGING = 16
TASK_MAINTENANCE = 17
TASK_DRONE_FLIGHT = 18
TASK_DRONE_FLIGHT_BACK = 19

TASK_NAMES = {
    TASK_IDLE: "空闲待命",
    TASK_LOADING: "货物装载",
    TASK_UNLOADING: "货物卸载",
    TASK_DRONE_LAUNCH: "无人机发射",
    TASK_DRONE_RECOVERY: "无人机回收",
    TASK_SERVICE: "客户服务",
    TASK_CHARGING: "电池充电",
    TASK_MAINTENANCE: "车辆维护",
    TASK_DRONE_FLIGHT: "无人机飞行-前往客户点",
    TASK_DRONE_FLIGHT_BACK: "无人机飞行-返回回收点"
}
import os
import glob

def initial_route(node, DEPOT_nodeID, V, T, vehicle, uav_travel, veh_distance, veh_travel, N, N_zero, N_plus, A_total, A_cvtp, A_vtp, A_aerial_relay_node, G_air, G_ground, air_matrix, ground_matrix, air_node_types, ground_node_types, A_c, xeee, customer_time_windows_h, early_arrival_cost, late_arrival_cost, points_num, vehicle_num, uav_num):
    vtp_index = A_vtp

    vtp_coords = np.array([node[i].position for i in vtp_index])
    len_customer_num = len(A_c)

    num_trucks = len(T)
    num_clusters = min(num_trucks, len(vtp_index))

    generator = DiverseRouteGenerator(node, DEPOT_nodeID, A_vtp, V, T, vehicle, uav_travel, veh_distance, veh_travel, vtp_coords, num_clusters, G_air, G_ground, air_matrix, ground_matrix, air_node_types, ground_node_types, A_c, xeee, customer_time_windows_h, early_arrival_cost, late_arrival_cost)

    vehicle_task_data = generator._create_initial_vehicle_task_data()
    files = list_saved_files()

    input_filename = f"com_r_{len(A_c)}cust_{vehicle_num}v_{uav_num}d_{points_num}n"

    save_dir = SAVE_DIR_ABS

    os.makedirs(save_dir, exist_ok=True)
    saved_file_path = find_saved_solution_file(save_dir, input_filename)

    if saved_file_path is None:

        num_solutions = 1
        air_vtp_solutions, vehicle_candidate_solutions, total_important_vtps = generator.generate_diverse_solutions(num_solutions)

        best_customer_plan, best_uav_plan, best_plan_cost, best_vehicle_route, vehicle_task_data, vehicle_arrival_time, best_total_important_vtps = generator.generate_uav_solutions(vehicle_candidate_solutions, vehicle_task_data, total_important_vtps)
        if len(best_customer_plan) != len_customer_num:

            best_customer_plan, best_uav_plan, best_plan_cost, best_vehicle_route, vehicle_task_data, vehicle_arrival_time, best_total_important_vtps = generator.generate_greedy_uav_solutions(vehicle_candidate_solutions, vehicle_task_data, total_important_vtps)
        uav_task_dict = defaultdict(dict)
        for customer_id in best_customer_plan:
            drone_id, launch_node, customer_id, recovery_node, launch_vehicle, recovery_vehicle = best_customer_plan[customer_id]
            if drone_id not in uav_task_dict:
                uav_task_dict[drone_id] = []
            uav_task_dict[drone_id].append(best_customer_plan[customer_id])

        input_data = {
        'uav_task_dict': uav_task_dict,
        'best_customer_plan': best_customer_plan,
        'best_uav_plan': best_uav_plan,
        'best_vehicle_route': best_vehicle_route,
        'vehicle_task_data': vehicle_task_data,
        'vehicle_arrival_time': vehicle_arrival_time,
        'node': node,
        'DEPOT_nodeID': DEPOT_nodeID,
        'V': V,
        'T': T,
        'vehicle': vehicle,
        'uav_travel': uav_travel,
        'veh_distance': veh_distance,
        'veh_travel': veh_travel,
        'N': N,
        'N_zero': N_zero,
        'N_plus': N_plus,
        'A_total': A_total,
        'A_cvtp': A_cvtp,
        'A_vtp': A_vtp,
        'A_aerial_relay_node': A_aerial_relay_node,
        'G_air': G_air,
        'G_ground': G_ground,
        'air_matrix': air_matrix,
        'ground_matrix': ground_matrix,
        'air_node_types': air_node_types,
        'ground_node_types': ground_node_types,
        'A_c': A_c,
        'xeee': xeee,
        'best_total_important_vtps': best_total_important_vtps
        }

        custom_name = f"com_r_{len(A_c)}cust_{vehicle_num}v_{uav_num}d_{points_num}n"

        input_filename = save_input_data_with_name(input_data, custom_name)

    uav_task_dict, best_customer_plan, best_uav_plan, best_vehicle_route, vehicle_task_data, vehicle_arrival_time,node, DEPOT_nodeID, V, T, vehicle, uav_travel, veh_distance, veh_travel, N, N_zero, N_plus, A_total, A_cvtp, A_vtp, A_aerial_relay_node, G_air, G_ground, air_matrix, ground_matrix, air_node_types, ground_node_types, A_c, xeee = run_low_update_time_from_saved(input_filename)

    sorted_mission_keys = list(best_uav_plan.keys())
    time_uav_task_dict, time_customer_plan, time_uav_plan, vehicle_plan_time, vehicle_task_data = low_update_time(uav_task_dict, best_uav_plan,
    best_vehicle_route, vehicle_task_data, vehicle_arrival_time, node, V, T, vehicle, uav_travel)

    best_uav_plan, best_uav_cost, vehicle_plan_time, best_vehicle_task_data, global_reservation_table = rolling_time_cbs(vehicle_arrival_time,
    best_vehicle_route, time_uav_task_dict, time_customer_plan, time_uav_plan, vehicle_plan_time, vehicle_task_data, node, DEPOT_nodeID,
    V, T, vehicle, uav_travel, veh_distance, veh_travel, N, N_zero, N_plus, A_total, A_cvtp, A_vtp, A_aerial_relay_node, G_air, G_ground,
    air_matrix, ground_matrix, air_node_types, ground_node_types, A_c, xeee)

    best_total_cost = calculate_plan_cost(best_uav_cost, best_vehicle_route, vehicle, T, V, veh_distance)

    return  best_total_cost, best_uav_plan, best_customer_plan, time_uav_task_dict, best_uav_cost, best_vehicle_route, vehicle_plan_time, best_vehicle_task_data, global_reservation_table

import os
import glob

def find_saved_solution_file(save_dir: str, base_name: str,
                            prefixes=("input_data_", "input_")):

    save_dir = os.path.abspath(save_dir)

    candidates = []
    for pref in prefixes:
        name = f"{pref}{base_name}"
        candidates += glob.glob(os.path.join(save_dir, name + ".pkl"))
        candidates += glob.glob(os.path.join(save_dir, name + "_*.pkl"))

    if not candidates:
        return None

    candidates.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    return candidates[0]

class DiverseRouteGenerator:

    def __init__(self, node, depot_id, vtp_indices, uav_ids, truck_ids, vehicle, uav_travel, veh_distance, veh_travel, vtp_coords, num_clusters, G_air, G_ground, air_matrix, ground_matrix, air_node_types, ground_node_types, A_c, xeee, customer_time_windows_h, early_arrival_cost, late_arrival_cost):
        self.node = node
        self.depot_id = depot_id
        self.vtp_indices = vtp_indices
        self.uav_ids = uav_ids
        self.truck_ids = truck_ids
        self.vehicle = vehicle
        self.uav_travel = uav_travel
        self.veh_distance = veh_distance
        self.veh_travel = veh_travel
        self.num_trucks = len(truck_ids)
        self.num_uavs = len(uav_ids)
        self.num_clusters = num_clusters
        self.vtp_coords = vtp_coords
        self.G_air = G_air
        self.G_ground = G_ground
        self.air_matrix = air_matrix
        self.ground_matrix = ground_matrix
        self.air_node_types = air_node_types
        self.ground_node_types = ground_node_types
        self.A_c = A_c
        self.vtp_index = {}
        self.xeee = xeee
        self.customer_time_windows_h = customer_time_windows_h
        self.early_arrival_cost = early_arrival_cost
        self.late_arrival_cost = late_arrival_cost
        self.save_path = r'saved_solutions'

        self.depot_pos = np.array([node[depot_id].latDeg, node[depot_id].lonDeg, node[depot_id].altMeters])
        self.vtp_positions = vtp_coords

        self.depot_index = 0
        vtp_index = 0

        for node_id in self.node:
            if self.node[node_id].nodeType == 'VTP Takeoff/Landing Node':
                self.vtp_index[node_id] = vtp_index
                vtp_index += 1

        self._compute_distance_matrices()

    def  generate_uav_solutions(self, vehicle_candidate_solutions, vehicle_task_data, total_important_vtps):
        uav_solutions = defaultdict(dict)
        insert_costs = []
        customer_uav_plans = {}
        diversity_costs = []
        best_uav_solution = None
        best_insert_cost = float('inf')
        task_arrival_times = defaultdict(dict)
        customer_plan = []
        total_window_cost = []
        uav_plan = []
        plan_cost = []
        total_vehicle_task = []
        total_vehicle_route = []
        feasible_plans = []
        num_plan_cost = []
        vehicle_arrival_time = []
        new_current_vehicle_task = self._create_initial_vehicle_task_data()

        for num_index, solution in enumerate(vehicle_candidate_solutions):
            veh_arrival_times = defaultdict(dict)

            for index, route in enumerate(solution, start=0):
                vehicle_id = self.truck_ids[index]
                for route_index, node_j in enumerate(route):
                    if route_index == 0:
                        veh_arrival_times[vehicle_id][node_j] = 0
                        continue
                    else:
                        node_i = route[route_index-1]
                        veh_arrival_times[vehicle_id][node_j] = veh_arrival_times[vehicle_id][node_i] + self.veh_travel[vehicle_id][node_i][node_j]

            current_vehicle_task = deep_copy_vehicle_task_data(new_current_vehicle_task)
            best_customer_plan, best_uav_plan, best_plan_cost, update_vehicle_task_data = self._find_feasible_uav_plans(solution, veh_arrival_times, current_vehicle_task)

            customer_plan.append(best_customer_plan)
            uav_plan.append(best_uav_plan)
            plan_cost.append(best_plan_cost)

            total_vehicle_task.append(update_vehicle_task_data)
            total_vehicle_route.append(solution)
            vehicle_arrival_time.append(veh_arrival_times)

            total_cost = calculate_plan_cost(best_plan_cost, solution, self.vehicle, self.truck_ids, self.uav_ids, self.veh_distance)

            window_cost,uav_tw_violate_cost, total_cost_dict = calculate_window_cost(best_customer_plan, best_plan_cost, veh_arrival_times, self.vehicle, self.customer_time_windows_h, self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node)
            total_window_total_cost = calculate_plan_cost(total_cost_dict, solution, self.vehicle, self.truck_ids, self.uav_ids, self.veh_distance)
            total_window_cost.append(total_window_total_cost)
            num_plan_cost.append(total_cost)

        num_plan_cost = np.array(total_window_cost)
        min_plan_cost_index = np.argmin(num_plan_cost)
        best_customer_plan = customer_plan[min_plan_cost_index]
        best_uav_plan = uav_plan[min_plan_cost_index]
        best_plan_cost = plan_cost[min_plan_cost_index]
        best_vehicle_task = total_vehicle_task[min_plan_cost_index]
        best_vehicle_route = total_vehicle_route[min_plan_cost_index]
        best_vehicle_arrival_time = vehicle_arrival_time[min_plan_cost_index]
        best_total_important_vtps = total_important_vtps[min_plan_cost_index]
        return best_customer_plan, best_uav_plan, best_plan_cost, best_vehicle_route, best_vehicle_task, best_vehicle_arrival_time, best_total_important_vtps

    def generate_greedy_uav_solutions(self, vehicle_candidate_solutions, vehicle_task_data, total_important_vtps):
        uav_solutions = defaultdict(dict)
        insert_costs = []
        customer_uav_plans = {}
        diversity_costs = []
        best_uav_solution = None
        best_insert_cost = float('inf')
        task_arrival_times = defaultdict(dict)
        customer_plan = []
        total_window_cost = []
        uav_plan = []
        plan_cost = []
        total_vehicle_task = []
        total_vehicle_route = []
        feasible_plans = []
        num_plan_cost = []
        vehicle_arrival_time = []
        new_current_vehicle_task = self._create_initial_vehicle_task_data()
        fix_unserved = []

        for num_index, solution in enumerate(vehicle_candidate_solutions):
            veh_arrival_times = defaultdict(dict)

            for index, route in enumerate(solution, start=0):
                vehicle_id = self.truck_ids[index]
                for route_index, node_j in enumerate(route):
                    if route_index == 0:
                        veh_arrival_times[vehicle_id][node_j] = 0
                        continue
                    else:
                        node_i = route[route_index-1]
                        veh_arrival_times[vehicle_id][node_j] = veh_arrival_times[vehicle_id][node_i] + self.veh_travel[vehicle_id][node_i][node_j]

            current_vehicle_task = deep_copy_vehicle_task_data(new_current_vehicle_task)
            self._fix_last_unserved = []
            best_customer_plan, best_uav_plan, best_plan_cost, update_vehicle_task_data = self._find_all_feasible_uav_plans(solution, veh_arrival_times, current_vehicle_task, self.early_arrival_cost, self.late_arrival_cost)
            fix_unserved.append(len(self._fix_last_unserved))

            customer_plan.append(best_customer_plan)
            uav_plan.append(best_uav_plan)
            plan_cost.append(best_plan_cost)

            total_vehicle_task.append(update_vehicle_task_data)
            total_vehicle_route.append(solution)
            vehicle_arrival_time.append(veh_arrival_times)

            total_cost = calculate_plan_cost(best_plan_cost, solution, self.vehicle, self.truck_ids, self.uav_ids, self.veh_distance)

            window_cost,uav_tw_violate_cost, total_cost_dict = calculate_window_cost(best_customer_plan, best_plan_cost, veh_arrival_times, self.vehicle, self.customer_time_windows_h, self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node)
            total_window_total_cost = calculate_plan_cost(total_cost_dict, solution, self.vehicle, self.truck_ids, self.uav_ids, self.veh_distance)
            total_window_cost.append(total_window_total_cost)
            num_plan_cost.append(total_cost)

        num_plan_cost = np.array(total_window_cost)
        min_plan_cost_index = np.argmin(num_plan_cost)
        if any(fix_unserved):

            min_plan_cost_index = min(range(len(fix_unserved)), key=lambda k: (fix_unserved[k], num_plan_cost[k], k))
            print(f'[fast-fix][init] unserved per candidate={fix_unserved}; chosen={min_plan_cost_index} unserved={fix_unserved[min_plan_cost_index]}')
        best_customer_plan = customer_plan[min_plan_cost_index]
        best_uav_plan = uav_plan[min_plan_cost_index]
        best_plan_cost = plan_cost[min_plan_cost_index]
        best_vehicle_task = total_vehicle_task[min_plan_cost_index]
        best_vehicle_route = total_vehicle_route[min_plan_cost_index]
        best_vehicle_arrival_time = vehicle_arrival_time[min_plan_cost_index]
        best_total_important_vtps = total_important_vtps[min_plan_cost_index]
        return best_customer_plan, best_uav_plan, best_plan_cost, best_vehicle_route, best_vehicle_task, best_vehicle_arrival_time, best_total_important_vtps

    def _create_initial_vehicle_task_data(self):

            def create_dict():
                return defaultdict(lambda: None)
            vehicle_task_data = defaultdict(create_dict)

            for vehicle_id, veh in self.vehicle.items():
                for node_id in self.node:
                    vehicle_task_data[vehicle_id][node_id] = vehicle_task(
                        vehicle_id, veh.vehicleType, node_id, self.node
                    )
                    if veh.vehicleType == TYPE_TRUCK:
                        drone_list = list(veh.drones.keys())
                        vehicle_task_data[vehicle_id][node_id].update_drone_list(drone_list)

            for vehicle_id, veh in self.vehicle.items():
                if veh.vehicleType == TYPE_TRUCK:
                    drone_list = list(veh.drones.keys())
                    belong_vehicle_id = vehicle_id
                    for drone_id in drone_list:
                        for node_id in self.node:
                            vehicle_task_data[drone_id][node_id].update_drone_belong(
                                drone_id, belong_vehicle_id
                            )

                            if vehicle_task_data[drone_id][node_id] is None:
                                drone_task = vehicle_task(drone_id, TYPE_UAV, node_id, self.node)
                                vehicle_task_data[drone_id][node_id] = drone_task

                            drone_task = vehicle_task_data[drone_id][node_id]
                            if vehicle_id in drone_task.dict_vehicle:
                                drone_task.dict_vehicle[vehicle_id]['drone_belong'] = vehicle_id
                                drone_task.dict_vehicle[vehicle_id]['prcise_arrive_time'] = 0
                                drone_task.dict_vehicle[vehicle_id]['prcise_departure_time'] = 0
                                drone_task.dict_vehicle[vehicle_id]['launch_drone_list'] = []
                                drone_task.dict_vehicle[vehicle_id]['recovery_drone_list'] = []

            return vehicle_task_data

    def greedy_insert_feasible_plan(self, un_visit_customer, vehicle_route, vehicle_arrival_time, vehicle_task_data, best_customer_plan):

        time_diff = {}
        for customer, plan in best_customer_plan.items():
            drone_id, launch_node, _, recovery_node, launch_vehicle, recovery_vehicle = plan
            launch_time = vehicle_arrival_time[launch_vehicle][launch_node]
            recovery_time = vehicle_arrival_time[recovery_vehicle][recovery_node]
            time_diff[customer] = recovery_time - launch_time

        sorted_customers = sorted(time_diff.items(), key=lambda x: x[1], reverse=True)

        best_cost = float('inf')
        best_plan = None
        best_original_customer = None
        plan_cost = {}
        plan_y = {}
        plan_time = {}
        plan_uav_route = {}

        try:
            for customer, time_difference in sorted_customers:

                drone_id, orig_launch_node, _, orig_recovery_node, launch_vehicle, recovery_vehicle = best_customer_plan[customer]

                test_vehicle_task_data = deep_copy_vehicle_task_data(vehicle_task_data)

                remove_vehicle_task_data = remove_vehicle_task(test_vehicle_task_data, best_customer_plan[customer], vehicle_route)
                launch_vehicle_index = launch_vehicle - 1
                recovery_vehicle_index = recovery_vehicle -1
                launch_node_index = vehicle_route[launch_vehicle_index].index(orig_launch_node)
                recovery_node_index = vehicle_route[recovery_vehicle_index].index(orig_recovery_node)
                total_customer = [customer]
                total_customer.append(un_visit_customer)
                if launch_vehicle == recovery_vehicle:
                    route_segment = vehicle_route[launch_vehicle_index][launch_node_index:recovery_node_index+1]

                    for c in total_customer:
                        plan, cost, time, uav_route = self.find_total_customer_plan(c, route_segment, drone_id, launch_vehicle)
                        plan_cost[c] = cost
                        plan_y[c] = plan
                        plan_time[c] = time
                        plan_uav_route[c] = uav_route

                    sort_cost, sort_plan, sort_time, sort_uav_route = sort_customer_plans(plan_cost, plan_y, plan_time, plan_uav_route)
                    for new_index, new_y in enumerate(sort_plan[un_visit_customer]):

                        new_vehicle_task_data = deep_copy_vehicle_task_data(remove_vehicle_task_data)
                        drone_id, new_launch_node, new_customer, new_recovery_node, new_launch_vehicle, new_recovery_vehicle = new_y
                        new_cost = sort_cost[un_visit_customer][new_index]
                        new_time = sort_time[un_visit_customer][new_index]
                        new_uav_route = sort_uav_route[un_visit_customer][new_index]
                        new_plan = {
                                    'drone_id': drone_id,
                                    'launch_vehicle': launch_vehicle,
                                    'recovery_vehicle': recovery_vehicle,
                                    'launch_node': new_launch_node,
                                    'recovery_node': new_recovery_node,
                                    'customer': new_customer,
                                    'launch_time': vehicle_arrival_time[new_launch_vehicle][new_launch_node],
                                    'recovery_time': vehicle_arrival_time[new_recovery_vehicle][new_recovery_node],
                                    'energy': self.xeee[drone_id][self.node[new_launch_node].map_key][new_customer][self.node[new_recovery_node].map_key],
                                    'cost': new_cost,
                                    'time': new_time,
                                    'uav_route': new_uav_route
                                }
                        new_vehicle_task_data = update_vehicle_task(
                            new_vehicle_task_data, new_y, vehicle_route
                        )

                        for orig_index, y in enumerate(sort_plan[customer]):
                            drone_id, orig_launch_node, orig_customer, orig_recovery_node, launch_vehicle, recovery_vehicle = y
                            orig_cost = sort_cost[customer][orig_index]
                            orig_time = sort_time[customer][orig_index]
                            orig_uav_route = sort_uav_route[customer][orig_index]
                            orig_plan = {
                                'drone_id': drone_id,
                                'launch_vehicle': launch_vehicle,
                                'recovery_vehicle': recovery_vehicle,
                                'launch_node': orig_launch_node,
                                'recovery_node': orig_recovery_node,
                                'customer': orig_customer,
                                'launch_time': vehicle_arrival_time[launch_vehicle][orig_launch_node],
                                'recovery_time': vehicle_arrival_time[recovery_vehicle][orig_recovery_node],
                                'energy': self.xeee[drone_id][self.node[orig_launch_node].map_key][orig_customer][self.node[orig_recovery_node].map_key],
                                'cost': orig_cost,
                                'time': orig_time,
                                'uav_route': orig_uav_route
                            }
                            is_valid_plan = check_same_vehicle_conflict(
                                        v_id=launch_vehicle,
                                        drone_id=drone_id,
                                        i_vtp=orig_launch_node,
                                        j_vtp=orig_recovery_node,
                                        solution_route=vehicle_route,
                                        solution=vehicle_arrival_time,
                                        vehicle_task_data=new_vehicle_task_data,
                                        vehicle = self.vehicle
                                    )
                            if not is_valid_plan:
                                continue
                            else:
                                if orig_cost + new_cost < best_cost:
                                    best_orig_y = y
                                    best_new_y = new_y
                                    best_orig_cost = orig_cost
                                    best_new_cost = new_cost
                                    best_cost = orig_cost + new_cost
                                    best_orig_y_cijkdu_plan = orig_plan
                                    best_new_y_cijkdu_plan = new_plan
                else:
                    launch_vehicle_segment = []
                    recovery_vehicle_segment = []
                    launch_vehicle_node_time = vehicle_arrival_time[launch_vehicle][orig_launch_node]
                    for index, node in enumerate(vehicle_route[launch_vehicle_index][launch_node_index:],start = launch_node_index):
                        if drone_id not in vehicle_task_data[launch_vehicle][node].recovery_drone_list:
                            if node != self.depot_id:
                                launch_vehicle_segment.append(node)

                    for i in range(len(vehicle_route[recovery_vehicle_index][:recovery_node_index]),-1,-1):

                        if vehicle_arrival_time[recovery_vehicle][vehicle_route[recovery_vehicle_index][i]] > launch_vehicle_node_time:
                            if drone_id not in vehicle_task_data[recovery_vehicle][vehicle_route[recovery_vehicle_index][i]].launch_drone_list:
                                if vehicle_route[recovery_vehicle_index][i] != self.depot_id:
                                    recovery_vehicle_segment.append(vehicle_route[recovery_vehicle_index][i])
                    recovery_vehicle_segment.reverse()

                    sub_route = vehicle_route[recovery_vehicle_index][recovery_node_index+1:]
                    for index, node in enumerate(sub_route):
                        if vehicle_arrival_time[recovery_vehicle][node] > launch_vehicle_node_time:
                            if drone_id not in vehicle_task_data[recovery_vehicle][node].launch_drone_list:
                                if node != self.depot_id:
                                        recovery_vehicle_segment.append(node)

                    for c in total_customer:
                        plan, cost, time, uav_route = self.find_cross_total_customer_plan(c, launch_vehicle_segment, recovery_vehicle_segment, drone_id, launch_vehicle, recovery_vehicle, vehicle_arrival_time)
                        plan_cost[c] = cost
                        plan_y[c] = plan
                        plan_time[c] = time
                        plan_uav_route[c] = uav_route

                    sort_cost, sort_plan, sort_time, sort_uav_route = sort_customer_plans(plan_cost, plan_y, plan_time, plan_uav_route)
                    for new_index, new_y in enumerate(sort_plan[un_visit_customer]):

                        new_vehicle_task_data = deep_copy_vehicle_task_data(remove_vehicle_task_data)
                        drone_id, new_launch_node, new_customer, new_recovery_node, new_launch_vehicle, new_recovery_vehicle = new_y
                        new_cost = sort_cost[un_visit_customer][new_index]
                        new_time = sort_time[un_visit_customer][new_index]
                        new_uav_route = sort_uav_route[un_visit_customer][new_index]
                        new_plan = {
                                    'drone_id': drone_id,
                                    'launch_vehicle': launch_vehicle,
                                    'recovery_vehicle': recovery_vehicle,
                                    'launch_node': new_launch_node,
                                    'recovery_node': new_recovery_node,
                                    'customer': new_customer,
                                    'launch_time': vehicle_arrival_time[new_launch_vehicle][new_launch_node],
                                    'recovery_time': vehicle_arrival_time[new_recovery_vehicle][new_recovery_node],
                                    'energy': self.xeee[drone_id][self.node[new_launch_node].map_key][new_customer][self.node[new_recovery_node].map_key],
                                    'cost': new_cost,
                                    'time': new_time,
                                    'uav_route': new_uav_route
                                }

                        if new_launch_vehicle == new_recovery_vehicle:
                            is_valid_plan = check_same_vehicle_conflict(
                                        v_id=new_launch_vehicle,
                                        drone_id=drone_id,
                                        i_vtp=new_launch_node,
                                        j_vtp=new_recovery_node,
                                        solution_route=vehicle_route,
                                        solution=vehicle_arrival_time,
                                        vehicle_task_data=new_vehicle_task_data,
                                        vehicle = self.vehicle
                                    )
                        else:
                            is_valid_plan = check_cross_vehicle_conflict_fixed(
                                new_launch_vehicle,
                                new_recovery_vehicle,
                                drone_id,
                                new_launch_node,
                                new_recovery_node,
                                vehicle_route,
                                vehicle_arrival_time,
                                new_vehicle_task_data,
                                vehicle_arrival_time[new_launch_vehicle][new_launch_node],
                                vehicle_arrival_time[new_recovery_vehicle][new_recovery_node],
                                self.vehicle
                            )
                        if not is_valid_plan:
                            continue
                        else:
                            new_vehicle_task_data = update_vehicle_task(
                                new_vehicle_task_data, new_y, vehicle_route
                            )

                        for orig_index, y in enumerate(sort_plan[customer]):
                            drone_id, orig_launch_node, orig_customer, orig_recovery_node, launch_vehicle, recovery_vehicle = y
                            orig_cost = sort_cost[customer][orig_index]
                            orig_time = sort_time[customer][orig_index]
                            orig_uav_route = sort_uav_route[customer][orig_index]
                            orig_plan = {
                                'drone_id': drone_id,
                                'launch_vehicle': launch_vehicle,
                                'recovery_vehicle': recovery_vehicle,
                                'launch_node': orig_launch_node,
                                'recovery_node': orig_recovery_node,
                                'customer': orig_customer,
                                'launch_time': vehicle_arrival_time[launch_vehicle][orig_launch_node],
                                'recovery_time': vehicle_arrival_time[recovery_vehicle][orig_recovery_node],
                                'energy': self.xeee[drone_id][self.node[orig_launch_node].map_key][orig_customer][self.node[orig_recovery_node].map_key],
                                'cost': orig_cost,
                                'time': orig_time,
                                'uav_route': orig_uav_route
                            }

                            if launch_vehicle == recovery_vehicle:
                                is_valid_plan = check_same_vehicle_conflict(
                                            v_id=launch_vehicle,
                                            drone_id=drone_id,
                                            i_vtp=orig_launch_node,
                                            j_vtp=orig_recovery_node,
                                            solution_route=vehicle_route,
                                            solution=vehicle_arrival_time,
                                            vehicle_task_data=new_vehicle_task_data,
                                            vehicle = self.vehicle
                                        )
                            else:
                                is_valid_plan = check_cross_vehicle_conflict_fixed(
                                    launch_vehicle,
                                    recovery_vehicle,
                                    drone_id,
                                    orig_launch_node,
                                    orig_recovery_node,
                                    vehicle_route,
                                    vehicle_arrival_time,
                                    new_vehicle_task_data,
                                    vehicle_arrival_time[launch_vehicle][orig_launch_node],
                                    vehicle_arrival_time[recovery_vehicle][orig_recovery_node],
                                    self.vehicle
                                )
                            if not is_valid_plan:
                                continue
                            else:
                                if orig_cost + new_cost < best_cost:
                                    best_orig_y = y
                                    best_new_y = new_y
                                    best_orig_cost = orig_cost
                                    best_new_cost = new_cost
                                    best_cost = orig_cost + new_cost
                                    best_orig_y_cijkdu_plan = orig_plan
                                    best_new_y_cijkdu_plan = new_plan
            return best_orig_y, best_new_y, best_orig_cost, best_new_cost, best_orig_y_cijkdu_plan, best_new_y_cijkdu_plan
        except Exception as e:
            print(f'Error in find_cross_total_customer_plan: {e}')
            return None, None, None, None, None, None

    def find_cross_total_customer_plan(self, customer, launch_vehicle_segment, recovery_vehicle_segment, drone_id, launch_vehicle, recovery_vehicle, vehicle_arrival_time):
        y_plan = []
        y_cost = []
        y_time = []
        y_uav_route = []
        launch_vehicle_index = launch_vehicle - 1
        recovery_vehicle_index = recovery_vehicle - 1

        for i in range(len(launch_vehicle_segment)):
            for j in range(i+1, len(launch_vehicle_segment)):
                if launch_vehicle_segment[i] == launch_vehicle_segment[j]:
                    continue

                air_i = self.node[launch_vehicle_segment[i]].map_key
                air_j = self.node[launch_vehicle_segment[j]].map_key
                energy = self.xeee[drone_id][air_i][customer][air_j]
                if energy > 0 and energy is not None:
                    y = (drone_id, launch_vehicle_segment[i], customer, launch_vehicle_segment[j], launch_vehicle, launch_vehicle)
                    y_plan.append(y)
                    cost, time, uav_route = cal_low_cost(launch_vehicle_segment[i], customer, launch_vehicle_segment[j], launch_vehicle, launch_vehicle, drone_id, self.uav_travel, self.veh_distance, self.veh_travel, self.node, self.vehicle, 2, self.xeee)
                    y_cost.append(cost)
                    y_time.append(time)
                    y_uav_route.append(uav_route)

        for i in range(len(recovery_vehicle_segment)):
            for j in range(i+1, len(recovery_vehicle_segment)):
                if recovery_vehicle_segment[i] == recovery_vehicle_segment[j]:
                    continue

                air_i = self.node[recovery_vehicle_segment[i]].map_key
                air_j = self.node[recovery_vehicle_segment[j]].map_key
                energy = self.xeee[drone_id][air_i][customer][air_j]
                if energy > 0 and energy is not None:
                    y = (drone_id, recovery_vehicle_segment[i], customer, recovery_vehicle_segment[j], recovery_vehicle, recovery_vehicle)
                    y_plan.append(y)
                    cost, time, uav_route = cal_low_cost(recovery_vehicle_segment[i], customer, recovery_vehicle_segment[j], recovery_vehicle, recovery_vehicle, drone_id, self.uav_travel, self.veh_distance, self.veh_travel, self.node, self.vehicle, 2, self.xeee)
                    y_cost.append(cost)
                    y_time.append(time)
                    y_uav_route.append(uav_route)

        for i in range(len(launch_vehicle_segment)):
            for j in range(len(recovery_vehicle_segment)):
                if launch_vehicle_segment[i] == recovery_vehicle_segment[j]:
                    continue

                air_i = self.node[launch_vehicle_segment[i]].map_key
                air_j = self.node[recovery_vehicle_segment[j]].map_key
                energy = self.xeee[drone_id][air_i][customer][air_j]
                if energy > 0 and energy is not None:
                    if vehicle_arrival_time[launch_vehicle][launch_vehicle_segment[i]] < vehicle_arrival_time[recovery_vehicle][recovery_vehicle_segment[j]]:
                        y = (drone_id, launch_vehicle_segment[i], customer, recovery_vehicle_segment[j], launch_vehicle, recovery_vehicle)
                        y_plan.append(y)
                        cost, time, uav_route = cal_low_cost(launch_vehicle_segment[i], customer, recovery_vehicle_segment[j], launch_vehicle, recovery_vehicle, drone_id, self.uav_travel, self.veh_distance, self.veh_travel, self.node, self.vehicle, 2, self.xeee)
                        y_cost.append(cost)
                        y_time.append(time)
                        y_uav_route.append(uav_route)

        for j in range(len(recovery_vehicle_segment)):
            for i in range(len(launch_vehicle_segment)):
                if launch_vehicle_segment[i] == recovery_vehicle_segment[j]:
                    continue

                air_i = self.node[recovery_vehicle_segment[j]].map_key
                air_j = self.node[launch_vehicle_segment[i]].map_key
                energy = self.xeee[drone_id][air_i][customer][air_j]
                if energy > 0 and energy is not None:
                    if vehicle_arrival_time[recovery_vehicle][recovery_vehicle_segment[j]] < vehicle_arrival_time[launch_vehicle][launch_vehicle_segment[i]]:
                        y = (drone_id, recovery_vehicle_segment[j], customer, launch_vehicle_segment[i], recovery_vehicle, launch_vehicle)
                        y_plan.append(y)
                        cost, time, uav_route = cal_low_cost(recovery_vehicle_segment[j], customer, launch_vehicle_segment[i], recovery_vehicle, launch_vehicle, drone_id, self.uav_travel, self.veh_distance, self.veh_travel, self.node, self.vehicle, 2, self.xeee)
                        y_cost.append(cost)
                        y_time.append(time)
                        y_uav_route.append(uav_route)
        return y_plan, y_cost, y_time, y_uav_route

    def find_total_customer_plan(self, customer, segment, drone_id, vehicle_id):

        y_plan = []
        y_cost = []
        y_time = []
        y_uav_route = []

        for i in range(len(segment)):
            for j in range(i+1, len(segment)):
                if segment[i] == segment[j]:
                    continue

                air_i = self.node[segment[i]].map_key
                air_j = self.node[segment[j]].map_key
                energy = self.xeee[drone_id][air_i][customer][air_j]
                if energy > 0 and energy is not None:
                    y = (drone_id, segment[i], customer, segment[j], vehicle_id, vehicle_id)
                    y_plan.append(y)
                    cost, time, uav_route = cal_low_cost(segment[i], customer, segment[j], vehicle_id, vehicle_id, drone_id, self.uav_travel, self.veh_distance, self.veh_travel, self.node, self.vehicle, 2, self.xeee)
                    y_cost.append(cost)
                    y_time.append(time)
                    y_uav_route.append(uav_route)
        return y_plan, y_cost, y_time, y_uav_route

    import bisect
    from collections import defaultdict
    def find_next_customer_plan(self, customers, solution_route, solution, vehicle_task_data, early_arrival_cost, late_arrival_cost):

        y_cijkdu = {uav_id: [] for uav_id in self.uav_ids}

        best_completion_time = float('inf')
        best_cost_metric = float('inf')
        best_plan = None
        best_customer = None

        potential_nodes = []
        for v_id, route_nodes in enumerate(solution_route):
            for seq_idx, node_id in enumerate(route_nodes):
                if node_id not in self.node:
                    continue
                node_key = self.node[node_id].map_key
                vehicle_id = v_id +1

                arrival_t = float('inf')
                if vehicle_id in solution:
                    arrival_t = solution[vehicle_id].get(node_id, float('inf'))

                potential_nodes.append({
                    'v_id': vehicle_id,
                    'node_id': node_id,
                    'node_key': node_key,
                    'arrival_time': arrival_t,
                    'seq_idx': seq_idx
                })

        sorted_nodes = sorted(potential_nodes, key=lambda x: x['arrival_time'])

        pending_customer_set = set(customers)

        for l_info in sorted_nodes:
            v_id = l_info['v_id']
            i_vtp = l_info['node_key']
            n_id = l_info['node_id']
            launch_time = l_info['arrival_time'] + abs(self.vehicle[v_id].launchTime)

            if launch_time >= best_completion_time:
                break

            for drone_id in self.xeee:

                if drone_id not in vehicle_task_data[v_id][n_id].drone_list:
                    continue

                if i_vtp not in self.xeee[drone_id]:
                    continue

                feasible_customers_at_launch = self.xeee[drone_id][i_vtp]

                valid_candidates = pending_customer_set.intersection(feasible_customers_at_launch.keys())

                if not valid_candidates:
                    continue

                for r_info in sorted_nodes:
                    recv_v_id = r_info['v_id']
                    j_vtp = r_info['node_key']
                    j_n_id = r_info['node_id']
                    recovery_arrival_time = r_info['arrival_time']

                    if recovery_arrival_time <= launch_time:
                        continue

                    recovery_time = recovery_arrival_time + abs(self.vehicle[recv_v_id].recoveryTime)
                    if recovery_time >= best_completion_time:
                        break

                    if v_id == recv_v_id:
                        if r_info['seq_idx'] <= l_info['seq_idx']:
                            continue

                    is_valid_plan = False
                    if v_id == recv_v_id:
                        is_valid_plan = check_same_vehicle_conflict(
                            v_id=v_id, drone_id=drone_id, i_vtp=n_id, j_vtp=j_n_id,
                            solution_route=solution_route, solution=solution,
                            vehicle_task_data=vehicle_task_data, vehicle=self.vehicle
                        )
                    else:
                        is_valid_plan = check_cross_vehicle_conflict_fixed(
                            launch_v_id=v_id, recover_v_id=recv_v_id, drone_id=drone_id,
                            i_vtp=n_id, j_vtp=j_n_id,
                            solution_route=solution_route, solution=solution,
                            vehicle_task_data=vehicle_task_data,
                            launch_time=launch_time, recovery_time=recovery_time,
                            vehicle=self.vehicle
                        )

                    if not is_valid_plan:
                        continue

                    for cand_customer in valid_candidates:

                        energy_dict = self.xeee[drone_id][i_vtp][cand_customer]
                        if j_vtp not in energy_dict:
                            continue

                        energy = energy_dict[j_vtp]
                        if not (energy > 0 and energy is not None):
                            continue

                        cost, time_flight, uav_route = cal_low_cost(
                            n_id, cand_customer, j_n_id, v_id, recv_v_id, drone_id,
                            self.uav_travel, self.veh_distance, self.veh_travel,
                            self.node, self.vehicle, 2, self.xeee
                        )

                        if recovery_time < best_completion_time:
                            best_completion_time = recovery_time
                            best_cost_metric = cost

                            best_customer = cand_customer
                            best_plan = {
                                'drone_id': drone_id,
                                'launch_vehicle': v_id,
                                'recovery_vehicle': recv_v_id,
                                'launch_node': n_id,
                                'recovery_node': j_n_id,
                                'customer': cand_customer,
                                'launch_time': launch_time,
                                'recovery_time': recovery_time,
                                'energy': energy,
                                'cost': cost,
                                'time': time_flight,
                                'uav_route': uav_route
                            }

                        elif recovery_time == best_completion_time:

                            if cost < best_cost_metric:
                                best_cost_metric = cost
                                best_customer = cand_customer
                                best_plan = {
                                    'drone_id': drone_id,
                                    'launch_vehicle': v_id,
                                    'recovery_vehicle': recv_v_id,
                                    'launch_node': n_id,
                                    'recovery_node': j_n_id,
                                    'customer': cand_customer,
                                    'launch_time': launch_time,
                                    'recovery_time': recovery_time,
                                    'energy': energy,
                                    'cost': cost,
                                    'time': time_flight,
                                    'uav_route': uav_route
                                }

        final_y_cijkdu = {uav_id: [] for uav_id in self.uav_ids}
        final_y_plan = {}
        final_y_cost = {}

        if best_plan is not None:
            drone_id = best_plan['drone_id']

            key = (drone_id, best_plan['launch_node'], best_customer, best_plan['recovery_node'],
                   best_plan['launch_vehicle'], best_plan['recovery_vehicle'])

            final_y_cijkdu[drone_id].append(best_plan)
            final_y_plan[key] = best_plan
            final_y_cost[key] = best_cost_metric

        return final_y_cijkdu, final_y_plan, final_y_cost, best_plan, best_cost_metric

    def find_nearest_customer_plan(self, customer, solution_route, solution, vehicle_task_data):
        y_cijkdu = {uav_id: [] for uav_id in self.uav_ids}
        y_plan = {}
        y_cost = {}
        best_y_cost = float('inf')
        best_y_cijkdu_plan = None
        xeee = self.xeee.copy()

        vehicle_arrive_time = solution

        for drone_id in xeee:
            for i in xeee[drone_id]:
                i_vtp = self.node[i].map_key
                if customer not in xeee[drone_id][i]:
                    continue

                for j in xeee[drone_id][i][customer]:
                    j_vtp = self.node[j].map_key
                    energy = xeee[drone_id][i][customer][j]
                    if not (energy > 0 and energy is not None):
                        continue

                    for v_id, route_info in solution.items():

                        if i_vtp not in route_info or drone_id not in vehicle_task_data[v_id][i_vtp].drone_list:
                            continue

                        launch_time = route_info[i_vtp] + abs(self.vehicle[v_id].launchTime)

                        for recv_v_id, recv_route_info in solution.items():

                            if j_vtp not in recv_route_info:
                                continue

                            recovery_time = recv_route_info[j_vtp] + abs(self.vehicle[recv_v_id].recoveryTime)

                            if launch_time >= recovery_time:
                                continue

                            is_valid_plan = False
                            if v_id == recv_v_id:

                                is_valid_plan = check_same_vehicle_conflict(
                                    v_id=v_id,
                                    drone_id=drone_id,
                                    i_vtp=i_vtp,
                                    j_vtp=j_vtp,
                                    solution_route=solution_route,
                                    solution=solution,
                                    vehicle_task_data=vehicle_task_data,
                                    vehicle = self.vehicle
                                )
                            else:

                                is_valid_plan = check_cross_vehicle_conflict_fixed(
                                    launch_v_id=v_id,
                                    recover_v_id=recv_v_id,
                                    drone_id=drone_id,
                                    i_vtp=i_vtp,
                                    j_vtp=j_vtp,
                                    solution_route=solution_route,
                                    solution=solution,
                                    vehicle_task_data=vehicle_task_data,
                                    launch_time=launch_time,
                                    recovery_time=recovery_time,
                                    vehicle=self.vehicle
                                )

                            if not is_valid_plan:
                                continue

                            cost, time, uav_route = cal_low_cost(i_vtp, customer, j_vtp, v_id, recv_v_id, drone_id, self.uav_travel, self.veh_distance, self.veh_travel, self.node, self.vehicle, 2, self.xeee)

                            insert_plan = {}
                            insert_plan[customer] = (drone_id, i_vtp, customer, j_vtp, v_id, recv_v_id)
                            cost+=calculate_customer_window_cost(insert_plan, self.vehicle, vehicle_arrive_time, self.customer_time_windows_h, self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node)
                            plan = {
                                'drone_id': drone_id,
                                'launch_vehicle': v_id,
                                'recovery_vehicle': recv_v_id,
                                'launch_node': i_vtp,
                                'recovery_node': j_vtp,
                                'customer': customer,
                                'launch_time': launch_time,
                                'recovery_time': recovery_time,
                                'energy': energy,
                                'cost': cost,
                                'time': time,
                                'uav_route': uav_route
                            }

                            key = (drone_id, i_vtp, customer, j_vtp, v_id, recv_v_id)

                            y_cijkdu[drone_id].append(plan)
                            y_plan[key] = plan
                            y_cost[key] = cost

                            if cost < best_y_cost:
                                best_y_cost = cost
                                best_y_cijkdu_plan = plan

        return y_cijkdu, y_plan, y_cost, best_y_cijkdu_plan, best_y_cost

    def _find_all_feasible_uav_plans(self, vehicle_route, vehicle_arrival_time, vehicle_task_data, early_arrival_cost, late_arrival_cost):

        best_customer_plan = defaultdict(dict)
        best_uav_plan = defaultdict(dict)
        best_plan_cost = defaultdict(dict)
        customer_service_count = defaultdict(int)
        for c in self.A_c:
            for drone_id in self.xeee:
                for vtp_i in self.xeee[drone_id]:
                    for vtp_j in self.xeee[drone_id][vtp_i][c]:
                        if vtp_i == vtp_j:
                            continue
                        if self.xeee[drone_id][vtp_i][c][vtp_j] is not None:
                            customer_service_count[c] += 1

        sorted_customers = sorted(customer_service_count.items(),
                                key=lambda x: x[1],
                                reverse=True)
        pending_customers = [cust for cust, _cnt in sorted_customers]

        max_total_attempts = max(1000, len(pending_customers) * 20)
        total_attempts = 0
        retry_counter = defaultdict(int)
        max_retry_per_customer = 50

        from task_data import deep_remove_vehicle_task
        while pending_customers:
            total_attempts += 1

            [y_cijkdu, y_plan, y_cost, best_y_cijkdu_plan, best_y_cost] = self.find_next_customer_plan(pending_customers, vehicle_route, vehicle_arrival_time, vehicle_task_data, early_arrival_cost, late_arrival_cost)
            if best_y_cijkdu_plan is None:

                self._fix_last_unserved = list(pending_customers)
                print(f'[fast-fix][init] candidate route set leaves {len(pending_customers)} customers without a feasible UAV insertion: {pending_customers}')
                break
            pop_customer = best_y_cijkdu_plan['customer']
            customer = pop_customer
            pending_customers.remove(pop_customer)

            if best_y_cijkdu_plan is None:

                print('找不到可行的贪婪插入方案，在生成解方案方案失败。检查车辆和无人机的数量配比配置')
            else:

                drone_id = best_y_cijkdu_plan['drone_id']
                launch_node = best_y_cijkdu_plan['launch_node']
                recovery_node = best_y_cijkdu_plan['recovery_node']
                launch_vehicle = best_y_cijkdu_plan['launch_vehicle']
                recovery_vehicle = best_y_cijkdu_plan['recovery_vehicle']

                y = (drone_id, launch_node, customer, recovery_node, launch_vehicle, recovery_vehicle)

                best_customer_plan[customer] = y
                best_plan_cost[customer] = best_y_cost
                best_uav_plan[y] = best_y_cijkdu_plan

                vehicle_task_data = update_vehicle_task(vehicle_task_data, y, vehicle_route)

        return best_customer_plan, best_uav_plan, best_plan_cost, vehicle_task_data

    def _find_feasible_uav_plans(self, vehicle_route, vehicle_arrival_time, vehicle_task_data):

            best_customer_plan = defaultdict(dict)
            best_uav_plan = defaultdict(dict)
            best_plan_cost = defaultdict(dict)
            customer_service_count = defaultdict(int)
            for c in self.A_c:
                for drone_id in self.xeee:
                    for vtp_i in self.xeee[drone_id]:
                        for vtp_j in self.xeee[drone_id][vtp_i][c]:
                            if vtp_i == vtp_j:
                                continue
                            if self.xeee[drone_id][vtp_i][c][vtp_j] is not None:
                                customer_service_count[c] += 1

            sorted_customers = sorted(customer_service_count.items(),
                                    key=lambda x: x[1],
                                    reverse=True)
            pending_customers = [cust for cust, _cnt in sorted_customers]

            max_total_attempts = max(1000, len(pending_customers) * 20)
            total_attempts = 0
            retry_counter = defaultdict(int)
            max_retry_per_customer = 50

            from task_data import deep_remove_vehicle_task
            while pending_customers:
                total_attempts += 1
                if total_attempts > max_total_attempts:
                    print("[WARN] pending_customers 插入循环超过最大尝试次数，已赋予最大值，提前退出以避免死循环。")
                    best_plan_cost[customer] = float('inf')
                    return best_customer_plan, best_uav_plan, best_plan_cost, vehicle_task_data

                customer = pending_customers.pop(0)
                retry_counter[customer] += 1
                if retry_counter[customer] > max_retry_per_customer:

                    print(f"[WARN] customer={customer} 重试次数过多，跳过。")
                    continue

                if customer in best_customer_plan:
                    continue

                [y_cijkdu, y_plan, y_cost, best_y_cijkdu_plan, best_y_cost] = self.find_nearest_customer_plan(customer, vehicle_route, vehicle_arrival_time, vehicle_task_data)

                if best_y_cijkdu_plan is None:

                    (best_orig_y, best_new_y,
                    best_orig_cost, best_new_cost,
                    best_orig_y_cijkdu_plan, best_new_y_cijkdu_plan) = \
                        self.greedy_insert_feasible_plan(customer, vehicle_route, vehicle_arrival_time, vehicle_task_data, best_customer_plan)

                    if best_orig_y is None or best_new_y is None:

                        pending_customers.append(customer)
                        continue

                    orig_drone_id, orig_launch_node, orig_customer, orig_recovery_node, orig_launch_vehicle, orig_recovery_vehicle = best_orig_y
                    new_drone_id, new_launch_node, new_customer, new_recovery_node, new_launch_vehicle, new_recovery_vehicle = best_new_y

                    if orig_customer == customer:
                        remove_customer = new_customer
                    else:
                        remove_customer = orig_customer

                    if remove_customer in best_customer_plan:
                        assignment_to_remove = best_customer_plan[remove_customer]
                        orig_vehicle_id = assignment_to_remove[4]

                        vehicle_task_data = remove_vehicle_task(vehicle_task_data, assignment_to_remove, vehicle_route)

                        need_to_remove_tasks = find_chain_tasks(
                            assignment_to_remove,
                            best_customer_plan,
                            vehicle_route,
                            vehicle_task_data
                        )
                        if need_to_remove_tasks != []:
                            best_plan_cost[customer] = float('inf')
                            return best_customer_plan, best_uav_plan, best_plan_cost, vehicle_task_data

                        best_customer_plan.pop(remove_customer, None)
                        best_plan_cost.pop(remove_customer, None)
                        best_uav_plan.pop(assignment_to_remove, None)

                        chain_customers_removed = []

                        for chain_customer, chain_assignment in need_to_remove_tasks:
                            if chain_customer in best_customer_plan:

                                best_customer_plan.pop(chain_customer, None)
                                best_plan_cost.pop(chain_customer, None)
                                best_uav_plan.pop(chain_assignment, None)

                                vehicle_task_data = deep_remove_vehicle_task(
                                    vehicle_task_data,
                                    chain_assignment,
                                    vehicle_route,
                                    orig_vehicle_id
                                )

                                chain_customers_removed.append(chain_customer)

                        if chain_customers_removed:

                            for cc in reversed(chain_customers_removed):
                                if cc not in pending_customers and cc not in best_customer_plan:
                                    pending_customers.insert(0, cc)

                    if best_new_y_cijkdu_plan['launch_time'] < best_orig_y_cijkdu_plan['launch_time']:

                        best_customer_plan[new_customer] = best_new_y
                        best_plan_cost[new_customer] = best_new_cost
                        best_uav_plan[best_new_y] = best_new_y_cijkdu_plan

                        best_customer_plan[orig_customer] = best_orig_y
                        best_plan_cost[orig_customer] = best_orig_cost
                        best_uav_plan[best_orig_y] = best_orig_y_cijkdu_plan

                        vehicle_task_data = update_vehicle_task(vehicle_task_data, best_new_y, vehicle_route)
                        vehicle_task_data = update_vehicle_task(vehicle_task_data, best_orig_y, vehicle_route)
                    else:

                        best_customer_plan[orig_customer] = best_orig_y
                        best_plan_cost[orig_customer] = best_orig_cost
                        best_uav_plan[best_orig_y] = best_orig_y_cijkdu_plan

                        best_customer_plan[new_customer] = best_new_y
                        best_plan_cost[new_customer] = best_new_cost
                        best_uav_plan[best_new_y] = best_new_y_cijkdu_plan

                        vehicle_task_data = update_vehicle_task(vehicle_task_data, best_orig_y, vehicle_route)
                        vehicle_task_data = update_vehicle_task(vehicle_task_data, best_new_y, vehicle_route)

                    if customer not in best_customer_plan:
                        pending_customers.insert(0, customer)

                else:

                    drone_id = best_y_cijkdu_plan['drone_id']
                    launch_node = best_y_cijkdu_plan['launch_node']
                    recovery_node = best_y_cijkdu_plan['recovery_node']
                    launch_vehicle = best_y_cijkdu_plan['launch_vehicle']
                    recovery_vehicle = best_y_cijkdu_plan['recovery_vehicle']

                    y = (drone_id, launch_node, customer, recovery_node, launch_vehicle, recovery_vehicle)

                    best_customer_plan[customer] = y
                    best_plan_cost[customer] = best_y_cost
                    best_uav_plan[y] = best_y_cijkdu_plan

                    vehicle_task_data = update_vehicle_task(vehicle_task_data, y, vehicle_route)

            return best_customer_plan, best_uav_plan, best_plan_cost, vehicle_task_data

    def air_route_convert_ground_route(self, solution, node):

        ground_route = []
        for vtp_index in solution:
            ground_route.append(node[vtp_index].map_key)
        return ground_route

    def _compute_distance_matrices(self):

        self.vtp_distances = cdist(self.vtp_positions, self.vtp_positions)

        self.depot_vtp_distances = cdist([self.depot_pos], self.vtp_positions)[0]

        customer_nodes = [n for n in self.node if self.node[n].nodeType == 'CUSTOMER']
        if customer_nodes:
            self.customer_positions = np.array([[self.node[i].latDeg, self.node[i].lonDeg, self.node[i].altMeters]
                                               for i in customer_nodes])
            self.vtp_customer_distances = cdist(self.vtp_positions, self.customer_positions)
        else:
            self.customer_positions = np.array([])
            self.vtp_customer_distances = np.array([])

    def generate_diverse_solutions(self, num_solutions):

        solutions = []
        vtp_solutions = []
        total_important_vtps = []

        clustering_params = self._generate_clustering_params(max(1, num_solutions // 3))
        for params in clustering_params:
            clusters_route = []
            clusters = self._cluster_vtps(**params)
            routes = self._solve_tsp_for_clusters(clusters)

            solutions.append(routes)

            for route in routes:
                ground_route = self.air_route_convert_ground_route(route, self.node)
                clusters_route.append(ground_route)
            vtp_solutions.append(clusters_route)

        if solutions:
            important_vtps = self._evaluate_vtp_importance()
            base_solution = solutions.copy()
            diversified_solutions = self._redistribute_important_nodes(base_solution, important_vtps)

            solutions.extend(diversified_solutions)
            for num_index, solution in enumerate(diversified_solutions):
                diversified_vtp_solutions = []
                for route in solution:
                    ground_route = self.air_route_convert_ground_route(route, self.node)
                    diversified_vtp_solutions.append(ground_route)
                vtp_solutions.append(diversified_vtp_solutions)
                total_important_vtps.extend(important_vtps)
        return solutions, vtp_solutions, total_important_vtps

    def _generate_clustering_params(self, num_params):

        params_list = []

        init_methods = ['k-means++', 'random']
        random_states = [42, 123, 456, 789, 1000, 1234, 1456, 1678, 1890, 2000]
        use_depot_distance = [True, False]

        for i in range(num_params):
            params = {
                'init_method': init_methods[i % len(init_methods)],
                'random_state': random_states[i % len(random_states)],
                'use_depot_distance': use_depot_distance[i % len(use_depot_distance)]
            }
            params_list.append(params)

        return params_list

    def _cluster_vtps(self, init_method='k-means++', random_state=42, use_depot_distance=True):

        if len(self.vtp_indices) == 0:
            return {i: [] for i in range(self.num_trucks)}

        if use_depot_distance:
            features = np.column_stack([
                self.vtp_positions,
                self.depot_vtp_distances.reshape(-1, 1)
            ])
        else:
            features = self.vtp_positions

        n_clusters = min(self.num_trucks, len(self.vtp_indices))

        kmeans = KMeans(n_clusters=n_clusters, init=init_method, random_state=random_state)
        labels = kmeans.fit_predict(features)

        clusters = {i: [] for i in range(self.num_trucks)}
        for idx, (vtp_idx, cluster_id) in enumerate(zip(self.vtp_indices, labels)):
            clusters[cluster_id].append(vtp_idx)

        return clusters

    def _solve_tsp_for_clusters(self, clusters):

        routes = []

        for truck_idx in range(self.num_trucks):
            vtp_indices = clusters.get(truck_idx, [])

            if len(vtp_indices) == 0:

                route = []
            elif len(vtp_indices) == 1:

                route = vtp_indices
                route.insert(0, self.depot_index)

            else:

                vtp_indices.insert(0, self.depot_index)

                route = self._solve_tsp_gurobi(vtp_indices)
            routes.append(route)

        return routes

    def _solve_tsp_gurobi(self, vtp_indices):

        nodes = vtp_indices.copy()
        n = len(nodes)

        dist_matrix = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                if i != j:
                    node_i = nodes[i]
                    node_j = nodes[j]
                    if node_i == self.depot_index and node_j != self.depot_index:
                        vtp_pos_idx = self.vtp_index[node_j]
                        dist_matrix[i, j] = self.depot_vtp_distances[vtp_pos_idx]
                    elif node_i != self.depot_index and node_j == self.depot_index:
                        vtp_pos_idx = self.vtp_index[node_i]
                        dist_matrix[i, j] = self.depot_vtp_distances[vtp_pos_idx]
                    else:
                        vtp_i_idx = self.vtp_index[node_i]
                        vtp_j_idx = self.vtp_index[node_j]
                        dist_matrix[i, j] = self.vtp_distances[vtp_i_idx, vtp_j_idx]

        visited = [False] * n
        visited[0] = True
        order_indices = [0]

        for _ in range(n - 1):
            current = order_indices[-1]

            best_next = -1
            best_dist = float('inf')
            for j in range(n):
                if not visited[j] and dist_matrix[current, j] < best_dist:
                    best_dist = dist_matrix[current, j]
                    best_next = j
            visited[best_next] = True
            order_indices.append(best_next)

        improved = True
        while improved:
            improved = False
            for i in range(1, n - 1):
                for j in range(i + 1, n):

                    a, b = order_indices[i - 1], order_indices[i]
                    c, d = order_indices[j], order_indices[(j + 1) % n]
                    old_cost = dist_matrix[a, b] + dist_matrix[c, d]
                    new_cost = dist_matrix[a, c] + dist_matrix[b, d]
                    if new_cost < old_cost - 1e-10:
                        order_indices[i:j + 1] = order_indices[i:j + 1][::-1]
                        improved = True

        route = [self.depot_id]
        for idx in order_indices[1:]:
            route.append(nodes[idx])
        route.append(self.depot_id)
        return route

    def plot_vehicle_routes(self, routes):

        plt.figure(figsize=(12, 10))
        plt.rcParams.update({
        'font.family': 'Arial',
        'font.size': 12,
        'axes.linewidth': 1.5,
        'axes.labelsize': 14,
        'axes.titlesize': 16,
        'xtick.labelsize': 12,
        'ytick.labelsize': 12,
        'legend.fontsize': 10,
        'legend.frameon': True,
        'legend.framealpha': 0.7,
        'legend.edgecolor': 'k'
    })

        if not isinstance(routes[0], list):
            routes = [routes]

        for edge in self.G_air.edges():
            node1, node2 = edge
            x1, y1 = self.node[node1].latDeg, self.node[node1].lonDeg
            x2, y2 = self.node[node2].latDeg, self.node[node2].lonDeg

            plt.plot([x1, x2], [y1, y2],
                    color='#666666',
                    linestyle='--',
                    linewidth=1.0,
                    alpha=0.4,
                    zorder=1)

        colors = plt.cm.jet(np.linspace(0, 1, len(routes)))

        all_nodes = set()
        for route in routes:
            all_nodes.update(route)

        for i, route in enumerate(routes):
            x_coords = []
            y_coords = []

            for node in route:

                x, y = self.node[node].latDeg, self.node[node].lonDeg

                x_coords.append(x)
                y_coords.append(y)

            plt.plot(x_coords, y_coords, 'o-', color=colors[i], linewidth=2, markersize=8, label=f'route {i+1}')

            for j, node in enumerate(route):
                x, y = self.node[node].latDeg, self.node[node].lonDeg

                if self.node[node].nodeType == 'DEPOT':
                    plt.plot(x, y, 'ks', markersize=12)
                    plt.text(x, y+0.1, f'depot', fontsize=12, ha='center')
                else:
                    plt.text(x, y+0.1, f'{node}', fontsize=10, ha='center')

        for node in self.A_c:
            x, y = self.node[node].latDeg, self.node[node].lonDeg
            plt.plot(x, y, 'b*', markersize=15)

        plt.title('Vehicle Route Visualization', fontsize=15)
        plt.xlabel('X Coordinate', fontsize=12)
        plt.ylabel('Y Coordinate', fontsize=12)
        plt.grid(True, linestyle='--', alpha=0.7)

        plt.tight_layout()
        plt.ion()
        plt.show()

        if self.save_path:
            plt.savefig(self.save_path, dpi=300, bbox_inches='tight', pad_inches=0.1)
        plt.close('all')
        print('路径可视化完成')

    def _greedy_tsp(self, vtp_indices):

        if not vtp_indices:
            return []

        route = []
        remaining = set(vtp_indices)

        current_pos = self.depot_pos

        while remaining:
            best_dist = float('inf')
            best_node = None

            for vtp in remaining:
                vtp_pos = np.array([self.node[vtp].latDeg, self.node[vtp].lonDeg])
                dist = np.linalg.norm(current_pos - vtp_pos)

                if dist < best_dist:
                    best_dist = dist
                    best_node = vtp

            if best_node is not None:
                route.append(best_node)
                remaining.remove(best_node)
                current_pos = np.array([self.node[best_node].latDeg, self.node[best_node].lonDeg])

        return route

    def _evaluate_vtp_importance(self):

        importance_scores = {}

        drone_range = 10

        weights = {
            'connectivity': 0.3,
            'centrality': 0.2,
            'customer_proximity': 0.4,
            'depot_proximity': 0.1
        }

        all_scores = {metric: [] for metric in weights.keys()}

        for vtp_id in self.vtp_indices:
            air_index = self.node[vtp_id].map_index

            air_degree = sum(self.air_matrix[air_index] > 0)
            connectivity_score = air_degree / len(self.air_matrix)
            all_scores['connectivity'].append(connectivity_score)

            reachable_vtp_count = 0
            total_vtp_nodes = len(self.vtp_indices)
            for other_vtp_id in self.vtp_indices:
                if other_vtp_id != vtp_id:
                    distance = np.linalg.norm(
                        np.array([self.node[vtp_id].latDeg, self.node[vtp_id].lonDeg]) -
                        np.array([self.node[other_vtp_id].latDeg, self.node[other_vtp_id].lonDeg])
                    )
                    if distance <= drone_range:
                        reachable_vtp_count += 1

            centrality_score = reachable_vtp_count / (total_vtp_nodes - 1) if total_vtp_nodes > 1 else 0
            all_scores['centrality'].append(centrality_score)

            customer_distances = []
            for air_node_id in self.A_c:
                dist = np.linalg.norm(
                    np.array([self.node[vtp_id].latDeg, self.node[vtp_id].lonDeg]) -
                    np.array([self.node[air_node_id].latDeg, self.node[air_node_id].lonDeg])
                )
                customer_distances.append(dist)
            customer_proximity_score = 1 / (np.mean(customer_distances) + 1e-10)
            all_scores['customer_proximity'].append(customer_proximity_score)

            depot_distance = np.linalg.norm(
                np.array([self.node[vtp_id].latDeg, self.node[vtp_id].lonDeg]) -
                np.array([self.node[self.depot_id].latDeg, self.node[self.depot_id].lonDeg])
            )
            depot_proximity_score = 1 / (depot_distance + 1e-10)
            all_scores['depot_proximity'].append(depot_proximity_score)

            importance_scores[vtp_id] = {
                'connectivity': connectivity_score,
                'centrality': centrality_score,
                'customer_proximity': customer_proximity_score,
                'depot_proximity': depot_proximity_score
            }

        for metric in weights.keys():
            max_val = max(all_scores[metric])
            min_val = min(all_scores[metric])
            range_val = max_val - min_val

            if range_val > 0:
                for vtp_id in importance_scores:
                    importance_scores[vtp_id][metric] = (importance_scores[vtp_id][metric] - min_val) / range_val
            else:

                for vtp_id in importance_scores:
                    importance_scores[vtp_id][metric] = 1.0

        final_scores = {}
        for vtp_id in importance_scores:
            final_scores[vtp_id] = sum(importance_scores[vtp_id][metric] * weights[metric] for metric in weights)

        return final_scores

    def _redistribute_important_nodes(self, base_solutions, vtp_scores, insertion_prob_base=0.7):

        diversified_solutions = []

        max_score = max(vtp_scores.values()) if vtp_scores else 1.0
        for solution in base_solutions:

            new_solution =[]
            add_solution = []
            tsp_solution = {}
            vehicle_id = 0

            existing_vtps = set()
            for route in solution:
                current_route = route.copy()

                current_route.pop(0)
                current_route.pop()
                existing_vtps.update(node for node in route if node in vtp_scores)

                missing_vtps = [vtp for vtp in vtp_scores if vtp not in existing_vtps]

                missing_vtps.sort(key=lambda x: vtp_scores[x], reverse=True)

                for vtp in missing_vtps:

                    normalized_score = vtp_scores[vtp] / max_score
                    insertion_prob = insertion_prob_base * normalized_score

                    if random.random() < insertion_prob:

                        if current_route:
                            current_route.append(vtp)
                new_solution.append(current_route)
                tsp_solution[vehicle_id] = current_route
                vehicle_id += 1
            add_solution = self._solve_tsp_for_clusters(tsp_solution)

            diversified_solutions.append(add_solution)
        return diversified_solutions

    def _find_best_insertion_position(self, route, vtp):

        if not route:
            return 0

        min_increase = float('inf')
        best_pos = 0

        vtp_pos = np.array([self.node[vtp].latDeg, self.node[vtp].lonDeg])

        for pos in range(len(route) + 1):

            if pos == 0:

                if route:
                    first_pos = np.array([self.node[route[0]].latDeg, self.node[route[0]].lonDeg])
                    increase = (np.linalg.norm(self.depot_pos - vtp_pos) +
                              np.linalg.norm(vtp_pos - first_pos) -
                              np.linalg.norm(self.depot_pos - first_pos))
                else:
                    increase = np.linalg.norm(self.depot_pos - vtp_pos)
            elif pos == len(route):

                last_pos = np.array([self.node[route[-1]].latDeg, self.node[route[-1]].lonDeg])
                increase = (np.linalg.norm(last_pos - vtp_pos) +
                          np.linalg.norm(vtp_pos - self.depot_pos) -
                          np.linalg.norm(last_pos - self.depot_pos))
            else:

                prev_pos = np.array([self.node[route[pos-1]].latDeg, self.node[route[pos-1]].lonDeg])
                next_pos = np.array([self.node[route[pos]].latDeg, self.node[route[pos]].lonDeg])
                increase = (np.linalg.norm(prev_pos - vtp_pos) +
                          np.linalg.norm(vtp_pos - next_pos) -
                          np.linalg.norm(prev_pos - next_pos))

            if increase < min_increase:
                min_increase = increase
                best_pos = pos

        return best_pos

    def _apply_local_search(self, routes):

        new_routes = [route.copy() for route in routes]

        operations = ['swap_within', 'swap_between', 'relocate', '2-opt']
        operation = random.choice(operations)

        if operation == 'swap_within':

            truck_idx = random.randint(0, self.num_trucks - 1)
            if len(new_routes[truck_idx]) >= 2:
                i, j = random.sample(range(len(new_routes[truck_idx])), 2)
                new_routes[truck_idx][i], new_routes[truck_idx][j] = \
                    new_routes[truck_idx][j], new_routes[truck_idx][i]

        elif operation == 'swap_between':

            if self.num_trucks >= 2:
                truck1, truck2 = random.sample(range(self.num_trucks), 2)
                if new_routes[truck1] and new_routes[truck2]:
                    i = random.randint(0, len(new_routes[truck1]) - 1)
                    j = random.randint(0, len(new_routes[truck2]) - 1)
                    new_routes[truck1][i], new_routes[truck2][j] = \
                        new_routes[truck2][j], new_routes[truck1][i]

        elif operation == 'relocate':

            non_empty = [i for i in range(self.num_trucks) if len(new_routes[i]) > 0]
            if non_empty:
                src_truck = random.choice(non_empty)
                if len(new_routes[src_truck]) > 0:
                    node_idx = random.randint(0, len(new_routes[src_truck]) - 1)
                    node = new_routes[src_truck].pop(node_idx)

                    dst_truck = random.randint(0, self.num_trucks - 1)
                    if dst_truck == src_truck and new_routes[dst_truck]:

                        new_pos = random.randint(0, len(new_routes[dst_truck]))
                        new_routes[dst_truck].insert(new_pos, node)
                    else:

                        pos = self._find_best_insertion_position(new_routes[dst_truck], node)
                        new_routes[dst_truck].insert(pos, node)

        elif operation == '2-opt':

            truck_idx = random.randint(0, self.num_trucks - 1)
            if len(new_routes[truck_idx]) >= 4:
                i = random.randint(0, len(new_routes[truck_idx]) - 3)
                j = random.randint(i + 2, len(new_routes[truck_idx]) - 1)
                new_routes[truck_idx][i+1:j+1] = new_routes[truck_idx][i+1:j+1][::-1]

        return new_routes

    def _create_empty_solution(self):

        return [[] for _ in range(self.num_trucks)]

    def _create_random_solution(self):

        routes = [[] for _ in range(self.num_trucks)]
        remaining = self.vtp_indices.copy()
        random.shuffle(remaining)

        for i, vtp in enumerate(remaining):
            truck_idx = i % self.num_trucks
            routes[truck_idx].append(vtp)

        return routes

    def select_best_solution(self, solutions):

        best_solution = None
        best_cost = float('inf')

        for solution in solutions:
            cost = self._evaluate_solution(solution)
            if cost < best_cost:
                best_cost = cost
                best_solution = solution

        return best_solution if best_solution is not None else self._create_empty_solution()

    def _evaluate_solution(self, solution):

        total_cost = 0

        for route in solution:
            if not route:
                continue

            route_cost = 0

            if route:
                first_pos = np.array([self.node[route[0]].latDeg, self.node[route[0]].lonDeg])
                route_cost += np.linalg.norm(self.depot_pos - first_pos)

            for i in range(len(route) - 1):
                pos1 = np.array([self.node[route[i]].latDeg, self.node[route[i]].lonDeg])
                pos2 = np.array([self.node[route[i+1]].latDeg, self.node[route[i+1]].lonDeg])
                route_cost += np.linalg.norm(pos1 - pos2)

            if route:
                last_pos = np.array([self.node[route[-1]].latDeg, self.node[route[-1]].lonDeg])
                route_cost += np.linalg.norm(last_pos - self.depot_pos)

            total_cost += route_cost

        route_lengths = [len(route) for route in solution]
        if route_lengths:
            balance_penalty = np.std(route_lengths) * 10
            total_cost += balance_penalty

        return total_cost

def make_dict():
	return defaultdict(make_dict)

