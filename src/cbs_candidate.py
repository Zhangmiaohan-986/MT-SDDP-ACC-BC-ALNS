import numpy as np
import copy
from task_data import vehicle_task
from cost_y import calculate_plan_cost
from task_data import *
from cost_y import *
from call_function import *
from initialize import *
import heapq
import itertools
from collections import deque
import networkx as nx
import time
import itertools

def low_update_time(uav_task_dict, best_uav_plan, best_vehicle_route, vehicle_task_data, vehicle_arrival_time, node, V, T, vehicle, uav_travel):

    detailed_vehicle_task_data = deep_copy_vehicle_task_data(vehicle_task_data)
    trucks_id = T
    uav_id = V
    time_uav_task_dict = {}
    time_customer_plan = {}
    vehicle_plan_time = {}
    for id in uav_id:
        time_uav_task_dict[id] = []
    vehicle_events = defaultdict(lambda: defaultdict(list))
    for drone_id, missions in uav_task_dict.items():
        for flight_tuple in missions:
            _, launch_node, _, recovery_node, launch_veh_id, recovery_veh_id = flight_tuple
            vehicle_events[launch_veh_id][launch_node].append({
                "type": "launch", "task_type": TASK_DRONE_LAUNCH, "drone_id": drone_id, "flight_tuple": flight_tuple
            })
            vehicle_events[recovery_veh_id][recovery_node].append({
                "type": "recovery", "task_type": TASK_DRONE_RECOVERY, "drone_id": drone_id, "flight_tuple": flight_tuple
            })
    sorted_items = sorted(best_uav_plan.items(), key=lambda item: item[1]['launch_time'])
    sorted_mission_keys = [item[0] for item in sorted_items]

    for id in trucks_id:
        idx = id -1
        vehicle_route = best_vehicle_route[idx]

        for index, node_id in enumerate(vehicle_route[:-1]):
            if index == 0:

                arrive_time = vehicle_arrival_time[id][node_id]
                departure_time = vehicle_arrival_time[id][node_id]
                detailed_vehicle_task_data[id][node_id].add_node(node_id, arrive_time, departure_time)
                detailed_vehicle_task_data[id][node_id].prcise_arrive_time = arrive_time
                detailed_vehicle_task_data[id][node_id].prcise_departure_time = departure_time

                carry_drone_list = detailed_vehicle_task_data[id][node_id].drone_list
                launch_drone_list = detailed_vehicle_task_data[id][node_id].launch_drone_list
                recovery_drone_list = detailed_vehicle_task_data[id][node_id].recovery_drone_list
                launch_drone_list = normalize_input_data(launch_drone_list)
                carry_drone_list = normalize_input_data(carry_drone_list)
                recovery_drone_list = normalize_input_data(recovery_drone_list)

                if carry_drone_list is not None:
                    for drone_id in carry_drone_list:
                        detailed_vehicle_task_data[drone_id][node_id].drone_belong = id
                        detailed_vehicle_task_data[drone_id][node_id].add_node(node_id, arrive_time, departure_time)
                        detailed_vehicle_task_data[drone_id][node_id].prcise_arrive_time = arrive_time
                        detailed_vehicle_task_data[drone_id][node_id].dict_vehicle[id]['prcise_arrive_time'] = arrive_time
                        detailed_vehicle_task_data[drone_id][node_id].prcise_departure_time = departure_time
                        detailed_vehicle_task_data[drone_id][node_id].dict_vehicle[id]['prcise_departure_time'] = departure_time
                        detailed_vehicle_task_data[drone_id][node_id].arrive_times.append(arrive_time)
                        detailed_vehicle_task_data[drone_id][node_id].departure_times.append(arrive_time)
                if launch_drone_list is not None:
                    for drone_id in launch_drone_list:
                        detailed_vehicle_task_data[drone_id][node_id].drone_belong = id
                        detailed_vehicle_task_data[drone_id][node_id].add_node(node_id, arrive_time, departure_time)
                        detailed_vehicle_task_data[drone_id][node_id].prcise_arrive_time = arrive_time
                        detailed_vehicle_task_data[drone_id][node_id].dict_vehicle[id]['prcise_arrive_time'] = arrive_time
                        detailed_vehicle_task_data[drone_id][node_id].prcise_departure_time = departure_time
                        detailed_vehicle_task_data[drone_id][node_id].dict_vehicle[id]['prcise_departure_time'] = departure_time
                        detailed_vehicle_task_data[drone_id][node_id].arrive_times.append(arrive_time)
                        detailed_vehicle_task_data[drone_id][node_id].departure_times.append(arrive_time)
            else:
                arrive_time = vehicle_arrival_time[id][node_id]
                departure_time = arrive_time
                detailed_vehicle_task_data[id][node_id].add_node(node_id, arrive_time, departure_time)
                detailed_vehicle_task_data[id][node_id].prcise_arrive_time = arrive_time

                detailed_vehicle_task_data[id][node_id].prcise_departure_time = departure_time

                detailed_vehicle_task_data[id][node_id].arrive_times.append(arrive_time)

                detailed_vehicle_task_data[id][node_id].departure_times.append(departure_time)

                carry_drone_list = detailed_vehicle_task_data[id][node_id].drone_list
                launch_drone_list = detailed_vehicle_task_data[id][node_id].launch_drone_list
                recovery_drone_list = detailed_vehicle_task_data[id][node_id].recovery_drone_list
                carry_drone_list = normalize_input_data(carry_drone_list)
                launch_drone_list = normalize_input_data(launch_drone_list)
                recovery_drone_list = normalize_input_data(recovery_drone_list)
                if carry_drone_list is not None:
                    for drone_id in carry_drone_list:
                        detailed_vehicle_task_data[drone_id][node_id].drone_belong = id
                        detailed_vehicle_task_data[drone_id][node_id].add_node(node_id, arrive_time, departure_time)
                        detailed_vehicle_task_data[drone_id][node_id].prcise_arrive_time = arrive_time
                        detailed_vehicle_task_data[drone_id][node_id].dict_vehicle[id]['prcise_arrive_time'] = arrive_time
                        detailed_vehicle_task_data[drone_id][node_id].prcise_departure_time = departure_time
                        detailed_vehicle_task_data[drone_id][node_id].dict_vehicle[id]['prcise_departure_time'] = departure_time
                        detailed_vehicle_task_data[drone_id][node_id].arrive_times.append(arrive_time)
                        detailed_vehicle_task_data[drone_id][node_id].departure_times.append(arrive_time)
                if launch_drone_list is not None:
                    for drone_id in launch_drone_list:
                        detailed_vehicle_task_data[drone_id][node_id].drone_belong = id
                        detailed_vehicle_task_data[drone_id][node_id].add_node(node_id, arrive_time, departure_time)
                        detailed_vehicle_task_data[drone_id][node_id].prcise_arrive_time = arrive_time
                        detailed_vehicle_task_data[drone_id][node_id].dict_vehicle[id]['prcise_arrive_time'] = arrive_time
                        detailed_vehicle_task_data[drone_id][node_id].prcise_departure_time = departure_time
                        detailed_vehicle_task_data[drone_id][node_id].dict_vehicle[id]['prcise_departure_time'] = departure_time
                        detailed_vehicle_task_data[drone_id][node_id].arrive_times.append(arrive_time)
                        detailed_vehicle_task_data[drone_id][node_id].departure_times.append(departure_time)

    for index, y_ijkd in enumerate(sorted_mission_keys):
        drone_id, launch_node, customer_node, recovery_node, launch_veh_id, recovery_veh_id = y_ijkd
        time_customer_plan[customer_node] = y_ijkd
        time_uav_task_dict[drone_id].append(y_ijkd)
        launch_map_key = node[launch_node].map_key
        recovery_map_key = node[recovery_node].map_key
        truck_departure_time = detailed_vehicle_task_data[launch_veh_id][launch_node].prcise_departure_time
        drone_start_launch_time = truck_departure_time
        drone_end_launch_time = drone_start_launch_time + vehicle[drone_id].launchTime

        detailed_vehicle_task_data[drone_id][launch_node].add_task(launch_node, 13, drone_start_launch_time, drone_end_launch_time)

        detailed_vehicle_task_data[launch_veh_id][launch_node].prcise_departure_time = drone_end_launch_time
        detailed_vehicle_task_data[drone_id][launch_node].prcise_departure_time = drone_end_launch_time
        detailed_vehicle_task_data[drone_id][launch_node].dict_vehicle[launch_veh_id]['prcise_departure_time'] = drone_end_launch_time
        detailed_vehicle_task_data[drone_id][launch_node].departure_times.append(drone_end_launch_time)
        detailed_vehicle_task_data[drone_id][launch_node].dict_vehicle[launch_veh_id]['departure_times'].append(drone_end_launch_time)
        detailed_vehicle_task_data[launch_veh_id][launch_node].departure_times.append(drone_end_launch_time)

        uav_to_c_time = uav_travel[drone_id][launch_map_key][customer_node].totalTime
        uav_arr_c_time = drone_end_launch_time + uav_to_c_time
        detailed_vehicle_task_data[drone_id][launch_node].add_task(launch_node, 18, drone_end_launch_time, uav_arr_c_time)

        customer_service_time = node[customer_node].serviceTimeUAV
        uav_depart_customer_time = uav_arr_c_time + customer_service_time
        detailed_vehicle_task_data[drone_id][customer_node].add_task(customer_node, 15, uav_arr_c_time, uav_depart_customer_time)

        detailed_vehicle_task_data[drone_id][customer_node].prcise_arrive_time = uav_arr_c_time
        detailed_vehicle_task_data[drone_id][customer_node].dict_vehicle[launch_veh_id]['prcise_arrive_time'] = uav_arr_c_time
        detailed_vehicle_task_data[drone_id][customer_node].prcise_departure_time = uav_depart_customer_time
        detailed_vehicle_task_data[drone_id][customer_node].dict_vehicle[launch_veh_id]['prcise_departure_time'] = uav_depart_customer_time
        detailed_vehicle_task_data[drone_id][customer_node].arrive_times.append(uav_arr_c_time)
        detailed_vehicle_task_data[drone_id][customer_node].departure_times.append(uav_depart_customer_time)
        detailed_vehicle_task_data[drone_id][customer_node].dict_vehicle[launch_veh_id]['arrive_times'].append(uav_arr_c_time)
        detailed_vehicle_task_data[drone_id][customer_node].dict_vehicle[launch_veh_id]['departure_times'].append(uav_depart_customer_time)

        uav_to_j_time = uav_travel[drone_id][customer_node][recovery_map_key].totalTime
        uav_arr_j_time = uav_depart_customer_time + uav_to_j_time
        detailed_vehicle_task_data[drone_id][recovery_node].add_task(recovery_node, 19, uav_depart_customer_time, uav_arr_j_time)

        detailed_vehicle_task_data[drone_id][recovery_node].arrive_times.append(uav_arr_j_time)
        detailed_vehicle_task_data[drone_id][recovery_node].dict_vehicle[recovery_veh_id]['arrive_times'].append(uav_arr_j_time)
        detailed_vehicle_task_data[drone_id][recovery_node].prcise_arrive_time = uav_arr_j_time
        detailed_vehicle_task_data[drone_id][recovery_node].dict_vehicle[recovery_veh_id]['prcise_arrive_time'] = uav_arr_j_time

        delta_time = detailed_vehicle_task_data[launch_veh_id][launch_node].prcise_departure_time - drone_start_launch_time
        detailed_vehicle_task_data = update_delta_time(delta_time, detailed_vehicle_task_data, best_vehicle_route, y_ijkd, vehicle)

    uav_plan_time = update_uav_plan(detailed_vehicle_task_data, best_uav_plan)
    vehicle_plan_time = update_vehicle_arrive_time(detailed_vehicle_task_data, vehicle_arrival_time)

    sorted_uav_plan_time = sorted(uav_plan_time.items(), key=lambda item: item[1]['launch_time'])
    uav_plan_time = {k: v for k, v in sorted_uav_plan_time}

    return time_uav_task_dict, time_customer_plan, uav_plan_time, vehicle_plan_time, detailed_vehicle_task_data

def rolling_time_cbs(
    vehicle_arrive_time, best_vehicle_route, time_uav_task_dict, time_customer_plan, time_uav_plan, vehicle_plan_time,
    vehicle_task_data, node, DEPOT_nodeID, V, T, vehicle, uav_travel, veh_distance,
    veh_travel, N, N_zero, N_plus, A_total, A_cvtp, A_vtp, A_aerial_relay_node, G_air, G_ground,
    air_matrix, ground_matrix, air_node_types, ground_node_types, A_c, xeee
):

    sorted_missions = sorted(time_uav_plan.items(), key=lambda item: item[1]['launch_time'])
    unplanned_missions_q = deque(sorted_missions)

    current_batch = []

    global_reservation_table = {}
    final_uav_plan = {}
    final_solution = {}
    best_uav_cost = {}
    cbs_solver = Time_cbs_Batch_Solver(global_reservation_table, time_uav_task_dict, time_customer_plan, time_uav_plan, vehicle_plan_time, vehicle_task_data,
    node, DEPOT_nodeID, V, T, vehicle, uav_travel, veh_distance, veh_travel, N, N_zero, N_plus, A_total,
    A_cvtp, A_vtp, A_aerial_relay_node, G_air, G_ground, air_matrix, ground_matrix, air_node_types, ground_node_types,
    A_c, xeee)
    decimals = 4

    while unplanned_missions_q:

        anchor_mission_tuple, anchor_mission_dict = unplanned_missions_q[0]

        t_start = anchor_mission_dict['launch_time']

        t_end_window = anchor_mission_dict['recovery_time'] * 1.0
        current_batch_missions = []
        MAX_BATCH_SIZE = 15
        selected_drone_ids = set()
        temp_q = unplanned_missions_q.copy()
        while temp_q and len(current_batch_missions) < MAX_BATCH_SIZE:
            mission_tuple, mission_dict = temp_q.popleft()
            drone_id = mission_tuple[0]
            if mission_dict['launch_time'] <= t_end_window and drone_id not in selected_drone_ids:

                current_batch_missions.append((mission_tuple, mission_dict))
                selected_drone_ids.add(drone_id)
            else:
                break

        batch_ids_for_log = [m[0] for m in current_batch_missions]

        batch_result = cbs_solver.solve_for_batch(current_batch_missions, global_reservation_table)

        if batch_result is None:

            print(f"Warning: CBS failed for batch. Retrying with anchor mission only.")
            current_batch_missions = [unplanned_missions_q[0]]
            batch_result = cbs_solver.solve_for_batch(current_batch_missions, global_reservation_table)

            if batch_result is None:
                raise RuntimeError(f"Fatal: Could not find a path even for a single mission: {anchor_mission_tuple}")

        best_uav_plan = {k: v.copy() for k, v in time_uav_plan.items()}

        update_launch_task = {}
        update_recovery_task = {}
        for mission_tuple, path_data in batch_result.items():

            final_solution[mission_tuple] = path_data
            drone_id,launch_node, customer, recovery_node, launch_vehicle,recovery_vehicle = mission_tuple
            route_list = [item[0] for item in path_data]
            launch_vtp_node = node[route_list[0]].map_key
            recovery_vtp_node = node[route_list[-1]].map_key
            route_list.insert(0, launch_vtp_node)
            route_list.append(recovery_vtp_node)
            global_reservation_table = cbs_solver._update_reservation_table_with_path(global_reservation_table, path_data, drone_id, mission_tuple)
            best_uav_plan[mission_tuple]['uav_route'] = route_list
            launch_time, recovery_time = update_launch_recovery_time(node, path_data, vehicle, drone_id)
            if round(launch_time, decimals) != round(best_uav_plan[mission_tuple]['launch_time'], decimals):
                best_uav_plan[mission_tuple]['launch_time'] = launch_time
                update_launch_task[mission_tuple] = launch_time

            if round(recovery_time, decimals) != round(best_uav_plan[mission_tuple]['recovery_time'], decimals):
                best_uav_plan[mission_tuple]['recovery_time'] = recovery_time
                update_recovery_task[mission_tuple] = recovery_time

            work_time = recovery_time - launch_time
            best_uav_plan[mission_tuple]['time'] = work_time
            uav_route_cost, uav_time_cost = update_uav_cost(node, route_list, work_time, vehicle, drone_id, G_air, G_ground, mission_tuple)
            best_uav_plan[mission_tuple]['uav_route_cost'] = uav_route_cost
            best_uav_plan[mission_tuple]['uav_time_cost'] = uav_time_cost
            best_uav_cost[customer] = uav_route_cost

        if update_recovery_task:
            sorted_recovery_task = sorted(update_recovery_task.items(), key=lambda item: item[1])
            for mission_tuple, recovery_time in sorted_recovery_task:
                drone_id, launch_node, customer, recovery_node, launch_vehicle, recovery_vehicle = mission_tuple
                delta_time = time_uav_plan[mission_tuple]['recovery_time'] - best_uav_plan[mission_tuple]['recovery_time']
                if delta_time > 0:
                    vehicle_task_data = update_delta_time(delta_time, vehicle_task_data, best_vehicle_route, mission_tuple, vehicle)

        for _ in range(len(batch_result)):
            unplanned_missions_q.popleft()
    vehicle_plan_time = update_vehicle_arrive_time(vehicle_task_data, vehicle_arrive_time)
    print("[Rolling Horizon] All missions planned successfully.")

    return best_uav_plan, best_uav_cost, vehicle_plan_time, vehicle_task_data, global_reservation_table

def update_launch_recovery_time(node, path_data, vehicle, drone_id):
    launch_node = path_data[0][0]
    recovery_node = path_data[-1][0]
    air_launch_time = path_data[0][1]
    air_recovery_time = path_data[-1][1]
    init_node_alt = abs(node[launch_node].map_position[2] - node[launch_node].position[2])
    start_air_time = init_node_alt / vehicle[drone_id].takeoffSpeed
    end_node_alt = abs(node[recovery_node].map_position[2] - node[recovery_node].position[2])
    end_air_time = end_node_alt / vehicle[drone_id].landingSpeed
    launch_time = air_launch_time - start_air_time
    recovery_time = end_air_time + air_recovery_time
    return launch_time, recovery_time

def update_uav_cost(node, route_list, work_time, vehicle, drone_id, G_air, G_ground, mission_tuple):
    uav_route_cost = 0
    route_length = 0
    uav_time_cost = vehicle[drone_id].time_cost * work_time
    launch_node = route_list[0]
    recovery_node = route_list[-1]
    air_route_list = route_list[1:-1]
    for i in range(len(air_route_list) - 1):
        current_node = air_route_list[i]
        next_node = air_route_list[i+1]
        if current_node == next_node:
            continue
        else:
            route_length += G_air[current_node][next_node]['weight']
    init_node_alt = abs(node[launch_node].map_position[2] - node[launch_node].position[2])
    end_node_alt = abs(node[recovery_node].map_position[2] - node[recovery_node].position[2])
    drone_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle = mission_tuple

    cust_node_alt = 2 * abs(node[customer_node].position[2] - node[customer_node].map_position[2])
    route_length += init_node_alt + end_node_alt + cust_node_alt
    uav_route_cost = route_length * vehicle[drone_id].per_cost

    return uav_route_cost, uav_time_cost

def get_path_location(path, time):

    if not path:
        return None

    if time < path[0][1]:
        return None

    for i in range(len(path)):
        if path[i][1] >= time:

            if path[i][1] == time:
                return path[i][0]

            elif i > 0:

                return path[i-1][0]
            else:
                return path[0][0]

    return path[-1][0]

def detect_all_collisions(paths_dict):

    collisions = []
    uav_ids = list(paths_dict.keys())

    max_time = 0
    for path in paths_dict.values():
        if path:
            max_time = max(max_time, path[-1][1])

    for uav1_id, uav2_id in itertools.combinations(uav_ids, 2):
        path1 = paths_dict[uav1_id]
        path2 = paths_dict[uav2_id]

        for t in range(max_time + 1):
            loc1 = get_path_location(path1, t)
            loc2 = get_path_location(path2, t)

            if loc1 is not None and loc1 == loc2:
                collisions.append({'uavs': [uav1_id, uav2_id], 'loc': [loc1], 'timestep': t, 'type': 'vertex'})
                break

            next_loc1 = get_path_location(path1, t + 1)
            next_loc2 = get_path_location(path2, t + 1)
            if loc1 is not None and loc2 is not None and loc1 == next_loc2 and loc2 == next_loc1:
                collisions.append({'uavs': [uav1_id, uav2_id], 'loc': [loc1, loc2], 'timestep': t + 1, 'type': 'edge'})
                break

    return collisions

def standard_splitting(collision):

    constraints = []
    uav1, uav2 = collision['uavs']

    constraints.append({
        'agent': uav1,
        'loc': collision['loc'],
        'timestep': collision['timestep']
    })
    constraints.append({
        'agent': uav2,
        'loc': list(reversed(collision['loc'])) if len(collision['loc']) > 1 else collision['loc'],
        'timestep': collision['timestep']
    })
    return constraints

def get_sum_of_cost(paths_dict):

    total_cost = 0
    for path in paths_dict.values():
        if path:
            total_cost += (path[-1][1] - path[0][1])
    return total_cost

class Time_cbs_Batch_Solver:
    def __init__(self, global_reservation_table, time_uav_task_dict, time_customer_plan, time_uav_plan, vehicle_plan_time, vehicle_task_data,
    node, DEPOT_nodeID, V, T, vehicle, uav_travel, veh_distance, veh_travel, N, N_zero, N_plus, A_total,
    A_cvtp, A_vtp, A_aerial_relay_node, G_air, G_ground, air_matrix, ground_matrix, air_node_types, ground_node_types,
    A_c, xeee):
        self.global_reservation_table = global_reservation_table
        self.time_uav_task_dict = time_uav_task_dict
        self.time_customer_plan = time_customer_plan
        self.time_uav_plan = time_uav_plan
        self.vehicle_plan_time = vehicle_plan_time
        self.vehicle_task_data = vehicle_task_data
        self.node = node
        self.DEPOT_nodeID = DEPOT_nodeID
        self.V = V
        self.T = T
        self.vehicle = vehicle
        self.uav_travel = uav_travel
        self.veh_distance = veh_distance
        self.veh_travel = veh_travel
        self.N = N
        self.N_zero = N_zero
        self.N_plus = N_plus
        self.A_total = A_total
        self.A_cvtp = A_cvtp
        self.A_vtp = A_vtp
        self.A_aerial_relay_node = A_aerial_relay_node
        self.G_air = G_air
        self.G_ground = G_ground
        self.air_matrix = air_matrix
        self.ground_matrix = ground_matrix
        self.air_node_types = air_node_types
        self.ground_node_types = ground_node_types
        self.A_c = A_c
        self.xeee = xeee
        self.uav_ids = V
        self.truck_ids = T
        self.G_air = G_air

        self.open_list = []
        self.num_generated = 0
        self.num_expanded = 0

        self.heuristics = {}
        for node in self.G_air.nodes():
            self.heuristics[node] = nx.single_source_dijkstra_path_length(self.G_air, node, weight='weight')

    def solve_for_batch(self, batch_missions, reservation_table, timeout=60):

        open_list = []
        node_id_counter = 0

        reservation_table_for_root = copy.copy(reservation_table)

        root_paths = {}
        root_cost = 0

        sorted_missions = sorted(batch_missions, key=lambda x: x[1]['launch_time'])
        for mission_tuple, mission_dict in sorted_missions:
            drone_id = mission_tuple[0]

            path, reservation_table_for_root = self._plan_full_mission_path(mission_tuple, mission_dict, [],
            reservation_table_for_root)

            if path is None:
                print(f"错误: CBS初始化规划失败, 无人机 {drone_id} 的任务 {mission_tuple} 无解")
                return None

            root_paths[mission_tuple] = path

            completion_time = path[-1][1]
            root_cost += completion_time

        root_collisions = self._detect_collisions(root_paths, [])

        root_node = {
            'cost': root_cost,
            'constraints': [],
            'paths': root_paths,
            'collisions': root_collisions
        }

        heapq.heappush(open_list, (root_node['cost'], node_id_counter, root_node))
        node_id_counter += 1

        while open_list:

            _, _, current_node = heapq.heappop(open_list)

            if not current_node['collisions']:

                return current_node['paths']

            collision = current_node['collisions'][0]

            participants = collision['participants']

            for participant in participants:
                agent_id = participant['agent_id']
                mission_tuple_to_replan = participant['mission_tuple']

                mission_tuple, mission_dict = next((m for m in batch_missions if m[0] == mission_tuple_to_replan), (None, None))

                if mission_dict is None: continue

                new_constraint = self._create_constraint_from_collision(collision, agent_id, mission_tuple)
                child_constraints = current_node['constraints'] + [new_constraint]

                temp_reservation_table = self._rebuild_reservation_table(
                    reservation_table,
                    current_node['paths'],
                    agent_id,
                    mission_tuple
                )

                new_path_tuple = self._plan_full_mission_path(
                    mission_tuple,
                    mission_dict,
                    child_constraints,
                    temp_reservation_table,
                )
                new_path, _ = new_path_tuple

                child_paths = current_node['paths'].copy()
                child_paths[mission_tuple] = new_path

                child_cost = self._calculate_solution_cost(child_paths)
                child_collisions = self._detect_collisions(child_paths, child_constraints)

                child_node = {
                    'cost': child_cost,
                    'constraints': child_constraints,
                    'paths': child_paths,
                    'collisions': child_collisions
                }

                heapq.heappush(open_list, (child_node['cost'], node_id_counter, child_node))
                node_id_counter += 1

        print("CBS求解失败，未找到解决方案。")
        return None

    def _init_global_reservation_table(self, batch_missions, reservation_table):

        updated_table = copy.copy(reservation_table)

        for mission_tuple, mission_dict in batch_missions:

            drone_id = mission_tuple[0]
            launch_node = mission_tuple[1]
            launch_time = mission_dict.get('launch_time', 0.0)
            init_node_alt = abs(self.node[launch_node].map_position[2] - self.node[launch_node].position[2])
            start_air_time = init_node_alt / self.vehicle[drone_id].landSpeed
            end_air_time = start_air_time + launch_time

            if launch_time > 0:

                occupancy_interval = {
                    'start': start_air_time,
                    'end': end_air_time,
                    'drone_id': drone_id
                }

                if launch_node not in updated_table:
                    updated_table[launch_node] = []

                updated_table[launch_node].append(occupancy_interval)

        return updated_table

    def _rebuild_reservation_table(self, base_table, all_paths, agent_to_exclude, mission_to_exclude):

        temp_table = copy.deepcopy(base_table)
        for mission_tuple, path in all_paths.items():

            if mission_tuple != mission_to_exclude:
                drone_id = mission_tuple[0]
                self._update_reservation_table_with_path(temp_table, path, drone_id, mission_tuple)
        return temp_table

    def _calculate_solution_cost(self, paths_dict):

        total_cost = 0
        for mission_tuple, path in paths_dict.items():
            if not path: continue
            launch_time = path[0][1]
            completion_time = path[-1][1]
            total_cost += (completion_time - launch_time)
        return total_cost

    def _update_reservation_table_with_path(self, reservation_table, path, drone_id, mission_tuple):

        if not path or len(path) < 2:
            return

        launch_node, launch_ready_time = path[0]

        climb_altitude = abs(self.node[launch_node].map_position[2] - self.node[launch_node].position[2])
        climb_time = climb_altitude / self.vehicle[drone_id].takeoffSpeed

        climb_start_time = launch_ready_time - climb_time

        if launch_node not in reservation_table:
            reservation_table[launch_node] = []
        reservation_table[launch_node].append({
            'start': climb_start_time,
            'end': launch_ready_time,
            'arrive_time': launch_ready_time,
            'drone_id': drone_id,
            'mission_tuple': mission_tuple,
            'type': 'climb'
        })

        for i in range(len(path) - 1):
            start_node, start_time = path[i]
            end_node, end_time = path[i+1]

            if start_time >= end_time:
                continue

            nodes_to_reserve = [start_node, end_node]

            for index, node in enumerate(nodes_to_reserve):
                if node not in reservation_table:
                    reservation_table[node] = []
                if index == 0:
                    reservation_table[node].append({
                        'start': start_time,
                        'end': end_time,
                        'arrive_time': start_time,
                        'drone_id': drone_id,
                        'mission_tuple': mission_tuple,
                        'type': 'flight'
                    })
                else:
                    reservation_table[node].append({
                        'start': start_time,
                        'end': end_time,
                        'arrive_time': end_time,
                        'drone_id': drone_id,
                        'mission_tuple': mission_tuple,
                        'type': 'flight'
                    })

        land_node, land_arrival_time = path[-1]

        land_altitude = abs(self.node[land_node].map_position[2] - self.node[land_node].position[2])
        land_time = land_altitude / self.vehicle[drone_id].landingSpeed

        land_complete_time = land_arrival_time + land_time

        if land_node not in reservation_table:
            reservation_table[land_node] = []
        reservation_table[land_node].append({
            'start': land_arrival_time,
            'end': land_complete_time,
            'arrive_time': land_complete_time,
            'drone_id': drone_id,
            'mission_tuple': mission_tuple,
            'type': 'land'
        })
        return reservation_table

    def push_node(self, node):

        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_generated, node))
        self.num_generated += 1

    def pop_node(self):
        _, _, _, node = heapq.heappop(self.open_list)
        self.num_expanded += 1
        return node

    def _plan_full_mission_path(self, mission_tuple, mission_dict, constraints, reservation_table):

            drone_id, launch_node, customer_node, recovery_node, _, _ = mission_tuple
            service_time = self.node[customer_node].serviceTimeUAV

            climb_altitude = abs(self.node[launch_node].map_position[2] - self.node[launch_node].position[2])
            climb_time = climb_altitude / self.vehicle[drone_id].takeoffSpeed
            path1 = self.spatio_temporal_a_star(
                self.G_air,
                self.node[launch_node].map_key,
                customer_node,
                mission_dict['launch_time'] + climb_time,
                self.heuristics[customer_node],
                constraints,
                reservation_table,
                drone_id
            )

            if not path1:
                print(f"找不到可行路径！Debug: Path1 failed for {drone_id}")
                return None, None

            arrival_time_in_air = path1[-1][1]

            land_altitude = abs(self.node[customer_node].map_position[2] - self.node[customer_node].position[2])
            land_duration = land_altitude / self.vehicle[drone_id].landingSpeed

            climb_duration = land_altitude / self.vehicle[drone_id].takeoffSpeed

            departure_time_for_path2 = arrival_time_in_air + land_duration + service_time + climb_duration

            self._update_reservation_table_with_path(reservation_table, path1, drone_id, mission_tuple)

            temp_reservation_table = copy.copy(reservation_table)

            if customer_node not in temp_reservation_table:
                temp_reservation_table[customer_node] = []

            temp_reservation_table[customer_node].append({
                'start': arrival_time_in_air + land_duration,
                'end': arrival_time_in_air + land_duration + service_time,
                'arrive_time': arrival_time_in_air + land_duration,
                'drone_id': drone_id,
                'type': 'service_stop'
            })

            path2 = self.spatio_temporal_a_star(
                self.G_air,
                customer_node,
                self.node[recovery_node].map_key,
                departure_time_for_path2 ,
                self.heuristics[self.node[recovery_node].map_key],
                constraints,
                temp_reservation_table,
                drone_id
            )

            if not path2:

                return None, None

            self._update_reservation_table_with_path(temp_reservation_table, path2, drone_id, mission_tuple)

            full_path = path1 + path2[1:]

            return full_path, temp_reservation_table

    def _detect_collisions(self, paths, constraints):

        all_collisions = []

        temp_reservation_table = {}
        for mission_tuple, path in paths.items():
            drone_id = mission_tuple[0]
            self._update_reservation_table_with_path(temp_reservation_table, path, drone_id, mission_tuple)

        for node, intervals in temp_reservation_table.items():
            for interval1, interval2 in itertools.combinations(intervals, 2):
                if interval1['drone_id'] == interval2['drone_id']:
                    continue
                time_diff = abs(interval1['arrive_time'] - interval2['arrive_time'])
                if time_diff < 1e-6:
                    collision_time = interval1['arrive_time']
                    colliding_agents = sorted([interval1['drone_id'], interval2['drone_id']])
                    collision = {
                        'type': 'vertex',
                        'loc': [node],
                        'time': collision_time,
                        'participants': [
                            {'agent_id': interval1['drone_id'], 'mission_tuple': interval1['mission_tuple']},
                            {'agent_id': interval2['drone_id'], 'mission_tuple': interval2['mission_tuple']}
                        ]
                    }

                    is_duplicate = any(
                        c['type'] == 'vertex' and c['loc'][0] == node and
                        set(p['agent_id'] for p in c['participants']) == {interval1['drone_id'], interval2['drone_id']}
                        for c in all_collisions
                    )
                    if not is_duplicate:
                        all_collisions.append(collision)

        path_items = list(paths.items())
        for (mission1, path1), (mission2, path2) in itertools.combinations(path_items, 2):
            drone1_id = mission1[0]
            drone2_id = mission2[0]

            for i in range(len(path1) - 1):
                p1_start_node, p1_start_time = path1[i]
                p1_end_node, p1_end_time = path1[i+1]

                for j in range(len(path2) - 1):
                    p2_start_node, p2_start_time = path2[j]
                    p2_end_node, p2_end_time = path2[j+1]

                    is_swapping = (p1_start_node == p2_end_node and p1_end_node == p2_start_node)

                    if not is_swapping:
                        continue

                    time_overlap = max(p1_start_time, p2_start_time) < min(p1_end_time, p2_end_time)

                    if time_overlap:
                        collision = {
                        'type': 'edge',
                        'participants': [
                            {'agent_id': drone1_id, 'mission_tuple': mission1, 'loc': [p1_start_node, p1_end_node], 'time': p1_end_time},
                            {'agent_id': drone2_id, 'mission_tuple': mission2, 'loc': [p2_start_node, p2_end_node], 'time': p2_end_time}
                        ]}

                        nodes_involved = frozenset([p1_start_node, p1_end_node])
                        is_duplicate = any(
                            c['type'] == 'edge' and
                            set(p['agent_id'] for p in c['participants']) == {drone1_id, drone2_id} and
                            frozenset(c['participants'][0]['loc']) == nodes_involved
                            for c in all_collisions
                        )
                        if not is_duplicate:
                            all_collisions.append(collision)

        for constraint in constraints:

            agent_id = constraint['agent']
            constrained_mission_tuple = constraint['mission_tuple']
            constrained_loc = constraint['loc']
            constrained_time = constraint['time']

            path = paths.get(constrained_mission_tuple)
            if not path: continue

            if len(constrained_loc) == 1:
                node_to_check = constrained_loc[0]
                for node, time in path:
                    if node == node_to_check and abs(time - constrained_time) < 1e-9:
                        violation = {
                            'type': 'constraint_violation',
                            'constraint': constraint,
                            'participants': [{'agent_id': agent_id, 'mission_tuple': constrained_mission_tuple}]
                        }
                        if violation not in all_collisions:
                            all_collisions.append(violation)
                        break

            elif len(constrained_loc) == 2:
                [c_start, c_end] = constrained_loc
                for i in range(len(path) - 1):
                    p_start, _ = path[i]
                    p_end, p_end_time = path[i+1]
                    if p_start == c_start and p_end == c_end and abs(p_end_time - constrained_time) < 1e-9:
                        violation = {
                            'type': 'constraint_violation',
                            'constraint': constraint,
                            'participants': [{'agent_id': agent_id, 'mission_tuple': constrained_mission_tuple}]
                        }
                        if violation not in all_collisions:
                            all_collisions.append(violation)
                        break

        return all_collisions

    def _create_constraint_from_collision(self, collision, agent_id_to_constrain, mission_tuple):

        collision_type = collision['type']

        if collision_type == 'vertex':

            return {
                'agent': agent_id_to_constrain,
                'mission_tuple': mission_tuple,
                'loc': collision['loc'],
                'time': collision['time'],
                'type': 'vertex'
            }

        elif collision_type == 'edge':

            constraint_details = next(
                (p for p in collision['participants'] if p['agent_id'] == agent_id_to_constrain),
                None
            )

            if not constraint_details:
                return None

            return {
                'agent': agent_id_to_constrain,
                'mission_tuple': mission_tuple,
                'loc': constraint_details['loc'],
                'time': constraint_details['time'],
                'type': 'edge'
            }

        else:

            return None

    def spatio_temporal_a_star(self, graph, start_node, goal_node, launch_time, heuristic, constraints, reservation_table, drone_id):

        open_list = []

        heapq.heappush(open_list, (launch_time + heuristic[start_node]/self.vehicle[drone_id].cruiseSpeed, launch_time, (launch_time, start_node)))
        came_from = {}
        g_score = { (launch_time, start_node): launch_time }

        EPSILON = 1e-4
        while open_list:
            _, current_g, current_state = heapq.heappop(open_list)
            current_time, current_node = current_state

            if current_g > g_score.get(current_state, float('inf')):
                continue
            if current_node == goal_node:

                path = []
                temp = current_state
                while temp in came_from:
                    path.append((temp[1], temp[0]))
                    temp = came_from[temp]
                path.append((start_node, launch_time))
                return path[::-1]

            all_possible_neighbors = list(graph.neighbors(current_node)) + [current_node]
            for neighbor in all_possible_neighbors:

                if neighbor == current_node:

                    travel_time = 0.02
                else:

                    edge_data = graph.get_edge_data(current_node, neighbor)
                    distance = edge_data.get('weight', 1)
                    travel_time = distance / self.vehicle[drone_id].cruiseSpeed

                earliest_arrival_time = current_time + travel_time

                safe_arrival_time = earliest_arrival_time
                loop_count = 0
                MAX_LOOPS = 50
                while True:

                    last_conflict_end_time = -1

                    is_constrained = False
                    temp_arrival_time = safe_arrival_time

                    while True:
                        cbs_conflict_found = False

                        for c in constraints:
                            if len(c['loc']) == 1 and c['loc'][0] == neighbor and c['time'] == temp_arrival_time:
                                cbs_conflict_found = True
                                break

                        if not cbs_conflict_found and neighbor != current_node:
                            for c in constraints:
                                if len(c['loc']) == 2 and c['loc'] == [current_node, neighbor] and c['time'] == temp_arrival_time:
                                    cbs_conflict_found = True
                                    break

                        if cbs_conflict_found:
                            last_conflict_end_time = max(last_conflict_end_time, temp_arrival_time)

                            temp_arrival_time += 1

                        else:
                            break

                    safe_arrival_time = temp_arrival_time

                    reserved_intervals = reservation_table.get(neighbor, [])
                    reservation_conflict_found = False

                    for interval_dict in reserved_intervals:

                        start_t = interval_dict['start']
                        end_t = interval_dict['end']
                        if interval_dict['drone_id'] == drone_id:
                            continue

                        if start_t <= safe_arrival_time < end_t:

                            last_conflict_end_time = max(last_conflict_end_time, end_t)
                            reservation_conflict_found = True

                    if reservation_conflict_found or last_conflict_end_time != -1:
                        if cbs_conflict_found:

                                safe_arrival_time += 1

                        if reservation_conflict_found:
                            safe_arrival_time = max(safe_arrival_time, last_conflict_end_time)

                        continue

                    break

                next_state = (safe_arrival_time, neighbor)
                new_g = safe_arrival_time

                if new_g < g_score.get(next_state, float('inf')):
                    g_score[next_state] = new_g
                    h_value = heuristic.get(neighbor, 0) / self.vehicle[drone_id].cruiseSpeed
                    f_value = new_g + h_value
                    heapq.heappush(open_list, (f_value, new_g, next_state))

                    came_from[next_state] = current_state
        return None
        print('无人机的低空城市走廊规划任务中找到了无解的路径，请检查任务后重新规划')

import numpy as np
import copy
from task_data import vehicle_task
from cost_y import calculate_plan_cost
from task_data import *
from cost_y import *
from call_function import *
from initialize import *
import heapq
import itertools
from collections import deque
import networkx as nx
import time
import itertools

def low_update_time(uav_task_dict, best_uav_plan, best_vehicle_route, vehicle_task_data, vehicle_arrival_time, node, V, T, vehicle, uav_travel):

    detailed_vehicle_task_data = deep_copy_vehicle_task_data(vehicle_task_data)
    trucks_id = T
    uav_id = V
    time_uav_task_dict = {}
    time_customer_plan = {}
    vehicle_plan_time = {}
    for id in uav_id:
        time_uav_task_dict[id] = []
    vehicle_events = defaultdict(lambda: defaultdict(list))
    for drone_id, missions in uav_task_dict.items():
        for flight_tuple in missions:
            _, launch_node, _, recovery_node, launch_veh_id, recovery_veh_id = flight_tuple
            vehicle_events[launch_veh_id][launch_node].append({
                "type": "launch", "task_type": TASK_DRONE_LAUNCH, "drone_id": drone_id, "flight_tuple": flight_tuple
            })
            vehicle_events[recovery_veh_id][recovery_node].append({
                "type": "recovery", "task_type": TASK_DRONE_RECOVERY, "drone_id": drone_id, "flight_tuple": flight_tuple
            })
    sorted_items = sorted(best_uav_plan.items(), key=lambda item: item[1]['launch_time'])
    sorted_mission_keys = [item[0] for item in sorted_items]

    for id in trucks_id:
        idx = id -1
        vehicle_route = best_vehicle_route[idx]

        for index, node_id in enumerate(vehicle_route[:-1]):
            if index == 0:

                arrive_time = vehicle_arrival_time[id][node_id]
                departure_time = vehicle_arrival_time[id][node_id]
                detailed_vehicle_task_data[id][node_id].add_node(node_id, arrive_time, departure_time)
                detailed_vehicle_task_data[id][node_id].prcise_arrive_time = arrive_time
                detailed_vehicle_task_data[id][node_id].prcise_departure_time = departure_time

                carry_drone_list = detailed_vehicle_task_data[id][node_id].drone_list
                launch_drone_list = detailed_vehicle_task_data[id][node_id].launch_drone_list
                recovery_drone_list = detailed_vehicle_task_data[id][node_id].recovery_drone_list
                launch_drone_list = normalize_input_data(launch_drone_list)
                carry_drone_list = normalize_input_data(carry_drone_list)
                recovery_drone_list = normalize_input_data(recovery_drone_list)

                if carry_drone_list is not None:
                    for drone_id in carry_drone_list:
                        detailed_vehicle_task_data[drone_id][node_id].drone_belong = id
                        detailed_vehicle_task_data[drone_id][node_id].add_node(node_id, arrive_time, departure_time)
                        detailed_vehicle_task_data[drone_id][node_id].prcise_arrive_time = arrive_time
                        detailed_vehicle_task_data[drone_id][node_id].dict_vehicle[id]['prcise_arrive_time'] = arrive_time
                        detailed_vehicle_task_data[drone_id][node_id].prcise_departure_time = departure_time
                        detailed_vehicle_task_data[drone_id][node_id].dict_vehicle[id]['prcise_departure_time'] = departure_time
                        detailed_vehicle_task_data[drone_id][node_id].arrive_times.append(arrive_time)
                        detailed_vehicle_task_data[drone_id][node_id].departure_times.append(arrive_time)
                if launch_drone_list is not None:
                    for drone_id in launch_drone_list:
                        detailed_vehicle_task_data[drone_id][node_id].drone_belong = id
                        detailed_vehicle_task_data[drone_id][node_id].add_node(node_id, arrive_time, departure_time)
                        detailed_vehicle_task_data[drone_id][node_id].prcise_arrive_time = arrive_time
                        detailed_vehicle_task_data[drone_id][node_id].dict_vehicle[id]['prcise_arrive_time'] = arrive_time
                        detailed_vehicle_task_data[drone_id][node_id].prcise_departure_time = departure_time
                        detailed_vehicle_task_data[drone_id][node_id].dict_vehicle[id]['prcise_departure_time'] = departure_time
                        detailed_vehicle_task_data[drone_id][node_id].arrive_times.append(arrive_time)
                        detailed_vehicle_task_data[drone_id][node_id].departure_times.append(arrive_time)
            else:
                arrive_time = vehicle_arrival_time[id][node_id]
                departure_time = arrive_time
                detailed_vehicle_task_data[id][node_id].add_node(node_id, arrive_time, departure_time)
                detailed_vehicle_task_data[id][node_id].prcise_arrive_time = arrive_time

                detailed_vehicle_task_data[id][node_id].prcise_departure_time = departure_time

                detailed_vehicle_task_data[id][node_id].arrive_times.append(arrive_time)

                detailed_vehicle_task_data[id][node_id].departure_times.append(departure_time)

                carry_drone_list = detailed_vehicle_task_data[id][node_id].drone_list
                launch_drone_list = detailed_vehicle_task_data[id][node_id].launch_drone_list
                recovery_drone_list = detailed_vehicle_task_data[id][node_id].recovery_drone_list
                carry_drone_list = normalize_input_data(carry_drone_list)
                launch_drone_list = normalize_input_data(launch_drone_list)
                recovery_drone_list = normalize_input_data(recovery_drone_list)
                if carry_drone_list is not None:
                    for drone_id in carry_drone_list:
                        detailed_vehicle_task_data[drone_id][node_id].drone_belong = id
                        detailed_vehicle_task_data[drone_id][node_id].add_node(node_id, arrive_time, departure_time)
                        detailed_vehicle_task_data[drone_id][node_id].prcise_arrive_time = arrive_time
                        detailed_vehicle_task_data[drone_id][node_id].dict_vehicle[id]['prcise_arrive_time'] = arrive_time
                        detailed_vehicle_task_data[drone_id][node_id].prcise_departure_time = departure_time
                        detailed_vehicle_task_data[drone_id][node_id].dict_vehicle[id]['prcise_departure_time'] = departure_time
                        detailed_vehicle_task_data[drone_id][node_id].arrive_times.append(arrive_time)
                        detailed_vehicle_task_data[drone_id][node_id].departure_times.append(arrive_time)
                if launch_drone_list is not None:
                    for drone_id in launch_drone_list:
                        detailed_vehicle_task_data[drone_id][node_id].drone_belong = id
                        detailed_vehicle_task_data[drone_id][node_id].add_node(node_id, arrive_time, departure_time)
                        detailed_vehicle_task_data[drone_id][node_id].prcise_arrive_time = arrive_time
                        detailed_vehicle_task_data[drone_id][node_id].dict_vehicle[id]['prcise_arrive_time'] = arrive_time
                        detailed_vehicle_task_data[drone_id][node_id].prcise_departure_time = departure_time
                        detailed_vehicle_task_data[drone_id][node_id].dict_vehicle[id]['prcise_departure_time'] = departure_time
                        detailed_vehicle_task_data[drone_id][node_id].arrive_times.append(arrive_time)
                        detailed_vehicle_task_data[drone_id][node_id].departure_times.append(departure_time)

    for index, y_ijkd in enumerate(sorted_mission_keys):
        drone_id, launch_node, customer_node, recovery_node, launch_veh_id, recovery_veh_id = y_ijkd
        time_customer_plan[customer_node] = y_ijkd
        time_uav_task_dict[drone_id].append(y_ijkd)
        launch_map_key = node[launch_node].map_key
        recovery_map_key = node[recovery_node].map_key
        truck_departure_time = detailed_vehicle_task_data[launch_veh_id][launch_node].prcise_departure_time
        drone_start_launch_time = truck_departure_time
        drone_end_launch_time = drone_start_launch_time + vehicle[drone_id].launchTime

        detailed_vehicle_task_data[drone_id][launch_node].add_task(launch_node, 13, drone_start_launch_time, drone_end_launch_time)

        detailed_vehicle_task_data[launch_veh_id][launch_node].prcise_departure_time = drone_end_launch_time
        detailed_vehicle_task_data[drone_id][launch_node].prcise_departure_time = drone_end_launch_time
        detailed_vehicle_task_data[drone_id][launch_node].dict_vehicle[launch_veh_id]['prcise_departure_time'] = drone_end_launch_time
        detailed_vehicle_task_data[drone_id][launch_node].departure_times.append(drone_end_launch_time)
        detailed_vehicle_task_data[drone_id][launch_node].dict_vehicle[launch_veh_id]['departure_times'].append(drone_end_launch_time)
        detailed_vehicle_task_data[launch_veh_id][launch_node].departure_times.append(drone_end_launch_time)

        uav_to_c_time = uav_travel[drone_id][launch_map_key][customer_node].totalTime
        uav_arr_c_time = drone_end_launch_time + uav_to_c_time
        detailed_vehicle_task_data[drone_id][launch_node].add_task(launch_node, 18, drone_end_launch_time, uav_arr_c_time)

        customer_service_time = node[customer_node].serviceTimeUAV
        uav_depart_customer_time = uav_arr_c_time + customer_service_time
        detailed_vehicle_task_data[drone_id][customer_node].add_task(customer_node, 15, uav_arr_c_time, uav_depart_customer_time)

        detailed_vehicle_task_data[drone_id][customer_node].prcise_arrive_time = uav_arr_c_time
        detailed_vehicle_task_data[drone_id][customer_node].dict_vehicle[launch_veh_id]['prcise_arrive_time'] = uav_arr_c_time
        detailed_vehicle_task_data[drone_id][customer_node].prcise_departure_time = uav_depart_customer_time
        detailed_vehicle_task_data[drone_id][customer_node].dict_vehicle[launch_veh_id]['prcise_departure_time'] = uav_depart_customer_time
        detailed_vehicle_task_data[drone_id][customer_node].arrive_times.append(uav_arr_c_time)
        detailed_vehicle_task_data[drone_id][customer_node].departure_times.append(uav_depart_customer_time)
        detailed_vehicle_task_data[drone_id][customer_node].dict_vehicle[launch_veh_id]['arrive_times'].append(uav_arr_c_time)
        detailed_vehicle_task_data[drone_id][customer_node].dict_vehicle[launch_veh_id]['departure_times'].append(uav_depart_customer_time)

        uav_to_j_time = uav_travel[drone_id][customer_node][recovery_map_key].totalTime
        uav_arr_j_time = uav_depart_customer_time + uav_to_j_time
        detailed_vehicle_task_data[drone_id][recovery_node].add_task(recovery_node, 19, uav_depart_customer_time, uav_arr_j_time)

        detailed_vehicle_task_data[drone_id][recovery_node].arrive_times.append(uav_arr_j_time)
        detailed_vehicle_task_data[drone_id][recovery_node].dict_vehicle[recovery_veh_id]['arrive_times'].append(uav_arr_j_time)
        detailed_vehicle_task_data[drone_id][recovery_node].prcise_arrive_time = uav_arr_j_time
        detailed_vehicle_task_data[drone_id][recovery_node].dict_vehicle[recovery_veh_id]['prcise_arrive_time'] = uav_arr_j_time

        delta_time = detailed_vehicle_task_data[launch_veh_id][launch_node].prcise_departure_time - drone_start_launch_time
        detailed_vehicle_task_data = update_delta_time(delta_time, detailed_vehicle_task_data, best_vehicle_route, y_ijkd, vehicle)

    uav_plan_time = update_uav_plan(detailed_vehicle_task_data, best_uav_plan)
    vehicle_plan_time = update_vehicle_arrive_time(detailed_vehicle_task_data, vehicle_arrival_time)

    sorted_uav_plan_time = sorted(uav_plan_time.items(), key=lambda item: item[1]['launch_time'])
    uav_plan_time = {k: v for k, v in sorted_uav_plan_time}

    return time_uav_task_dict, time_customer_plan, uav_plan_time, vehicle_plan_time, detailed_vehicle_task_data

def rolling_time_cbs(
    vehicle_arrive_time, best_vehicle_route, time_uav_task_dict, time_customer_plan, time_uav_plan, vehicle_plan_time,
    vehicle_task_data, node, DEPOT_nodeID, V, T, vehicle, uav_travel, veh_distance,
    veh_travel, N, N_zero, N_plus, A_total, A_cvtp, A_vtp, A_aerial_relay_node, G_air, G_ground,
    air_matrix, ground_matrix, air_node_types, ground_node_types, A_c, xeee
):

    sorted_missions = sorted(time_uav_plan.items(), key=lambda item: item[1]['launch_time'])
    unplanned_missions_q = deque(sorted_missions)

    current_batch = []

    global_reservation_table = {}
    final_uav_plan = {}
    final_solution = {}
    best_uav_cost = {}
    cbs_solver = Time_cbs_Batch_Solver(global_reservation_table, time_uav_task_dict, time_customer_plan, time_uav_plan, vehicle_plan_time, vehicle_task_data,
    node, DEPOT_nodeID, V, T, vehicle, uav_travel, veh_distance, veh_travel, N, N_zero, N_plus, A_total,
    A_cvtp, A_vtp, A_aerial_relay_node, G_air, G_ground, air_matrix, ground_matrix, air_node_types, ground_node_types,
    A_c, xeee)
    decimals = 4

    while unplanned_missions_q:

        anchor_mission_tuple, anchor_mission_dict = unplanned_missions_q[0]

        t_start = anchor_mission_dict['launch_time']

        t_end_window = anchor_mission_dict['recovery_time'] * 1.0
        current_batch_missions = []
        MAX_BATCH_SIZE = 15
        selected_drone_ids = set()
        temp_q = unplanned_missions_q.copy()
        while temp_q and len(current_batch_missions) < MAX_BATCH_SIZE:
            mission_tuple, mission_dict = temp_q.popleft()
            drone_id = mission_tuple[0]
            if mission_dict['launch_time'] <= t_end_window and drone_id not in selected_drone_ids:

                current_batch_missions.append((mission_tuple, mission_dict))
                selected_drone_ids.add(drone_id)
            else:
                break

        batch_ids_for_log = [m[0] for m in current_batch_missions]

        batch_result = cbs_solver.solve_for_batch(current_batch_missions, global_reservation_table)

        if batch_result is None:

            print(f"Warning: CBS failed for batch. Retrying with anchor mission only.")
            current_batch_missions = [unplanned_missions_q[0]]
            batch_result = cbs_solver.solve_for_batch(current_batch_missions, global_reservation_table)

            if batch_result is None:
                raise RuntimeError(f"Fatal: Could not find a path even for a single mission: {anchor_mission_tuple}")

        best_uav_plan = {k: v.copy() for k, v in time_uav_plan.items()}

        update_launch_task = {}
        update_recovery_task = {}
        for mission_tuple, path_data in batch_result.items():

            final_solution[mission_tuple] = path_data
            drone_id,launch_node, customer, recovery_node, launch_vehicle,recovery_vehicle = mission_tuple
            route_list = [item[0] for item in path_data]
            launch_vtp_node = node[route_list[0]].map_key
            recovery_vtp_node = node[route_list[-1]].map_key
            route_list.insert(0, launch_vtp_node)
            route_list.append(recovery_vtp_node)
            global_reservation_table = cbs_solver._update_reservation_table_with_path(global_reservation_table, path_data, drone_id, mission_tuple)
            best_uav_plan[mission_tuple]['uav_route'] = route_list
            launch_time, recovery_time = update_launch_recovery_time(node, path_data, vehicle, drone_id)
            if round(launch_time, decimals) != round(best_uav_plan[mission_tuple]['launch_time'], decimals):
                best_uav_plan[mission_tuple]['launch_time'] = launch_time
                update_launch_task[mission_tuple] = launch_time
                print('发射时间产生改变。')
            if round(recovery_time, decimals) != round(best_uav_plan[mission_tuple]['recovery_time'], decimals):
                best_uav_plan[mission_tuple]['recovery_time'] = recovery_time
                update_recovery_task[mission_tuple] = recovery_time
                print('回收时间产生改变。')
            work_time = recovery_time - launch_time
            best_uav_plan[mission_tuple]['time'] = work_time
            uav_route_cost, uav_time_cost = update_uav_cost(node, route_list, work_time, vehicle, drone_id, G_air, G_ground, mission_tuple)
            best_uav_plan[mission_tuple]['uav_route_cost'] = uav_route_cost
            best_uav_plan[mission_tuple]['uav_time_cost'] = uav_time_cost
            best_uav_cost[customer] = uav_route_cost

        if update_recovery_task:
            sorted_recovery_task = sorted(update_recovery_task.items(), key=lambda item: item[1])
            for mission_tuple, recovery_time in sorted_recovery_task:
                drone_id, launch_node, customer, recovery_node, launch_vehicle, recovery_vehicle = mission_tuple
                delta_time = time_uav_plan[mission_tuple]['recovery_time'] - best_uav_plan[mission_tuple]['recovery_time']
                if delta_time > 0:
                    vehicle_task_data = update_delta_time(delta_time, vehicle_task_data, best_vehicle_route, mission_tuple, vehicle)

        for _ in range(len(batch_result)):
            unplanned_missions_q.popleft()
    vehicle_plan_time = update_vehicle_arrive_time(vehicle_task_data, vehicle_arrive_time)
    print("[Rolling Horizon] All missions planned successfully.")

    return best_uav_plan, best_uav_cost, vehicle_plan_time, vehicle_task_data, global_reservation_table

def update_launch_recovery_time(node, path_data, vehicle, drone_id):
    launch_node = path_data[0][0]
    recovery_node = path_data[-1][0]
    air_launch_time = path_data[0][1]
    air_recovery_time = path_data[-1][1]
    init_node_alt = abs(node[launch_node].map_position[2] - node[launch_node].position[2])
    start_air_time = init_node_alt / vehicle[drone_id].takeoffSpeed
    end_node_alt = abs(node[recovery_node].map_position[2] - node[recovery_node].position[2])
    end_air_time = end_node_alt / vehicle[drone_id].landingSpeed
    launch_time = air_launch_time - start_air_time
    recovery_time = end_air_time + air_recovery_time
    return launch_time, recovery_time

def update_uav_cost(node, route_list, work_time, vehicle, drone_id, G_air, G_ground, mission_tuple):
    uav_route_cost = 0
    route_length = 0
    uav_time_cost = vehicle[drone_id].time_cost * work_time
    launch_node = route_list[0]
    recovery_node = route_list[-1]
    air_route_list = route_list[1:-1]
    for i in range(len(air_route_list) - 1):
        current_node = air_route_list[i]
        next_node = air_route_list[i+1]
        if current_node == next_node:
            continue
        else:
            route_length += G_air[current_node][next_node]['weight']
    init_node_alt = abs(node[launch_node].map_position[2] - node[launch_node].position[2])
    end_node_alt = abs(node[recovery_node].map_position[2] - node[recovery_node].position[2])
    drone_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle = mission_tuple

    cust_node_alt = 2 * abs(node[customer_node].position[2] - node[customer_node].map_position[2])
    route_length += init_node_alt + end_node_alt + cust_node_alt
    uav_route_cost = route_length * vehicle[drone_id].per_cost

    return uav_route_cost, uav_time_cost

def get_path_location(path, time):

    if not path:
        return None

    if time < path[0][1]:
        return None

    for i in range(len(path)):
        if path[i][1] >= time:

            if path[i][1] == time:
                return path[i][0]

            elif i > 0:

                return path[i-1][0]
            else:
                return path[0][0]

    return path[-1][0]

def detect_all_collisions(paths_dict):

    collisions = []
    uav_ids = list(paths_dict.keys())

    max_time = 0
    for path in paths_dict.values():
        if path:
            max_time = max(max_time, path[-1][1])

    for uav1_id, uav2_id in itertools.combinations(uav_ids, 2):
        path1 = paths_dict[uav1_id]
        path2 = paths_dict[uav2_id]

        for t in range(max_time + 1):
            loc1 = get_path_location(path1, t)
            loc2 = get_path_location(path2, t)

            if loc1 is not None and loc1 == loc2:
                collisions.append({'uavs': [uav1_id, uav2_id], 'loc': [loc1], 'timestep': t, 'type': 'vertex'})
                break

            next_loc1 = get_path_location(path1, t + 1)
            next_loc2 = get_path_location(path2, t + 1)
            if loc1 is not None and loc2 is not None and loc1 == next_loc2 and loc2 == next_loc1:
                collisions.append({'uavs': [uav1_id, uav2_id], 'loc': [loc1, loc2], 'timestep': t + 1, 'type': 'edge'})
                break

    return collisions

def standard_splitting(collision):

    constraints = []
    uav1, uav2 = collision['uavs']

    constraints.append({
        'agent': uav1,
        'loc': collision['loc'],
        'timestep': collision['timestep']
    })
    constraints.append({
        'agent': uav2,
        'loc': list(reversed(collision['loc'])) if len(collision['loc']) > 1 else collision['loc'],
        'timestep': collision['timestep']
    })
    return constraints

def get_sum_of_cost(paths_dict):

    total_cost = 0
    for path in paths_dict.values():
        if path:
            total_cost += (path[-1][1] - path[0][1])
    return total_cost

class Time_cbs_Batch_Solver:
    def __init__(self, global_reservation_table, time_uav_task_dict, time_customer_plan, time_uav_plan, vehicle_plan_time, vehicle_task_data,
    node, DEPOT_nodeID, V, T, vehicle, uav_travel, veh_distance, veh_travel, N, N_zero, N_plus, A_total,
    A_cvtp, A_vtp, A_aerial_relay_node, G_air, G_ground, air_matrix, ground_matrix, air_node_types, ground_node_types,
    A_c, xeee):
        self.global_reservation_table = global_reservation_table
        self.time_uav_task_dict = time_uav_task_dict
        self.time_customer_plan = time_customer_plan
        self.time_uav_plan = time_uav_plan
        self.vehicle_plan_time = vehicle_plan_time
        self.vehicle_task_data = vehicle_task_data
        self.node = node
        self.DEPOT_nodeID = DEPOT_nodeID
        self.V = V
        self.T = T
        self.vehicle = vehicle
        self.uav_travel = uav_travel
        self.veh_distance = veh_distance
        self.veh_travel = veh_travel
        self.N = N
        self.N_zero = N_zero
        self.N_plus = N_plus
        self.A_total = A_total
        self.A_cvtp = A_cvtp
        self.A_vtp = A_vtp
        self.A_aerial_relay_node = A_aerial_relay_node
        self.G_air = G_air
        self.G_ground = G_ground
        self.air_matrix = air_matrix
        self.ground_matrix = ground_matrix
        self.air_node_types = air_node_types
        self.ground_node_types = ground_node_types
        self.A_c = A_c
        self.xeee = xeee
        self.uav_ids = V
        self.truck_ids = T
        self.G_air = G_air

        self.open_list = []
        self.num_generated = 0
        self.num_expanded = 0

        self.heuristics = {}
        for node in self.G_air.nodes():
            self.heuristics[node] = nx.single_source_dijkstra_path_length(self.G_air, node, weight='weight')

    def solve_for_batch(self, batch_missions, reservation_table, timeout=60):

        open_list = []
        node_id_counter = 0

        reservation_table_for_root = copy.copy(reservation_table)

        root_paths = {}
        root_cost = 0

        sorted_missions = sorted(batch_missions, key=lambda x: x[1]['launch_time'])
        for mission_tuple, mission_dict in sorted_missions:
            drone_id = mission_tuple[0]

            path, reservation_table_for_root = self._plan_full_mission_path(mission_tuple, mission_dict, [],
            reservation_table_for_root)

            if path is None:
                print(f"错误: CBS初始化规划失败, 无人机 {drone_id} 的任务 {mission_tuple} 无解")
                return None

            root_paths[mission_tuple] = path

            completion_time = path[-1][1]
            root_cost += completion_time

        root_collisions = self._detect_collisions(root_paths, [])

        root_node = {
            'cost': root_cost,
            'constraints': [],
            'paths': root_paths,
            'collisions': root_collisions
        }

        heapq.heappush(open_list, (root_node['cost'], node_id_counter, root_node))
        node_id_counter += 1

        while open_list:

            _, _, current_node = heapq.heappop(open_list)

            if not current_node['collisions']:

                return current_node['paths']

            collision = current_node['collisions'][0]

            participants = collision['participants']

            for participant in participants:
                agent_id = participant['agent_id']
                mission_tuple_to_replan = participant['mission_tuple']

                mission_tuple, mission_dict = next((m for m in batch_missions if m[0] == mission_tuple_to_replan), (None, None))

                if mission_dict is None: continue

                new_constraint = self._create_constraint_from_collision(collision, agent_id, mission_tuple)
                child_constraints = current_node['constraints'] + [new_constraint]

                temp_reservation_table = self._rebuild_reservation_table(
                    reservation_table,
                    current_node['paths'],
                    agent_id,
                    mission_tuple
                )

                new_path_tuple = self._plan_full_mission_path(
                    mission_tuple,
                    mission_dict,
                    child_constraints,
                    temp_reservation_table,
                )
                new_path, _ = new_path_tuple

                child_paths = current_node['paths'].copy()
                child_paths[mission_tuple] = new_path

                child_cost = self._calculate_solution_cost(child_paths)
                child_collisions = self._detect_collisions(child_paths, child_constraints)

                child_node = {
                    'cost': child_cost,
                    'constraints': child_constraints,
                    'paths': child_paths,
                    'collisions': child_collisions
                }

                heapq.heappush(open_list, (child_node['cost'], node_id_counter, child_node))
                node_id_counter += 1

        print("CBS求解失败，未找到解决方案。")
        return None

    def _init_global_reservation_table(self, batch_missions, reservation_table):

        updated_table = copy.copy(reservation_table)

        for mission_tuple, mission_dict in batch_missions:

            drone_id = mission_tuple[0]
            launch_node = mission_tuple[1]
            launch_time = mission_dict.get('launch_time', 0.0)
            init_node_alt = abs(self.node[launch_node].map_position[2] - self.node[launch_node].position[2])
            start_air_time = init_node_alt / self.vehicle[drone_id].landSpeed
            end_air_time = start_air_time + launch_time

            if launch_time > 0:

                occupancy_interval = {
                    'start': start_air_time,
                    'end': end_air_time,
                    'drone_id': drone_id
                }

                if launch_node not in updated_table:
                    updated_table[launch_node] = []

                updated_table[launch_node].append(occupancy_interval)

        return updated_table

    def _rebuild_reservation_table(self, base_table, all_paths, agent_to_exclude, mission_to_exclude):

        temp_table = copy.deepcopy(base_table)
        for mission_tuple, path in all_paths.items():

            if mission_tuple != mission_to_exclude:
                drone_id = mission_tuple[0]
                self._update_reservation_table_with_path(temp_table, path, drone_id, mission_tuple)
        return temp_table

    def _calculate_solution_cost(self, paths_dict):

        total_cost = 0
        for mission_tuple, path in paths_dict.items():
            if not path: continue
            launch_time = path[0][1]
            completion_time = path[-1][1]
            total_cost += (completion_time - launch_time)
        return total_cost

    def _update_reservation_table_with_path(self, reservation_table, path, drone_id, mission_tuple):

        if not path or len(path) < 2:
            return

        launch_node, launch_ready_time = path[0]

        climb_altitude = abs(self.node[launch_node].map_position[2] - self.node[launch_node].position[2])
        climb_time = climb_altitude / self.vehicle[drone_id].takeoffSpeed

        climb_start_time = launch_ready_time - climb_time

        if launch_node not in reservation_table:
            reservation_table[launch_node] = []
        reservation_table[launch_node].append({
            'start': climb_start_time,
            'end': launch_ready_time,
            'arrive_time': launch_ready_time,
            'drone_id': drone_id,
            'mission_tuple': mission_tuple,
            'type': 'climb'
        })

        for i in range(len(path) - 1):
            start_node, start_time = path[i]
            end_node, end_time = path[i+1]

            if start_time >= end_time:
                continue

            nodes_to_reserve = [start_node, end_node]

            for index, node in enumerate(nodes_to_reserve):
                if node not in reservation_table:
                    reservation_table[node] = []
                if index == 0:
                    reservation_table[node].append({
                        'start': start_time,
                        'end': end_time,
                        'arrive_time': start_time,
                        'drone_id': drone_id,
                        'mission_tuple': mission_tuple,
                        'type': 'flight'
                    })
                else:
                    reservation_table[node].append({
                        'start': start_time,
                        'end': end_time,
                        'arrive_time': end_time,
                        'drone_id': drone_id,
                        'mission_tuple': mission_tuple,
                        'type': 'flight'
                    })

        land_node, land_arrival_time = path[-1]

        land_altitude = abs(self.node[land_node].map_position[2] - self.node[land_node].position[2])
        land_time = land_altitude / self.vehicle[drone_id].landingSpeed

        land_complete_time = land_arrival_time + land_time

        if land_node not in reservation_table:
            reservation_table[land_node] = []
        reservation_table[land_node].append({
            'start': land_arrival_time,
            'end': land_complete_time,
            'arrive_time': land_complete_time,
            'drone_id': drone_id,
            'mission_tuple': mission_tuple,
            'type': 'land'
        })
        return reservation_table

    def push_node(self, node):

        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_generated, node))
        self.num_generated += 1

    def pop_node(self):
        _, _, _, node = heapq.heappop(self.open_list)
        self.num_expanded += 1
        return node

    def _plan_full_mission_path(self, mission_tuple, mission_dict, constraints, reservation_table):

            drone_id, launch_node, customer_node, recovery_node, _, _ = mission_tuple
            service_time = self.node[customer_node].serviceTimeUAV

            climb_altitude = abs(self.node[launch_node].map_position[2] - self.node[launch_node].position[2])
            climb_time = climb_altitude / self.vehicle[drone_id].takeoffSpeed
            path1 = self.spatio_temporal_a_star(
                self.G_air,
                self.node[launch_node].map_key,
                customer_node,
                mission_dict['launch_time'] + climb_time,
                self.heuristics[customer_node],
                constraints,
                reservation_table,
                drone_id
            )

            if not path1:
                print(f"找不到可行路径！Debug: Path1 failed for {drone_id}")
                return None, None

            arrival_time_in_air = path1[-1][1]

            land_altitude = abs(self.node[customer_node].map_position[2] - self.node[customer_node].position[2])
            land_duration = land_altitude / self.vehicle[drone_id].landingSpeed

            climb_duration = land_altitude / self.vehicle[drone_id].takeoffSpeed

            departure_time_for_path2 = arrival_time_in_air + land_duration + service_time + climb_duration

            self._update_reservation_table_with_path(reservation_table, path1, drone_id, mission_tuple)

            temp_reservation_table = copy.copy(reservation_table)

            if customer_node not in temp_reservation_table:
                temp_reservation_table[customer_node] = []

            temp_reservation_table[customer_node].append({
                'start': arrival_time_in_air + land_duration,
                'end': arrival_time_in_air + land_duration + service_time,
                'arrive_time': arrival_time_in_air + land_duration,
                'drone_id': drone_id,
                'type': 'service_stop'
            })

            path2 = self.spatio_temporal_a_star(
                self.G_air,
                customer_node,
                self.node[recovery_node].map_key,
                departure_time_for_path2 ,
                self.heuristics[self.node[recovery_node].map_key],
                constraints,
                temp_reservation_table,
                drone_id
            )

            if not path2:

                return None, None

            self._update_reservation_table_with_path(temp_reservation_table, path2, drone_id, mission_tuple)

            full_path = path1 + path2[1:]

            return full_path, temp_reservation_table

    def _detect_collisions(self, paths, constraints):

        all_collisions = []

        temp_reservation_table = {}
        for mission_tuple, path in paths.items():
            drone_id = mission_tuple[0]
            self._update_reservation_table_with_path(temp_reservation_table, path, drone_id, mission_tuple)

        for node, intervals in temp_reservation_table.items():
            for interval1, interval2 in itertools.combinations(intervals, 2):
                if interval1['drone_id'] == interval2['drone_id']:
                    continue
                time_diff = abs(interval1['arrive_time'] - interval2['arrive_time'])
                if time_diff < 1e-6:
                    collision_time = interval1['arrive_time']
                    colliding_agents = sorted([interval1['drone_id'], interval2['drone_id']])
                    collision = {
                        'type': 'vertex',
                        'loc': [node],
                        'time': collision_time,
                        'participants': [
                            {'agent_id': interval1['drone_id'], 'mission_tuple': interval1['mission_tuple']},
                            {'agent_id': interval2['drone_id'], 'mission_tuple': interval2['mission_tuple']}
                        ]
                    }

                    is_duplicate = any(
                        c['type'] == 'vertex' and c['loc'][0] == node and
                        set(p['agent_id'] for p in c['participants']) == {interval1['drone_id'], interval2['drone_id']}
                        for c in all_collisions
                    )
                    if not is_duplicate:
                        all_collisions.append(collision)

        path_items = list(paths.items())
        for (mission1, path1), (mission2, path2) in itertools.combinations(path_items, 2):
            drone1_id = mission1[0]
            drone2_id = mission2[0]

            for i in range(len(path1) - 1):
                p1_start_node, p1_start_time = path1[i]
                p1_end_node, p1_end_time = path1[i+1]

                for j in range(len(path2) - 1):
                    p2_start_node, p2_start_time = path2[j]
                    p2_end_node, p2_end_time = path2[j+1]

                    is_swapping = (p1_start_node == p2_end_node and p1_end_node == p2_start_node)

                    if not is_swapping:
                        continue

                    time_overlap = max(p1_start_time, p2_start_time) < min(p1_end_time, p2_end_time)

                    if time_overlap:
                        collision = {
                        'type': 'edge',
                        'participants': [
                            {'agent_id': drone1_id, 'mission_tuple': mission1, 'loc': [p1_start_node, p1_end_node], 'time': p1_end_time},
                            {'agent_id': drone2_id, 'mission_tuple': mission2, 'loc': [p2_start_node, p2_end_node], 'time': p2_end_time}
                        ]}

                        nodes_involved = frozenset([p1_start_node, p1_end_node])
                        is_duplicate = any(
                            c['type'] == 'edge' and
                            set(p['agent_id'] for p in c['participants']) == {drone1_id, drone2_id} and
                            frozenset(c['participants'][0]['loc']) == nodes_involved
                            for c in all_collisions
                        )
                        if not is_duplicate:
                            all_collisions.append(collision)

        for constraint in constraints:

            agent_id = constraint['agent']
            constrained_mission_tuple = constraint['mission_tuple']
            constrained_loc = constraint['loc']
            constrained_time = constraint['time']

            path = paths.get(constrained_mission_tuple)
            if not path: continue

            if len(constrained_loc) == 1:
                node_to_check = constrained_loc[0]
                for node, time in path:
                    if node == node_to_check and abs(time - constrained_time) < 1e-9:
                        violation = {
                            'type': 'constraint_violation',
                            'constraint': constraint,
                            'participants': [{'agent_id': agent_id, 'mission_tuple': constrained_mission_tuple}]
                        }
                        if violation not in all_collisions:
                            all_collisions.append(violation)
                        break

            elif len(constrained_loc) == 2:
                [c_start, c_end] = constrained_loc
                for i in range(len(path) - 1):
                    p_start, _ = path[i]
                    p_end, p_end_time = path[i+1]
                    if p_start == c_start and p_end == c_end and abs(p_end_time - constrained_time) < 1e-9:
                        violation = {
                            'type': 'constraint_violation',
                            'constraint': constraint,
                            'participants': [{'agent_id': agent_id, 'mission_tuple': constrained_mission_tuple}]
                        }
                        if violation not in all_collisions:
                            all_collisions.append(violation)
                        break

        return all_collisions

    def _create_constraint_from_collision(self, collision, agent_id_to_constrain, mission_tuple):

        collision_type = collision['type']

        if collision_type == 'vertex':

            return {
                'agent': agent_id_to_constrain,
                'mission_tuple': mission_tuple,
                'loc': collision['loc'],
                'time': collision['time'],
                'type': 'vertex'
            }

        elif collision_type == 'edge':

            constraint_details = next(
                (p for p in collision['participants'] if p['agent_id'] == agent_id_to_constrain),
                None
            )

            if not constraint_details:
                return None

            return {
                'agent': agent_id_to_constrain,
                'mission_tuple': mission_tuple,
                'loc': constraint_details['loc'],
                'time': constraint_details['time'],
                'type': 'edge'
            }

        else:

            return None

    def spatio_temporal_a_star(self, graph, start_node, goal_node, launch_time, heuristic, constraints, reservation_table, drone_id):

        open_list = []

        heapq.heappush(open_list, (launch_time + heuristic[start_node]/self.vehicle[drone_id].cruiseSpeed, launch_time, (launch_time, start_node)))
        came_from = {}
        g_score = { (launch_time, start_node): launch_time }

        EPSILON = 1e-4
        while open_list:
            _, current_g, current_state = heapq.heappop(open_list)
            current_time, current_node = current_state

            if current_g > g_score.get(current_state, float('inf')):
                continue
            if current_node == goal_node:

                path = []
                temp = current_state
                while temp in came_from:
                    path.append((temp[1], temp[0]))
                    temp = came_from[temp]
                path.append((start_node, launch_time))
                return path[::-1]

            all_possible_neighbors = list(graph.neighbors(current_node)) + [current_node]
            for neighbor in all_possible_neighbors:

                if neighbor == current_node:

                    travel_time = 0.02
                else:

                    edge_data = graph.get_edge_data(current_node, neighbor)
                    distance = edge_data.get('weight', 1)
                    travel_time = distance / self.vehicle[drone_id].cruiseSpeed

                earliest_arrival_time = current_time + travel_time

                safe_arrival_time = earliest_arrival_time
                loop_count = 0
                MAX_LOOPS = 50
                while loop_count < MAX_LOOPS:
                    loop_count += 1
                    found_conflict = False
                    max_conflict_end_time = -1

                    for c in constraints:

                        if len(c['loc']) == 1 and c['loc'][0] == neighbor:
                            if abs(c['time'] - safe_arrival_time) < EPSILON:
                                found_conflict = True

                                max_conflict_end_time = max(max_conflict_end_time, c['time'] + 0.05)

                        if neighbor != current_node and len(c['loc']) == 2:
                            if c['loc'] == [current_node, neighbor] and abs(c['time'] - safe_arrival_time) < EPSILON:
                                found_conflict = True
                                max_conflict_end_time = max(max_conflict_end_time, c['time'] + 0.05)

                    reserved_intervals = reservation_table.get(neighbor, [])
                    for interval in reserved_intervals:
                        if interval['drone_id'] == drone_id:
                            continue

                        start_t, end_t = interval['start'], interval['end']

                        if start_t - EPSILON <= safe_arrival_time < end_t - EPSILON:
                            found_conflict = True

                            max_conflict_end_time = max(max_conflict_end_time, end_t)

                    if found_conflict:

                        if max_conflict_end_time > safe_arrival_time:
                            safe_arrival_time = max_conflict_end_time + EPSILON
                        else:

                            safe_arrival_time += 0.05
                        continue
                    else:

                        break

                if loop_count >= MAX_LOOPS:

                    continue

                    safe_arrival_time = temp_arrival_time

                    reserved_intervals = reservation_table.get(neighbor, [])
                    reservation_conflict_found = False

                    for interval_dict in reserved_intervals:

                        start_t = interval_dict['start']
                        end_t = interval_dict['end']
                        if interval_dict['drone_id'] == drone_id:
                            continue

                        if start_t <= safe_arrival_time < end_t:

                            last_conflict_end_time = max(last_conflict_end_time, end_t)
                            reservation_conflict_found = True

                    if reservation_conflict_found or last_conflict_end_time != -1:
                        if cbs_conflict_found:

                                safe_arrival_time += 1

                        if reservation_conflict_found:
                            safe_arrival_time = max(safe_arrival_time, last_conflict_end_time)

                        continue

                    break

                next_state = (safe_arrival_time, neighbor)
                new_g = safe_arrival_time

                if new_g < g_score.get(next_state, float('inf')):
                    g_score[next_state] = new_g
                    h_value = heuristic.get(neighbor, 0) / self.vehicle[drone_id].cruiseSpeed
                    f_value = new_g + h_value
                    heapq.heappush(open_list, (f_value, new_g, next_state))

                    came_from[next_state] = current_state
        return None
        print('无人机的低空城市走廊规划任务中找到了无解的路径，请检查任务后重新规划')
