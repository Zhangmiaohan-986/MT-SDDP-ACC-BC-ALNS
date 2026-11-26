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

class _VehiclePlanTimeDict(dict):

    __slots__ = ("cust_arrive_h",)
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.cust_arrive_h = {}

def _propagate_coupled_timetable(best_vehicle_route, T, veh_travel, DEPOT_nodeID, committed_info, vehicle):

    stop_pos = {}
    for v_id in T:
        route = best_vehicle_route[v_id - 1] if 1 <= v_id <= len(best_vehicle_route) else []
        stop_pos[v_id] = {nid: idx for idx, nid in enumerate(route[:-1])} if len(route) > 1 else {}

    launches_at = defaultdict(list)
    recoveries_at = defaultdict(list)
    for m, info in committed_info.items():
        launches_at[(info['launch_vehicle'], info['launch_node'])].append((m, info))
        recoveries_at[(info['recovery_vehicle'], info['recovery_node'])].append((m, info))

    truck_arr = {v_id: {} for v_id in T}
    truck_dep = {v_id: {} for v_id in T}
    truck_ret = {v_id: 0.0 for v_id in T}
    rec_complete = {m: info['ground_touchdown'] + float(vehicle[info['drone_id']].recoveryTime)
                    for m, info in committed_info.items()}

    max_passes = len(T) + len(committed_info) + 4
    for _ in range(max_passes):
        changed = 0.0
        for v_id in T:
            route = best_vehicle_route[v_id - 1] if 1 <= v_id <= len(best_vehicle_route) else []
            if not route:
                continue
            prev_dep = 0.0
            for idx, nid in enumerate(route):
                if idx == 0:
                    arr = 0.0
                    dep = arr
                    for _, linfo in launches_at.get((v_id, nid), ()):
                        dep = max(dep, float(linfo['launch_time']))
                    for m_rec, rinfo in recoveries_at.get((v_id, nid), ()):
                        rc = max(float(rinfo['ground_touchdown']), arr) + float(vehicle[rinfo['drone_id']].recoveryTime)
                        changed = max(changed, abs(rc - rec_complete.get(m_rec, rc)))
                        rec_complete[m_rec] = rc
                        dep = max(dep, rc)
                    changed = max(changed, abs(arr - truck_arr[v_id].get(nid, -1.0)), abs(dep - truck_dep[v_id].get(nid, -1.0)))
                    truck_arr[v_id][nid] = arr
                    truck_dep[v_id][nid] = dep
                    prev_dep = dep
                elif idx < len(route) - 1:
                    prev_nid = route[idx - 1]
                    arr = prev_dep + float(veh_travel[v_id][prev_nid][nid])
                    dep = arr
                    for _, linfo in launches_at.get((v_id, nid), ()):
                        dep = max(dep, float(linfo['launch_time']))
                    for m_rec, rinfo in recoveries_at.get((v_id, nid), ()):
                        rc = max(float(rinfo['ground_touchdown']), arr) + float(vehicle[rinfo['drone_id']].recoveryTime)
                        changed = max(changed, abs(rc - rec_complete.get(m_rec, rc)))
                        rec_complete[m_rec] = rc
                        dep = max(dep, rc)
                    changed = max(changed, abs(arr - truck_arr[v_id].get(nid, -1.0)), abs(dep - truck_dep[v_id].get(nid, -1.0)))
                    truck_arr[v_id][nid] = arr
                    truck_dep[v_id][nid] = dep
                    prev_dep = dep
                else:
                    prev_nid = route[idx - 1]
                    ret_t = prev_dep + float(veh_travel[v_id][prev_nid][nid])
                    changed = max(changed, abs(ret_t - truck_ret[v_id]))
                    truck_ret[v_id] = ret_t
        if changed < 1e-12:
            break

    drone_ready = defaultdict(float)
    for m, rc in rec_complete.items():
        d_id = m[0]
        if rc > drone_ready[d_id]:
            drone_ready[d_id] = rc
    return stop_pos, truck_arr, truck_dep, truck_ret, rec_complete, drone_ready

def _build_conflict_free_path(mission_tuple, launch_time_ground, node, vehicle, uav_travel):

    drone_id, launch_node, customer_node, recovery_node, _, _ = mission_tuple
    l_air = node[launch_node].map_key
    r_air = node[recovery_node].map_key
    climb_alt = abs(node[l_air].map_position[2] - node[l_air].position[2])
    climb_t = climb_alt / vehicle[drone_id].takeoffSpeed
    leg1 = uav_travel[drone_id][l_air][customer_node]
    leg2 = uav_travel[drone_id][customer_node][r_air]

    t_cur = float(launch_time_ground) + climb_t
    path1 = [(l_air, t_cur)]
    p1_nodes = list(leg1.path)
    if not p1_nodes or p1_nodes[0] != l_air:
        p1_nodes = [l_air] + [x for x in p1_nodes if x != l_air]
    if p1_nodes[-1] != customer_node:
        p1_nodes.append(customer_node)

    for a, b in zip(p1_nodes, p1_nodes[1:]):
        dist_ab = math.hypot(node[a].position[0] - node[b].position[0], node[a].position[1] - node[b].position[1])

        t_cur += dist_ab / vehicle[drone_id].cruiseSpeed
        path1.append((b, t_cur))

    cust_land_alt = abs(node[customer_node].map_position[2] - node[customer_node].position[2])
    cust_land_t = cust_land_alt / vehicle[drone_id].landingSpeed
    cust_climb_t = cust_land_alt / vehicle[drone_id].takeoffSpeed
    service_t = node[customer_node].serviceTimeUAV

    t_dep2 = path1[-1][1] + cust_land_t + service_t + cust_climb_t
    path2 = [(customer_node, t_dep2)]
    p2_nodes = list(leg2.path)
    if not p2_nodes or p2_nodes[0] != customer_node:
        p2_nodes = [customer_node] + [x for x in p2_nodes if x != customer_node]
    if p2_nodes[-1] != r_air:
        p2_nodes.append(r_air)
    t_cur2 = t_dep2
    for a, b in zip(p2_nodes, p2_nodes[1:]):
        dist_ab = math.hypot(node[a].position[0] - node[b].position[0], node[a].position[1] - node[b].position[1])
        t_cur2 += dist_ab / vehicle[drone_id].cruiseSpeed
        path2.append((b, t_cur2))
    return path1 + path2[1:]

def rolling_time_cbs(
    vehicle_arrive_time, best_vehicle_route, time_uav_task_dict, time_customer_plan, time_uav_plan, vehicle_plan_time,
    vehicle_task_data, node, DEPOT_nodeID, V, T, vehicle, uav_travel, veh_distance,
    veh_travel, N, N_zero, N_plus, A_total, A_cvtp, A_vtp, A_aerial_relay_node, G_air, G_ground,
    air_matrix, ground_matrix, air_node_types, ground_node_types, A_c, xeee
):

    import os as _os
    cbs_mode = (_os.environ.get("VDAC_CBS_MODE") or "on").strip().lower()

    best_uav_plan = {k: v.copy() for k, v in time_uav_plan.items()}
    orig_order = {}
    for d_id, m_list in (time_uav_task_dict or {}).items():
        for rank_idx, m_tup in enumerate(m_list):
            orig_order[m_tup] = (rank_idx, float(time_uav_plan.get(m_tup, {}).get('launch_time', 0.0)))
    for rank_idx, (m_tup, m_dict) in enumerate(sorted(best_uav_plan.items(), key=lambda x: x[1]['launch_time'])):
        if m_tup not in orig_order:
            orig_order[m_tup] = (rank_idx, float(m_dict.get('launch_time', 0.0)))

    unplanned_pool = {k: v for k, v in best_uav_plan.items()}
    global_reservation_table = {}
    final_solution = {}
    best_uav_cost = {}
    committed_info = {}
    cust_ground_arrive_map = {}

    cbs_solver = Time_cbs_Batch_Solver(
        global_reservation_table, time_uav_task_dict, time_customer_plan, best_uav_plan, vehicle_plan_time, vehicle_task_data,
        node, DEPOT_nodeID, V, T, vehicle, uav_travel, veh_distance, veh_travel, N, N_zero, N_plus, A_total,
        A_cvtp, A_vtp, A_aerial_relay_node, G_air, G_ground, air_matrix, ground_matrix, air_node_types, ground_node_types,
        A_c, xeee
    )
    shift_counts = defaultdict(int)

    while unplanned_pool:
        stop_pos, truck_arr, truck_dep, truck_ret, rec_complete, drone_ready = _propagate_coupled_timetable(
            best_vehicle_route, T, veh_travel, DEPOT_nodeID, committed_info, vehicle
        )

        drone_first = {}
        for m_tup, m_dict in unplanned_pool.items():
            d_id, l_node, _, _, l_veh, _ = m_tup
            s_L = float(vehicle[d_id].launchTime)
            arr_lb = float(truck_arr.get(l_veh, {}).get(l_node, 0.0)) + s_L
            drn_lb = float(drone_ready.get(d_id, 0.0)) + s_L
            new_lb = max(float(m_dict['launch_time']), arr_lb, drn_lb)
            if new_lb > float(m_dict['launch_time']):
                dur_nom = max(0.05, float(m_dict['recovery_time']) - float(m_dict['launch_time']))
                m_dict['launch_time'] = new_lb
                m_dict['recovery_time'] = max(float(m_dict['recovery_time']), new_lb + dur_nom)
            cand_key = (orig_order.get(m_tup, (0, 0.0)), m_tup)
            if d_id not in drone_first or cand_key < drone_first[d_id][0]:
                drone_first[d_id] = (cand_key, m_tup)

        first_per_drone = {pair[1] for pair in drone_first.values()}

        ready_items = []
        for m_tup in first_per_drone:
            m_dict = unplanned_pool[m_tup]
            d_id, l_node, _, _, l_veh, _ = m_tup
            idx_l = stop_pos.get(l_veh, {}).get(l_node, 0)
            blocked = False
            for m2 in unplanned_pool:
                if m2 == m_tup:
                    continue
                _, l2, _, r2, lv2, rv2 = m2
                if lv2 == l_veh and stop_pos.get(l_veh, {}).get(l2, 0) < idx_l:
                    blocked = True
                    break
                if rv2 == l_veh and stop_pos.get(l_veh, {}).get(r2, 0) < idx_l:
                    blocked = True
                    break
            if not blocked:
                ready_items.append((m_tup, m_dict))

        if not ready_items:

            earliest_m = min(unplanned_pool.items(), key=lambda kv: (kv[1]['launch_time'], orig_order.get(kv[0], (0, 0.0))))
            ready_items = [earliest_m]

        ready_items.sort(key=lambda kv: (kv[1]['launch_time'], orig_order.get(kv[0], (0, 0.0))))
        anchor_mission_tuple, anchor_mission_dict = ready_items[0]
        t_start = float(anchor_mission_dict['launch_time'])
        anchor_dur = max(0.5, float(anchor_mission_dict['recovery_time']) - t_start)
        ROLLING_FACTOR = 1.2
        t_end_window = t_start + anchor_dur * ROLLING_FACTOR

        MAX_BATCH_SIZE = 15
        current_batch_missions = []
        selected_drone_ids = set()
        for m_tup, m_dict in ready_items:
            d_id = m_tup[0]
            if float(m_dict['launch_time']) <= t_end_window and d_id not in selected_drone_ids:
                current_batch_missions.append((m_tup, m_dict))
                selected_drone_ids.add(d_id)
                if len(current_batch_missions) >= MAX_BATCH_SIZE:
                    break

        if cbs_mode == "off":
            batch_result = {}
            for m_tup, m_dict in current_batch_missions:
                d_id, l_node, c_node, r_node, _, _ = m_tup
                l_air = node[l_node].map_key
                r_air = node[r_node].map_key
                climb_alt = abs(node[l_air].map_position[2] - node[l_air].position[2])
                climb_t = climb_alt / vehicle[d_id].takeoffSpeed
                p1 = cbs_solver.spatio_temporal_a_star(
                    G_air, l_air, c_node, float(m_dict['launch_time']) + climb_t,
                    cbs_solver.heuristics[c_node], [], {}, d_id
                )
                cust_land_alt = abs(node[c_node].map_position[2] - node[c_node].position[2])
                cust_land_t = cust_land_alt / vehicle[d_id].landingSpeed
                cust_climb_t = cust_land_alt / vehicle[d_id].takeoffSpeed
                dep2 = p1[-1][1] + cust_land_t + node[c_node].serviceTimeUAV + cust_climb_t
                p2 = cbs_solver.spatio_temporal_a_star(
                    G_air, c_node, r_air, dep2,
                    cbs_solver.heuristics[r_air], [], {}, d_id
                )
                batch_result[m_tup] = p1 + p2[1:]
        else:
            batch_result = None
            batch_try = list(current_batch_missions)
            timeout_s = 60
            while True:
                batch_result = cbs_solver.solve_for_batch(batch_try, global_reservation_table, timeout=timeout_s)
                if batch_result is not None:
                    break
                if len(batch_try) <= 1:
                    break
                batch_try = batch_try[: max(1, len(batch_try) // 2)]
                timeout_s = max(15, int(timeout_s * 0.7))

            if batch_result is None:
                shift_counts[anchor_mission_tuple] += 1
                if shift_counts[anchor_mission_tuple] > 400:
                    raise RuntimeError(f"CBS failed after 400 launch shifts for mission {anchor_mission_tuple}")
                anchor_mission_dict['launch_time'] = float(anchor_mission_dict['launch_time']) + getattr(cbs_solver, "TIME_STEP", 0.05)
                continue

        for mission_tuple, path_data in batch_result.items():
            final_solution[mission_tuple] = path_data
            drone_id, launch_node, customer, recovery_node, launch_vehicle, recovery_vehicle = mission_tuple
            route_list = [item[0] for item in path_data]
            launch_vtp_node = node[route_list[0]].map_key
            recovery_vtp_node = node[route_list[-1]].map_key
            route_list.insert(0, launch_vtp_node)
            route_list.append(recovery_vtp_node)
            if cbs_mode != "off":
                global_reservation_table = cbs_solver._update_reservation_table_with_path(
                    global_reservation_table, path_data, drone_id, mission_tuple
                )
            best_uav_plan[mission_tuple]['uav_route'] = route_list
            launch_time, ground_touchdown = update_launch_recovery_time(node, path_data, vehicle, drone_id)
            cust_air_arr = next((t for nd, t in path_data if nd == customer), path_data[0][1])
            cust_land_alt = abs(node[customer].map_position[2] - node[customer].position[2])
            cust_ground_arr = float(cust_air_arr) + cust_land_alt / vehicle[drone_id].landingSpeed
            cust_ground_arrive_map[customer] = cust_ground_arr

            best_uav_plan[mission_tuple]['launch_time'] = float(launch_time)
            best_uav_plan[mission_tuple]['ground_touchdown_time'] = float(ground_touchdown)
            best_uav_plan[mission_tuple]['customer_arrive_time'] = float(cust_ground_arr)
            best_uav_plan[mission_tuple]['recovery_time'] = float(ground_touchdown) + float(vehicle[drone_id].recoveryTime)

            work_time = best_uav_plan[mission_tuple]['recovery_time'] - float(launch_time)
            best_uav_plan[mission_tuple]['time'] = work_time
            uav_route_cost, uav_time_cost = update_uav_cost(
                node, route_list, work_time, vehicle, drone_id, G_air, G_ground, mission_tuple
            )
            best_uav_plan[mission_tuple]['uav_route_cost'] = uav_route_cost
            best_uav_plan[mission_tuple]['uav_cost'] = uav_route_cost
            best_uav_plan[mission_tuple]['uav_time_cost'] = uav_time_cost
            best_uav_cost[customer] = uav_route_cost

            committed_info[mission_tuple] = {
                'drone_id': drone_id,
                'launch_node': launch_node,
                'recovery_node': recovery_node,
                'launch_vehicle': launch_vehicle,
                'recovery_vehicle': recovery_vehicle,
                'launch_time': float(launch_time),
                'ground_touchdown': float(ground_touchdown),
            }
            unplanned_pool.pop(mission_tuple, None)

    _, truck_arr, truck_dep, truck_ret, rec_complete, _ = _propagate_coupled_timetable(
        best_vehicle_route, T, veh_travel, DEPOT_nodeID, committed_info, vehicle
    )
    for mission_tuple, rc in rec_complete.items():
        best_uav_plan[mission_tuple]['recovery_time'] = float(rc)
        best_uav_plan[mission_tuple]['time'] = float(rc) - float(best_uav_plan[mission_tuple]['launch_time'])

    out_vehicle_plan_time = _VehiclePlanTimeDict()
    out_vehicle_plan_time.cust_arrive_h = dict(cust_ground_arrive_map)
    for v_id in T:
        out_vehicle_plan_time[v_id] = {}
        route = best_vehicle_route[v_id - 1] if 1 <= v_id <= len(best_vehicle_route) else []
        for nid in (vehicle_arrive_time.get(v_id) or {}):
            out_vehicle_plan_time[v_id][nid] = [0.0, 0.0]
        for nid in route[1:-1]:
            arr_t = float(truck_arr[v_id].get(nid, 0.0))
            dep_t = float(truck_dep[v_id].get(nid, arr_t))
            out_vehicle_plan_time[v_id][nid] = [arr_t, dep_t]
            if v_id in vehicle_task_data and nid in vehicle_task_data[v_id]:
                vehicle_task_data[v_id][nid].prcise_arrive_time = arr_t
                vehicle_task_data[v_id][nid].prcise_departure_time = dep_t
        ret_t = float(truck_ret.get(v_id, 0.0))
        out_vehicle_plan_time[v_id][DEPOT_nodeID] = [ret_t, ret_t]
        if v_id in vehicle_task_data and DEPOT_nodeID in vehicle_task_data[v_id]:
            vehicle_task_data[v_id][DEPOT_nodeID].prcise_arrive_time = ret_t
            vehicle_task_data[v_id][DEPOT_nodeID].prcise_departure_time = ret_t

    return best_uav_plan, best_uav_cost, out_vehicle_plan_time, vehicle_task_data, global_reservation_table

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
        self._launch_delay_seen = set()
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
        self.EPSILON = 1e-4
        self.TIME_STEP = 0.05
        self.MAX_WAIT_SHIFT = 5

        self.heuristics = {}
        for node in self.G_air.nodes():
            self.heuristics[node] = nx.single_source_dijkstra_path_length(self.G_air, node, weight='weight')

    def _rt_deepcopy(self, rt):
        return {k: [d.copy() for d in v] for k, v in rt.items()}

    def solve_for_batch(self, batch_missions, reservation_table, timeout=60):

        start_time = time.time()
        MAX_EXPANSIONS = 20000
        expansions = 0

        open_list = []
        node_id_counter = 0

        reservation_table_for_root = self._rt_deepcopy(reservation_table)

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
            'constraint_keys': set(),
            'paths': root_paths,
            'collisions': root_collisions
        }

        heapq.heappush(open_list, (root_node['cost'], node_id_counter, root_node))
        node_id_counter += 1

        while open_list:
            expansions += 1

            if expansions > MAX_EXPANSIONS:
                print(f"[CBS] Abort: expansions>{MAX_EXPANSIONS}. Return None.")
                return None
            if time.time() - start_time > timeout:
                print(f"[CBS] Timeout {timeout}s. expansions={expansions} open={len(open_list)} "
                f"root_collisions={len(root_collisions) if 'root_collisions' in locals() else 'NA'} "
                f"constraints={len(current_node['constraints']) if 'current_node' in locals() else 'NA'}")
                print(f"[CBS] Timeout {timeout}s. Return None.")
                return None

            _, _, current_node = heapq.heappop(open_list)

            if not current_node['collisions']:

                print(f"找到解决方案！成本: {current_node['cost']:.2f}。")
                return current_node['paths']

            collision = current_node['collisions'][0]

            participants = collision['participants']

            for participant in participants:
                agent_id = participant['agent_id']
                mission_tuple_to_replan = participant['mission_tuple']

                mission_tuple, mission_dict = next((m for m in batch_missions if m[0] == mission_tuple_to_replan), (None, None))

                if mission_dict is None: continue

                new_constraint = self._create_constraint_from_collision(collision, agent_id, mission_tuple)

                if new_constraint is None:
                    print("[CBS] Skip: cannot create constraint from collision type=",
                        collision.get('type'), "agent=", agent_id, "mission=", mission_tuple)
                    print("      collision keys:", list(collision.keys()))
                    if 'constraint' in collision:
                        print("      collision['constraint']=", collision['constraint'])
                    continue

                key = (new_constraint['agent_id'],
                    tuple(new_constraint['loc']),
                    round(new_constraint['t_start']/self.TIME_STEP),
                    round(new_constraint['t_end']/self.TIME_STEP),
                    new_constraint['kind'])

                if key in current_node.get('constraint_keys', set()):
                    continue

                child_constraints = current_node['constraints'] + [new_constraint]
                child_keys = set(current_node.get('constraint_keys', set()))
                child_keys.add(key)

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
                if new_path_tuple is None:
                    continue
                new_path, _ = new_path_tuple

                if new_path is None:
                    continue
                if new_path == current_node['paths'].get(mission_tuple):
                    continue

                child_paths = current_node['paths'].copy()
                child_paths[mission_tuple] = new_path

                child_cost = self._calculate_solution_cost(child_paths)
                child_collisions = self._detect_collisions(child_paths, child_constraints)

                child_node = {
                    'cost': child_cost,
                    'constraints': child_constraints,
                    'constraint_keys': child_keys,
                    'paths': child_paths,
                    'collisions': child_collisions
                }
                heapq.heappush(open_list, (child_node['cost'], node_id_counter, child_node))
                node_id_counter += 1

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
            return reservation_table

        EPS = getattr(self, "EPSILON", 1e-4)
        touched_nodes = set()

        def _append_only(node, itv):
            reservation_table.setdefault(node, []).append(itv)
            touched_nodes.add(node)

        def _merge_node(node):
            lst = reservation_table.get(node, [])
            if len(lst) <= 1:
                return
            lst.sort(key=lambda x: x['start'])
            merged = []
            for cur in lst:
                if not merged:
                    merged.append(cur)
                    continue
                last = merged[-1]
                same_owner = (
                    cur.get('drone_id') == last.get('drone_id')
                    and cur.get('mission_tuple') == last.get('mission_tuple')
                    and cur.get('type') == last.get('type')
                )
                if same_owner and cur['start'] <= last['end'] + EPS:
                    last['end'] = max(last['end'], cur['end'])
                    if 'arrive_time' in cur:
                        last['arrive_time'] = max(last.get('arrive_time', last['end']), cur['arrive_time'])
                else:
                    merged.append(cur)
            reservation_table[node] = merged

        launch_node, launch_ready_time = path[0]
        climb_altitude = abs(self.node[launch_node].map_position[2] - self.node[launch_node].position[2])
        climb_time = climb_altitude / self.vehicle[drone_id].takeoffSpeed
        climb_start_time = launch_ready_time - climb_time

        _append_only(launch_node, {
            'start': climb_start_time,
            'end': launch_ready_time,
            'arrive_time': launch_ready_time,
            'drone_id': drone_id,
            'mission_tuple': mission_tuple,
            'type': 'climb'
        })

        for i in range(len(path) - 1):
            start_node, start_time = path[i]
            end_node, end_time = path[i + 1]
            if start_time >= end_time:
                continue

            nodes_to_reserve = [start_node, end_node]

            for idx, node in enumerate(nodes_to_reserve):
                _append_only(node, {
                    'start': start_time,
                    'end': end_time,
                    'arrive_time': (start_time if idx == 0 else end_time),
                    'drone_id': drone_id,
                    'mission_tuple': mission_tuple,
                    'type': 'flight'
                })

        land_node, land_arrival_time = path[-1]
        land_altitude = abs(self.node[land_node].map_position[2] - self.node[land_node].position[2])
        land_time = land_altitude / self.vehicle[drone_id].landingSpeed
        land_complete_time = land_arrival_time + land_time

        _append_only(land_node, {
            'start': land_arrival_time,
            'end': land_complete_time,
            'arrive_time': land_complete_time,
            'drone_id': drone_id,
            'mission_tuple': mission_tuple,
            'type': 'land'
        })

        for n in touched_nodes:
            _merge_node(n)

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

        adjusted_launch_time = mission_dict['launch_time']

        launch_air_node = self.node[launch_node].map_key

        launch_intervals = reservation_table.get(launch_air_node, [])

        launch_intervals.sort(key=lambda x: x['start'])

        adjusted_launch_time = self._next_free_time(
            launch_air_node,
            mission_dict['launch_time'],
            reservation_table,
            eps=self.EPSILON,
            ignore_drone_id=None
        )

        if adjusted_launch_time > mission_dict['launch_time'] + self.EPSILON:
            key = (drone_id, launch_air_node,
                round(mission_dict['launch_time'], 4),
                round(adjusted_launch_time, 4))
            if key not in self._launch_delay_seen:
                self._launch_delay_seen.add(key)

        mission_dict['launch_time'] = adjusted_launch_time

        current_launch_time = adjusted_launch_time

        climb_altitude = abs(self.node[launch_node].map_position[2] - self.node[launch_node].position[2])
        climb_time = climb_altitude / self.vehicle[drone_id].takeoffSpeed

        path1_start_time = current_launch_time + climb_time

        path1_start_node = self.node[launch_node].map_key

        path1 = self.spatio_temporal_a_star(
            self.G_air,
            path1_start_node,
            customer_node,
            path1_start_time,
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

        reservation_table = self._update_reservation_table_with_path(
            reservation_table, path1, drone_id, mission_tuple
        )

        temp_reservation_table = self._rt_deepcopy(reservation_table)

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
            departure_time_for_path2,
            self.heuristics[self.node[recovery_node].map_key],
            constraints,
            temp_reservation_table,
            drone_id
        )

        if not path2:
            return None, None

        temp_reservation_table = self._update_reservation_table_with_path(
            temp_reservation_table, path2, drone_id, mission_tuple
        )

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

        import math

        TS  = getattr(self, "TIME_STEP", 0.05)
        EPS = getattr(self, "EPSILON", 1e-4)

        def q_down(t):
            return math.floor((t + EPS) / TS) * TS

        def q_up(t):
            return math.ceil((t - EPS) / TS) * TS

        collision_type = collision.get('type', None)
        if collision_type == 'constraint_violation':
            c = collision.get('constraint', None)
            if not c:
                return None

            cid = c.get('agent_id', c.get('agent', None))
            if cid != agent_id_to_constrain:
                return None

            loc  = c.get('loc', None)
            kind = c.get('kind', c.get('type', None))
            if not loc or kind not in ('vertex', 'edge'):
                return None

            t0 = c.get('t_start', c.get('time', None))
            t1 = c.get('t_end', None)
            if t0 is None:
                return None
            if t1 is None:
                t1 = t0 + TS

            t0, t1 = q_down(t0), q_up(t1)
            if t1 <= t0 + EPS:
                t1 = t0 + TS

            return {
                'agent': cid,
                'mission_tuple': c.get('mission_tuple', mission_tuple),
                'loc': loc,
                'time': c.get('time', t0),
                'type': kind,

                'agent_id': cid,
                'kind': kind,
                't_start': t0,
                't_end': t1,
            }
        if collision_type not in ('vertex', 'edge'):
            return None

        if collision_type == 'vertex':
            loc = collision.get('loc', None)
            if not loc:
                return None

            t = collision.get('time', None)
            if t is None:
                return None

            t0 = collision.get('t_start', t)
            t1 = collision.get('t_end', t + TS)
            t0, t1 = q_down(t0), q_up(t1)
            if t1 <= t0 + EPS:
                t1 = t0 + TS

            return {

                'agent': agent_id_to_constrain,
                'mission_tuple': mission_tuple,
                'loc': loc,
                'time': t,
                'type': 'vertex',

                'agent_id': agent_id_to_constrain,
                'kind': 'vertex',
                't_start': t0,
                't_end': t1,
            }

        else:
            participants = collision.get('participants', [])
            constraint_details = next(
                (p for p in participants if p.get('agent_id') == agent_id_to_constrain),
                None
            )
            if not constraint_details:
                return None

            loc = constraint_details.get('loc', None)
            t   = constraint_details.get('time', None)
            if not loc or t is None:
                return None

            t0 = constraint_details.get('t_start', collision.get('t_start', t))
            t1 = constraint_details.get('t_end',   collision.get('t_end',   t + TS))
            t0, t1 = q_down(t0), q_up(t1)
            if t1 <= t0 + EPS:
                t1 = t0 + TS

            return {

                'agent': agent_id_to_constrain,
                'mission_tuple': mission_tuple,
                'loc': loc,
                'time': t,
                'type': 'edge',

                'agent_id': agent_id_to_constrain,
                'kind': 'edge',
                't_start': t0,
                't_end': t1,
            }

    def spatio_temporal_a_star(self, graph, start_node, goal_node, launch_time,
                            heuristic, constraints, reservation_table, drone_id):
        import math
        hit = 0
        open_list = []
        came_from = {}

        TIME_STEP = self.TIME_STEP
        EPSILON   = self.EPSILON
        MAX_SHIFT = self.MAX_WAIT_SHIFT

        def _quantize_up(t):
            return math.ceil(t / TIME_STEP) * TIME_STEP

        launch_time = _quantize_up(launch_time)

        heapq.heappush(
            open_list,
            (launch_time + heuristic[start_node] / self.vehicle[drone_id].cruiseSpeed,
            launch_time,
            (launch_time, start_node))
        )

        g_score = {(launch_time, start_node): launch_time}

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
                    travel_time = TIME_STEP
                else:
                    edge_data = graph.get_edge_data(current_node, neighbor)
                    distance = edge_data.get('weight', 1)
                    travel_time = distance / self.vehicle[drone_id].cruiseSpeed

                earliest_arrival_time = _quantize_up(current_time + travel_time)
                safe_arrival_time = earliest_arrival_time

                ok = False
                for _ in range(10):

                    conflict = False
                    push_to = safe_arrival_time

                    for c in constraints:

                        if c.get('agent_id', c.get('agent')) not in (None, drone_id):
                            continue

                        loc = c.get('loc', [])

                        if 't_start' in c and 't_end' in c:
                            t0, t1 = c['t_start'], c['t_end']

                            if len(loc) == 1 and loc[0] == neighbor:
                                if t0 - EPSILON <= safe_arrival_time < t1 - EPSILON:
                                    conflict = True
                                    push_to = max(push_to, t1)

                            if neighbor != current_node and len(loc) == 2 and loc[0] == current_node and loc[1] == neighbor:
                                if t0 - EPSILON <= safe_arrival_time < t1 - EPSILON:
                                    conflict = True
                                    push_to = max(push_to, t1)

                        else:
                            if len(loc) == 1 and loc[0] == neighbor:
                                if abs(c['time'] - safe_arrival_time) < EPSILON:
                                    conflict = True
                                    push_to = max(push_to, c['time'] + TIME_STEP)
                            if neighbor != current_node and len(loc) == 2 and loc == [current_node, neighbor]:
                                if abs(c['time'] - safe_arrival_time) < EPSILON:
                                    conflict = True
                                    push_to = max(push_to, c['time'] + TIME_STEP)

                    if conflict:
                        safe_arrival_time = _quantize_up(push_to + EPSILON)
                        if safe_arrival_time - earliest_arrival_time > MAX_SHIFT:
                            break
                        continue

                    safe_arrival_time = _quantize_up(
                        self._next_free_time(
                            neighbor, safe_arrival_time, reservation_table,
                            eps=EPSILON, ignore_drone_id=drone_id
                        )
                    )

                    if safe_arrival_time - earliest_arrival_time > MAX_SHIFT:
                        break

                    intervals = reservation_table.get(neighbor, [])
                    in_conflict = False
                    for itv in intervals:
                        if itv.get('drone_id') == drone_id:
                            continue
                        if itv['start'] - EPSILON <= safe_arrival_time < itv['end'] - EPSILON:
                            in_conflict = True
                            safe_arrival_time = _quantize_up(itv['end'] + EPSILON)
                            break

                    if not in_conflict:
                        ok = True
                        break

                if not ok:
                    continue

                next_state = (safe_arrival_time, neighbor)
                new_g = safe_arrival_time

                if new_g < g_score.get(next_state, float('inf')):
                    g_score[next_state] = new_g
                    h = heuristic.get(neighbor, 0) / self.vehicle[drone_id].cruiseSpeed
                    f = new_g + h
                    heapq.heappush(open_list, (f, new_g, next_state))
                    came_from[next_state] = current_state

        return None

    def _next_free_time(self, node_id, t, reservation_table, eps=1e-4, ignore_drone_id=None):
        intervals = reservation_table.get(node_id, [])
        if not intervals:
            return t

        intervals = sorted(intervals, key=lambda x: x['start'])
        tt = t
        for it in intervals:
            if ignore_drone_id is not None and it.get('drone_id') == ignore_drone_id:
                continue
            s, e = it['start'], it['end']

            if e <= tt + eps:
                continue

            if tt < s - eps:
                break

            if s - eps <= tt < e - eps:
                tt = e + eps

        return tt

