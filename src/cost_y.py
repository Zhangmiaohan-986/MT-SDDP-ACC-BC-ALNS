import math
import numpy as np
NODE_TYPE_DEPOT	= 0
NODE_TYPE_CUST	= 1

TYPE_TRUCK 		= 1
TYPE_UAV 		= 2

def cal_low_cost(i_vtp,customre,j_vtp,v_id,recv_v_id, uav_id, uav_travel,veh_distance,veh_travel,node,vehicle,vehicle_type,xeee):

    path = []
    if vehicle_type == TYPE_UAV:
        air_i = node[i_vtp].map_key
        air_j = node[j_vtp].map_key
        per_cost = vehicle[uav_id].per_cost

        uav_time_ic = uav_travel[uav_id][air_i][customre].totalTime
        uav_distance_ic = uav_travel[uav_id][air_i][customre].totalDistance
        uav_route_ic = uav_travel[uav_id][air_i][customre].path
        uav_cost_ic = uav_distance_ic * per_cost
        path.extend(uav_route_ic)
        uav_time_cj = uav_travel[uav_id][customre][air_j].totalTime
        uav_distance_cj = uav_travel[uav_id][customre][air_j].totalDistance
        uav_route_cj = uav_travel[uav_id][customre][air_j].path
        uav_cost_cj = uav_distance_cj * per_cost
        path.extend(uav_route_cj)
        uav_time_ij = uav_time_ic + uav_time_cj
        uav_distance_ij = uav_distance_ic + uav_distance_cj
        uav_cost_ij = uav_distance_ij * per_cost

        return uav_cost_ij, uav_time_ij, path
    elif vehicle_type == TYPE_TRUCK:
        per_cost = vehicle[v_id].per_cost
        veh_distance = veh_distance[v_id][i_vtp][j_vtp]
        veh_cost = veh_distance * per_cost
        veh_time = veh_travel[v_id][i_vtp][j_vtp]
        return veh_cost, veh_time, path

def calculate_plan_cost(best_plan_cost, vehicle_route, vehicle, vehicle_id, uav_id, veh_distance):
    total_cost = 0
    np_best_plan_cost = np.array(list(best_plan_cost.values()))
    uav_total_cost = np.sum(np_best_plan_cost)
    vehicle_fix_cost = 0
    vehicle_total_route_cost = 0

    for veh_idx, route in enumerate(vehicle_route):
        veh_id = vehicle_id[veh_idx]
        vehicle_route_cost = calculate_veh_route_cost(route, vehicle[veh_id], veh_distance)
        vehicle_total_route_cost += vehicle_route_cost
    total_cost = uav_total_cost + vehicle_fix_cost + vehicle_total_route_cost
    return total_cost

def calculate_veh_route_cost(route, vehicle, veh_distance):
    veh_cost = 0
    veh_per_cost = vehicle.per_cost
    for i in range(len(route)-1):
        veh_cost += veh_distance[vehicle.id][route[i]][route[i+1]] * veh_per_cost
    return veh_cost

def update_calculate_plan_cost(best_uav_plan, vehicle_route, vehicle, vehicle_id, uav_id, veh_distance):
    total_cost = 0
    uav_total_cost = 0
    vehicle_fix_cost = 0
    vehicle_total_route_cost = 0

    for v_id in vehicle_id:
        fix_cost = vehicle[v_id].fix_cost
        vehicle_fix_cost += fix_cost

    for veh_idx, route in enumerate(vehicle_route):
        veh_id = vehicle_id[veh_idx]
        vehicle_route_cost = calculate_veh_route_cost(route, vehicle[veh_id], veh_distance)
        vehicle_total_route_cost += vehicle_route_cost
    for mission_tuple, mission_dict in best_uav_plan.items():
        uav_id = mission_dict['drone_id']
        uav_cost = mission_dict['uav_cost']
        uav_time = mission_dict['uav_time']
        uav_route = mission_dict['uav_route']
        uav_total_cost += uav_cost
    total_cost = uav_total_cost + vehicle_fix_cost + vehicle_total_route_cost
    return total_cost

def calculate_window_cost(best_customer_plan,
                          best_plan_cost,
                          veh_arrival_times,
                          vehicle,
                          customer_time_windows_h,
                          early_arrival_cost,
                          late_arrival_cost,
                          uav_fly_time_to_customer,
                          node):

    uav_tw_violation_cost = compute_uav_tw_violation_cost(
        best_customer_plan=best_customer_plan,
        veh_arrival_times=veh_arrival_times,
        vehicle=vehicle,
        customer_time_windows_h=customer_time_windows_h,
        early_arrival_cost=early_arrival_cost,
        late_arrival_cost=late_arrival_cost,
        uav_fly_time_to_customer=uav_fly_time_to_customer,
        node=node,
    )

    total_cost_dict = combine_plan_and_tw_cost(
        best_plan_cost=best_plan_cost,
        uav_tw_violation_cost=uav_tw_violation_cost
    )

    window_total_cost = float(sum(total_cost_dict.values()))

    return window_total_cost, uav_tw_violation_cost, total_cost_dict

def calculate_customer_window_cost(best_customer_plan,
                          vehicle,
                          veh_arrival_times,
                          customer_time_windows_h,
                          early_arrival_cost,
                          late_arrival_cost,
                          uav_fly_time_to_customer,
                          node):

    uav_tw_violation_cost = compute_uav_tw_violation_cost(
        best_customer_plan=best_customer_plan,
        vehicle=vehicle,
        veh_arrival_times=veh_arrival_times,
        customer_time_windows_h=customer_time_windows_h,
        early_arrival_cost=early_arrival_cost,
        late_arrival_cost=late_arrival_cost,
        uav_fly_time_to_customer=uav_fly_time_to_customer,
        node=node,
    )

    uav_tw_violation_cost = float(sum(uav_tw_violation_cost.values()))
    return uav_tw_violation_cost

def combine_plan_and_tw_cost(best_plan_cost, uav_tw_violation_cost):

    total_cost_dict = defaultdict(float)

    all_keys = set(best_plan_cost.keys()) | set(uav_tw_violation_cost.keys())
    for k in all_keys:
        base_cost = float(best_plan_cost.get(k, 0.0))
        tw_cost = float(uav_tw_violation_cost.get(k, 0.0))
        total_cost_dict[k] = base_cost + tw_cost

    return total_cost_dict

from collections import defaultdict

def compute_uav_tw_violation_cost(best_customer_plan,
                                  veh_arrival_times,
                                  vehicle,
                                  customer_time_windows_h,
                                  early_arrival_cost,
                                  late_arrival_cost,
                                  uav_fly_time_to_customer,node):

    early_penalty_per_hour = float(early_arrival_cost[0])
    late_penalty_per_hour = float(late_arrival_cost[0])

    uav_tw_violation_cost = defaultdict(float)

    for key_idx, plan in best_customer_plan.items():
        if plan is None:
            continue

        uav_id, i_vtp, customer_node, j_vtp, v_id, recv_v_id = plan
        map_i_vtp = node[i_vtp].map_key
        map_j_vtp = node[j_vtp].map_key

        tw = customer_time_windows_h.get(key_idx, None)
        if tw is None:

            uav_tw_violation_cost[key_idx] = 0.0
            print(f"没有时间窗信息，不罚")
            continue

        ready_h = float(tw['ready_h'])
        due_h = float(tw['due_h'])

        try:
            truck_arrival_time = float(veh_arrival_times[v_id][i_vtp])
        except KeyError:

            uav_tw_violation_cost[key_idx] = 0.0
            print(f"没有卡车到达时间，请检查")
            continue

        _realized_cust_arr = getattr(veh_arrival_times, "cust_arrive_h", None)
        if _realized_cust_arr is not None and customer_node in _realized_cust_arr:
            arrival_time_at_customer = float(_realized_cust_arr[customer_node])
        else:
            arrival_time_at_customer = uav_fly_time_to_customer[uav_id][map_i_vtp][customer_node].totalTime + truck_arrival_time

        early = max(0.0, ready_h - arrival_time_at_customer)
        late = max(0.0, arrival_time_at_customer - due_h)

        early_cost = early * early_penalty_per_hour
        late_cost = late * late_penalty_per_hour

        tw_cost = early_cost + late_cost
        uav_tw_violation_cost[key_idx] = tw_cost

    return uav_tw_violation_cost

def get_max_completion_time(best_arrive_time):

    vehicle_max_times = {}

    for vehicle_id, node_time_dict in best_arrive_time.items():
        if not node_time_dict:
            vehicle_max_times[vehicle_id] = 0.0
            continue

        node_times = list(node_time_dict.values())
        max_time = max(node_times)
        vehicle_max_times[vehicle_id] = max_time

    global_max_time = max(vehicle_max_times.values()) if vehicle_max_times else 0.0

    return vehicle_max_times, global_max_time

def sort_customer_plans(customer_costs, plan_y, plan_time, plan_uav_route):

    for customer in customer_costs.keys():

        indexed_costs = [(cost, idx) for idx, cost in enumerate(customer_costs[customer])]

        indexed_costs.sort(key=lambda x: x[0])

        sorted_indices = [idx for _, idx in indexed_costs]

        customer_costs[customer] = [customer_costs[customer][idx] for idx in sorted_indices]
        plan_y[customer] = [plan_y[customer][idx] for idx in sorted_indices]
        plan_time[customer] = [plan_time[customer][idx] for idx in sorted_indices]
        plan_uav_route[customer] = [plan_uav_route[customer][idx] for idx in sorted_indices]

    return customer_costs, plan_y, plan_time, plan_uav_route

def format_rm_empty_vehicle_arrive_time(final_vehicle_plan_time, vehicle_routes):

    formatted_data = {}

    for i, route in enumerate(vehicle_routes):
        v_id = i + 1

        if v_id not in final_vehicle_plan_time:
            continue

        formatted_data[v_id] = {}

        vehicle_times_source = final_vehicle_plan_time[v_id]

        for node_id in route:
            if node_id in vehicle_times_source:
                raw_value = vehicle_times_source[node_id]

                if isinstance(raw_value, (list, tuple)):
                    arrive_time = float(raw_value[0])
                else:

                    arrive_time = float(raw_value)

                formatted_data[v_id][node_id] = arrive_time

    return formatted_data
