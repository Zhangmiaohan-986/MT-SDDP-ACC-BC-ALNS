import math
import random
import numpy as np
import gurobipy as gp
from gurobipy import GRB

def check_same_vehicle_conflict(v_id, drone_id, i_vtp, j_vtp,
                               solution_route, solution, vehicle_task_data, vehicle):

    v_idx = v_id - 1
    i_idx = solution_route[v_idx].index(i_vtp)
    j_idx = solution_route[v_idx].index(j_vtp)

    if i_idx >= j_idx:
        return False

    if drone_id not in vehicle_task_data[v_id][i_vtp].drone_list:
        return False

    route_segment = solution_route[v_idx][i_idx:j_idx+1]

    for node in route_segment:
        task_data = vehicle_task_data[v_id][node]
        if drone_id in task_data.launch_drone_list or drone_id in task_data.recovery_drone_list:
            return False

    if len(vehicle_task_data[v_id][j_vtp].drone_list) >= vehicle[v_id].maxDrones:
        return False

    if solution[v_id][i_vtp] > solution[v_id][j_vtp]:
        return False

    return True

def check_cross_vehicle_conflict_fixed(launch_v_id, recover_v_id, drone_id,
                                       i_vtp, j_vtp, solution_route, solution,
                                       vehicle_task_data, launch_time, recovery_time, vehicle):

    launch_veh_route = solution_route[launch_v_id - 1]
    i_idx = launch_veh_route.index(i_vtp)

    if drone_id not in vehicle_task_data[launch_v_id][i_vtp].drone_list:
        return False

    for post_idx in range(i_idx + 1, len(launch_veh_route)-1):
        post_node = launch_veh_route[post_idx]
        post_task = vehicle_task_data[launch_v_id][post_node]

        if drone_id in post_task.launch_drone_list:
            has_intermediate_recovery = False

            route_slice = launch_veh_route[i_idx + 1:post_idx+1]
            for idx, mid_node in enumerate(route_slice):
                if idx == len(route_slice) - 1:
                    if drone_id in vehicle_task_data[launch_v_id][mid_node].recovery_drone_list:
                        has_intermediate_recovery = True
                        break
                if drone_id in vehicle_task_data[launch_v_id][mid_node].recovery_drone_list:

                    if recovery_time > solution[launch_v_id][mid_node]:
                        return False
                    has_intermediate_recovery = True
                    break
            if not has_intermediate_recovery:
                return False

        if drone_id in post_task.recovery_drone_list:

            if solution[launch_v_id][post_node] < recovery_time:
                return False

    recover_veh_route = solution_route[recover_v_id - 1]
    j_idx = recover_veh_route.index(j_vtp)

    if drone_id in vehicle_task_data[recover_v_id][j_vtp].drone_list or drone_id in vehicle_task_data[recover_v_id][j_vtp].recovery_drone_list or \
        len(vehicle_task_data[recover_v_id][j_vtp].drone_list) >= vehicle[recover_v_id].maxDrones:
        return False

    for pre_idx in range(j_idx - 1, -1, -1):
        pre_node = recover_veh_route[pre_idx]
        pre_task = vehicle_task_data[recover_v_id][pre_node]

        if drone_id in pre_task.launch_drone_list:

            has_intermediate_recovery = False
            for mid_idx in range(pre_idx + 1, j_idx - 1):
                mid_node = recover_veh_route[mid_idx]
                if drone_id in vehicle_task_data[recover_v_id][mid_node].recovery_drone_list:
                    has_intermediate_recovery = True
                    break
            if has_intermediate_recovery:
                return False
            break

        if drone_id in pre_task.launch_drone_list:

            if solution[recover_v_id][pre_node] > launch_time:
                 return False
            break

    for post_idx in range(j_idx + 1, len(recover_veh_route)-1):
        post_node = recover_veh_route[post_idx]
        post_task = vehicle_task_data[recover_v_id][post_node]

        if drone_id in post_task.launch_drone_list:

            for mid_node in recover_veh_route[j_idx+1:post_idx+1]:
                if drone_id in vehicle_task_data[recover_v_id][mid_node].recovery_drone_list:
                    return False
            break

        if drone_id in post_task.launch_drone_list:

            if recovery_time > solution[recover_v_id][post_node]:
                return False
            break

    recover_task_info = vehicle_task_data[recover_v_id][j_vtp]

    if len(recover_task_info.drone_list) >= vehicle[recover_v_id].maxDrones:
        return False

    return True

