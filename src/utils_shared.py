def find_keys_and_indices(dictionary, target_value):

    result = []
    for index, (key, value) in enumerate(dictionary.items()):
        if value == target_value:
            result.append((key, index, value))
    return result

def find_same_xy_different_z(positions_dict, target_position):

    target_x, target_y, target_z = target_position

    for key, position in positions_dict.items():

        if position == target_position:
            continue

        x, y, z = position

        if x == target_x and y == target_y and z != target_z:

            return (key, key, position)

    return None

def merge_and_renumber_dicts(air_node_types, ground_node_types):

    merged = {}
    current_index = 0

    for key in sorted(air_node_types.keys()):
        merged[current_index] = air_node_types[key]
        current_index += 1

    for key in sorted(ground_node_types.keys()):
        merged[current_index] = ground_node_types[key]
        current_index += 1

    return merged

def find_chain_tasks(assignment, customer_plan, vehicle_routes, vehicle_task_data):

    drone_id, launch_node, customer, recovery_node, launch_vehicle, recovery_vehicle = assignment

    if launch_vehicle == recovery_vehicle:
        return []

    origin_vehicle = launch_vehicle

    launch_map = {}
    for cid, a in customer_plan.items():
        d, ln, _, rn, lv, rv = a
        launch_map[(d, lv, ln)] = (cid, a)

    cur_vehicle = recovery_vehicle
    cur_route = vehicle_routes[cur_vehicle - 1]
    if recovery_node not in cur_route:
        print(f"回收节点 {recovery_node} 不在回收车辆 {recovery_vehicle} 路线中")
        return []

    cur_index = cur_route.index(recovery_node)

    need = []
    returned_to_origin = False

    visited = set()

    while True:
        state = (cur_vehicle, cur_index, returned_to_origin)
        if state in visited:

            break
        visited.add(state)

        route = vehicle_routes[cur_vehicle - 1]
        found_next = False

        for i in range(cur_index, len(route)):
            node = route[i]

            if (node in vehicle_task_data.get(cur_vehicle, {}) and
                hasattr(vehicle_task_data[cur_vehicle][node], "launch_drone_list") and
                drone_id in vehicle_task_data[cur_vehicle][node].launch_drone_list):

                if returned_to_origin and cur_vehicle == origin_vehicle:
                    found_next = True
                    break

                key = (drone_id, cur_vehicle, node)
                if key not in launch_map:

                    continue

                cid, a = launch_map[key]
                need.append((cid, a))

                _, _, _, rec_node, _, rec_vehicle = a

                if rec_vehicle == origin_vehicle:
                    returned_to_origin = True

                cur_vehicle = rec_vehicle
                rec_route = vehicle_routes[cur_vehicle - 1]
                if rec_node not in rec_route:
                    print(f"回收节点 {rec_node} 不在回收车辆 {cur_vehicle} 路线中")
                    return _dedupe_keep_order(need)

                cur_index = rec_route.index(rec_node)
                found_next = True
                break

        if not found_next:
            break

        if returned_to_origin and cur_vehicle == origin_vehicle:

            break

    return _dedupe_keep_order(need)

def _dedupe_keep_order(pairs):

    out = []
    seen = set()
    for cid, a in pairs:
        if cid not in seen:
            out.append((cid, a))
            seen.add(cid)
    return out

def is_time_feasible(customer_plan, rm_vehicle_arrive_time):

    for customer_node, plan in customer_plan.items():
        _, launch_node, _, recovery_node, launch_vehicle_id, recovery_vehicle_id = plan

        try:
            launch_time = rm_vehicle_arrive_time[launch_vehicle_id][launch_node]
            recovery_time = rm_vehicle_arrive_time[recovery_vehicle_id][recovery_node]

            if launch_time >= recovery_time:
                return False

        except KeyError:
            return False

    return True
