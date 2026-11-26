from collections import defaultdict
from typing import List, Dict, Tuple
import copy
from collections import defaultdict
from task_data import *
from cost_y import *
from call_function import *
from initialize import *
from cbs_plan import *

from down_data import *

def rm_empty_node(customer_plan, vehicle_route):

    filtered_vehicle_route = []
    empty_nodes_by_vehicle = defaultdict(list)

    for num_index, route in enumerate(vehicle_route):
        vehicle_id = num_index + 1

        task_route = route[1:-1] if len(route) > 2 else []

        mission_nodes = set()

        for mission_tuple in customer_plan.values():
            drone_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle = mission_tuple

            if launch_vehicle == vehicle_id or recovery_vehicle == vehicle_id:

                if launch_node in task_route:
                    mission_nodes.add(launch_node)

                if recovery_node in task_route:
                    mission_nodes.add(recovery_node)

        empty_nodes = [node for node in task_route if node not in mission_nodes]
        empty_nodes_by_vehicle[vehicle_id] = empty_nodes

        filtered_route = [route[0]]

        for node in task_route:
            if node in mission_nodes:
                filtered_route.append(node)

        if len(route) > 1:
            filtered_route.append(route[-1])

        filtered_vehicle_route.append(filtered_route)

    return filtered_vehicle_route, dict(empty_nodes_by_vehicle)

