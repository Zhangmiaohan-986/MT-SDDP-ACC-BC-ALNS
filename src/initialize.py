import numpy as np
import networkx as nx
import copy

def init_agent(node, vehicle, V, uav_travel, N, N_zero, N_plus, A_total, A_cvtp, A_vtp, A_aerial_relay_node, xP, sigmaprime, sL, sR):
	trucks = []
	drones = []
	for vehicle_id, vehicle_type in vehicle.items():
		if vehicle_type == 'truck':
			trucks.append(Vehicle(vehicle_id, vehicle[vehicle_id].capacity, vehicle[vehicle_id].start_node_id))
		else:
			drones.append(Drone(vehicle_id, vehicle[vehicle_id].endurance, vehicle[vehicle_id].task_speed, vehicle[vehicle_id].speed))

	return trucks, drones

def initialize_drone_vehicle_assignments(vehicle, V, T):

    num_drones = len(V)
    num_vehicles = len(T)

    print(f"分配 {num_drones} 架无人机给 {num_vehicles} 辆车辆")

    drones_per_vehicle = num_drones // num_vehicles
    remaining_drones = num_drones % num_vehicles

    vehicle_assignments = {vehicle_id: drones_per_vehicle for vehicle_id in T}

    for i in range(remaining_drones):
        vehicle_assignments[T[i]] += 1

    for vehicle_id in T:
        print(f"车辆 {vehicle_id} 将被分配 {vehicle_assignments[vehicle_id]} 架无人机")

    drone_index = 0
    for vehicle_id in T:
        drones_to_assign = vehicle_assignments[vehicle_id]
        for _ in range(drones_to_assign):
            if drone_index < len(V):
                drone_id = V[drone_index]

                vehicle[vehicle_id].add_drone(vehicle[drone_id])

                vehicle[drone_id].is_available = True
                print(f"已将无人机 {drone_id} 分配给车辆 {vehicle_id}")
                drone_index += 1
            else:
                break

    for vehicle_id in T:
        num_assigned = len(vehicle[vehicle_id].drones)
        print(f"车辆 {vehicle_id} 现在携带 {num_assigned} 架无人机: {list(vehicle[vehicle_id].drones.keys())}")

    return vehicle

class Vehicle:
    def __init__(self, vehicle_id, capacity, start_node_id, speed=60):

        self.id = vehicle_id
        self.capacity = capacity
        self.route = []
        self.real_route = []
        self.drones = []
        self.start_node_id = start_node_id
        self.speed = speed
        self.work_time = 3/60
        self.completion_time = 0
        self.type = 'vehicle'
        self.launch_records = []
        self.recover_records = []
        self.launch_recover = {}
        self.current_time = 0
        self.current_node_index = 0
        self.node_entry_time = {}
        self.node_entry_exit_time = {}
        self.drone_capacity = capacity

        self.vehicle_available_drones = copy.deepcopy(self.drones)
        self.route_refresh = False
        self.current_node = None
        self.current_time = 0
        self.end_time = 0
        self.available_drones = self.drones.copy()
        self.available_capacity = self.capacity
        self.task_finish = False
        self.mission_route = []

        self.waiting_times = {}
    def update_entry_exit_time(self, node_id, entry_time, exit_time):
        self.node_entry_time[node_id] = entry_time
        self.node_exit_time[node_id] = exit_time

    def can_enter_node(self, node_id, global_time, other_vehicles):

        for vehicle in other_vehicles:
            if node_id in vehicle.node_entry_time:
                if vehicle.node_entry_time[node_id] <= global_time < vehicle.node_exit_time[node_id]:
                    return False
        return True

    def launch_drone(self, drone, current_node):

        if self.drone_capacity > 0:

            drone.is_available = False
            drone.current_node = current_node
            drone.start_time = self.current_time + self.work_time

            self.drone_capacity -= 1
            if current_node not in self.launch:
                self.launch[current_node] = []
            self.launch[current_node].append(drone.id)

            self.current_time += self.work_time
            return True
        else:
            return False

    def recover_drone(self, drone, current_node):

        drone.is_available = True
        drone.current_node = current_node
        drone.end_time = self.current_time + self.work_time

        self.drone_capacity += 1
        if current_node not in self.recover:
            self.recover[current_node] = []
        self.recover[current_node].append(drone.id)

        self.current_time += self.work_time

    def add_drone(self, drone):

        self.drones.append(drone)

    def set_route(self, route):

        self.route = route.copy()

    def add_node_to_route(self, node_id):

        self.route.append(node_id)

    def encode_route(self):

        return self.route.copy()

    def decode_route(self, encoded_route):

        self.route = encoded_route.copy()

    def get_completion_time(self, ground_graph):

        total_distance = 0
        for i in range(len(self.route) - 1):
            node1 = self.route[i]
            node2 = self.route[i + 1]
            edge = ground_graph.edges.get((node1, node2))
            if edge:
                total_distance += edge
            else:

                total_distance += np.inf
        self.completion_time = total_distance
        return self.completion_time

class Drone:
    def __init__(self, drone_id, endurance=0.77, task_speed=36, speed=61.2):

        self.id = drone_id
        self.endurance = endurance
        self.task_speed = task_speed
        self.speed = speed
        self.route = []
        self.time = []
        self.completion_time = 0
        self.type = 'drone'
        self.launch = {}
        self.recover = {}
        self.launch_recover = {}
        self.is_available = True
        self.current_node = None
        self.start_time = 0
        self.end_time = 0
        self.task_finish = False
        self.remaining_endurance = endurance
        self.current_node = None
        self.assigned_vehicle = None
        self.launch_time = None
        self.recover_time = None
        self.task_assigned = None
        self.current_time = 0
        self.need_back = False
        self.returnable_nodes = []
        self.inspection_nodes = []
        self.inspection_time = 0
        self.inspection_nodes_index = 0
        self.violate = False
        self.recover_vehicle = None
        self.no_task = False

    def assign_task(self, path):

        self.route = path.copy()
        self.is_available = False

    def update_status(self, time_spent):

        self.remaining_time -= time_spent
        if self.remaining_time <= 0:
            self.is_available = False

    def set_route(self, route):

        self.route = route.copy()

    def add_node_to_route(self, node_id):

        self.route.append(node_id)

    def encode_route(self):

        return self.route.copy()

    def decode_route(self, encoded_route):

        self.route = encoded_route.copy()

    def get_completion_time(self, air_graph):

        total_distance = 0
        for i in range(len(self.route) - 1):
            node1 = self.route[i]
            node2 = self.route[i + 1]
            edge = air_graph.edges.get((node1, node2))
            if edge:
                total_distance += edge
            else:

                total_distance += np.inf
        self.completion_time = total_distance / self.speed
        return self.completion_time

from collections import defaultdict

from collections import defaultdict
import copy

def make_dict():
    return defaultdict(make_dict)

def deep_copy_vehicle_task_data(original_data):

    copied_data = original_data.copy()

    for vehicle_id, inner_dict in copied_data.items():

        new_inner = inner_dict.copy()

        for node_id, task_object in new_inner.items():

            new_inner[node_id] = task_object.fast_copy()

        copied_data[vehicle_id] = new_inner

    return copied_data

