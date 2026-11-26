import numpy as np
from collections import defaultdict
from typing import Any, List, Dict, Tuple
import copy
from collections import defaultdict

NODE_TYPE_DEPOT	= 0
NODE_TYPE_CUST	= 1

TYPE_TRUCK 		= 1
TYPE_UAV 		= 2

NUM_POINTS = 50
SEED = 6
Z_COORD = 5
UAV_DISTANCE = 15

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

class Task:

    def __init__(self, task_type, start_time, end_time, details=None):
        self.task_type = task_type
        self.start_time = start_time
        self.end_time = end_time
        self.details = details or {}

    def __str__(self):
        task_name = TASK_NAMES.get(self.task_type, "未知任务")
        return f"{task_name} ({self.start_time:.2f}-{self.end_time:.2f})"

    def to_dict(self):

        return {
            'task_type': self.task_type,
            'task_name': TASK_NAMES.get(self.task_type, "未知任务"),
            'start_time': self.start_time,
            'end_time': self.end_time,
            'duration': self.end_time - self.start_time,
            'details': self.details
        }

    def copy(self):

        return Task(
            task_type=self.task_type,
            start_time=self.start_time,
            end_time=self.end_time,
            details=self.details.copy() if self.details else None
        )

class VehicleInfo:
    def __init__(self):
        self.drone_belong = None
        self.precise_arrive_time = 0
        self.precise_departure_time = 0
        self.launch_time = []
        self.recovery_time = []

        self.task = {}

    def copy(self):
        new_info = VehicleInfo()
        new_info.drone_belong = self.drone_belong
        new_info.precise_arrive_time = self.precise_arrive_time
        new_info.precise_departure_time = self.precise_departure_time
        new_info.launch_time = self.launch_time.copy()
        new_info.recovery_time = self.recovery_time.copy()

        new_info.task = {k: v.copy() for k, v in self.task.items()}
        return new_info

def create_vehicle_info_dict():

    return {
        "drone_belong": None,
        "prcise_arrive_time": 0,
        "prcise_departure_time": 0,
        "arrive_times": [],
        "departure_times": [],
        "task": {}
    }

def copy_vehicle_info_dict(d):

    return {
        "drone_belong": d["drone_belong"],
        "precise_arrive_time": d["precise_arrive_time"],
        "precise_departure_time": d["precise_departure_time"],
        "arrive_times": d["arrive_times"].copy(),
        "departure_times": d["departure_times"].copy(),
        "task": []
    }

MAX_DICT_VEHICLE_ID = 10

def set_max_dict_vehicle_id(num_trucks):
    global MAX_DICT_VEHICLE_ID
    MAX_DICT_VEHICLE_ID = max(10, int(num_trucks))

def _default_dict_vehicle():
    return {vid: {"drone_belong": None, "prcise_arrive_time": 0, "prcise_departure_time": 0,
                  "arrive_times": [], "departure_times": [], "task": {}}
            for vid in range(1, MAX_DICT_VEHICLE_ID + 1)}

class _LazyDictVehicle:

    def __get__(self, obj, objtype=None):
        if obj is None:
            return self
        d = _default_dict_vehicle()
        obj.__dict__['dict_vehicle'] = d
        return d

_object_new = object.__new__

class vehicle_task:
    dict_vehicle = _LazyDictVehicle()

    def __init__(self, id, vehicleType, node_id, node):
        self.vehicleType = vehicleType
        self.id = id
        self.node = node
        self.node_id = node_id
        self.tasks = {}
        self.arrive_times = []
        self.departure_times = []
        self.prcise_arrive_time = None
        self.prcise_departure_time = None
        self.dict_vehicle = {}
        if node[node_id].nodeType == 'DEPOT':
            self.is_task = True
        else:
            self.is_task = False
        self.drone_list = None
        self.launch_drone_list = None
        self.recovery_drone_list = None
        self.drone_belong = None
        if self.vehicleType == TYPE_TRUCK:

            self.drone_list = []
            self.launch_drone_list = []
            self.recovery_drone_list = []
        else:

            self.drone_belong = None

            self.dict_vehicle = {}

            self.init_dict_vehicle()

    def init_dict_vehicle(self):

        for vehicle_id in range(1, MAX_DICT_VEHICLE_ID + 1):
            self.dict_vehicle[vehicle_id] = create_vehicle_info_dict()

    def update_drone_list(self, drone_id):

        if  not isinstance(drone_id, list):
            self.drone_list.append(drone_id)
        else:
            self.drone_list.extend(drone_id)

    def delete_drone_list(self, drone_id):
        if not isinstance(drone_id, list):
            self.drone_list.remove(drone_id)
        else:
            for i in drone_id:
                self.drone_list.remove(i)

    def update_drone_belong(self, drone_id, vehicle_id):
        if drone_id == self.id:
            self.drone_belong = vehicle_id

    def delete_drone_belong_to_vehicle(self, drone_id):
        if not isinstance(drone_id, list):
            self.drone_belong_to_vehicle.pop(drone_id)
        else:
            for i in drone_id:
                self.drone_belong_to_vehicle.pop(i)

    def add_node(self, node_id, arrive_time=0, departure_time=0):

        self.tasks[node_id] = []
        self.arrive_times.append(arrive_time)
        self.departure_times.append(departure_time)

    def add_task(self, node_id, task_type, start_time, end_time, details=None):

        if node_id not in self.tasks:
            self.add_node(node_id)

        task = Task(task_type, start_time, end_time, details)
        self.tasks[node_id].append(task)
        return task

    def get_node_tasks(self, node_id):

        return self.tasks.get(node_id, [])

    def set_arrive_time(self, time):

        self.arrive_times.append(time)

    def set_departure_time(self, time):

        self.departure_times.append(time)

    def find_task(self, node_id, task_type):

        tasks_at_node = self.tasks.get(node_id, [])

        for task in tasks_at_node:
            if task.task_type == task_type:
                return task

        return None

    def delete_task(self, node_id, task_type):

        if node_id not in self.tasks:
            return False

        original_tasks = self.tasks[node_id]

        task_to_delete = None
        for task in original_tasks:
            if task.task_type == task_type:
                task_to_delete = task
                break

        if task_to_delete:

            original_tasks.remove(task_to_delete)
            return True

        return False

    def to_dict(self):

        result = {
            'vehicle_id': self.id,
            'vehicle_type': self.vehicleType,
            'nodes': {}
        }

        for node_id in self.tasks:
            node_dict = {
                'arrive_time': self.arrive_times.get(node_id, 0),
                'departure_time': self.departure_times.get(node_id, 0),
                'tasks': [task.to_dict() for task in self.tasks[node_id]]
            }
            result['nodes'][str(node_id)] = node_dict

        return result

    def fast_copy(self):

        new_task = _object_new(vehicle_task)
        new_task.vehicleType = self.vehicleType
        new_task.id = self.id
        new_task.node = self.node
        new_task.node_id = self.node_id
        new_task.tasks = {node_id: task_list.copy() for node_id, task_list in self.tasks.items()}
        new_task.arrive_times = self.arrive_times.copy()
        new_task.departure_times = self.departure_times.copy()
        new_task.prcise_arrive_time = None
        new_task.prcise_departure_time = None
        if self.vehicleType == TYPE_TRUCK:
            new_task.dict_vehicle = {}
        new_task.is_task = self.is_task
        if self.vehicleType == TYPE_TRUCK:
            new_task.drone_list = [d_id for d_id in self.drone_list]
            new_task.launch_drone_list = [d_id for d_id in self.launch_drone_list]
            new_task.recovery_drone_list = [d_id for d_id in self.recovery_drone_list]
            new_task.drone_belong = None
        else:
            new_task.drone_list = None
            new_task.launch_drone_list = None
            new_task.recovery_drone_list = None
            new_task.drone_belong = self.drone_belong if self.vehicleType == TYPE_UAV else None
        return new_task

    def _fast_copy_legacy(self):
        new_task = vehicle_task(self.id, self.vehicleType, self.node_id, self.node)
        new_task.is_task = self.is_task
        new_task.tasks = {node_id: task_list.copy() for node_id, task_list in self.tasks.items()}
        new_task.arrive_times = self.arrive_times.copy()
        new_task.departure_times = self.departure_times.copy()
        if self.vehicleType == TYPE_TRUCK:
            new_task.drone_list = [d_id for d_id in self.drone_list]
            new_task.launch_drone_list = [d_id for d_id in self.launch_drone_list]
            new_task.recovery_drone_list = [d_id for d_id in self.recovery_drone_list]
        elif self.vehicleType == TYPE_UAV:
            new_task.drone_belong = self.drone_belong
        return new_task

    def print_tasks(self):

        print(f"\n===== 车辆 ID: {self.id} ({self.vehicleType}) 任务报告 =====")

        if not self.tasks:
            print("该车辆没有分配任务")
            return

        for node_id in sorted(self.tasks.keys()):
            print(f"\n节点 {node_id}:")
            print(f"  到达时间: {self.arrive_times.get(node_id, 0):.2f}")
            print(f"  离开时间: {self.departure_times.get(node_id, 0):.2f}")

            if not self.tasks[node_id]:
                print("  无任务执行")
                continue

            print("  任务列表:")
            for i, task in enumerate(sorted(self.tasks[node_id], key=lambda t: t.start_time), 1):
                task_name = TASK_NAMES.get(task.task_type, "未知任务")
                print(f"    {i}. {task_name} ({task.start_time:.2f} → {task.end_time:.2f})")

                if task.details:
                    details_str = ", ".join(f"{k}={v}" for k, v in task.details.items())
                    print(f"       详情: {details_str}")

def find_belong_vehicle_id(vehicle, uav, node_id):
    for vehicle_id in vehicle:
        drone_list = list(vehicle[vehicle_id][node_id].drone_list)
        if uav.id in drone_list:
            return vehicle_id
    print(f"无人机 {uav.id} 在节点 {node_id} 没有归属车辆")
    return None

def remove_vehicle_task(vehicle_task_data, y, vehicle_route):
    drone_id, vtp_i, customer, vtp_j, v_id, recv_v_id = y
    veh_launch_index = v_id -1
    veh_recovery_index = recv_v_id -1
    update_vheicle_route = vehicle_route.copy()

    if v_id == recv_v_id:
        task_route = vehicle_route[veh_launch_index]
        task_route_launch_index = task_route.index(vtp_i)
        task_route_recovery_index = task_route.index(vtp_j)
        remove_uav_route = task_route[task_route_launch_index:task_route_recovery_index+1]
        for node_index, node in enumerate(remove_uav_route):
            if node_index == 0:
                vehicle_task_data[v_id][node].launch_drone_list.remove(drone_id)
                vehicle_task_data[v_id][node].drone_list.append(drone_id)
                vehicle_task_data[drone_id][node].dict_vehicle[v_id]['drone_belong'] = v_id
            elif node_index == len(remove_uav_route)-1:
                vehicle_task_data[v_id][node].recovery_drone_list.remove(drone_id)
                if drone_id not in vehicle_task_data[v_id][node].launch_drone_list:
                    vehicle_task_data[v_id][node].drone_list.append(drone_id)
                    vehicle_task_data[v_id][node].drone_list = remove_duplicates(vehicle_task_data[v_id][node].drone_list)
                    vehicle_task_data[drone_id][node].dict_vehicle[v_id]['drone_belong'] = v_id
                else:
                    vehicle_task_data[drone_id][node].dict_vehicle[v_id]['drone_belong'] = v_id
            else:
                vehicle_task_data[v_id][node].drone_list.append(drone_id)
                vehicle_task_data[drone_id][node].dict_vehicle[v_id]['drone_belong'] = v_id
    else:
        task_launch_route = vehicle_route[veh_launch_index]
        task_recovery_route = vehicle_route[veh_recovery_index]
        task_launch_index = task_launch_route.index(vtp_i)
        task_recovery_index = task_recovery_route.index(vtp_j)
        remove_launch_uav_route = task_launch_route[task_launch_index:]
        remove_recovery_uav_route = task_recovery_route[task_recovery_index:]
        for node_index, node in enumerate(remove_launch_uav_route):
            if node_index == 0:
                vehicle_task_data[v_id][node].launch_drone_list.remove(drone_id)
                vehicle_task_data[v_id][node].drone_list.append(drone_id)
                vehicle_task_data[v_id][node].drone_list = remove_duplicates(vehicle_task_data[v_id][node].drone_list)
                vehicle_task_data[drone_id][node].dict_vehicle[v_id]['drone_belong'] = v_id
                continue
            vehicle_carry_drone_list = vehicle_task_data[v_id][node].drone_list
            if drone_id not in vehicle_task_data[v_id][node].recovery_drone_list and drone_id not in vehicle_task_data[v_id][node].drone_list:
                vehicle_task_data[v_id][node].drone_list.append(drone_id)
                vehicle_task_data[drone_id][node].dict_vehicle[v_id]['drone_belong'] = v_id
            elif drone_id in vehicle_task_data[v_id][node].recovery_drone_list:

                vehicle_task_data[drone_id][node].dict_vehicle[v_id]['drone_belong'] = v_id
                break
        for node_index, node in enumerate(remove_recovery_uav_route):
            if node_index == 0:
                vehicle_task_data[recv_v_id][node].recovery_drone_list.remove(drone_id)
                vehicle_task_data[drone_id][node].dict_vehicle[recv_v_id]['drone_belong'] = None

                if drone_id not in vehicle_task_data[recv_v_id][node].launch_drone_list:
                    vehicle_task_data[recv_v_id][node].drone_list.remove(drone_id)
                    vehicle_task_data[recv_v_id][node].drone_list = remove_duplicates(vehicle_task_data[recv_v_id][node].drone_list)

                elif drone_id in vehicle_task_data[recv_v_id][node].launch_drone_list:
                    vehicle_task_data[drone_id][node].dict_vehicle[recv_v_id]['drone_belong'] = recv_v_id
                    break

            if drone_id not in vehicle_task_data[recv_v_id][node].launch_drone_list and drone_id in vehicle_task_data[recv_v_id][node].drone_list:
                vehicle_task_data[recv_v_id][node].drone_list.remove(drone_id)
                vehicle_task_data[recv_v_id][node].drone_list = remove_duplicates(vehicle_task_data[recv_v_id][node].drone_list)
                vehicle_task_data[drone_id][node].dict_vehicle[recv_v_id]['drone_belong'] = None
            elif drone_id in vehicle_task_data[recv_v_id][node].launch_drone_list:
                vehicle_task_data[drone_id][node].dict_vehicle[recv_v_id]['drone_belong'] = recv_v_id
                break

    return vehicle_task_data
from initialize import deep_copy_vehicle_task_data
def restore_vehicle_task_data_for_vehicles(
    temp_vehicle_task_data,
    original_vehicle_task_data,
    changed_vehicle_ids
):

    from collections import defaultdict

    changed_vehicle_ids = set(changed_vehicle_ids)

    if temp_vehicle_task_data is None:
        temp_vehicle_task_data = {}

    restored_data = temp_vehicle_task_data.copy()

    for veh_id in changed_vehicle_ids:

        if veh_id not in original_vehicle_task_data:

            continue

        inner_dict = original_vehicle_task_data[veh_id]

        new_inner = inner_dict.copy()

        for node_id, task_obj in inner_dict.items():
            new_inner[node_id] = task_obj.fast_copy()

        restored_data[veh_id] = new_inner

    return restored_data

def deep_remove_vehicle_task(vehicle_task_data, y, vehicle_route, orig_vehicle_id):
    drone_id, vtp_i, customer, vtp_j, v_id, recv_v_id = y
    veh_launch_index = v_id -1
    veh_recovery_index = recv_v_id -1

    if v_id == recv_v_id:
        task_route = vehicle_route[veh_launch_index]
        task_route_launch_index = task_route.index(vtp_i)
        task_route_recovery_index = task_route.index(vtp_j)
        remove_uav_route = task_route[task_route_launch_index:task_route_recovery_index+1]
        for node_index, node in enumerate(remove_uav_route):
            if node_index == 0:
                vehicle_task_data[v_id][node].launch_drone_list.remove(drone_id)

                vehicle_task_data[drone_id][node].dict_vehicle[v_id]['drone_belong'] = None
            elif node_index == len(remove_uav_route)-1:
                vehicle_task_data[v_id][node].recovery_drone_list.remove(drone_id)
                if drone_id in vehicle_task_data[v_id][node].drone_list:
                    vehicle_task_data[v_id][node].drone_list.remove(drone_id)
                    vehicle_task_data[v_id][node].drone_list = remove_duplicates(vehicle_task_data[v_id][node].drone_list)
                    vehicle_task_data[drone_id][node].dict_vehicle[v_id]['drone_belong'] = None

            else:

                vehicle_task_data[drone_id][node].dict_vehicle[v_id]['drone_belong'] = None

        rm_route_list = task_route[task_route_recovery_index:]
        is_launch_task = False

        for index, node in enumerate(rm_route_list):
            if index == 0:
                if drone_id in vehicle_task_data[v_id][node].launch_drone_list:
                    is_launch_task = True
                    break
            else:
                if drone_id not in vehicle_task_data[v_id][node].launch_drone_list and drone_id in vehicle_task_data[v_id][node].drone_list and drone_id not in vehicle_task_data[v_id][node].recovery_drone_list:
                    vehicle_task_data[v_id][node].drone_list.remove(drone_id)
                if drone_id in vehicle_task_data[v_id][node].launch_drone_list:
                    is_launch_task = True
                    break
        if not is_launch_task:
            for index, node in enumerate(rm_route_list):
                if index == 0:
                    continue
                else:
                    if drone_id in vehicle_task_data[v_id][node].drone_list:
                        vehicle_task_data[v_id][node].drone_list.remove(drone_id)

                        vehicle_task_data[drone_id][node].dict_vehicle[v_id]['drone_belong'] = None
    else:
        task_launch_route = vehicle_route[veh_launch_index]
        task_recovery_route = vehicle_route[veh_recovery_index]
        task_launch_index = task_launch_route.index(vtp_i)
        task_recovery_index = task_recovery_route.index(vtp_j)
        remove_launch_uav_route = task_launch_route[task_launch_index:]
        remove_recovery_uav_route = task_recovery_route[task_recovery_index:]
        for node_index, node in enumerate(remove_launch_uav_route):
            if node_index == 0:
                vehicle_task_data[v_id][node].launch_drone_list.remove(drone_id)

                vehicle_task_data[drone_id][node].dict_vehicle[v_id]['drone_belong'] = None
                continue
            vehicle_carry_drone_list = vehicle_task_data[v_id][node].drone_list
            if drone_id not in vehicle_task_data[v_id][node].recovery_drone_list and drone_id not in vehicle_task_data[v_id][node].drone_list:

                vehicle_task_data[drone_id][node].dict_vehicle[v_id]['drone_belong'] = None

            if drone_id not in vehicle_task_data[v_id][node].recovery_drone_list and drone_id in vehicle_task_data[v_id][node].drone_list:
                vehicle_task_data[v_id][node].drone_list.remove(drone_id)

                vehicle_task_data[drone_id][node].dict_vehicle[v_id]['drone_belong'] = None
            if drone_id in vehicle_task_data[v_id][node].recovery_drone_list:

                vehicle_task_data[drone_id][node].dict_vehicle[v_id]['drone_belong'] = v_id
                break
            if drone_id in vehicle_task_data[v_id][node].launch_drone_list:
                vehicle_task_data[drone_id][node].dict_vehicle[v_id]['drone_belong'] = v_id
                break

        for node_index, node in enumerate(remove_recovery_uav_route):
            if node_index == 0:
                vehicle_task_data[recv_v_id][node].recovery_drone_list.remove(drone_id)
                vehicle_task_data[drone_id][node].dict_vehicle[recv_v_id]['drone_belong'] = recv_v_id
                if orig_vehicle_id == recv_v_id:
                    vehicle_task_data[drone_id][node].dict_vehicle[recv_v_id]['drone_belong'] = recv_v_id
                    break
                elif orig_vehicle_id != recv_v_id and drone_id not in vehicle_task_data[recv_v_id][node].launch_drone_list and drone_id in vehicle_task_data[recv_v_id][node].drone_list:
                    vehicle_task_data[recv_v_id][node].drone_list.remove(drone_id)

                    vehicle_task_data[drone_id][node].dict_vehicle[recv_v_id]['drone_belong'] = None
                elif drone_id in vehicle_task_data[recv_v_id][node].launch_drone_list:
                    vehicle_task_data[drone_id][node].dict_vehicle[recv_v_id]['drone_belong'] = recv_v_id
                    break

            if drone_id not in vehicle_task_data[recv_v_id][node].launch_drone_list and drone_id in vehicle_task_data[recv_v_id][node].drone_list:
                vehicle_task_data[recv_v_id][node].drone_list.remove(drone_id)

                vehicle_task_data[drone_id][node].dict_vehicle[recv_v_id]['drone_belong'] = recv_v_id
            elif drone_id in vehicle_task_data[recv_v_id][node].launch_drone_list:
                vehicle_task_data[drone_id][node].dict_vehicle[recv_v_id]['drone_belong'] = recv_v_id
                break

    return vehicle_task_data

def update_vehicle_task(vehicle_task_data, y, vehicle_route):
    drone_id, vtp_i, customer, vtp_j, v_id, recv_v_id = y

    veh_launch_index = v_id -1
    veh_recovery_index = recv_v_id -1
    update_vheicle_route = vehicle_route.copy()

    if v_id == recv_v_id:
        task_route = vehicle_route[veh_launch_index]
        task_route_launch_index = task_route.index(vtp_i)
        task_route_recovery_index = task_route.index(vtp_j)
        remove_uav_route = task_route[task_route_launch_index:task_route_recovery_index+1]
        for node_index, node in enumerate(remove_uav_route):
            if node_index == 0:
                vehicle_task_data[v_id][node].launch_drone_list.append(drone_id)
                vehicle_task_data[drone_id][node].dict_vehicle[v_id]['drone_belong'] = v_id
                vehicle_task_data[v_id][node].drone_list.remove(drone_id)
            elif node_index == len(remove_uav_route)-1:
                vehicle_task_data[v_id][node].recovery_drone_list.append(drone_id)
                vehicle_task_data[drone_id][node].dict_vehicle[v_id]['drone_belong'] = v_id

                if drone_id not in vehicle_task_data[v_id][node].launch_drone_list:
                    vehicle_task_data[v_id][node].drone_list.append(drone_id)

                    vehicle_task_data[v_id][node].drone_list = remove_duplicates(vehicle_task_data[v_id][node].drone_list)

            else:
                vehicle_task_data[v_id][node].drone_list.remove(drone_id)

    else:
        task_launch_route = vehicle_route[veh_launch_index]
        task_recovery_route = vehicle_route[veh_recovery_index]
        task_launch_index = task_launch_route.index(vtp_i)
        task_recovery_index = task_recovery_route.index(vtp_j)
        remove_launch_uav_route = task_launch_route[task_launch_index:]
        remove_recovery_uav_route = task_recovery_route[task_recovery_index:]
        for node_index, node in enumerate(remove_launch_uav_route):
            if node_index == 0:
                vehicle_task_data[v_id][node].launch_drone_list.append(drone_id)
                vehicle_task_data[v_id][node].drone_list.remove(drone_id)
                vehicle_task_data[v_id][node].drone_list = remove_duplicates(vehicle_task_data[v_id][node].drone_list)
                vehicle_task_data[drone_id][node].dict_vehicle[v_id]['drone_belong'] = v_id
                continue
            vehicle_carry_drone_list = vehicle_task_data[v_id][node].drone_list
            if drone_id not in vehicle_task_data[v_id][node].recovery_drone_list:
                vehicle_task_data[v_id][node].drone_list.remove(drone_id)
                vehicle_task_data[drone_id][node].dict_vehicle[v_id]['drone_belong'] = None
            else:
                vehicle_task_data[drone_id][node].dict_vehicle[v_id]['drone_belong'] = v_id
                break
        for node_index, node in enumerate(remove_recovery_uav_route):
            if node_index == 0:
                vehicle_task_data[recv_v_id][node].recovery_drone_list.append(drone_id)
                vehicle_task_data[drone_id][node].dict_vehicle[recv_v_id]['drone_belong'] = recv_v_id

                if drone_id not in vehicle_task_data[recv_v_id][node].launch_drone_list:
                    vehicle_task_data[recv_v_id][node].drone_list.append(drone_id)
                    vehicle_task_data[recv_v_id][node].drone_list = remove_duplicates(vehicle_task_data[recv_v_id][node].drone_list)

                continue
            vehicle_carry_drone_list = vehicle_task_data[recv_v_id][node].drone_list
            if drone_id not in vehicle_task_data[recv_v_id][node].launch_drone_list:
                vehicle_task_data[recv_v_id][node].drone_list.append(drone_id)
                vehicle_task_data[recv_v_id][node].drone_list = remove_duplicates(vehicle_task_data[recv_v_id][node].drone_list)
                vehicle_task_data[drone_id][node].dict_vehicle[recv_v_id]['drone_belong'] = recv_v_id
            else:
                vehicle_task_data[drone_id][node].dict_vehicle[recv_v_id]['drone_belong'] = recv_v_id
                break

    return vehicle_task_data

def remove_duplicates(lst):
    result = []
    seen = set()
    for num in lst:
        if num not in seen:
            result.append(num)
            seen.add(num)
    return result

from typing import List, Union, Any

def normalize_input_data(data: Union[int, float, List[Any]]) -> List[Any]:

    if not isinstance(data, list):

        if isinstance(data, (int, float)):
            return [data]

        else:
            raise TypeError(f"输入类型不受支持，期望是 int, float 或 list，但得到的是 {type(data).__name__}")

    is_nested = any(isinstance(item, list) for item in data)

    if not is_nested:
        return data
    else:
        return [
            item
            for sublist_or_item in data
            for item in (sublist_or_item if isinstance(sublist_or_item, list) else [sublist_or_item])
        ]

def update_delta_time(delta_time, detailed_vehicle_task_data, vehicle_route, y_ijkd, vehicle):

    drone_id, vtp_i, customer, vtp_j, v_id, recv_v_id = y_ijkd

    if v_id == recv_v_id:
        task_route = vehicle_route[v_id - 1]

        try:
            launch_node_index = task_route.index(vtp_i)
        except ValueError:

            print(f"错误：发射节点 {vtp_i} 不在车辆 {v_id} 的路线中。")
            return detailed_vehicle_task_data

        remain_route = task_route[launch_node_index + 1:]

        detailed_vehicle_task_data = update_delta_route(
            delta_time,
            detailed_vehicle_task_data,
            v_id,
            remain_route,
            vehicle
        )
    else:
        v_id_task_route = vehicle_route[v_id - 1]
        recv_v_id_task_route = vehicle_route[recv_v_id - 1]
        launch_node_index = v_id_task_route.index(vtp_i)
        recovery_node_index = recv_v_id_task_route.index(vtp_j)
        launch_remain_route = v_id_task_route[launch_node_index + 1:]
        recovery_remain_route = recv_v_id_task_route[recovery_node_index:]
        detailed_vehicle_task_data = update_delta_route(
            delta_time,
            detailed_vehicle_task_data,
            v_id,
            launch_remain_route,
            vehicle
        )
        detailed_vehicle_task_data = update_delta_route(
            0,
            detailed_vehicle_task_data,
            recv_v_id,
            recovery_remain_route,
            vehicle
        )

    return detailed_vehicle_task_data

def update_delta_route(
    delta_time: float,
    detailed_vehicle_task_data: dict,
    v_id: int,
    remain_route: list,
    vehicle: dict,
):

    accumulated_delay = delta_time

    for node_id in remain_route:
        task_obj = detailed_vehicle_task_data[v_id][node_id]
        task_obj_prcise_arrive_time = task_obj.prcise_arrive_time
        task_obj_prcise_departure_time = task_obj.prcise_departure_time

        task_obj.prcise_arrive_time = task_obj_prcise_arrive_time + accumulated_delay
        task_obj_prcise_arrive_time += accumulated_delay
        if task_obj_prcise_departure_time < task_obj_prcise_arrive_time:
            task_obj.prcise_departure_time = task_obj_prcise_arrive_time
            task_obj_prcise_departure_time = task_obj_prcise_arrive_time

        task_obj.arrive_times.append(task_obj.prcise_arrive_time)
        task_obj.departure_times.append(task_obj.prcise_departure_time)

        drones_to_recover = task_obj.recovery_drone_list
        drones_to_launch = task_obj.launch_drone_list
        drones_to_carry = task_obj.drone_list
        copy_drones_to_carry = (drones_to_carry or []) + (drones_to_launch or [])

        if not drones_to_recover:
            if not copy_drones_to_carry:
                task_obj.prcise_departure_time = task_obj.prcise_arrive_time
                continue
            else:
                for drone_to_carry_id in copy_drones_to_carry:
                    drone_task_obj = detailed_vehicle_task_data[drone_to_carry_id][node_id]
                    drone_task_obj.prcise_arrive_time = task_obj.prcise_arrive_time
                    drone_task_obj.dict_vehicle[v_id]['prcise_arrive_time'] = task_obj.prcise_arrive_time
                    if drone_task_obj.prcise_departure_time < drone_task_obj.prcise_arrive_time:
                        drone_task_obj.prcise_departure_time = drone_task_obj.prcise_arrive_time
                        drone_task_obj.dict_vehicle[v_id]['prcise_departure_time'] = task_obj_prcise_departure_time
                continue
        else:

            pending_recoveries = []
            for drone_to_recover_id in drones_to_recover:

                drone_task_obj = detailed_vehicle_task_data[drone_to_recover_id][node_id]

                drone_arrival_time = detailed_vehicle_task_data[drone_to_recover_id][node_id].dict_vehicle[v_id]['prcise_arrive_time']

                pending_recoveries.append({
                    "drone_id": drone_to_recover_id,
                    "arrival_time": drone_arrival_time
                })

            pending_recoveries.sort(key=lambda x: x['arrival_time'])

            node_service_timeline = task_obj.prcise_arrive_time

            for recovery_event in pending_recoveries:
                drone_id = recovery_event['drone_id']
                drone_arrival = recovery_event['arrival_time']

                recovery_start_time = max(node_service_timeline, drone_arrival)

                recovery_duration = vehicle[drone_id].recoveryTime

                recovery_end_time = recovery_start_time + recovery_duration

                node_service_timeline = recovery_end_time

                uav_task_found = drone_task_obj.find_task(node_id, 14)
                if uav_task_found:
                    drone_task_obj.delete_task(node_id, 14)
                drone_task_obj.add_task(node_id, 14, drone_arrival, recovery_end_time)

                drone_task_obj.departure_times.append(recovery_end_time)
                drone_task_obj.dict_vehicle[v_id]['departure_times'].append(recovery_end_time)

            task_obj.prcise_departure_time = node_service_timeline
            for drone_to_recover_id in drones_to_recover:
                drone_task_obj = detailed_vehicle_task_data[drone_to_recover_id][node_id]
                drone_task_obj.prcise_departure_time = node_service_timeline
                drone_task_obj.dict_vehicle[v_id]['prcise_departure_time'] = node_service_timeline

            for drone_to_launch_id in drones_to_launch:
                drone_task_obj = detailed_vehicle_task_data[drone_to_launch_id][node_id]
                drone_task_obj.prcise_departure_time = node_service_timeline
                drone_task_obj.dict_vehicle[v_id]['prcise_departure_time'] = node_service_timeline

            if node_service_timeline > task_obj.prcise_arrive_time:
                original_arrival_time = task_obj.prcise_arrive_time
                accumulated_delay += (task_obj.prcise_departure_time - original_arrival_time)
            else:
                original_arrival_time = task_obj.prcise_arrive_time
                accumulated_delay += (task_obj.prcise_departure_time - original_arrival_time)

    return detailed_vehicle_task_data

def update_uav_plan(detailed_vehicle_task_data, best_uav_plan):

    uav_plan_time = copy.copy(best_uav_plan)

    for y_ijkd in best_uav_plan:
        uav_id, launch_node, customer, recovery_node, launch_veh_id, recovery_veh_id = y_ijkd

        uav_plan_time[y_ijkd]['launch_time'] = detailed_vehicle_task_data[uav_id][launch_node].dict_vehicle[launch_veh_id]['prcise_departure_time']

        uav_plan_time[y_ijkd]['recovery_time'] = detailed_vehicle_task_data[uav_id][recovery_node].dict_vehicle[recovery_veh_id]['prcise_arrive_time']
    return uav_plan_time

def update_vehicle_arrive_time(detailed_vehicle_task_data, vehicle_arrival_time):
    update_vehicle_arrive_time = defaultdict(lambda: defaultdict(list))
    for v_id, key_values in vehicle_arrival_time.items():
        for key in key_values.keys():
            arrive_time = detailed_vehicle_task_data[v_id][key].prcise_arrive_time
            departure_time = detailed_vehicle_task_data[v_id][key].prcise_departure_time
            update_vehicle_arrive_time[v_id][key] = [arrive_time, departure_time]
    return update_vehicle_arrive_time

def update_re_uav_plan(state, re_uav_plan):

    vehicle_arrive_time = state.rm_empty_vehicle_arrive_time
    customer_plan = state.customer_plan
    customer_cost = state.uav_cost
    uav_travel = state.uav_travel
    node_collection = state.node
    xeee_matrix = state.xeee

    temp_tasks = []

    for customer_id, plan in customer_plan.items():

        drone_id = plan[0]
        launch_node = plan[1]
        target_customer = plan[2]
        recovery_node = plan[3]
        launch_vehicle = plan[4]
        recovery_vehicle = plan[5]

        mission_key = (drone_id, launch_node, target_customer, recovery_node, launch_vehicle, recovery_vehicle)

        l_map_key = node_collection[launch_node].map_key
        r_map_key = node_collection[recovery_node].map_key

        try:
            l_arrival_time = vehicle_arrive_time[launch_vehicle][launch_node]
        except KeyError:
            l_arrival_time = float('inf')

        try:
            r_arrival_time = vehicle_arrive_time[recovery_vehicle][recovery_node]
        except KeyError:
            r_arrival_time = float('inf')

        travel_obj = uav_travel[drone_id][l_map_key][r_map_key]

        uav_route_path = travel_obj.path

        route_time = getattr(travel_obj, 'totalTime', 0)

        energy_val = xeee_matrix[drone_id][l_map_key][target_customer][r_map_key]

        task_info = {
            'drone_id': drone_id,
            'launch_vehicle': launch_vehicle,
            'recovery_vehicle': recovery_vehicle,
            'launch_node': launch_node,
            'recovery_node': recovery_node,
            'customer': target_customer,
            'launch_time': l_arrival_time,
            'recovery_time': r_arrival_time,
            'energy': energy_val,
            'cost': customer_cost[target_customer],
            'time': route_time,
            'uav_route': uav_route_path
        }

        temp_tasks.append((l_arrival_time, mission_key, task_info))

    temp_tasks.sort(key=lambda x: x[0])

    sorted_re_uav_plan = {item[1]: item[2] for item in temp_tasks}

    state.re_uav_plan = sorted_re_uav_plan

    return sorted_re_uav_plan

import os
import json
from typing import Any, Dict, List
import pandas as pd

def save_alns_results(
    instance_name: str,
    y_best,
    y_cost,
    win_cost,
    uav_route_cost,
    vehicle_route_cost,
    strategy_weights,
    operator_weights,
    elapsed_time,
    best_objective,
    best_vehicle_max_times,
    best_global_max_time,
    best_arrive_time,
    best_window_total_cost,
    best_uav_tw_violation_cost,
    best_total_cost_dict,
    best_state,

    best_final_uav_cost=None,
    best_final_objective=None,
    best_final_win_cost=None,
    best_total_win_cost=None,

    best_final_vehicle_route_cost=None,

    final_uav_cost=None,
    final_total_list=None,
    final_win_cost=None,
    final_total_objective=None,
    final_vehicle_route_cost=None,

    best_final_vehicle_max_times=None,
    best_final_global_max_time=None,
    work_time=None,
    final_work_time=None,

    best_final_state=None,

    operator_statistics=None,
    operator_combinations=None,
    best_operator_combo=None,
    iteration_operator_log=None,
    best_destroy_by_avg_score=None,
    best_repair_by_avg_score=None,
    best_combo_by_avg_score=None,
    best_combo_by_new_best=None,
    operator_history=None,

    base_dir = None,
) -> None:

    base_dir = os.path.abspath(base_dir)
    os.makedirs(base_dir, exist_ok=True)

    case_dir = os.path.join(base_dir, instance_name)
    os.makedirs(case_dir, exist_ok=True)

    prefix = os.path.join(case_dir, instance_name)

    if best_uav_tw_violation_cost is None:
        buv_for_summary = None
    elif hasattr(best_uav_tw_violation_cost, "items"):
        buv_for_summary = {k: float(v) for k, v in best_uav_tw_violation_cost.items()}
    else:
        buv_for_summary = float(best_uav_tw_violation_cost)

    if best_total_cost_dict is None:
        btcd_for_summary = None
    elif hasattr(best_total_cost_dict, "items"):
        btcd_for_summary = {k: float(v) for k, v in best_total_cost_dict.items()}
    else:
        btcd_for_summary = best_total_cost_dict

    if best_vehicle_max_times is None:
        bvt_for_summary = None
    elif hasattr(best_vehicle_max_times, "items"):
        bvt_for_summary = {k: float(v) for k, v in best_vehicle_max_times.items()}
    else:
        bvt_for_summary = float(best_vehicle_max_times)

    if best_final_vehicle_max_times is None:
        bfvt_for_summary = None
    elif hasattr(best_final_vehicle_max_times, "items"):
        bfvt_for_summary = {k: float(v) for k, v in best_final_vehicle_max_times.items()}
    else:
        bfvt_for_summary = float(best_final_vehicle_max_times)

    summary_data = {
        "instance_name": instance_name,
        "best_objective": float(best_objective) if best_objective is not None else None,
        "elapsed_time": float(elapsed_time) if elapsed_time is not None else None,
        "best_global_max_time": float(best_global_max_time) if best_global_max_time is not None else None,
        "len_y_cost": len(y_cost) if y_cost is not None else 0,
        "len_y_best": len(y_best) if y_best is not None else 0,
        "len_win_cost": len(win_cost) if win_cost is not None else 0,
        "len_uav_route_cost": len(uav_route_cost) if uav_route_cost is not None else 0,
        "len_vehicle_route_cost": len(vehicle_route_cost) if vehicle_route_cost is not None else 0,
        "strategy_weights": strategy_weights,
        "operator_weights": operator_weights,
        "best_vehicle_max_times": bvt_for_summary,
        "best_arrive_time_brief": {
            "num_vehicles": len(best_arrive_time),
            "total_records": sum(len(v) for v in best_arrive_time.items())
            if hasattr(best_arrive_time, "items")
            else sum(len(v) for v in best_arrive_time.values()),
        },
        "best_window_total_cost": float(best_window_total_cost) if best_window_total_cost is not None else None,
        "best_total_cost_dict": btcd_for_summary,
        "best_uav_tw_violation_cost": buv_for_summary,
        "best_state_total_cost_attr": getattr(best_state, "_total_cost", None),

        "best_final_uav_cost": float(best_final_uav_cost) if best_final_uav_cost is not None else None,
        "best_final_objective": float(best_final_objective) if best_final_objective is not None else None,
        "best_final_win_cost": float(best_final_win_cost) if best_final_win_cost is not None else None,
        "best_total_win_cost": float(best_total_win_cost) if best_total_win_cost is not None else None,

        "best_final_vehicle_route_cost": float(best_final_vehicle_route_cost) if best_final_vehicle_route_cost is not None else None,
        "best_final_state_total_cost_attr": getattr(best_final_state, "_total_cost", None) if best_final_state else None,

        "best_final_vehicle_max_times": bfvt_for_summary,
        "best_final_global_max_time": float(best_final_global_max_time) if best_final_global_max_time is not None else None,

        "len_final_uav_cost": len(final_uav_cost) if final_uav_cost is not None else 0,
        "len_final_total_list": len(final_total_list) if final_total_list is not None else 0,
        "len_final_win_cost": len(final_win_cost) if final_win_cost is not None else 0,
        "len_final_total_objective": len(final_total_objective) if final_total_objective is not None else 0,
        "len_final_vehicle_route_cost": len(final_vehicle_route_cost) if final_vehicle_route_cost is not None else 0,
        "len_work_time": len(work_time) if work_time is not None else 0,
        "len_final_work_time": len(final_work_time) if final_work_time is not None else 0,
    }

    summary_txt_path = f"{prefix}_summary.txt"

    with open(summary_txt_path, "w", encoding="utf-8") as f:
        f.write("==== ALNS Result Summary ====\n")
        for k, v in summary_data.items():
            if isinstance(v, (dict, list)):
                v_clean = make_json_friendly(v)
                f.write(f"{k}:\n{json.dumps(v_clean, ensure_ascii=False, indent=2)}\n\n")
            else:
                f.write(f"{k}: {v}\n")

    curves_xlsx_path = f"{prefix}_curves.xlsx"

    def _pad_list(lst: List[Any], target_len: int) -> List[Any]:
        if lst is None:
            return [None] * target_len
        lst = list(lst)
        if len(lst) < target_len:
            lst = lst + [None] * (target_len - len(lst))
        return lst

    max_len = max(
        len(y_cost),
        len(y_best),
        len(win_cost),
        len(uav_route_cost),
        len(vehicle_route_cost),
        len(final_uav_cost) if final_uav_cost is not None else 0,
        len(final_total_list) if final_total_list is not None else 0,
        len(final_win_cost) if final_win_cost is not None else 0,
        len(final_total_objective) if final_total_objective is not None else 0,
        len(final_vehicle_route_cost) if final_vehicle_route_cost is not None else 0,
        len(work_time) if work_time is not None else 0,
        len(final_work_time) if final_work_time is not None else 0,
    )

    df_curves = pd.DataFrame({
        "iteration": list(range(max_len)),
        "y_cost": _pad_list(y_cost, max_len),
        "y_best": _pad_list(y_best, max_len),
        "win_cost": _pad_list(win_cost, max_len),
        "uav_route_cost": _pad_list(uav_route_cost, max_len),
        "vehicle_route_cost": _pad_list(vehicle_route_cost, max_len),

        "final_uav_cost": _pad_list(final_uav_cost, max_len),
        "final_total_list": _pad_list(final_total_list, max_len),
        "final_win_cost": _pad_list(final_win_cost, max_len),
        "final_total_objective": _pad_list(final_total_objective, max_len),
        "final_vehicle_route_cost": _pad_list(final_vehicle_route_cost, max_len),

        "work_time": _pad_list(work_time, max_len),
        "final_work_time": _pad_list(final_work_time, max_len),
    })

    df_strategy_weights = pd.DataFrame(
        [{"strategy": k, "weight": v} for k, v in strategy_weights.items()]
    )

    op_rows = []
    for strat, group in operator_weights.items():
        for op_type, w_dict in group.items():
            for op_name, w in w_dict.items():
                op_rows.append({
                    "strategy": strat,
                    "op_type": op_type,
                    "operator": op_name,
                    "weight": w,
                })
    df_operator_weights = pd.DataFrame(op_rows)

    with pd.ExcelWriter(curves_xlsx_path) as writer:
        df_curves.to_excel(writer, sheet_name="curves", index=False)
        df_strategy_weights.to_excel(writer, sheet_name="strategy_weights", index=False)
        df_operator_weights.to_excel(writer, sheet_name="operator_weights", index=False)

    summary_xlsx_path = f"{prefix}_summary.xlsx"

    if best_vehicle_max_times is not None and hasattr(best_vehicle_max_times, "items"):
        df_vehicle_finish = pd.DataFrame(
            [{"vehicle_id": vid, "completion_time": t} for vid, t in best_vehicle_max_times.items()]
        )
    else:
        df_vehicle_finish = pd.DataFrame([{"vehicle_id": "all", "completion_time": best_vehicle_max_times}])

    if best_uav_tw_violation_cost is not None and hasattr(best_uav_tw_violation_cost, "items"):
        df_tw_violation = pd.DataFrame(
            [{"key": k, "violation_cost": float(v)} for k, v in best_uav_tw_violation_cost.items()]
        )
    else:
        df_tw_violation = pd.DataFrame(columns=["key", "violation_cost"])

    if best_total_cost_dict is not None and hasattr(best_total_cost_dict, "items"):
        df_total_cost_dict = pd.DataFrame(
            [{"key": k, "value": float(v)} for k, v in best_total_cost_dict.items()]
        )
    else:
        df_total_cost_dict = pd.DataFrame(columns=["key", "value"])

    df_summary_scalar = pd.DataFrame(
        [
            {
                "instance_name": instance_name,
                "best_objective": best_objective,
                "elapsed_time": elapsed_time,
                "best_global_max_time": best_global_max_time,
                "best_window_total_cost": best_window_total_cost,
                "best_state_total_cost_attr": getattr(best_state, "_total_cost", None),

                "best_final_uav_cost": best_final_uav_cost,
                "best_final_objective": best_final_objective,
                "best_final_win_cost": best_final_win_cost,
                "best_total_win_cost": best_total_win_cost,

                "best_final_vehicle_route_cost": best_final_vehicle_route_cost,
                "best_final_state_total_cost_attr": getattr(best_final_state, "_total_cost", None) if best_final_state else None,

                "best_final_vehicle_max_times": str(best_final_vehicle_max_times),
                "best_final_global_max_time": best_final_global_max_time,
            }
        ]
    )

    with pd.ExcelWriter(summary_xlsx_path) as writer:
        df_summary_scalar.to_excel(writer, sheet_name="summary", index=False)
        df_vehicle_finish.to_excel(writer, sheet_name="vehicle_finish_time", index=False)
        df_tw_violation.to_excel(writer, sheet_name="tw_violation", index=False)
        df_total_cost_dict.to_excel(writer, sheet_name="total_cost_detail", index=False)

    arrive_xlsx_path = f"{prefix}_best_arrive_time.xlsx"
    arrive_rows = []
    if best_arrive_time:
        for vid, node_times in best_arrive_time.items():
            for node_id, t in node_times.items():
                arrive_rows.append(
                    {"vehicle_id": vid, "node_id": node_id, "arrive_time": t}
                )
    df_arrive = pd.DataFrame(arrive_rows)
    df_arrive.to_excel(arrive_xlsx_path, index=False, sheet_name="arrive_time")

    best_state_xlsx_path = f"{prefix}_best_state.xlsx"

    cp_rows = []
    for cid, assign in best_state.customer_plan.items():
        row = {"customer_id": cid}
        if isinstance(assign, (list, tuple)):
            for i, v in enumerate(assign):
                row[f"field_{i}"] = v
        else:
            row["assignment"] = assign
        cp_rows.append(row)
    df_customer_plan = pd.DataFrame(cp_rows)

    uav_cost = getattr(best_state, "uav_cost", {})
    if isinstance(uav_cost, dict):
        df_uav_cost = pd.DataFrame(
            [{"uav_id": k, "cost": v} for k, v in uav_cost.items()]
        )
    else:
        df_uav_cost = pd.DataFrame(columns=["uav_id", "cost"])

    vr = getattr(best_state, "vehicle_routes", [])
    vr_rows = []
    if isinstance(vr, dict):
        for vid, route in vr.items():
            for idx, node in enumerate(route):
                vr_rows.append(
                    {"vehicle_id": vid, "seq": idx, "node_id": node}
                )
    else:
        for i, route in enumerate(vr):
            vid = i + 1
            for idx, node in enumerate(route):
                vr_rows.append(
                    {"vehicle_id": vid, "seq": idx, "node_id": node}
                )
    df_vehicle_routes = pd.DataFrame(vr_rows)

    uav_plan = getattr(best_state, "uav_plan", None)
    try:
        uav_plan_json = json.dumps(make_json_friendly(uav_plan), ensure_ascii=False)
    except TypeError:
        uav_plan_json = repr(uav_plan)
    df_uav_plan = pd.DataFrame([{"uav_plan_json": uav_plan_json}])

    df_state_scalar = pd.DataFrame(
        [
            {
                "_total_cost_attr": getattr(best_state, "_total_cost", None),
                "objective_now": best_state.objective()
                if hasattr(best_state, "objective")
                else None,
            }
        ]
    )

    final_uav_plan_data_bs = getattr(best_state, "final_uav_plan", None)

    with pd.ExcelWriter(best_state_xlsx_path) as writer:
        df_customer_plan.to_excel(writer, sheet_name="customer_plan", index=False)
        df_uav_cost.to_excel(writer, sheet_name="uav_cost", index=False)
        df_vehicle_routes.to_excel(writer, sheet_name="vehicle_routes", index=False)
        df_uav_plan.to_excel(writer, sheet_name="uav_plan_raw", index=False)
        df_state_scalar.to_excel(writer, sheet_name="state_scalar", index=False)

        if final_uav_plan_data_bs is not None:
            final_rows_bs = []
            for key, info in final_uav_plan_data_bs.items():
                row = {}
                if isinstance(key, tuple) and len(key) >= 6:
                    row["key_drone_id"] = key[0]
                    row["key_launch_node"] = key[1]
                    row["key_customer"] = key[2]
                    row["key_recovery_node"] = key[3]
                    row["key_launch_vehicle"] = key[4]
                    row["key_recovery_vehicle"] = key[5]
                else:
                    row["key_raw"] = str(key)

                if isinstance(info, dict):
                    for field in ["drone_id", "launch_vehicle", "recovery_vehicle",
                                  "launch_node", "recovery_node", "customer",
                                  "launch_time", "recovery_time", "energy",
                                  "cost", "time", "uav_route_cost", "uav_time_cost"]:
                        row[field] = info.get(field)

                    route = info.get("uav_route")
                    try:
                        row["uav_route"] = json.dumps(make_json_friendly(route), ensure_ascii=False)
                    except Exception:
                        row["uav_route"] = str(route)

                    try:
                        row["uav_route_len"] = len(route) if route is not None else 0
                    except TypeError:
                        row["uav_route_len"] = None
                else:
                    row["info_raw"] = str(info)
                final_rows_bs.append(row)

            df_final_uav_bs = pd.DataFrame(final_rows_bs)
            df_final_uav_bs.to_excel(writer, sheet_name="final_uav_plan", index=False)

    if best_final_state is not None:
        best_final_state_xlsx_path = f"{prefix}_best_final_state.xlsx"

        cp_rows = []
        for cid, assign in best_final_state.customer_plan.items():
            row = {"customer_id": cid}
            if isinstance(assign, (list, tuple)):
                for i, v in enumerate(assign):
                    row[f"field_{i}"] = v
            else:
                row["assignment"] = assign
            cp_rows.append(row)
        df_customer_plan = pd.DataFrame(cp_rows)

        uav_cost = getattr(best_final_state, "uav_cost", {})
        if isinstance(uav_cost, dict):
            df_uav_cost = pd.DataFrame(
                [{"uav_id": k, "cost": v} for k, v in uav_cost.items()]
            )
        else:
            df_uav_cost = pd.DataFrame(columns=["uav_id", "cost"])

        vr = getattr(best_final_state, "vehicle_routes", [])
        vr_rows = []
        if isinstance(vr, dict):
            for vid, route in vr.items():
                for idx, node in enumerate(route):
                    vr_rows.append(
                        {"vehicle_id": vid, "seq": idx, "node_id": node}
                    )
        else:
            for i, route in enumerate(vr):
                vid = i + 1
                for idx, node in enumerate(route):
                    vr_rows.append(
                        {"vehicle_id": vid, "seq": idx, "node_id": node}
                    )
        df_vehicle_routes = pd.DataFrame(vr_rows)

        uav_plan = getattr(best_final_state, "uav_plan", None)
        try:
            uav_plan_json = json.dumps(make_json_friendly(uav_plan), ensure_ascii=False)
        except TypeError:
            uav_plan_json = repr(uav_plan)
        df_uav_plan = pd.DataFrame([{"uav_plan_json": uav_plan_json}])

        df_state_scalar = pd.DataFrame(
            [
                {
                    "_total_cost_attr": getattr(best_final_state, "_total_cost", None),
                    "objective_now": best_final_state.objective()
                    if hasattr(best_final_state, "objective")
                    else None,
                }
            ]
        )

        final_uav_plan_data = getattr(best_final_state, "final_uav_plan", None)

        with pd.ExcelWriter(best_final_state_xlsx_path) as writer:
            df_customer_plan.to_excel(writer, sheet_name="customer_plan", index=False)
            df_uav_cost.to_excel(writer, sheet_name="uav_cost", index=False)
            df_vehicle_routes.to_excel(writer, sheet_name="vehicle_routes", index=False)
            df_uav_plan.to_excel(writer, sheet_name="uav_plan_raw", index=False)
            df_state_scalar.to_excel(writer, sheet_name="state_scalar", index=False)

            if final_uav_plan_data is not None:
                final_rows = []
                for key, info in final_uav_plan_data.items():
                    row = {}
                    if isinstance(key, tuple) and len(key) >= 6:
                        row["key_drone_id"] = key[0]
                        row["key_launch_node"] = key[1]
                        row["key_customer"] = key[2]
                        row["key_recovery_node"] = key[3]
                        row["key_launch_vehicle"] = key[4]
                        row["key_recovery_vehicle"] = key[5]
                    else:
                        row["key_raw"] = str(key)

                    if isinstance(info, dict):

                        for field in ["drone_id", "launch_vehicle", "recovery_vehicle",
                                      "launch_node", "recovery_node", "customer",
                                      "launch_time", "recovery_time", "energy",
                                      "cost", "time", "uav_route_cost", "uav_time_cost"]:
                            row[field] = info.get(field)

                        route = info.get("uav_route")
                        try:
                            row["uav_route"] = json.dumps(make_json_friendly(route), ensure_ascii=False)
                        except Exception:
                            row["uav_route"] = str(route)

                        try:
                            row["uav_route_len"] = len(route) if route is not None else 0
                        except TypeError:
                            row["uav_route_len"] = None
                    else:
                        row["info_raw"] = str(info)

                    final_rows.append(row)

                df_final_uav = pd.DataFrame(final_rows)
                df_final_uav.to_excel(writer, sheet_name="final_uav_plan", index=False)

        print(f"[save_alns_results] best_final_state 已保存到: {best_final_state_xlsx_path}")

    if operator_statistics is not None:
        operator_stats_xlsx_path = f"{prefix}_operator_statistics.xlsx"

        destroy_stats_rows = []
        for op_name, stats in operator_statistics.get('destroy', {}).items():
            destroy_stats_rows.append({
                'operator_name': op_name,
                'total_uses': stats.get('total_uses', 0),
                'total_score': stats.get('total_score', 0),
                'avg_score': stats.get('avg_score', 0),
                'improvements': stats.get('improvements', 0),
                'new_best_count': stats.get('new_best_count', 0),
                'better_current_count': stats.get('better_current_count', 0),
                'total_improvement': stats.get('total_improvement', 0),
                'avg_improvement': stats.get('avg_improvement', 0),
            })
        df_destroy_stats = pd.DataFrame(destroy_stats_rows)

        repair_stats_rows = []
        for op_name, stats in operator_statistics.get('repair', {}).items():
            repair_stats_rows.append({
                'operator_name': op_name,
                'total_uses': stats.get('total_uses', 0),
                'total_score': stats.get('total_score', 0),
                'avg_score': stats.get('avg_score', 0),
                'improvements': stats.get('improvements', 0),
                'new_best_count': stats.get('new_best_count', 0),
                'better_current_count': stats.get('better_current_count', 0),
                'total_improvement': stats.get('total_improvement', 0),
                'avg_improvement': stats.get('avg_improvement', 0),
            })
        df_repair_stats = pd.DataFrame(repair_stats_rows)

        combo_stats_rows = []
        if operator_combinations:
            for combo_key, stats in operator_combinations.items():
                if isinstance(combo_key, tuple) and len(combo_key) >= 2:
                    d_name, r_name = combo_key[0], combo_key[1]
                else:
                    d_name, r_name = str(combo_key), ""
                combo_stats_rows.append({
                    'destroy_operator': d_name,
                    'repair_operator': r_name,
                    'uses': stats.get('uses', 0),
                    'total_score': stats.get('total_score', 0),
                    'avg_score': stats.get('total_score', 0) / stats.get('uses', 1) if stats.get('uses', 0) > 0 else 0,
                    'improvements': stats.get('improvements', 0),
                    'new_best_count': stats.get('new_best_count', 0),
                    'better_current_count': stats.get('better_current_count', 0),
                    'total_improvement': stats.get('total_improvement', 0),
                    'best_improvement': stats.get('best_improvement', 0),
                })
        df_combo_stats = pd.DataFrame(combo_stats_rows)

        best_operators_summary = []

        if best_destroy_by_avg_score:
            if isinstance(best_destroy_by_avg_score, tuple) and len(best_destroy_by_avg_score) >= 2:
                best_operators_summary.append({
                    'category': '最优破坏算子(按平均得分)',
                    'operator_name': best_destroy_by_avg_score[0],
                    'metric_value': best_destroy_by_avg_score[1].get('avg_score', 0) if isinstance(best_destroy_by_avg_score[1], dict) else 0,
                    'total_uses': best_destroy_by_avg_score[1].get('total_uses', 0) if isinstance(best_destroy_by_avg_score[1], dict) else 0,
                    'new_best_count': best_destroy_by_avg_score[1].get('new_best_count', 0) if isinstance(best_destroy_by_avg_score[1], dict) else 0,
                })

        if best_repair_by_avg_score:
            if isinstance(best_repair_by_avg_score, tuple) and len(best_repair_by_avg_score) >= 2:
                best_operators_summary.append({
                    'category': '最优修复算子(按平均得分)',
                    'operator_name': best_repair_by_avg_score[0],
                    'metric_value': best_repair_by_avg_score[1].get('avg_score', 0) if isinstance(best_repair_by_avg_score[1], dict) else 0,
                    'total_uses': best_repair_by_avg_score[1].get('total_uses', 0) if isinstance(best_repair_by_avg_score[1], dict) else 0,
                    'new_best_count': best_repair_by_avg_score[1].get('new_best_count', 0) if isinstance(best_repair_by_avg_score[1], dict) else 0,
                })

        if best_combo_by_avg_score:
            if isinstance(best_combo_by_avg_score, tuple) and len(best_combo_by_avg_score) >= 2:
                combo_stats = operator_combinations.get(best_combo_by_avg_score, {}) if operator_combinations else {}
                best_operators_summary.append({
                    'category': '最优组合(按平均得分)',
                    'operator_name': f"{best_combo_by_avg_score[0]} + {best_combo_by_avg_score[1]}",
                    'metric_value': combo_stats.get('total_score', 0) / max(combo_stats.get('uses', 1), 1),
                    'total_uses': combo_stats.get('uses', 0),
                    'new_best_count': combo_stats.get('new_best_count', 0),
                })

        if best_combo_by_new_best:
            if isinstance(best_combo_by_new_best, tuple) and len(best_combo_by_new_best) >= 2:
                combo_stats = operator_combinations.get(best_combo_by_new_best, {}) if operator_combinations else {}
                best_operators_summary.append({
                    'category': '最优组合(按找到新最优次数)',
                    'operator_name': f"{best_combo_by_new_best[0]} + {best_combo_by_new_best[1]}",
                    'metric_value': combo_stats.get('new_best_count', 0),
                    'total_uses': combo_stats.get('uses', 0),
                    'new_best_count': combo_stats.get('new_best_count', 0),
                })

        if best_operator_combo and isinstance(best_operator_combo, dict) and best_operator_combo.get('destroy'):
            best_operators_summary.append({
                'category': '最大改进组合',
                'operator_name': f"{best_operator_combo.get('destroy', '')} + {best_operator_combo.get('repair', '')}",
                'metric_value': best_operator_combo.get('best_improvement', 0),
                'total_uses': None,
                'new_best_count': None,
                'iteration': best_operator_combo.get('iteration'),
                'objective_before': best_operator_combo.get('objective_before'),
                'objective_after': best_operator_combo.get('objective_after'),
            })

        df_best_summary = pd.DataFrame(best_operators_summary)

        with pd.ExcelWriter(operator_stats_xlsx_path) as writer:
            df_destroy_stats.to_excel(writer, sheet_name='destroy_operators', index=False)
            df_repair_stats.to_excel(writer, sheet_name='repair_operators', index=False)
            if not df_combo_stats.empty:
                df_combo_stats.to_excel(writer, sheet_name='operator_combinations', index=False)
            if not df_best_summary.empty:
                df_best_summary.to_excel(writer, sheet_name='best_operators_summary', index=False)

        print(f"[save_alns_results] 算子统计已保存到: {operator_stats_xlsx_path}")

    if iteration_operator_log is not None and len(iteration_operator_log) > 0:
        iteration_log_xlsx_path = f"{prefix}_iteration_operator_log.xlsx"
        df_iteration_log = pd.DataFrame(iteration_operator_log)
        df_iteration_log.to_excel(iteration_log_xlsx_path, index=False, sheet_name='iteration_log')
        print(f"[save_alns_results] 迭代算子日志已保存到: {iteration_log_xlsx_path}")

    if operator_history is not None and len(operator_history) > 0:
        operator_history_xlsx_path = f"{prefix}_operator_history.xlsx"
        df_operator_history = pd.DataFrame(operator_history)
        df_operator_history.to_excel(operator_history_xlsx_path, index=False, sheet_name='operator_history')
        print(f"[save_alns_results] 算子历史已保存到: {operator_history_xlsx_path}")

    print(f"[save_alns_results] 结果已保存到目录: {case_dir}")

def update_init_vehicle_task_data(new_vehicle_task_data, state):

    customer_plan = state.customer_plan
    vehicle_routes = state.vehicle_routes
    vehicle_id = state.T
    uav_id = state.V
    vehicle_arrive_time = state.rm_empty_vehicle_arrive_time

    for veh_id in vehicle_id:
        vehicle_route = vehicle_routes[veh_id-1]
        for node_id in vehicle_route:
            if node_id in new_vehicle_task_data[veh_id]:
                new_vehicle_task_data[veh_id][node_id].prcise_arrive_time = vehicle_arrive_time[veh_id][node_id]
                new_vehicle_task_data[veh_id][node_id].prcise_departure_time = vehicle_arrive_time[veh_id][node_id]
            else:
                print(f"node_id {node_id} not in new_vehicle_task_data[veh_id]")
                new_vehicle_task_data[veh_id][node_id].prcise_arrive_time = 0
                new_vehicle_task_data[veh_id][node_id].prcise_departure_time = 0
    return new_vehicle_task_data

def update_init_sorted_mission_keys(state):

    vehicle_arrive_time = state.rm_empty_vehicle_arrive_time
    customer_plan = state.customer_plan

    temp_list = []

    for plan in customer_plan.values():
        launch_node = plan[1]
        launch_vehicle = plan[4]

        try:
            arrive_time = vehicle_arrive_time[launch_vehicle][launch_node]
        except KeyError:

            arrive_time = float('inf')

        temp_list.append((arrive_time, tuple(plan)))

    temp_list.sort(key=lambda x: x[0])

    sorted_mission_keys = [item[1] for item in temp_list]

    return sorted_mission_keys

import numpy as np

import os
import json
from typing import Any, Dict, List
import pandas as pd

try:
    import numpy as np
except ImportError:
    np = None

def make_json_friendly(obj: Any) -> Any:

    if isinstance(obj, (str, int, float, bool)) or obj is None:
        return obj

    if np is not None and isinstance(obj, np.generic):

        if "int" in type(obj).__name__.lower():
            return int(obj)
        if "float" in type(obj).__name__.lower():
            return float(obj)
        return obj.item()

    if isinstance(obj, (list, tuple, set)):
        return [make_json_friendly(x) for x in obj]

    if isinstance(obj, dict):
        new_dict = {}
        for k, v in obj.items():
            k_clean = make_json_friendly(k)
            v_clean = make_json_friendly(v)
            new_dict[str(k_clean)] = v_clean
        return new_dict

    return str(obj)

from collections import defaultdict
from typing import Dict, Any

class _ArriveTimeDict(dict):

    __slots__ = ("cust_arrive_h",)
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.cust_arrive_h = None

def extract_arrive_time_from_plan(
    vehicle_plan_time: Dict[Any, Dict[Any, list]]
) -> Dict[int, Dict[int, float]]:

    result: _ArriveTimeDict = _ArriveTimeDict()

    if vehicle_plan_time is None:
        return result

    cust_arr = getattr(vehicle_plan_time, "cust_arrive_h", None)
    if cust_arr:
        result.cust_arrive_h = dict(cust_arr)

    for veh_id, node_dict in vehicle_plan_time.items():

        try:
            veh_key = int(veh_id)
        except (TypeError, ValueError):
            veh_key = veh_id

        result[veh_key] = {}

        for node_id, times in node_dict.items():
            try:
                node_key = int(node_id)
            except (TypeError, ValueError):
                node_key = node_id

            if isinstance(times, (list, tuple)) and len(times) > 0:
                arrive_time = float(times[0])
            else:

                try:
                    arrive_time = float(times)
                except (TypeError, ValueError):

                    arrive_time = None

            result[veh_key][node_key] = arrive_time

    return result
