try:
    from r12_timing import TimedList as _R12TimedList
except Exception:
    _R12TimedList = list
import copy
import numpy as np
import numpy.random as rnd
from collections import defaultdict
import time
from parseCSV import *
from utils_shared import *
from collections import defaultdict
import copy
from initialize import init_agent, initialize_drone_vehicle_assignments
from create_vehicle_route import *

import os

from mfstsp_heuristic_1_partition import *
from mfstsp_heuristic_2_asgn_uavs import *
from mfstsp_heuristic_3_timing import *
from task_data import deep_remove_vehicle_task
from local_search import *
from rm_node_sort_node import rm_empty_node
from task_data import *
import main
import endurance_calculator
import distance_functions
visualize_plan = None
import random
from typing import List, Dict, Tuple
from dataclasses import dataclass

from destroy_repair_operator import *
from initialize import deep_copy_vehicle_task_data
from cost_y import calculate_plan_cost
from create_vehicle_route import DiverseRouteGenerator
from constraint_validator import validate_state_constraints, quick_validate
from constraints_satisfied import is_constraints_satisfied

import cProfile
import pstats
import io

def _fast_update_is_noop(vehicle_task_data, state):

    try:
        arrive = state.rm_empty_vehicle_arrive_time
        routes = state.vehicle_routes
        for veh_id in state.T:
            if veh_id not in vehicle_task_data:
                return False
            inner = vehicle_task_data[veh_id]
            at = arrive[veh_id]
            for node_id in routes[veh_id - 1]:
                if node_id not in inner or node_id not in at:
                    return False
        return True
    except Exception:
        return False

class FastMfstspState:

    def __init__(self, vehicle_routes, uav_assignments, customer_plan,
                 vehicle_task_data, global_reservation_table, total_cost=None, init_uav_plan=None, uav_cost=None,
                 init_vehicle_plan_time=None,
                 node=None, DEPOT_nodeID=None, V=None, T=None, vehicle=None, uav_travel=None, veh_distance=None,
                 veh_travel=None, N=None, N_zero=None, N_plus=None, A_total=None, A_cvtp=None, A_vtp=None,
                 A_aerial_relay_node=None, G_air=None, G_ground=None, air_matrix=None, ground_matrix=None,
                 air_node_types=None, ground_node_types=None, A_c=None, xeee=None, customer_time_windows_h=None, early_arrival_cost=None, late_arrival_cost=None):

        self.vehicle_routes = vehicle_routes
        self.uav_assignments = uav_assignments
        self.customer_plan = customer_plan
        self.vehicle_task_data = vehicle_task_data
        self.global_reservation_table = global_reservation_table
        self._total_cost = total_cost
        self.uav_plan = init_uav_plan
        self.uav_cost = uav_cost
        self.vehicle_plan_time = init_vehicle_plan_time
        self.node = node
        self.DEPOT_nodeID = DEPOT_nodeID
        self.vehicle = vehicle
        self.uav_travel = uav_travel
        self.veh_distance = veh_distance
        self.veh_travel = veh_travel
        self.N = N
        self.V = V
        self.T = T
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
        self.customer_time_windows_h = customer_time_windows_h
        self.early_arrival_cost = early_arrival_cost
        self.late_arrival_cost = late_arrival_cost
        self.rm_empty_vehicle_task_data = deep_copy_vehicle_task_data(self.vehicle_task_data)

        self._modification_history = []

    def calculate_rm_empty_vehicle_arrive_time(self, vehicle_route):

        rm_empty_vehicle_arrive_time = {}

        veh_travel = self.veh_travel
        for vehicle_id, route in enumerate(vehicle_route):
            vehicle_id = vehicle_id + 1
            arrive_time_dict = {}
            if len(route) > 0:
                prev = route[0]
                t = 0
                arrive_time_dict[prev] = 0
                if len(route) > 1:
                    vt = veh_travel[vehicle_id]
                    for node_j in route[1:]:
                        t = t + vt[prev][node_j]
                        arrive_time_dict[node_j] = t
                        prev = node_j
            rm_empty_vehicle_arrive_time[vehicle_id] = arrive_time_dict
        return rm_empty_vehicle_arrive_time

    def fast_arrive_time_memo(self, vehicle_route):

        key = tuple(tuple(r) for r in vehicle_route)
        memo = self.__dict__.get('_fast_arrive_memo')
        if memo is not None and memo[0] == key:
            return memo[1]
        res = self.calculate_rm_empty_vehicle_arrive_time(vehicle_route)
        self.__dict__['_fast_arrive_memo'] = (key, res)
        return res

    def re_update_time(self, vehicle_route, vehicle_arrive_time, vehicle_task_data, state):

        if _fast_update_is_noop(vehicle_task_data, state):
            new_vehicle_task_data = vehicle_task_data
        else:
            new_vehicle_task_data = deep_copy_vehicle_task_data(vehicle_task_data)

            new_vehicle_task_data = update_init_vehicle_task_data(new_vehicle_task_data, state)
        re_uav_assignment = state.uav_assignments
        re_uav_plan = {k: v for k, v in state.uav_plan.items()}
        sorted_mission_keys = update_re_uav_plan(state, re_uav_plan)
        re_time_uav_task_dict, re_time_customer_plan, re_time_uav_plan, re_vehicle_plan_time, re_vehicle_task_data = low_update_time(re_uav_assignment,
        sorted_mission_keys, vehicle_route, new_vehicle_task_data, vehicle_arrive_time,
        self.node, self.V, self.T, self.vehicle, self.uav_travel)

        final_uav_plan, final_uav_cost, final_vehicle_plan_time, final_vehicle_task_data, final_global_reservation_table = rolling_time_cbs(vehicle_arrive_time,
        vehicle_route, re_time_uav_task_dict, re_time_customer_plan, re_time_uav_plan,
        re_vehicle_plan_time, re_vehicle_task_data, self.node, self.DEPOT_nodeID, self.V, self.T, self.vehicle,
        self.uav_travel, self.veh_distance, self.veh_travel, self.N, self.N_zero, self.N_plus, self.A_total, self.A_cvtp,
        self.A_vtp, self.A_aerial_relay_node, self.G_air, self.G_ground, self.air_matrix, self.ground_matrix,
        self.air_node_types, self.ground_node_types, self.A_c, self.xeee)
        final_total_cost = calculate_plan_cost(final_uav_cost, vehicle_route, self.vehicle, self.T, self.V, self.veh_distance)
        return final_uav_plan, final_uav_cost, final_vehicle_plan_time, final_vehicle_task_data, final_global_reservation_table

    def check_all_vehicle_finish_task_time(self, vehicle_plan_time):

        total_task_time = 0
        for vehicle_id, node_times in vehicle_plan_time.items():
            if node_times:

                latest_finish_time_for_vehicle = max(times[1] for times in node_times.values())
            total_task_time += latest_finish_time_for_vehicle

        return total_task_time

    def objective(self):

        total_cost = calculate_plan_cost(self.uav_cost, self.vehicle_routes, self.vehicle, self.T, self.V, self.veh_distance)
        return total_cost

    def win_total_objective(self):

        vehicle_arrive_time = self.calculate_rm_empty_vehicle_arrive_time(self.vehicle_routes)
        route_total_cost = calculate_plan_cost(self.uav_cost, self.vehicle_routes, self.vehicle, self.T, self.V, self.veh_distance)
        win_total_cost, win_penalty_cost, total_dict_cost = calculate_window_cost(self.customer_plan,
                          self.uav_cost,
                          vehicle_arrive_time,
                          self.vehicle,
                          self.customer_time_windows_h,
                          self.early_arrival_cost,
                          self.late_arrival_cost,
                          self.uav_travel,
                          self.node)
        total_win_penalty_cost = float(sum(win_penalty_cost.values()))
        total_cost = route_total_cost + total_win_penalty_cost
        return total_cost

    def validate_constraints(self, verbose=True):

        return validate_state_constraints(self, verbose)

    def is_constraints_satisfied(self):

        return quick_validate(self)

    def update_rm_empty_task(self):
        rm_empty_vehicle_route, empty_nodes_by_vehicle = rm_empty_node(self.customer_plan, self.vehicle_routes)
        self.rm_empty_vehicle_route = rm_empty_vehicle_route
        self.empty_nodes_by_vehicle = empty_nodes_by_vehicle

        return self.rm_empty_vehicle_route, self.empty_nodes_by_vehicle

    def update_calculate_plan_cost(self, uav_cost, empty_vehicle_route):
        empty_node_cost = calculate_plan_cost(uav_cost, empty_vehicle_route, self.vehicle, self.T, self.V, self.veh_distance)
        return empty_node_cost

    def sorted_import_node(self, destroyed_plan, current_plan):

        vehicle_node_levels = {}

        for vehicle_id, route in self.vehicle_routes.items():
            if vehicle_id not in vehicle_node_levels:
                vehicle_node_levels[vehicle_id] = {
                    'level1': [],
                    'level2': [],
                    'level3': []
                }

            current_launch_nodes = set()
            current_recovery_nodes = set()
            for customer, assignment in current_plan.items():
                drone_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle = assignment
                if launch_vehicle == vehicle_id:
                    current_launch_nodes.add(launch_node)
                if recovery_vehicle == vehicle_id:
                    current_recovery_nodes.add(recovery_node)

            destroyed_launch_nodes = set()
            destroyed_recovery_nodes = set()
            for customer, assignment in destroyed_plan.items():
                drone_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle = assignment
                if launch_vehicle == vehicle_id:
                    destroyed_launch_nodes.add(launch_node)
                if recovery_vehicle == vehicle_id:
                    destroyed_recovery_nodes.add(recovery_node)

            for node in route:
                if node in current_launch_nodes or node in current_recovery_nodes:

                    vehicle_node_levels[vehicle_id]['level1'].append(node)
                elif node in destroyed_launch_nodes or node in destroyed_recovery_nodes:

                    vehicle_node_levels[vehicle_id]['level2'].append(node)
                else:

                    vehicle_node_levels[vehicle_id]['level3'].append(node)

        return vehicle_node_levels

    def fast_copy(self):

        cls = self.__class__

        new_state = cls.__new__(cls)

        new_state.__dict__ = self.__dict__.copy()

        new_state.vehicle_routes = [r[:] for r in self.vehicle_routes]
        new_state.rm_empty_vehicle_route = [r[:] for r in new_state.vehicle_routes]

        new_state.uav_assignments = {k: v[:] for k, v in self.uav_assignments.items()}

        new_state.customer_time_windows_h = self.customer_time_windows_h
        new_state.early_arrival_cost = self.early_arrival_cost
        new_state.late_arrival_cost = self.late_arrival_cost

        if isinstance(self.customer_plan, defaultdict):
            new_state.customer_plan = self.customer_plan.copy()
            for k, v in new_state.customer_plan.items():
                new_state.customer_plan[k] = list(v) if isinstance(v, (list, tuple)) else v
        else:

            new_state.customer_plan = {k: (v[:] if isinstance(v, list) else v)
                                    for k, v in self.customer_plan.items()}

        new_state.vehicle_task_data = self._fast_copy_nested_task_data(self.vehicle_task_data)

        new_state.global_reservation_table = {
            k: [item.copy() for item in v]
            for k, v in self.global_reservation_table.items()
        }

        new_state._modification_history = []

        if getattr(self, 'destroyed_customers_info', None):
            new_state.destroyed_customers_info = {k: v[:] for k, v in self.destroyed_customers_info.items()}
        else:
            new_state.destroyed_customers_info = {}

        if getattr(self, 'empty_nodes_by_vehicle', None):
            new_state.empty_nodes_by_vehicle = {k: v[:] for k, v in self.empty_nodes_by_vehicle.items()}

        new_state.uav_cost = {k: v for k, v in self.uav_cost.items()}
        if hasattr(self, 'vehicle_plan_time'):
            new_state.vehicle_plan_time = self.vehicle_plan_time.copy()
            for v_id, inner_dict in self.vehicle_plan_time.items():

                new_inner = inner_dict.copy()

                for node_id, time_list in inner_dict.items():
                    new_inner[node_id] = time_list[:]
                new_state.vehicle_plan_time[v_id] = new_inner
        if hasattr(self, 'vehicle_arrive_time'):
            new_state.vehicle_arrive_time = {n_id: t for n_id, t in self.vehicle_arrive_time.items()}
        if hasattr(self, 'rm_vehicle_arrive_time'):
            new_state.rm_vehicle_arrive_time = {n_id: t for n_id, t in self.rm_vehicle_arrive_time.items()}
        return new_state

    def temp_fast_copy(self, vehicles_to_copy=None):

        from collections import defaultdict

        cls = self.__class__

        new_state = cls.__new__(cls)

        new_state.__dict__ = self.__dict__.copy()

        new_state.vehicle_routes = [route[:] for route in self.vehicle_routes]

        new_state.uav_assignments = {
            k: v[:] for k, v in self.uav_assignments.items()
        }

        if isinstance(self.customer_plan, defaultdict):
            base_plan = dict(self.customer_plan)
        else:
            base_plan = self.customer_plan

        new_state.customer_plan = {
            cust: (plan[:] if isinstance(plan, list) else plan)
            for cust, plan in base_plan.items()
        }

        new_state.vehicle_task_data = self._fast_copy_nested_task_data(
            self.vehicle_task_data, vehicles_to_copy=vehicles_to_copy
        )

        if isinstance(self.uav_cost, dict):
            new_state.uav_cost = self.uav_cost.copy()
        else:
            new_state.uav_cost = self.uav_cost

        new_state._modification_history = []

        return new_state

    def _fast_copy_nested_task_data(self, original_data, vehicles_to_copy=None):

        new_data = original_data.copy()

        if vehicles_to_copy is None:
            for v_id, inner_dict in original_data.items():
                new_inner = inner_dict.copy()
                for t_id, task_obj in inner_dict.items():
                    new_inner[t_id] = task_obj.fast_copy()
                new_data[v_id] = new_inner
            return new_data

        vehicles_to_copy = set(vehicles_to_copy)

        for v_id in vehicles_to_copy:
            inner_dict = original_data.get(v_id)
            if inner_dict is None:
                continue

            new_inner = inner_dict.copy()
            for t_id, task_obj in inner_dict.items():
                new_inner[t_id] = task_obj.fast_copy()
            new_data[v_id] = new_inner

        return new_data

    def record_modification(self, operation, data):

        self._modification_history.append((operation, data))

    def rollback_last_modification(self):

        if self._modification_history:
            operation, data = self._modification_history.pop()

            if operation == "remove_customer":
                customer, assignment = data
                self.customer_plan[customer] = assignment
                uav_id, _, _, _, _ = assignment
                if uav_id in self.uav_assignments:
                    self.uav_assignments[uav_id].append(assignment)
            elif operation == "modify_route":
                vehicle_id, old_route = data
                self.vehicle_routes[vehicle_id] = old_route

            self._total_cost = None

class IncrementalALNS:

    def __init__(self, node, DEPOT_nodeID, V, T, vehicle, uav_travel, veh_distance, veh_travel, N,
    N_zero, N_plus, A_total, A_cvtp, A_vtp,
		A_aerial_relay_node, G_air, G_ground,air_matrix, ground_matrix, air_node_types,
        ground_node_types, A_c, xeee, customer_time_windows_h, early_arrival_cost, late_arrival_cost, problemName,
        iter, max_iterations, summary_dir=None, max_runtime=60, seed=None, algo_seed=None):
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
        self.customer_time_windows_h = customer_time_windows_h
        self.early_arrival_cost = early_arrival_cost
        self.late_arrival_cost = late_arrival_cost
        self.iter = iter
        self.summary_dir = summary_dir
        self.problemName = problemName

        self.max_iterations = max_iterations

        self.initial_temperature = max_iterations

        self.max_runtime = max_runtime

        if algo_seed is None:
            algo_seed = 42
        self.rng = rnd.default_rng(algo_seed)
        random.seed(algo_seed)
        np.random.seed(algo_seed)
        self.vtp_coords = np.array([self.node[i].position for i in self.A_vtp])
        self.num_clusters = min(len(self.T), len(self.A_vtp))
        self.dis_k = 25
        self.base_drone_assignment = self.base_drone_assigment()

        self.customer_destroy_ratio = (0.2, 0.4)

        self.vtp_destroy_quantity = {'random': (1, 2), 'worst': 1, 'shaw': 2}
        self.cluster_vtp_dict, self.map_cluster_vtp_dict = self.cluster_vtp_for_customers(k=self.dis_k)

        self.destroy_operators = [self.destroy_random_removal, self.destroy_worst_removal, self.destroy_comprehensive_removal,self.destroy_shaw_rebalance_removal]

        self.repair_operators = [self.repair_random_greedy_insertion, self.repair_drone_first_truck_second_insertion, self.repair_regret_insertion, self.repair_equal_priority_insertion, self.repair_kNN_regret]

        self.params = {
            'k_neighbors': 3,
            'W_base_synergy': 560.0,
            'W_base_partner': 45.0,
            'alpha': 1.5,
            'k_traditional_pairs':5,
            'k_expansion_partners':5,
            'k_synergy_check_partners':3
        }
        self.M_PENALTY = 1000

        self.reward_scores = {
            'new_best': 10,
            'better_than_current': 5,
            'accepted': 2
        }
        self.reaction_factor = 0.5
        self.strategy_weights = {
            'structural': 1.0,
            'internal': 1.0
        }

        self.operator_weights = {
            'structural': {
                'destroy': {op.__name__: 1.0 for op in self.destroy_operators},
                'repair':  {op.__name__: 1.0 for op in self.repair_operators}
            },
            'internal': {
                'destroy': {op.__name__: 1.0 for op in self.destroy_operators},
                'repair':  {op.__name__: 1.0 for op in self.repair_operators}
            }
        }
        from improved_operators import patch_incremental_alns
        patch_incremental_alns(self)

    def base_drone_assigment(self):

        vehicle_ids = self.T
        drone_ids = self.V

        num_vehicles = len(vehicle_ids)
        num_drones = len(drone_ids)

        base_assignment = {}

        drones_per_vehicle = num_drones // num_vehicles

        remainder = num_drones % num_vehicles

        current_drone_idx = 0

        for i, v_id in enumerate(vehicle_ids):

            count = drones_per_vehicle + 1 if i < remainder else drones_per_vehicle

            assigned_drones = drone_ids[current_drone_idx : current_drone_idx + count]

            base_assignment[v_id] = assigned_drones

            current_drone_idx += count

        return base_assignment

    def repair_random_greedy_insertion(self, state, strategic_bonus=0, num_destroyed=1, force_vtp_mode=True):

        repaired_state = state.fast_copy()
        repaired_state.repair_objective = 0
        destroy_node = list(state.destroyed_customers_info.keys())
        insert_plan = []

        force_vtp_mode = True
        if force_vtp_mode:
            num_repaired = 0

            while len(destroy_node) > 0:

                target_idx = self.rng.integers(0, len(destroy_node))
                customer = destroy_node.pop(target_idx)

                try:
                    current_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(repaired_state.vehicle_routes)
                    base_total_cost, uav_tw_violation_cost, total_cost_dict = calculate_window_cost(
                        repaired_state.customer_plan, repaired_state.uav_cost, current_arrive_time,
                        self.vehicle, self.customer_time_windows_h,
                        self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node
                    )
                except Exception:
                    base_total_cost = float('inf')

                vehicle_route = repaired_state.vehicle_routes
                vehicle_task_data = repaired_state.vehicle_task_data
                vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(repaired_state.vehicle_routes)

                customer_candidates = []

                traditional_result, is_heuristic_swap = self._evaluate_traditional_insertion(
                    customer, vehicle_route, vehicle_task_data, vehicle_arrive_time,
                    base_total_cost, uav_tw_violation_cost, total_cost_dict, repaired_state
                )

                if traditional_result[0] or traditional_result[1] is not None:
                    traditional_cost, traditional_scheme = traditional_result
                    if is_heuristic_swap:
                        current_total_cost = traditional_scheme['total_cost']
                        deta_total_cost = traditional_scheme['win_cost']
                        customer_candidates.append({
                            'customer': customer, 'scheme': traditional_scheme,
                            'cost': traditional_cost, 'win_cost': deta_total_cost,
                            'total_cost': current_total_cost, 'type': 'heuristic_swap', 'vtp_node': None
                        })
                    else:
                        current_total_cost = traditional_cost + base_total_cost
                        customer_candidates.append({
                            'customer': customer, 'scheme': traditional_scheme,
                            'cost': traditional_cost, 'win_cost': traditional_cost,
                            'total_cost': current_total_cost, 'type': 'traditional', 'vtp_node': None
                        })

                vtp_result, vtp_infor = self._evaluate_vtp_expansion_insertion(
                    customer, vehicle_route, vehicle_task_data, vehicle_arrive_time, repaired_state
                )
                if vtp_infor is not None:
                    vtp_node, vtp_insert_vehicle_id, vtp_insert_index = vtp_infor
                    if vtp_result is not None:
                        vtp_cost, vtp_scheme = vtp_result
                        customer_candidates.append({
                            'customer': customer, 'scheme': vtp_scheme,
                            'cost': vtp_cost, 'total_cost': vtp_cost,
                            'type': 'vtp_expansion', 'vtp_node': vtp_node,
                            'vtp_insert_vehicle_id': vtp_insert_vehicle_id,
                            'vtp_insert_index': vtp_insert_index,
                            'original_cost': vtp_cost
                        })

                customer_candidates = [item for item in customer_candidates if item['scheme'] is not None]
                candidates_plan = sorted(customer_candidates, key=lambda x: x['total_cost'])

                success = False

                for candidate in candidates_plan:

                    if candidate['type'] == 'traditional':
                        best_scheme = candidate['scheme']
                        best_cost = self.drone_insert_cost(best_scheme[0], best_scheme[2], best_scheme[1], best_scheme[3])
                        drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id = best_scheme

                        temp_customer_plan = {k: v for k, v in repaired_state.customer_plan.items()}
                        temp_customer_plan[customer_node] = best_scheme
                        temp_rm_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(vehicle_route)

                        if not is_time_feasible(temp_customer_plan, temp_rm_vehicle_arrive_time):
                            continue

                        repaired_state.customer_plan[customer_node] = best_scheme
                        if drone_id not in repaired_state.uav_assignments:
                            repaired_state.uav_assignments[drone_id] = []
                        repaired_state.uav_assignments[drone_id].append(best_scheme)

                        if repaired_state.uav_cost is None: repaired_state.uav_cost = {}
                        repaired_state.uav_cost[customer_node] = best_cost

                        vehicle_task_data = update_vehicle_task(vehicle_task_data, best_scheme, vehicle_route)
                        insert_plan.append((customer, best_scheme, best_cost, 'traditional'))
                        success = True
                        break

                    elif candidate['type'] == 'heuristic_swap':
                        orig_scheme = candidate['scheme']['orig_scheme']
                        new_scheme = candidate['scheme']['new_scheme']
                        orig_cost = candidate['scheme']['orig_cost']
                        new_cost = candidate['scheme']['new_cost']

                        orig_drone_id, orig_launch_node, orig_customer, orig_recovery_node, orig_launch_vehicle, orig_recovery_vehicle = orig_scheme
                        new_drone_id, new_launch_node, new_customer, new_recovery_node, new_launch_vehicle, new_recovery_vehicle = new_scheme
                        delete_customer = orig_customer

                        temp_customer_plan = {k: v for k, v in repaired_state.customer_plan.items()}
                        delete_task_plan = temp_customer_plan[orig_customer]
                        del temp_customer_plan[orig_customer]
                        temp_customer_plan[orig_customer] = orig_scheme
                        temp_customer_plan[new_customer] = new_scheme
                        temp_rm_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(vehicle_route)

                        if not is_time_feasible(temp_customer_plan, temp_rm_vehicle_arrive_time):
                            continue
                        else:
                            del repaired_state.customer_plan[delete_customer]
                            repaired_state.customer_plan[orig_customer] = orig_scheme
                            repaired_state.customer_plan[new_customer] = new_scheme

                            if orig_drone_id not in repaired_state.uav_assignments: repaired_state.uav_assignments[orig_drone_id] = []
                            repaired_state.uav_assignments[orig_drone_id].append(orig_scheme)
                            if new_drone_id not in repaired_state.uav_assignments: repaired_state.uav_assignments[new_drone_id] = []
                            repaired_state.uav_assignments[new_drone_id].append(new_scheme)

                            del repaired_state.uav_cost[delete_customer]
                            repaired_state.uav_cost[orig_customer] = orig_cost
                            repaired_state.uav_cost[new_customer] = new_cost

                            vehicle_task_data = remove_vehicle_task(vehicle_task_data, delete_task_plan, vehicle_route)
                            orig_launch_time = temp_rm_vehicle_arrive_time[orig_launch_vehicle][orig_launch_node]
                            new_launch_time = temp_rm_vehicle_arrive_time[new_launch_vehicle][new_launch_node]
                            if orig_launch_time <= new_launch_time:
                                vehicle_task_data = update_vehicle_task(vehicle_task_data, orig_scheme, vehicle_route)
                                vehicle_task_data = update_vehicle_task(vehicle_task_data, new_scheme, vehicle_route)
                            else:
                                vehicle_task_data = update_vehicle_task(vehicle_task_data, new_scheme, vehicle_route)
                                vehicle_task_data = update_vehicle_task(vehicle_task_data, orig_scheme, vehicle_route)

                            insert_plan.append((delete_customer, orig_scheme, orig_cost, 'heuristic_swap'))
                            insert_plan.append((customer, new_scheme, new_cost, 'heuristic_swap'))
                            success = True
                            break

                    elif candidate['type'] == 'vtp_expansion':
                        vtp_node = candidate['vtp_node']
                        vtp_insert_index = candidate['vtp_insert_index']
                        vtp_insert_vehicle_id = candidate['vtp_insert_vehicle_id']
                        best_scheme = candidate['scheme']

                        drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id = best_scheme
                        original_cost = self.drone_insert_cost(drone_id, customer_node, launch_node, recovery_node)

                        temp_customer_plan = {k: v for k, v in repaired_state.customer_plan.items()}
                        temp_customer_plan[customer_node] = best_scheme
                        temp_vehicle_route = [route[:] for route in vehicle_route]
                        temp_route = temp_vehicle_route[vtp_insert_vehicle_id - 1]
                        temp_route.insert(vtp_insert_index, vtp_node)
                        temp_vehicle_route[vtp_insert_vehicle_id - 1] = temp_route
                        repaired_state.temp_vehicle_routes = temp_vehicle_route
                        temp_rm_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(temp_vehicle_route)

                        if not is_time_feasible(temp_customer_plan, temp_rm_vehicle_arrive_time):
                            continue
                        else:
                            route = vehicle_route[vtp_insert_vehicle_id - 1]
                            route.insert(vtp_insert_index, vtp_node)
                            last_customer_node = route[vtp_insert_index - 1]

                            if vtp_insert_index == 1 or last_customer_node == self.DEPOT_nodeID:
                                last_drone_list = self.base_drone_assignment[vtp_insert_vehicle_id][:]
                            else:
                                last_drone_list = vehicle_task_data[vtp_insert_vehicle_id][last_customer_node].drone_list[:]

                            vehicle_task_data[vtp_insert_vehicle_id][vtp_node].drone_list = last_drone_list
                            vehicle_task_data[vtp_insert_vehicle_id][vtp_node].launch_drone_list = []
                            vehicle_task_data[vtp_insert_vehicle_id][vtp_node].recovery_drone_list = []

                            vehicle_task_data = update_vehicle_task(vehicle_task_data, best_scheme, vehicle_route)

                            repaired_state.customer_plan[customer_node] = best_scheme
                            if drone_id not in repaired_state.uav_assignments:
                                repaired_state.uav_assignments[drone_id] = []
                            repaired_state.uav_assignments[drone_id].append(best_scheme)

                            if repaired_state.uav_cost is None: repaired_state.uav_cost = {}
                            repaired_state.uav_cost[customer_node] = original_cost

                            repaired_state.rm_empty_vehicle_route = [route[:] for route in repaired_state.vehicle_routes]
                            repaired_state.rm_empty_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(repaired_state.rm_empty_vehicle_route)
                            insert_plan.append((customer, best_scheme, original_cost, 'vtp_expansion'))
                            success = True
                            break

                if not success:
                    print(f"客户点 {customer} 的所有候选方案都不满足约束，修复失败")
                    repaired_state.repair_objective = float('inf')
                    repaired_state.destroyed_customers_info = {}
                    return repaired_state, insert_plan

                num_repaired += 1

        repaired_state._total_cost = repaired_state.win_total_objective()
        repaired_state.destroyed_customers_info = {}

        return repaired_state, insert_plan

    def repair_greedy_insertion(self, state, strategic_bonus, num_destroyed, force_vtp_mode):

        repaired_state = state.fast_copy()
        repaired_state.repair_objective = 0
        destroy_node = list(state.destroyed_customers_info.keys())
        insert_plan = []

        force_vtp_mode = True
        if force_vtp_mode:
            num_repaired = 0
            while len(destroy_node) > 0:

                try:
                    current_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(repaired_state.vehicle_routes)
                    base_total_cost, uav_tw_violation_cost, total_cost_dict = calculate_window_cost(
                        repaired_state.customer_plan, repaired_state.uav_cost, current_arrive_time,
                        self.vehicle, self.customer_time_windows_h,
                        self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node
                    )
                except Exception:
                    base_total_cost = float('inf')

                best_option_overall = None
                best_customer_to_insert = None
                min_overall_eval_cost = float('inf')

                tactical_multiplier = (num_destroyed - num_repaired) / num_destroyed
                final_bonus = strategic_bonus * tactical_multiplier * 0.3
                final_bonus = 0

                vehicle_route = repaired_state.vehicle_routes
                vehicle_task_data = repaired_state.vehicle_task_data

                vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(repaired_state.vehicle_routes)

                all_candidates = []
                customer_candidates = []

                for customer in destroy_node:

                    traditional_result,is_heuristic_swap = self._evaluate_traditional_insertion(customer, vehicle_route, vehicle_task_data, vehicle_arrive_time,
                    base_total_cost, uav_tw_violation_cost, total_cost_dict, repaired_state)

                    if traditional_result[0] or traditional_result[1] is not None:
                        traditional_cost, traditional_scheme = traditional_result
                        if is_heuristic_swap:

                            current_total_cost = traditional_scheme['total_cost']
                            deta_total_cost = traditional_scheme['win_cost']
                            customer_candidates.append({
                                'customer': customer,
                                'scheme': traditional_scheme,
                                'cost': traditional_cost,
                                'win_cost': deta_total_cost,
                                'total_cost': current_total_cost,
                                'type': 'heuristic_swap',
                                'vtp_node': None
                            })
                        else:

                            current_total_cost = traditional_cost + base_total_cost
                            customer_candidates.append({
                                'customer': customer,
                                'scheme': traditional_scheme,
                                'cost': traditional_cost,
                                'win_cost': traditional_cost,
                                'total_cost': current_total_cost,
                                'type': 'traditional',
                                'vtp_node': None
                            })
                    else:

                        customer_candidates.append({
                            'customer': customer,
                            'scheme': None,
                            'cost': float('inf'),
                            'type': 'traditional',
                            'vtp_node': None
                        })

                    vtp_result,vtp_infor = self._evaluate_vtp_expansion_insertion(customer, vehicle_route, vehicle_task_data, vehicle_arrive_time, repaired_state)

                    if vtp_infor is not None:
                        vtp_node = vtp_infor[0]
                        vtp_insert_vehicle_id = vtp_infor[1]
                        vtp_insert_index = vtp_infor[2]
                        if vtp_result is not None:
                            vtp_cost, vtp_scheme = vtp_result

                            adjusted_cost = vtp_cost - final_bonus

                            customer_candidates.append({
                                'customer': customer,
                                'scheme': vtp_scheme,
                                'cost': adjusted_cost,
                                'total_cost': vtp_cost,
                                'type': 'vtp_expansion',
                                'vtp_node': vtp_node,
                                'vtp_insert_vehicle_id': vtp_insert_vehicle_id,
                                'vtp_insert_index': vtp_insert_index,
                                'original_cost': vtp_cost
                            })
                customer_candidates = [item for item in customer_candidates if item['scheme'] is not None]

                candidates_plan = sorted(customer_candidates, key=lambda x: x['total_cost'])

                success = False

                for candidate in candidates_plan:
                    customer = candidate['customer']

                    if candidate['type'] == 'traditional':

                        customer = candidate['customer']
                        best_scheme = candidate['scheme']
                        best_cost = self.drone_insert_cost(best_scheme[0], best_scheme[2], best_scheme[1], best_scheme[3])

                        drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id = best_scheme

                        temp_customer_plan = {k: v for k, v in repaired_state.customer_plan.items()}
                        temp_customer_plan[customer_node] = best_scheme
                        temp_rm_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(vehicle_route)

                        if not is_time_feasible(temp_customer_plan, temp_rm_vehicle_arrive_time):
                            print(f"传统方案时间约束不满足，尝试下一个候选方案")
                            continue

                        repaired_state.customer_plan[customer_node] = best_scheme

                        if drone_id not in repaired_state.uav_assignments:
                            repaired_state.uav_assignments[drone_id] = []
                        repaired_state.uav_assignments[drone_id].append(best_scheme)

                        if repaired_state.uav_cost is None:
                            repaired_state.uav_cost = {}
                        repaired_state.uav_cost[customer_node] = best_cost

                        vehicle_task_data = update_vehicle_task(
                            vehicle_task_data, best_scheme, vehicle_route
                        )

                        insert_plan.append((customer, best_scheme, best_cost, 'traditional'))

                        success = True
                        break

                    elif candidate['type'] == 'heuristic_swap':
                        best_cost = candidate['cost']
                        print(f"尝试使用启发式交换方案插入客户点 {customer}，成本: {best_cost:.2f}")

                        orig_scheme = candidate['scheme']['orig_scheme']
                        new_scheme = candidate['scheme']['new_scheme']
                        orig_cost = candidate['scheme']['orig_cost']
                        new_cost = candidate['scheme']['new_cost']
                        orig_plan = candidate['scheme']['orig_plan']
                        new_plan = candidate['scheme']['new_plan']

                        orig_drone_id, orig_launch_node, orig_customer, orig_recovery_node, orig_launch_vehicle, orig_recovery_vehicle = orig_scheme
                        new_drone_id, new_launch_node, new_customer, new_recovery_node, new_launch_vehicle, new_recovery_vehicle = new_scheme
                        customer = new_customer
                        delete_customer = orig_customer

                        temp_customer_plan = {k: v for k, v in repaired_state.customer_plan.items()}
                        delete_task_plan = temp_customer_plan[orig_customer]
                        del temp_customer_plan[orig_customer]
                        temp_customer_plan[orig_customer] = orig_scheme
                        temp_customer_plan[new_customer] = new_scheme
                        temp_rm_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(vehicle_route)
                        if not is_time_feasible(temp_customer_plan, temp_rm_vehicle_arrive_time):
                            print(f"启发式交换方案时间约束不满足，尝试下一个候选方案")
                            continue
                        else:

                            del repaired_state.customer_plan[delete_customer]
                            repaired_state.customer_plan[orig_customer] = orig_scheme
                            repaired_state.customer_plan[new_customer] = new_scheme

                            if orig_drone_id not in repaired_state.uav_assignments:
                                repaired_state.uav_assignments[orig_drone_id] = []
                            repaired_state.uav_assignments[orig_drone_id].append(orig_scheme)
                            if new_drone_id not in repaired_state.uav_assignments:
                                repaired_state.uav_assignments[new_drone_id] = []
                            repaired_state.uav_assignments[new_drone_id].append(new_scheme)

                            del repaired_state.uav_cost[delete_customer]
                            repaired_state.uav_cost[orig_customer] = orig_cost
                            repaired_state.uav_cost[new_customer] = new_cost

                            vehicle_task_data = remove_vehicle_task(vehicle_task_data, delete_task_plan, vehicle_route)
                            orig_launch_time = temp_rm_vehicle_arrive_time[orig_launch_vehicle][orig_launch_node]
                            new_launch_time = temp_rm_vehicle_arrive_time[new_launch_vehicle][new_launch_node]
                            if orig_launch_time <= new_launch_time:
                                vehicle_task_data = update_vehicle_task(vehicle_task_data, orig_scheme, vehicle_route)
                                vehicle_task_data = update_vehicle_task(vehicle_task_data, new_scheme, vehicle_route)
                            else:
                                vehicle_task_data = update_vehicle_task(vehicle_task_data, new_scheme, vehicle_route)
                                vehicle_task_data = update_vehicle_task(vehicle_task_data, orig_scheme, vehicle_route)

                            insert_plan.append((delete_customer, orig_scheme, orig_cost, 'heuristic_swap'))
                            insert_plan.append((customer, new_scheme, new_cost, 'heuristic_swap'))
                            success = True
                            break

                    elif candidate['type'] == 'vtp_expansion':

                        customer = candidate['customer']
                        vtp_node = candidate['vtp_node']
                        vtp_insert_index = candidate['vtp_insert_index']
                        vtp_insert_vehicle_id = candidate['vtp_insert_vehicle_id']
                        best_scheme = candidate['scheme']

                        drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id = best_scheme
                        original_cost = self.drone_insert_cost(drone_id, customer_node, launch_node, recovery_node)

                        temp_customer_plan = {k: v for k, v in repaired_state.customer_plan.items()}
                        temp_customer_plan[customer_node] = best_scheme

                        temp_vehicle_route = [route[:] for route in vehicle_route]
                        temp_route = temp_vehicle_route[vtp_insert_vehicle_id - 1]
                        temp_route.insert(vtp_insert_index, vtp_node)
                        temp_vehicle_route[vtp_insert_vehicle_id - 1] = temp_route
                        repaired_state.temp_vehicle_routes = temp_vehicle_route

                        temp_rm_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(temp_vehicle_route)

                        time_feasible = is_time_feasible(temp_customer_plan, temp_rm_vehicle_arrive_time)

                        if not time_feasible:

                            print(f"VTP扩展方案时间约束不满足，尝试下一个候选方案")
                            continue
                        else:

                            route = vehicle_route[vtp_insert_vehicle_id - 1]

                            route.insert(vtp_insert_index, vtp_node)

                            last_customer_node = route[vtp_insert_index - 1]

                            if vtp_insert_index == 1 or last_customer_node == self.DEPOT_nodeID:
                                last_drone_list = self.base_drone_assignment[vtp_insert_vehicle_id][:]
                            else:
                                last_drone_list = vehicle_task_data[vtp_insert_vehicle_id][last_customer_node].drone_list[:]

                            vehicle_task_data[vtp_insert_vehicle_id][vtp_node].drone_list = last_drone_list
                            vehicle_task_data[vtp_insert_vehicle_id][vtp_node].launch_drone_list = []
                            vehicle_task_data[vtp_insert_vehicle_id][vtp_node].recovery_drone_list = []

                            vehicle_task_data = update_vehicle_task(vehicle_task_data, best_scheme, vehicle_route)

                            repaired_state.customer_plan[customer_node] = best_scheme

                            if drone_id not in repaired_state.uav_assignments:
                                repaired_state.uav_assignments[drone_id] = []
                            repaired_state.uav_assignments[drone_id].append(best_scheme)

                            if repaired_state.uav_cost is None:
                                repaired_state.uav_cost = {}
                            repaired_state.uav_cost[customer_node] = original_cost

                            repaired_state.rm_empty_vehicle_route = [route[:] for route in repaired_state.vehicle_routes]
                            repaired_state.rm_empty_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(repaired_state.rm_empty_vehicle_route)

                            insert_plan.append((customer, best_scheme, original_cost, 'vtp_expansion'))

                            success = True
                            break

                if not success:
                    print(f"客户点 {customer} 的所有候选方案都不满足约束，跳过")
                    repaired_state.repair_objective = float('inf')

                    repaired_state.destroyed_customers_info = {}
                    return repaired_state, insert_plan

                if customer in destroy_node:
                    destroy_node.remove(customer)

                num_repaired += 1

        repaired_state._total_cost = repaired_state.win_total_objective()

        repaired_state.destroyed_customers_info = {}
        print(f'修复策略完成，修复后总成本计算完成')
        print(f"修复后总成本: {repaired_state._total_cost}")

        return repaired_state, insert_plan

    def get_near_node_list(self, best_scheme, k, vehicle_route):

        _, _, customer, _, launch_vehicle_id, recovery_vehicle_id = best_scheme
        customer_vtp_dict = self.cluster_vtp_for_customers()
        near_vtp_candidates = customer_vtp_dict.get(customer, [])

        route_launch = vehicle_route[launch_vehicle_id - 1]
        route_launch_set = set(route_launch)
        filtered_launch = [vtp for vtp in near_vtp_candidates if vtp not in self.node[route_launch_set].map_key]
        launch_list = filtered_launch[:k]

        if launch_vehicle_id == recovery_vehicle_id:
            return launch_list
        else:

            route_recovery = vehicle_route[recovery_vehicle_id - 1]
            route_recovery_set = set(route_recovery)
            filtered_recovery = [vtp for vtp in near_vtp_candidates if vtp not in route_recovery_set]
            recovery_list = filtered_recovery[:k]
            return {launch_vehicle_id: launch_list, recovery_vehicle_id: recovery_list}

    def drone_insert_cost(self, drone_id, customer, launch_node, recovery_node):

        launch_node_map_index = self.node[launch_node].map_key
        recovery_node_map_index = self.node[recovery_node].map_key
        customer_map_index = self.node[customer].map_key
        insert_cost = self.uav_travel[drone_id][launch_node_map_index][customer].totalDistance+ self.uav_travel[drone_id][customer][recovery_node_map_index].totalDistance
        per_cost = self.vehicle[drone_id].per_cost
        insert_cost = insert_cost * per_cost
        return insert_cost

    def repair_drone_first_truck_second_insertion(self, state, strategic_bonus=0, num_destroyed=1, force_vtp_mode=True):

        repaired_state = state.fast_copy()
        repaired_state.repair_objective = 0
        destroy_node = list(state.destroyed_customers_info.keys())
        insert_plan = []

        num_repaired = 0
        while len(destroy_node) > 0:

            try:
                current_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(repaired_state.vehicle_routes)
                base_total_cost, uav_tw_violation_cost, total_cost_dict = calculate_window_cost(
                    repaired_state.customer_plan, repaired_state.uav_cost, current_arrive_time,
                    self.vehicle, self.customer_time_windows_h,
                    self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node
                )
            except Exception:
                base_total_cost = float('inf')

            vehicle_route = repaired_state.vehicle_routes
            vehicle_task_data = repaired_state.vehicle_task_data
            vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(repaired_state.vehicle_routes)

            best_candidate = None

            min_drone_cost = float('inf')
            best_drone_candidate = None

            for customer in destroy_node:
                traditional_result, is_heuristic_swap = self._evaluate_traditional_insertion(
                    customer, vehicle_route, vehicle_task_data, vehicle_arrive_time,
                    base_total_cost, uav_tw_violation_cost, total_cost_dict, repaired_state
                )

                if traditional_result[0] or traditional_result[1] is not None:
                    traditional_cost, traditional_scheme = traditional_result
                    if is_heuristic_swap:
                        cand_cost = traditional_scheme['total_cost']
                        cand_type = 'heuristic_swap'
                    else:
                        cand_cost = traditional_cost + base_total_cost
                        cand_type = 'traditional'

                    if cand_cost < min_drone_cost:
                        min_drone_cost = cand_cost
                        best_drone_candidate = {
                            'customer': customer, 'scheme': traditional_scheme, 'cost': traditional_cost,
                            'total_cost': cand_cost, 'type': cand_type, 'vtp_node': None
                        }

            if best_drone_candidate is not None:

                best_candidate = best_drone_candidate
            else:

                min_truck_cost = float('inf')
                best_truck_candidate = None

                for customer in destroy_node:
                    vtp_result, vtp_infor = self._evaluate_vtp_expansion_insertion(
                        customer, vehicle_route, vehicle_task_data, vehicle_arrive_time, repaired_state
                    )

                    if vtp_infor is not None and vtp_result is not None:
                        vtp_cost, vtp_scheme = vtp_result
                        if vtp_cost < min_truck_cost:
                            min_truck_cost = vtp_cost
                            best_truck_candidate = {
                                'customer': customer, 'scheme': vtp_scheme, 'cost': vtp_cost,
                                'total_cost': vtp_cost, 'type': 'vtp_expansion',
                                'vtp_node': vtp_infor[0], 'vtp_insert_vehicle_id': vtp_infor[1],
                                'vtp_insert_index': vtp_infor[2], 'original_cost': vtp_cost
                            }

                best_candidate = best_truck_candidate

            if best_candidate is None:
                print(f"剩余 {len(destroy_node)} 个客户点连卡车兜底都无法修复，停止修复")
                repaired_state.repair_objective = float('inf')
                repaired_state.destroyed_customers_info = {}
                return repaired_state, insert_plan

            candidate = best_candidate
            customer = candidate['customer']
            success = False

            if candidate['type'] == 'traditional':
                best_scheme = candidate['scheme']
                best_cost = self.drone_insert_cost(best_scheme[0], best_scheme[2], best_scheme[1], best_scheme[3])
                drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id = best_scheme

                temp_customer_plan = {k: v for k, v in repaired_state.customer_plan.items()}
                temp_customer_plan[customer_node] = best_scheme
                temp_rm_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(vehicle_route)

                if is_time_feasible(temp_customer_plan, temp_rm_vehicle_arrive_time):
                    repaired_state.customer_plan[customer_node] = best_scheme
                    if drone_id not in repaired_state.uav_assignments: repaired_state.uav_assignments[drone_id] = []
                    repaired_state.uav_assignments[drone_id].append(best_scheme)
                    if repaired_state.uav_cost is None: repaired_state.uav_cost = {}
                    repaired_state.uav_cost[customer_node] = best_cost
                    vehicle_task_data = update_vehicle_task(vehicle_task_data, best_scheme, vehicle_route)
                    insert_plan.append((customer, best_scheme, best_cost, 'traditional'))
                    success = True

            elif candidate['type'] == 'heuristic_swap':
                orig_scheme = candidate['scheme']['orig_scheme']
                new_scheme = candidate['scheme']['new_scheme']
                orig_cost = candidate['scheme']['orig_cost']
                new_cost = candidate['scheme']['new_cost']
                orig_drone_id, orig_launch_node, orig_customer, orig_recovery_node, orig_launch_vehicle, orig_recovery_vehicle = orig_scheme
                new_drone_id, new_launch_node, new_customer, new_recovery_node, new_launch_vehicle, new_recovery_vehicle = new_scheme
                delete_customer = orig_customer

                temp_customer_plan = {k: v for k, v in repaired_state.customer_plan.items()}
                delete_task_plan = temp_customer_plan[orig_customer]
                del temp_customer_plan[orig_customer]
                temp_customer_plan[orig_customer] = orig_scheme
                temp_customer_plan[new_customer] = new_scheme
                temp_rm_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(vehicle_route)

                if is_time_feasible(temp_customer_plan, temp_rm_vehicle_arrive_time):
                    del repaired_state.customer_plan[delete_customer]
                    repaired_state.customer_plan[orig_customer] = orig_scheme
                    repaired_state.customer_plan[new_customer] = new_scheme

                    if orig_drone_id not in repaired_state.uav_assignments: repaired_state.uav_assignments[orig_drone_id] = []
                    repaired_state.uav_assignments[orig_drone_id].append(orig_scheme)
                    if new_drone_id not in repaired_state.uav_assignments: repaired_state.uav_assignments[new_drone_id] = []
                    repaired_state.uav_assignments[new_drone_id].append(new_scheme)
                    del repaired_state.uav_cost[delete_customer]
                    repaired_state.uav_cost[orig_customer] = orig_cost
                    repaired_state.uav_cost[new_customer] = new_cost

                    vehicle_task_data = remove_vehicle_task(vehicle_task_data, delete_task_plan, vehicle_route)
                    orig_launch_time = temp_rm_vehicle_arrive_time[orig_launch_vehicle][orig_launch_node]
                    new_launch_time = temp_rm_vehicle_arrive_time[new_launch_vehicle][new_launch_node]
                    if orig_launch_time <= new_launch_time:
                        vehicle_task_data = update_vehicle_task(vehicle_task_data, orig_scheme, vehicle_route)
                        vehicle_task_data = update_vehicle_task(vehicle_task_data, new_scheme, vehicle_route)
                    else:
                        vehicle_task_data = update_vehicle_task(vehicle_task_data, new_scheme, vehicle_route)
                        vehicle_task_data = update_vehicle_task(vehicle_task_data, orig_scheme, vehicle_route)

                    insert_plan.append((delete_customer, orig_scheme, orig_cost, 'heuristic_swap'))
                    insert_plan.append((customer, new_scheme, new_cost, 'heuristic_swap'))
                    success = True

            elif candidate['type'] == 'vtp_expansion':
                vtp_node = candidate['vtp_node']
                vtp_insert_index = candidate['vtp_insert_index']
                vtp_insert_vehicle_id = candidate['vtp_insert_vehicle_id']
                best_scheme = candidate['scheme']
                drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id = best_scheme
                original_cost = self.drone_insert_cost(drone_id, customer_node, launch_node, recovery_node)

                temp_customer_plan = {k: v for k, v in repaired_state.customer_plan.items()}
                temp_customer_plan[customer_node] = best_scheme
                temp_vehicle_route = [route[:] for route in vehicle_route]
                temp_route = temp_vehicle_route[vtp_insert_vehicle_id - 1]
                temp_route.insert(vtp_insert_index, vtp_node)
                temp_vehicle_route[vtp_insert_vehicle_id - 1] = temp_route
                repaired_state.temp_vehicle_routes = temp_vehicle_route
                temp_rm_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(temp_vehicle_route)

                if is_time_feasible(temp_customer_plan, temp_rm_vehicle_arrive_time):
                    route = vehicle_route[vtp_insert_vehicle_id - 1]
                    route.insert(vtp_insert_index, vtp_node)
                    last_customer_node = route[vtp_insert_index - 1]
                    if vtp_insert_index == 1 or last_customer_node == self.DEPOT_nodeID:
                        last_drone_list = self.base_drone_assignment[vtp_insert_vehicle_id][:]
                    else:
                        last_drone_list = vehicle_task_data[vtp_insert_vehicle_id][last_customer_node].drone_list[:]

                    vehicle_task_data[vtp_insert_vehicle_id][vtp_node].drone_list = last_drone_list
                    vehicle_task_data[vtp_insert_vehicle_id][vtp_node].launch_drone_list = []
                    vehicle_task_data[vtp_insert_vehicle_id][vtp_node].recovery_drone_list = []
                    vehicle_task_data = update_vehicle_task(vehicle_task_data, best_scheme, vehicle_route)

                    repaired_state.customer_plan[customer_node] = best_scheme
                    if drone_id not in repaired_state.uav_assignments: repaired_state.uav_assignments[drone_id] = []
                    repaired_state.uav_assignments[drone_id].append(best_scheme)
                    if repaired_state.uav_cost is None: repaired_state.uav_cost = {}
                    repaired_state.uav_cost[customer_node] = original_cost

                    repaired_state.rm_empty_vehicle_route = [route[:] for route in repaired_state.vehicle_routes]
                    repaired_state.rm_empty_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(repaired_state.rm_empty_vehicle_route)
                    insert_plan.append((customer, best_scheme, original_cost, 'vtp_expansion'))
                    success = True

            if not success:

                pass

            destroy_node.remove(customer)
            num_repaired += 1

        repaired_state._total_cost = repaired_state.win_total_objective()
        repaired_state.destroyed_customers_info = {}

        return repaired_state, insert_plan

    def repair_regret_insertion(self, state, strategic_bonus=0, num_destroyed=1, force_vtp_mode=False):

        repaired_state = state.fast_copy()
        repaired_state.repair_objective = 0
        destroy_node = list(state.destroyed_customers_info.keys())
        insert_plan = []
        force_vtp_mode = True
        if force_vtp_mode:
            num_repaired = 0
            while len(destroy_node) > 0:
                try:
                    current_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(repaired_state.vehicle_routes)
                    base_total_cost, uav_tw_violation_cost, total_cost_dict = calculate_window_cost(
                        repaired_state.customer_plan, repaired_state.uav_cost, current_arrive_time,
                        self.vehicle, self.customer_time_windows_h,
                        self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node
                    )
                except Exception:
                    base_total_cost = float('inf')

                tactical_multiplier = (num_destroyed - num_repaired) / max(num_destroyed, 1)
                final_bonus = strategic_bonus * tactical_multiplier * 0.3
                final_bonus = 0

                vehicle_route = repaired_state.vehicle_routes
                vehicle_task_data = repaired_state.vehicle_task_data
                vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(repaired_state.vehicle_routes)

                per_customer_candidates = {}
                regret_list = []

                for customer in destroy_node:
                    candidates = []

                    traditional_result, is_heuristic_swap = self._regret_evaluate_traditional_insertion(customer, vehicle_route, vehicle_task_data, vehicle_arrive_time, base_total_cost, uav_tw_violation_cost, total_cost_dict, repaired_state)
                    candidates.extend(traditional_result)

                    total_options = self._regret_evaluate_vtp_expansion_insertion(customer, vehicle_route, vehicle_task_data, vehicle_arrive_time, repaired_state)
                    candidates.extend(total_options)

                    if len(candidates) == 0:
                        print(f'在regret的修复策略中，客户点{customer}没有可行的插入方案，包括传统插入和VTP扩展插入,跳过')
                        continue

                    candidates_sorted = sorted(candidates, key=lambda x: x['eval_cost'])
                    best_cost = candidates_sorted[0]['total_cost']
                    best_type = candidates_sorted[0]['type']
                    second_best_cost = candidates_sorted[1]['total_cost'] if len(candidates_sorted) >= 2 else best_cost
                    second_best_type = candidates_sorted[1]['type'] if len(candidates_sorted) >= 2 else None

                    regret_value = second_best_cost - best_cost
                    per_customer_candidates[customer] = candidates_sorted
                    regret_list.append({'customer': customer, 'regret': regret_value, 'best_cost': best_cost, 'best_type': best_type, 'second_best_cost': second_best_cost, 'second_best_type': second_best_type})

                if not regret_list:

                    break

                regret_list.sort(key=lambda x: (-x['regret'], x['best_cost']))

                success_any = False
                for entry in regret_list:
                    customer = entry['customer']
                    candidates_sorted = per_customer_candidates[customer]
                    candidates_sorted = [item for item in candidates_sorted if item.get('scheme') is not None]

                    for candidate in candidates_sorted:

                        if candidate['type'] == 'traditional':

                            best_scheme = candidate['scheme']
                            customer = best_scheme[2]
                            best_cost = self.drone_insert_cost(best_scheme[0], best_scheme[2], best_scheme[1], best_scheme[3])
                            temp_customer_plan = {k: v for k, v in repaired_state.customer_plan.items()}
                            temp_customer_plan[best_scheme[2]] = best_scheme
                            temp_rm_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(vehicle_route)
                            if not is_time_feasible(temp_customer_plan, temp_rm_vehicle_arrive_time):
                                continue

                            drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id = best_scheme

                            repaired_state.customer_plan[customer_node] = best_scheme
                            if drone_id not in repaired_state.uav_assignments:
                                repaired_state.uav_assignments[drone_id] = []
                            repaired_state.uav_assignments[drone_id].append(best_scheme)
                            if repaired_state.uav_cost is None:
                                repaired_state.uav_cost = {}
                            repaired_state.uav_cost[customer_node] = best_cost
                            vehicle_task_data = update_vehicle_task(vehicle_task_data, best_scheme, vehicle_route)
                            insert_plan.append((customer, best_scheme, best_cost, 'traditional'))
                            success_any = True
                            break
                        elif candidate['type'] == 'heuristic_swap':
                            print(f"尝试使用启发式交换方案插入客户点 {customer}，成本: {best_cost:.2f}")

                            orig_scheme = candidate['orig_scheme']
                            new_scheme = candidate['new_scheme']
                            orig_cost = candidate['orig_cost']
                            new_cost = candidate['new_cost']

                            temp_customer_plan = {k: v for k, v in repaired_state.customer_plan.items()}
                            orig_drone_id, orig_launch_node, orig_customer, orig_recovery_node, orig_launch_vehicle, orig_recovery_vehicle = orig_scheme
                            new_drone_id, new_launch_node, new_customer, new_recovery_node, new_launch_vehicle, new_recovery_vehicle = new_scheme
                            customer = orig_customer
                            delete_task_plan = temp_customer_plan[orig_customer]
                            delete_customer = orig_customer

                            del temp_customer_plan[delete_customer]
                            temp_customer_plan[orig_customer] = orig_scheme
                            temp_customer_plan[new_customer] = new_scheme
                            temp_rm_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(vehicle_route)
                            if not is_time_feasible(temp_customer_plan, temp_rm_vehicle_arrive_time):
                                print(f"启发式交换方案时间约束不满足，尝试下一个候选方案")
                                continue
                            else:

                                del repaired_state.customer_plan[delete_customer]
                                repaired_state.customer_plan[orig_customer] = orig_scheme
                                repaired_state.customer_plan[new_customer] = new_scheme

                                if orig_drone_id not in repaired_state.uav_assignments:
                                    repaired_state.uav_assignments[orig_drone_id] = []
                                repaired_state.uav_assignments[orig_drone_id].append(orig_scheme)
                                if new_drone_id not in repaired_state.uav_assignments:
                                    repaired_state.uav_assignments[new_drone_id] = []
                                repaired_state.uav_assignments[new_drone_id].append(new_scheme)

                                del repaired_state.uav_cost[delete_customer]
                                repaired_state.uav_cost[orig_customer] = orig_cost
                                repaired_state.uav_cost[new_customer] = new_cost

                                vehicle_task_data = remove_vehicle_task(vehicle_task_data, delete_task_plan, vehicle_route)
                                orig_launch_time = repaired_state.rm_empty_vehicle_arrive_time[orig_launch_vehicle][orig_launch_node]
                                new_launch_time = repaired_state.rm_empty_vehicle_arrive_time[new_launch_vehicle][new_launch_node]
                                if orig_launch_time <= new_launch_time:
                                    vehicle_task_data = update_vehicle_task(vehicle_task_data, orig_scheme, vehicle_route)
                                    vehicle_task_data = update_vehicle_task(vehicle_task_data, new_scheme, vehicle_route)
                                else:
                                    vehicle_task_data = update_vehicle_task(vehicle_task_data, new_scheme, vehicle_route)
                                    vehicle_task_data = update_vehicle_task(vehicle_task_data, orig_scheme, vehicle_route)

                                insert_plan.append((delete_customer, orig_scheme, orig_cost, 'heuristic_swap'))
                                insert_plan.append((customer, new_scheme, new_cost, 'heuristic_swap'))
                                success_any = True
                                break
                        elif candidate['type'] == 'vtp_expansion':
                            customer = candidate['customer']
                            vtp_node = candidate['vtp_node']
                            vtp_insert_index = candidate['vtp_insert_index']
                            vtp_insert_vehicle_id = candidate['vtp_insert_vehicle_id']
                            drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id = candidate['scheme']
                            original_cost = self.drone_insert_cost(drone_id, customer_node, launch_node, recovery_node)

                            temp_customer_plan = {k: v for k, v in repaired_state.customer_plan.items()}
                            temp_customer_plan[customer_node] = candidate['scheme']
                            temp_vehicle_route = [route[:] for route in vehicle_route]
                            temp_route = temp_vehicle_route[vtp_insert_vehicle_id - 1]
                            temp_route.insert(vtp_insert_index, vtp_node)
                            temp_vehicle_route[vtp_insert_vehicle_id - 1] = temp_route
                            repaired_state.temp_vehicle_routes = temp_vehicle_route
                            temp_rm_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(temp_vehicle_route)
                            if not is_time_feasible(temp_customer_plan, temp_rm_vehicle_arrive_time):
                                continue

                            route = vehicle_route[vtp_insert_vehicle_id - 1]
                            route.insert(vtp_insert_index, vtp_node)

                            last_customer_node = route[vtp_insert_index - 1]
                            if vtp_insert_index == 1 or last_customer_node == self.DEPOT_nodeID:
                                last_drone_list = self.base_drone_assignment[vtp_insert_vehicle_id][:]
                            else:
                                last_drone_list = vehicle_task_data[vtp_insert_vehicle_id][last_customer_node].drone_list[:]
                            vehicle_task_data[vtp_insert_vehicle_id][vtp_node].drone_list = last_drone_list
                            vehicle_task_data[vtp_insert_vehicle_id][vtp_node].launch_drone_list = []
                            vehicle_task_data[vtp_insert_vehicle_id][vtp_node].recovery_drone_list = []

                            vehicle_task_data = update_vehicle_task(vehicle_task_data, candidate['scheme'], vehicle_route)

                            repaired_state.customer_plan[customer_node] = candidate['scheme']
                            if drone_id not in repaired_state.uav_assignments:
                                repaired_state.uav_assignments[drone_id] = []
                            repaired_state.uav_assignments[drone_id].append(candidate['scheme'])
                            if repaired_state.uav_cost is None:
                                repaired_state.uav_cost = {}
                            repaired_state.uav_cost[customer_node] = original_cost
                            repaired_state.rm_empty_vehicle_route = [route[:] for route in repaired_state.vehicle_routes]
                            repaired_state.rm_empty_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(repaired_state.rm_empty_vehicle_route)
                            insert_plan.append((customer, candidate['scheme'], original_cost, 'vtp_expansion'))
                            success_any = True
                            break

                    if success_any:
                        if customer in destroy_node:
                            destroy_node.remove(customer)
                        num_repaired += 1
                        break
                    else:
                        print(f'在regret的修复策略中，客户点{customer}没有可行的插入方案，跳过，插入方案失败')
                        repaired_state.repair_objective = float('inf')

                        repaired_state.destroyed_customers_info = {}
                        return repaired_state, insert_plan

                if not success_any:

                    break
        repaired_state._total_cost = repaired_state.win_total_objective()
        print(f'修复策略完成，修复后总成本计算完成')
        print(f"修复后总成本: {repaired_state._total_cost}")
        return repaired_state, insert_plan

    def repair_kNN_regret(self, state, strategic_bonus=0, num_destroyed=1, force_vtp_mode=False):

        try:
            repaired_state = state.fast_copy()
            repaired_state.repair_objective = 0
            vehicle_routes = repaired_state.vehicle_routes
            vehicle_task_data = repaired_state.vehicle_task_data
            vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(vehicle_routes)
            destroy_node = list(state.destroyed_customers_info.keys())
            insert_plan = []
            force_vtp_mode = True
            num_repaired = 0
            k_neighbors = self.params['k_neighbors']
            K_revest_position = 5
            k_neighbors = self.params.get('k_neighbors', 3)
            K_BEST_POSITIONS = self.params.get('K_BEST_POSITIONS', 5)
            w_impact = self.params.get('w_impact', 0.5)

            if force_vtp_mode:
                while len(destroy_node) > 0:
                    try:
                        current_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(repaired_state.vehicle_routes)
                        base_total_cost, uav_tw_violation_cost, total_cost_dict = calculate_window_cost(
                            repaired_state.customer_plan, repaired_state.uav_cost, current_arrive_time,
                            self.vehicle, self.customer_time_windows_h,
                            self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node
                        )
                    except Exception:
                        base_total_cost = float('inf')

                    final_bonus = 0.0
                    if force_vtp_mode and len(destroy_node) > 0:
                        tactical_multiplier = (len(destroy_node) - num_repaired) / max(len(destroy_node), 1)
                        final_bonus = strategic_bonus * tactical_multiplier
                    else:
                        final_bonus = 0.0
                    cached_baseline_costs = {}
                    regret_list = []
                    candidates = []

                    candidate_new_vtps = self._get_all_candidate_new_vtps(destroy_node, repaired_state)

                    used_vtps_set = {node for route in repaired_state.vehicle_routes for node in route[1:-1]}

                    temp_vtp_task_data = deep_copy_vehicle_task_data(repaired_state.vehicle_task_data)

                    for customer in destroy_node:

                        candidates = []
                        k_nearest_neighbors = self._find_k_nearest_unassigned(customer, k_neighbors, destroy_node)

                        for vtp_new in candidate_new_vtps:

                            temp_vtp_task_data = restore_vehicle_task_data_for_vehicles(temp_vtp_task_data, repaired_state.vehicle_task_data, self.T)

                            best_positions = self._find_k_best_vehicle_for_new_vtp(vtp_new, repaired_state,K_revest_position)
                            if not best_positions: continue

                            for (veh_id, insert_idx, veh_delta_cost) in best_positions:

                                total_benefit, affected_customers = self._calculate_vtp_benefits(
                                    vtp_new, (veh_id, insert_idx), repaired_state, customer, temp_vtp_task_data
                                )
                                for customer, scheme in affected_customers:
                                    cost = scheme[0]
                                    scheme_plan = scheme[1]
                                    candidates.append({'customer': customer, 'type': 'vtp_expansion', 'vtp_node': vtp_new, 'vtp_insert_index': insert_idx,
                                    'vtp_insert_vehicle_id': veh_id, 'scheme': scheme_plan, 'eval_cost': cost, 'total_cost': cost})
                        for vtp_shared in used_vtps_set:

                            best_shared_positions = self._find_k_best_vehicles_for_shared_vtp(vtp_shared, repaired_state, K_BEST_POSITIONS)

                            temp_vtp_task_data = restore_vehicle_task_data_for_vehicles(temp_vtp_task_data, repaired_state.vehicle_task_data, self.T)
                            if not best_shared_positions: continue

                            for (veh_id, insert_idx, veh_delta_cost) in best_shared_positions:

                                total_benefit, affected_customers = self._calculate_vtp_benefits(
                                    vtp_shared, (veh_id, insert_idx), repaired_state, customer, temp_vtp_task_data
                                )
                                for customer, scheme in affected_customers:
                                    cost = scheme[0]
                                    scheme_plan = scheme[1]
                                    candidates.append({'customer': customer, 'type': 'vtp_expansion', 'vtp_node': vtp_shared, 'vtp_insert_index': insert_idx,
                                    'vtp_insert_vehicle_id': veh_id, 'scheme': scheme_plan, 'eval_cost': cost, 'total_cost': cost})

                        traditional_options_list, is_heuristic_swap = self._regret_evaluate_traditional_insertion(
                            customer, vehicle_routes, vehicle_task_data, vehicle_arrive_time, base_total_cost, uav_tw_violation_cost, total_cost_dict, repaired_state)
                        if is_heuristic_swap:
                            for trad_opt in traditional_options_list:
                                if trad_opt['type'] == 'heuristic_swap':
                                    candidates.append({
                                    'customer': customer,
                                    'type': 'heuristic_swap',
                                    'vtp_node': None,
                                    'vtp_insert_index': None,
                                    'vtp_insert_vehicle_id': None,
                                    'orig_scheme': trad_opt['orig_scheme'],
                                    'new_scheme': trad_opt['new_scheme'],
                                    'orig_cost': trad_opt['orig_cost'],
                                    'new_cost': trad_opt['new_cost'],
                                    'eval_cost': trad_opt['eval_cost'],
                                    'total_cost': trad_opt['total_cost']
                                    })
                                else:
                                    candidates.append({
                                        'customer': customer,
                                        'type': 'traditional',
                                        'vtp_node': None,
                                        'vtp_insert_index': None,
                                        'vtp_insert_vehicle_id': None,
                                        'scheme': trad_opt['scheme'],
                                        'eval_cost': trad_opt['eval_cost'],
                                        'total_cost': trad_opt['total_cost']
                                    })
                        else:
                            for trad_opt in traditional_options_list:
                                candidates.append({
                                    'customer': customer,
                                    'type': 'traditional',
                                    'vtp_node': None,
                                    'vtp_insert_index': None,
                                    'vtp_insert_vehicle_id': None,
                                    'scheme': trad_opt['scheme'],
                                    'eval_cost': trad_opt['eval_cost'],
                                    'total_cost': trad_opt['total_cost']
                                })

                        candidates = [c for c in candidates if not np.isinf(c['total_cost'])]

                        candidates_sorted = sorted(candidates, key=lambda x: x['total_cost'])[:K_BEST_POSITIONS]
                        plan_scores = []

                        for option_tuple in candidates_sorted[:]:
                            temp_state_after = repaired_state.temp_fast_copy(vehicles_to_copy=self.T)

                            current_eval_cost = option_tuple['total_cost']
                            plan_type = option_tuple.get('type', 'traditional')
                            plan_scheme = option_tuple.get('scheme', None)
                            if plan_scheme is None:
                                print('plan_scheme is None')
                                continue
                            neigh = self._find_k_nearest_unassigned(option_tuple['customer'], k_neighbors, destroy_node)

                            future_impact = 0.0

                            if plan_type == 'vtp_expansion':
                                future_impact = self._calculate_future_impact(
                                    option_tuple, neigh, repaired_state, temp_state_after, base_total_cost, uav_tw_violation_cost, total_cost_dict
                                )
                            else:

                                future_impact = self._calculate_tradition_future_impact(option_tuple, neigh, repaired_state, temp_state_after, base_total_cost, uav_tw_violation_cost, total_cost_dict)
                            total_kNN_score = current_eval_cost + w_impact * future_impact

                            plan_scores.append({'kNN_score': total_kNN_score, 'option_dict': option_tuple})

                        if not plan_scores: continue
                        plan_scores.sort(key=lambda x: x['kNN_score'])

                        best_kNN_option = plan_scores[0]['option_dict']
                        best_kNN_score = plan_scores[0]['kNN_score']
                        second_best = plan_scores[1]['kNN_score'] if len(plan_scores) >= 2 else best_kNN_score
                        regret_value = second_best - best_kNN_score

                        regret_list.append({
                            'customer': customer,
                            'regret': regret_value,
                            'best_kNN_score': best_kNN_score,
                            'best_option': best_kNN_option,
                            'type': best_kNN_option['type'],
                        })

                    regret_list_sorted = sorted(regret_list, key=lambda x: (-x['regret'], x['best_kNN_score']))
                    if not regret_list_sorted:
                        repaired_state.repair_objective = float('inf')
                        return repaired_state, insert_plan

                    best_entry = regret_list_sorted[0]
                    best_opt = best_entry['best_option']
                    best_type = best_entry['type']
                    best_cust = best_entry['customer']
                    if best_type == 'traditional':
                        drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id = best_opt['scheme']
                        best_cost = self.drone_insert_cost(drone_id, customer_node, launch_node, recovery_node)

                        repaired_state.customer_plan[customer_node] = best_opt['scheme']
                        if drone_id not in repaired_state.uav_assignments:
                            repaired_state.uav_assignments[drone_id] = []
                        repaired_state.uav_assignments[drone_id].append(best_opt['scheme'])

                        if repaired_state.uav_cost is None:
                            repaired_state.uav_cost = {}
                        repaired_state.uav_cost[customer_node] = best_cost

                        vehicle_task_data = update_vehicle_task(vehicle_task_data, best_opt['scheme'], vehicle_routes)

                        insert_plan.append((best_cust, best_opt['scheme'], best_cost, 'traditional'))
                    elif best_type == 'vtp_expansion':
                        customer = best_opt['customer']
                        vtp_node = best_opt['vtp_node']
                        vtp_insert_index = best_opt['vtp_insert_index']
                        vtp_insert_vehicle_id = best_opt['vtp_insert_vehicle_id']
                        best_scheme = best_opt['scheme']
                        drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id = best_scheme
                        original_cost = self.drone_insert_cost(drone_id, customer_node, launch_node, recovery_node)

                        temp_customer_plan = {k: v for k, v in repaired_state.customer_plan.items()}
                        temp_customer_plan[customer_node] = best_scheme

                        temp_vehicle_route = [route[:] for route in repaired_state.vehicle_routes]
                        temp_route = temp_vehicle_route[vtp_insert_vehicle_id - 1]
                        temp_route.insert(vtp_insert_index, vtp_node)
                        temp_vehicle_route[vtp_insert_vehicle_id - 1] = temp_route
                        repaired_state.temp_vehicle_routes = temp_vehicle_route

                        temp_rm_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(temp_vehicle_route)
                        time_feasible = is_time_feasible(temp_customer_plan, temp_rm_vehicle_arrive_time)
                        if not time_feasible:

                            print(f"VTP扩展方案时间约束不满足，尝试下一个候选方案")
                            continue
                        else:

                            route = repaired_state.vehicle_routes[vtp_insert_vehicle_id - 1]

                            route.insert(vtp_insert_index, vtp_node)

                            last_customer_node = route[vtp_insert_index - 1]

                            if vtp_insert_index == 1 or last_customer_node == self.DEPOT_nodeID:
                                last_drone_list = self.base_drone_assignment[vtp_insert_vehicle_id][:]
                            else:
                                last_drone_list = vehicle_task_data[vtp_insert_vehicle_id][last_customer_node].drone_list[:]

                            vehicle_task_data[vtp_insert_vehicle_id][vtp_node].drone_list = last_drone_list
                            vehicle_task_data[vtp_insert_vehicle_id][vtp_node].launch_drone_list = []
                            vehicle_task_data[vtp_insert_vehicle_id][vtp_node].recovery_drone_list = []

                            vehicle_task_data = update_vehicle_task(vehicle_task_data, best_scheme, repaired_state.vehicle_routes)

                            repaired_state.customer_plan[customer_node] = best_scheme

                            if drone_id not in repaired_state.uav_assignments:
                                repaired_state.uav_assignments[drone_id] = []
                            repaired_state.uav_assignments[drone_id].append(best_scheme)

                            if repaired_state.uav_cost is None:
                                repaired_state.uav_cost = {}
                            repaired_state.uav_cost[customer_node] = original_cost

                            repaired_state.rm_empty_vehicle_route = [route[:] for route in repaired_state.vehicle_routes]
                            repaired_state.rm_empty_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(repaired_state.rm_empty_vehicle_route)

                            insert_plan.append((customer, best_scheme, original_cost, 'vtp_expansion'))

                    destroy_node.remove(best_cust)

                    vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(vehicle_routes)
                    num_repaired += 1
            return repaired_state, insert_plan
        except Exception as e:
            print(f"repair_kNN_regret 修复失败: {e}")
            repaired_state.repair_objective = float('inf')
            insert_plan = []
            return repaired_state, insert_plan

    def _calculate_tradition_future_impact(self, option_dict, k_neighbors, original_state, temp_state_after, base_total_cost, uav_tw_violation_cost, total_cost_dict):

        M_PENALTY = self.M_PENALTY
        total_impact = 0.0

        costs_before = {}
        orig_routes = original_state.vehicle_routes
        orig_task_data = original_state.vehicle_task_data
        orig_arrive_time = original_state.calculate_rm_empty_vehicle_arrive_time(orig_routes)
        orig_total_cost = sum(original_state.uav_cost.values())

        for customer in k_neighbors:

            trad_options, is_heuristic_swap = self._evaluate_traditional_insertion(
                customer, orig_routes, orig_task_data,
                orig_arrive_time, base_total_cost, uav_tw_violation_cost, total_cost_dict, original_state)
            if trad_options[0] or trad_options[1] is not None:
                trad_cost, traditional_scheme = trad_options
                if is_heuristic_swap:

                    current_total_cost = traditional_scheme['total_cost']
                    deta_total_cost = traditional_scheme['win_cost']
                    costs_before[customer] = current_total_cost
                else:

                    current_total_cost = trad_cost + base_total_cost
                    costs_before[customer] = current_total_cost
            else:
                costs_before[customer] = float('inf')

        try:
            if option_dict['type'] == 'heuristic_swap':
                orig_scheme = option_dict['orig_scheme']
                new_scheme = option_dict['new_scheme']
                orig_cost = option_dict['orig_cost']
                new_cost = option_dict['new_cost']
                delta_cost = orig_cost + new_cost - option_dict['delta_cost']
                orig_drone_id, orig_launch_node, orig_customer, orig_recovery_node, orig_launch_vehicle, orig_recovery_vehicle = orig_scheme
                new_drone_id, new_launch_node, new_customer, new_recovery_node, new_launch_vehicle, new_recovery_vehicle = new_scheme
                customer = new_customer
                delete_customer = orig_customer
                temp_customer_plan = {k: v for k, v in temp_state_after.customer_plan.items()}
                delete_task_plan = temp_customer_plan[orig_customer]
                del temp_customer_plan[orig_customer]
                temp_customer_plan[orig_customer] = orig_scheme
                temp_customer_plan[new_customer] = new_scheme
                temp_rm_vehicle_arrive_time = temp_state_after.calculate_rm_empty_vehicle_arrive_time(temp_state_after.vehicle_routes)
                if not is_time_feasible(temp_customer_plan, temp_rm_vehicle_arrive_time):
                    print(f"启发式交换方案时间约束不满足，尝试下一个候选方案")
                    return M_PENALTY

                del temp_customer_plan[delete_customer]
                temp_customer_plan.customer_plan[orig_customer] = orig_scheme
                temp_customer_plan.customer_plan[new_customer] = new_scheme

                if orig_drone_id not in temp_customer_plan.uav_assignments:
                    temp_customer_plan.uav_assignments[orig_drone_id] = []
                temp_customer_plan.uav_assignments[orig_drone_id].append(orig_scheme)
                if new_drone_id not in temp_customer_plan.uav_assignments:
                    temp_customer_plan.uav_assignments[new_drone_id] = []
                temp_customer_plan.uav_assignments[new_drone_id].append(new_scheme)

                del temp_customer_plan.uav_cost[delete_customer]
                temp_customer_plan.uav_cost[orig_customer] = orig_cost
                temp_customer_plan.uav_cost[new_customer] = new_cost

                temp_state_after.vehicle_task_data = remove_vehicle_task(temp_state_after.vehicle_task_data, delete_task_plan, temp_customer_plan.vehicle_routes)
                orig_launch_time = temp_rm_vehicle_arrive_time[orig_launch_vehicle][orig_launch_node]
                new_launch_time = temp_rm_vehicle_arrive_time[new_launch_vehicle][new_launch_node]
                if orig_launch_time <= new_launch_time:
                    temp_state_after.vehicle_task_data = update_vehicle_task(temp_state_after.vehicle_task_data, orig_scheme, temp_state_after.vehicle_routes)
                    temp_state_after.vehicle_task_data = update_vehicle_task(temp_state_after.vehicle_task_data, new_scheme, temp_state_after.vehicle_routes)
                else:
                    temp_state_after.vehicle_task_data = update_vehicle_task(temp_state_after.vehicle_task_data, new_scheme, temp_state_after.vehicle_routes)
                    temp_state_after.vehicle_task_data = update_vehicle_task(temp_state_after.vehicle_task_data, orig_scheme, temp_state_after.vehicle_routes)
            else:

                scheme = option_dict['scheme']
                drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id = scheme

                temp_customer_plan = {k: v for k, v in temp_state_after.customer_plan.items()}
                temp_customer_plan[customer_node] = scheme
                temp_rm_vehicle_arrive_time = temp_state_after.calculate_rm_empty_vehicle_arrive_time(
                    temp_state_after.vehicle_routes
                )

                if not is_time_feasible(temp_customer_plan, temp_rm_vehicle_arrive_time):
                    return M_PENALTY

                temp_state_after.customer_plan[customer_node] = scheme

                if drone_id not in temp_state_after.uav_assignments:
                    temp_state_after.uav_assignments[drone_id] = []
                temp_state_after.uav_assignments[drone_id].append(scheme)

                if temp_state_after.uav_cost is None:
                    temp_state_after.uav_cost = {}

                real_cost = self.drone_insert_cost(scheme[0], scheme[2], scheme[1], scheme[3])
                if real_cost is not None:
                    temp_state_after.uav_cost[customer_node] = real_cost

                temp_state_after.vehicle_task_data = update_vehicle_task(
                    temp_state_after.vehicle_task_data, scheme, temp_state_after.vehicle_routes
                )
        except Exception as e:

            return 0

        costs_after = {}
        temp_arrive_time = temp_rm_vehicle_arrive_time
        temp_base_total_cost, temp_uav_tw_violation_cost, temp_total_cost_dict = calculate_window_cost(
            temp_state_after.customer_plan, temp_state_after.uav_cost, temp_arrive_time,
            self.vehicle, self.customer_time_windows_h,
            self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node
        )

        for customer in k_neighbors:
            trad_options_after, is_heuristic_swap = self._evaluate_traditional_insertion(
                customer, temp_state_after.vehicle_routes, temp_state_after.vehicle_task_data,
                temp_arrive_time, temp_base_total_cost, temp_uav_tw_violation_cost, temp_total_cost_dict, temp_state_after)
            if is_heuristic_swap:

                current_total_cost = trad_options_after[1]['total_cost']
                deta_total_cost = trad_options_after[1]['win_cost']
                costs_after[customer] = current_total_cost
            else:
                if trad_options_after == (None,None):
                    costs_after[customer] = float('inf')
                    continue
                else:
                    cost = trad_options_after[0] + temp_base_total_cost
                    costs_after[customer] = cost

        for customer in k_neighbors:
            cost_b = M_PENALTY if costs_before.get(customer, float('inf')) == float('inf') else costs_before.get(customer)
            cost_a = M_PENALTY if costs_after.get(customer, float('inf')) == float('inf') else costs_after.get(customer)
            total_impact += (cost_a - cost_b)

        return total_impact

    def _calculate_future_impact(self, option_dict, k_neighbors, original_state, temp_state_after, base_total_cost, uav_tw_violation_cost, total_cost_dict):

        M_PENALTY = self.M_PENALTY
        total_impact = 0.0

        costs_before = {}
        orig_routes = original_state.vehicle_routes
        orig_task_data = original_state.vehicle_task_data
        orig_arrive_time = original_state.calculate_rm_empty_vehicle_arrive_time(orig_routes)

        for customer in k_neighbors:

            trad_options, is_heuristic_swap = self._evaluate_traditional_insertion(
                customer, orig_routes, orig_task_data,
                orig_arrive_time, base_total_cost, uav_tw_violation_cost, total_cost_dict, original_state)
            if trad_options:
                min_real_cost = trad_options[0]
                costs_before[customer] = min_real_cost
            else:
                costs_before[customer] = float('inf')

        try:

            plan_type = option_dict['type']

            if plan_type == 'vtp_expansion' or plan_type == 'investment' or plan_type == 'sharing':
                real_cost = option_dict['eval_cost']
                total_cost = option_dict['total_cost']
                plan = option_dict['scheme']
                vtp_node = option_dict['vtp_node']
                vtp_insert_index = option_dict['vtp_insert_index']
                vtp_insert_vehicle_id = option_dict['vtp_insert_vehicle_id']
            else:
                print(f"  > 错误: 未知插入方案类型: {plan_type}")
                return 0
            option_to_execute = (real_cost, plan, plan_type, vtp_node, vtp_insert_index, vtp_insert_vehicle_id, total_cost)
            self._execute_insertion(temp_state_after, option_to_execute)

        except Exception as e:

            return 0

        costs_after = {}
        temp_arrive_time = temp_state_after.calculate_rm_empty_vehicle_arrive_time(temp_state_after.vehicle_routes)
        base_total_cost, uav_tw_violation_cost, total_cost_dict = calculate_window_cost(
                        temp_state_after.customer_plan, temp_state_after.uav_cost, temp_arrive_time,
                        self.vehicle, self.customer_time_windows_h,
                        self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node
                    )
        for customer in k_neighbors:
            trad_options_after, is_heuristic_swap = self._evaluate_traditional_insertion(
                customer, temp_state_after.vehicle_routes, temp_state_after.vehicle_task_data,
                temp_arrive_time, base_total_cost, uav_tw_violation_cost, total_cost_dict, temp_state_after)
            if is_heuristic_swap:
                costs_after[customer] = float('inf')
                continue
            if trad_options_after == (None,None):
                costs_after[customer] = float('inf')
                continue
            cost = trad_options_after[0]
            costs_after[customer] = cost

        for customer in k_neighbors:
            cost_b = M_PENALTY if costs_before.get(customer, float('inf')) == float('inf') else costs_before.get(customer)
            cost_a = M_PENALTY if costs_after.get(customer, float('inf')) == float('inf') else costs_after.get(customer)
            total_impact += (cost_a - cost_b)

        return total_impact

    def _execute_insertion(self, state, option):

        real_cost, plan, plan_type, vtp_node, vtp_insert_index, vtp_insert_vehicle_id, total_cost = option
        drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id = plan
        routes = state.vehicle_routes
        task_data = state.vehicle_task_data
        arrive_time = state.calculate_rm_empty_vehicle_arrive_time(routes)
        routes[vtp_insert_vehicle_id - 1].insert(vtp_insert_index, vtp_node)

        if not is_time_feasible(state.customer_plan, arrive_time):
            return False
        state.vehicle_routes = routes
        last_node = routes[vtp_insert_vehicle_id - 1][vtp_insert_index - 1]
        if last_node == self.DEPOT_nodeID or vtp_insert_index == 1:
            drone_list = self.base_drone_assignment[vtp_insert_vehicle_id][:]
        else:
            drone_list = task_data[vtp_insert_vehicle_id][last_node].drone_list[:]
        task_data[vtp_insert_vehicle_id][vtp_node].drone_list = drone_list
        task_data[vtp_insert_vehicle_id][vtp_node].launch_drone_list = []
        state.vehicle_task_data = task_data
        arrive_time = state.calculate_rm_empty_vehicle_arrive_time(routes)
        state.vehicle_arrive_time = arrive_time
        state.customer_plan[customer_node] = (drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id)
        if drone_id not in state.uav_assignments:
            state.uav_assignments[drone_id] = []
        state.uav_assignments[drone_id].append((customer_node, launch_node, recovery_node, launch_vehicle_id, recovery_vehicle_id))
        state.uav_assignments[drone_id].append(plan)
        if state.uav_cost is None:
            state.uav_cost = {}
            state.uav_cost[customer_node] = real_cost
        task_data = update_vehicle_task(task_data, plan, routes)
        state.vehicle_task_data = task_data
        return True

    def repair_equal_priority_insertion(self, state, strategic_bonus=0, num_destroyed=1, force_vtp_mode=True):

        repaired_state = state.fast_copy()
        repaired_state.repair_objective = 0
        destroy_node = list(state.destroyed_customers_info.keys())
        insert_plan = []

        num_repaired = 0
        while len(destroy_node) > 0:

            try:
                current_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(repaired_state.vehicle_routes)
                base_total_cost, uav_tw_violation_cost, total_cost_dict = calculate_window_cost(
                    repaired_state.customer_plan, repaired_state.uav_cost, current_arrive_time,
                    self.vehicle, self.customer_time_windows_h,
                    self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node
                )
            except Exception:
                base_total_cost = float('inf')

            vehicle_route = repaired_state.vehicle_routes
            vehicle_task_data = repaired_state.vehicle_task_data
            vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(repaired_state.vehicle_routes)

            best_overall_candidate = None
            min_global_delta_f = float('inf')

            for customer in destroy_node:

                best_customer_candidate = None
                customer_min_cost = float('inf')

                traditional_result, is_heuristic_swap = self._evaluate_traditional_insertion(
                    customer, vehicle_route, vehicle_task_data, vehicle_arrive_time,
                    base_total_cost, uav_tw_violation_cost, total_cost_dict, repaired_state
                )

                if traditional_result[0] or traditional_result[1] is not None:
                    traditional_cost, traditional_scheme = traditional_result
                    if is_heuristic_swap:
                        cand_cost = traditional_scheme['total_cost']
                        cand_type = 'heuristic_swap'
                    else:
                        cand_cost = traditional_cost + base_total_cost
                        cand_type = 'traditional'

                    if cand_cost < customer_min_cost:
                        customer_min_cost = cand_cost
                        best_customer_candidate = {
                            'customer': customer, 'scheme': traditional_scheme, 'cost': traditional_cost,
                            'total_cost': cand_cost, 'type': cand_type, 'vtp_node': None
                        }

                vtp_result, vtp_infor = self._evaluate_vtp_expansion_insertion(
                    customer, vehicle_route, vehicle_task_data, vehicle_arrive_time, repaired_state
                )

                if vtp_infor is not None and vtp_result is not None:
                    vtp_cost, vtp_scheme = vtp_result

                    if vtp_cost < customer_min_cost:
                        customer_min_cost = vtp_cost
                        best_customer_candidate = {
                            'customer': customer, 'scheme': vtp_scheme, 'cost': vtp_cost,
                            'total_cost': vtp_cost, 'type': 'vtp_expansion',
                            'vtp_node': vtp_infor[0], 'vtp_insert_vehicle_id': vtp_infor[1],
                            'vtp_insert_index': vtp_infor[2], 'original_cost': vtp_cost
                        }

                if best_customer_candidate is not None and customer_min_cost < min_global_delta_f:
                    min_global_delta_f = customer_min_cost
                    best_overall_candidate = best_customer_candidate

            if best_overall_candidate is None:
                print(f"剩余 {len(destroy_node)} 个客户点均无可行插入方案，停止修复")
                repaired_state.repair_objective = float('inf')
                repaired_state.destroyed_customers_info = {}
                return repaired_state, insert_plan

            candidate = best_overall_candidate
            customer = candidate['customer']
            success = False

            if candidate['type'] == 'traditional':
                best_scheme = candidate['scheme']
                best_cost = self.drone_insert_cost(best_scheme[0], best_scheme[2], best_scheme[1], best_scheme[3])
                drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id = best_scheme

                temp_customer_plan = {k: v for k, v in repaired_state.customer_plan.items()}
                temp_customer_plan[customer_node] = best_scheme
                temp_rm_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(vehicle_route)

                if is_time_feasible(temp_customer_plan, temp_rm_vehicle_arrive_time):
                    repaired_state.customer_plan[customer_node] = best_scheme
                    if drone_id not in repaired_state.uav_assignments: repaired_state.uav_assignments[drone_id] = []
                    repaired_state.uav_assignments[drone_id].append(best_scheme)
                    if repaired_state.uav_cost is None: repaired_state.uav_cost = {}
                    repaired_state.uav_cost[customer_node] = best_cost
                    vehicle_task_data = update_vehicle_task(vehicle_task_data, best_scheme, vehicle_route)
                    insert_plan.append((customer, best_scheme, best_cost, 'traditional'))
                    success = True

            elif candidate['type'] == 'heuristic_swap':
                orig_scheme = candidate['scheme']['orig_scheme']
                new_scheme = candidate['scheme']['new_scheme']
                orig_cost = candidate['scheme']['orig_cost']
                new_cost = candidate['scheme']['new_cost']
                orig_drone_id, orig_launch_node, orig_customer, orig_recovery_node, orig_launch_vehicle, orig_recovery_vehicle = orig_scheme
                new_drone_id, new_launch_node, new_customer, new_recovery_node, new_launch_vehicle, new_recovery_vehicle = new_scheme
                delete_customer = orig_customer

                temp_customer_plan = {k: v for k, v in repaired_state.customer_plan.items()}
                delete_task_plan = temp_customer_plan[orig_customer]
                del temp_customer_plan[orig_customer]
                temp_customer_plan[orig_customer] = orig_scheme
                temp_customer_plan[new_customer] = new_scheme
                temp_rm_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(vehicle_route)

                if is_time_feasible(temp_customer_plan, temp_rm_vehicle_arrive_time):
                    del repaired_state.customer_plan[delete_customer]
                    repaired_state.customer_plan[orig_customer] = orig_scheme
                    repaired_state.customer_plan[new_customer] = new_scheme

                    if orig_drone_id not in repaired_state.uav_assignments: repaired_state.uav_assignments[orig_drone_id] = []
                    repaired_state.uav_assignments[orig_drone_id].append(orig_scheme)
                    if new_drone_id not in repaired_state.uav_assignments: repaired_state.uav_assignments[new_drone_id] = []
                    repaired_state.uav_assignments[new_drone_id].append(new_scheme)
                    del repaired_state.uav_cost[delete_customer]
                    repaired_state.uav_cost[orig_customer] = orig_cost
                    repaired_state.uav_cost[new_customer] = new_cost

                    vehicle_task_data = remove_vehicle_task(vehicle_task_data, delete_task_plan, vehicle_route)
                    orig_launch_time = temp_rm_vehicle_arrive_time[orig_launch_vehicle][orig_launch_node]
                    new_launch_time = temp_rm_vehicle_arrive_time[new_launch_vehicle][new_launch_node]
                    if orig_launch_time <= new_launch_time:
                        vehicle_task_data = update_vehicle_task(vehicle_task_data, orig_scheme, vehicle_route)
                        vehicle_task_data = update_vehicle_task(vehicle_task_data, new_scheme, vehicle_route)
                    else:
                        vehicle_task_data = update_vehicle_task(vehicle_task_data, new_scheme, vehicle_route)
                        vehicle_task_data = update_vehicle_task(vehicle_task_data, orig_scheme, vehicle_route)

                    insert_plan.append((delete_customer, orig_scheme, orig_cost, 'heuristic_swap'))
                    insert_plan.append((customer, new_scheme, new_cost, 'heuristic_swap'))
                    success = True

            elif candidate['type'] == 'vtp_expansion':
                vtp_node = candidate['vtp_node']
                vtp_insert_index = candidate['vtp_insert_index']
                vtp_insert_vehicle_id = candidate['vtp_insert_vehicle_id']
                best_scheme = candidate['scheme']
                drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id = best_scheme
                original_cost = self.drone_insert_cost(drone_id, customer_node, launch_node, recovery_node)

                temp_customer_plan = {k: v for k, v in repaired_state.customer_plan.items()}
                temp_customer_plan[customer_node] = best_scheme
                temp_vehicle_route = [route[:] for route in vehicle_route]
                temp_route = temp_vehicle_route[vtp_insert_vehicle_id - 1]
                temp_route.insert(vtp_insert_index, vtp_node)
                temp_vehicle_route[vtp_insert_vehicle_id - 1] = temp_route
                repaired_state.temp_vehicle_routes = temp_vehicle_route
                temp_rm_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(temp_vehicle_route)

                if is_time_feasible(temp_customer_plan, temp_rm_vehicle_arrive_time):
                    route = vehicle_route[vtp_insert_vehicle_id - 1]
                    route.insert(vtp_insert_index, vtp_node)
                    last_customer_node = route[vtp_insert_index - 1]
                    if vtp_insert_index == 1 or last_customer_node == self.DEPOT_nodeID:
                        last_drone_list = self.base_drone_assignment[vtp_insert_vehicle_id][:]
                    else:
                        last_drone_list = vehicle_task_data[vtp_insert_vehicle_id][last_customer_node].drone_list[:]

                    vehicle_task_data[vtp_insert_vehicle_id][vtp_node].drone_list = last_drone_list
                    vehicle_task_data[vtp_insert_vehicle_id][vtp_node].launch_drone_list = []
                    vehicle_task_data[vtp_insert_vehicle_id][vtp_node].recovery_drone_list = []
                    vehicle_task_data = update_vehicle_task(vehicle_task_data, best_scheme, vehicle_route)

                    repaired_state.customer_plan[customer_node] = best_scheme
                    if drone_id not in repaired_state.uav_assignments: repaired_state.uav_assignments[drone_id] = []
                    repaired_state.uav_assignments[drone_id].append(best_scheme)
                    if repaired_state.uav_cost is None: repaired_state.uav_cost = {}
                    repaired_state.uav_cost[customer_node] = original_cost

                    repaired_state.rm_empty_vehicle_route = [route[:] for route in repaired_state.vehicle_routes]
                    repaired_state.rm_empty_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(repaired_state.rm_empty_vehicle_route)
                    insert_plan.append((customer, best_scheme, original_cost, 'vtp_expansion'))
                    success = True

            if not success:

                print(f"  > 客户 {customer} 的最优方案复检失败（时间约束不满足），修复中止。")
                repaired_state.repair_objective = float('inf')
                repaired_state.destroyed_customers_info = {}
                return repaired_state, insert_plan

            destroy_node.remove(customer)
            num_repaired += 1

        expected_customer_count = len(state.customer_plan) + len(state.destroyed_customers_info)
        actual_customer_count = len(repaired_state.customer_plan)

        if actual_customer_count < expected_customer_count:
            print(f"  > 修复数量校验失败：预期 {expected_customer_count} 个客户点，"
                  f"实际修复后只有 {actual_customer_count} 个，标记为无效解。")
            repaired_state.repair_objective = float('inf')
            repaired_state.destroyed_customers_info = {}
            return repaired_state, insert_plan

        repaired_state._total_cost = repaired_state.win_total_objective()
        repaired_state.destroyed_customers_info = {}

        return repaired_state, insert_plan

    def noise_regret_insertion(self, state, strategic_bonus=0, num_destroyed=1, force_vtp_mode=False):

        repaired_state = state.fast_copy()
        repaired_state.repair_objective = 0
        destroy_node = list(state.destroyed_customers_info.keys())
        insert_plan = []

        rcs_k = 3
        noise_temperature = 1.0
        force_vtp_mode = True
        if force_vtp_mode:
            num_repaired = 0
            while len(destroy_node) > 0:

                try:
                    current_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(repaired_state.vehicle_routes)
                    base_total_cost, uav_tw_violation_cost, total_cost_dict = calculate_window_cost(
                        repaired_state.customer_plan, repaired_state.uav_cost, current_arrive_time,
                        self.vehicle, self.customer_time_windows_h,
                        self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node
                    )
                except Exception:
                    base_total_cost = float('inf')

                num_repaired = num_destroyed - len(destroy_node)
                tactical_multiplier = (len(destroy_node)) / num_destroyed
                final_bonus = strategic_bonus * tactical_multiplier * 0.3

                vehicle_route = repaired_state.vehicle_routes
                vehicle_task_data = repaired_state.vehicle_task_data
                vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(repaired_state.vehicle_routes)

                all_candidates = []
                customer_candidates = []
                global_best_moves = []

                for customer in destroy_node:

                    customer_specific_candidates = []
                    traditional_result,is_heuristic_swap = self._evaluate_traditional_insertion(customer, vehicle_route, vehicle_task_data, vehicle_arrive_time,base_total_cost, uav_tw_violation_cost, total_cost_dict, repaired_state)

                    if traditional_result[0] or traditional_result[1] is not None:
                        traditional_cost, traditional_scheme = traditional_result
                        if is_heuristic_swap:

                            current_total_cost = traditional_scheme['total_cost']
                            deta_total_cost = traditional_scheme['win_cost']
                            customer_candidates.append({
                                'customer': customer,
                                'scheme': traditional_scheme,
                                'cost': traditional_cost,
                                'win_cost': deta_total_cost,
                                'total_cost': current_total_cost,
                                'type': 'heuristic_swap',
                                'vtp_node': None
                            })
                        else:

                            current_total_cost = traditional_cost + base_total_cost
                            customer_candidates.append({
                                'customer': customer,
                                'scheme': traditional_scheme,
                                'cost': traditional_cost,
                                'win_cost': traditional_cost,
                                'total_cost': current_total_cost,
                                'type': 'traditional',
                                'vtp_node': None
                            })
                    else:

                        customer_candidates.append({
                            'customer': customer,
                            'scheme': None,
                            'cost': float('inf'),
                            'total_cost': float('inf'),
                            'type': 'traditional',
                            'vtp_node': None
                        })

                    vtp_result,vtp_infor = self._evaluate_vtp_expansion_insertion(customer, vehicle_route, vehicle_task_data, vehicle_arrive_time, repaired_state)

                    if vtp_infor is not None:
                        vtp_node = vtp_infor[0]
                        vtp_insert_vehicle_id = vtp_infor[1]
                        vtp_insert_index = vtp_infor[2]
                        if vtp_result is not None:
                            vtp_cost, vtp_scheme = vtp_result

                            adjusted_cost = vtp_cost - final_bonus

                            customer_candidates.append({
                                'customer': customer,
                                'scheme': vtp_scheme,
                                'cost': adjusted_cost,
                                'total_cost': vtp_cost,
                                'type': 'vtp_expansion',
                                'vtp_node': vtp_node,
                                'vtp_insert_vehicle_id': vtp_insert_vehicle_id,
                                'vtp_insert_index': vtp_insert_index,
                                'original_cost': vtp_cost
                            })
                    if customer_candidates:
                        customer_candidates = [item for item in customer_candidates if item['scheme'] is not None]
                        if customer_candidates:
                            customer_candidates.sort(key=lambda x: x['total_cost'])
                            best_move_for_this_customer = customer_candidates[0]
                            global_best_moves.append(best_move_for_this_customer)
                    customer_candidates = []
                if not global_best_moves:
                    print('在噪声的修复策略中，无法为剩余客户找到任何可行位置')
                    repaired_state.repair_objective = float('inf')
                    repaired_state.destroyed_customers_info = {}
                    return repaired_state, insert_plan
                global_best_moves.sort(key=lambda x: x['total_cost'])

                execution_queue = self.weighted_choice_sub(global_best_moves, rcs_k)

                success = False

                for candidate in execution_queue:
                    customer = candidate['customer']

                    if candidate['type'] == 'traditional':

                        customer = candidate['customer']
                        best_scheme = candidate['scheme']
                        best_cost = self.drone_insert_cost(best_scheme[0], best_scheme[2], best_scheme[1], best_scheme[3])

                        drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id = best_scheme

                        temp_customer_plan = {k: v for k, v in repaired_state.customer_plan.items()}
                        temp_customer_plan[customer_node] = best_scheme
                        temp_rm_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(vehicle_route)

                        if not is_time_feasible(temp_customer_plan, temp_rm_vehicle_arrive_time):
                            print(f"传统方案时间约束不满足，尝试下一个候选方案")
                            continue

                        repaired_state.customer_plan[customer_node] = best_scheme

                        if drone_id not in repaired_state.uav_assignments:
                            repaired_state.uav_assignments[drone_id] = []
                        repaired_state.uav_assignments[drone_id].append(best_scheme)

                        if repaired_state.uav_cost is None:
                            repaired_state.uav_cost = {}
                        repaired_state.uav_cost[customer_node] = best_cost

                        vehicle_task_data = update_vehicle_task(
                            vehicle_task_data, best_scheme, vehicle_route
                        )

                        insert_plan.append((customer, best_scheme, best_cost, 'traditional'))

                        success = True
                        break

                    elif candidate['type'] == 'heuristic_swap':
                        best_cost = candidate['cost']
                        print(f"尝试使用启发式交换方案插入客户点 {customer}，成本: {best_cost:.2f}")

                        orig_scheme = candidate['scheme']['orig_scheme']
                        new_scheme = candidate['scheme']['new_scheme']
                        orig_cost = candidate['scheme']['orig_cost']
                        new_cost = candidate['scheme']['new_cost']
                        orig_plan = candidate['scheme']['orig_plan']
                        new_plan = candidate['scheme']['new_plan']

                        orig_drone_id, orig_launch_node, orig_customer, orig_recovery_node, orig_launch_vehicle, orig_recovery_vehicle = orig_scheme
                        new_drone_id, new_launch_node, new_customer, new_recovery_node, new_launch_vehicle, new_recovery_vehicle = new_scheme
                        customer = new_customer
                        delete_customer = orig_customer

                        temp_customer_plan = {k: v for k, v in repaired_state.customer_plan.items()}
                        delete_task_plan = temp_customer_plan[orig_customer]
                        del temp_customer_plan[orig_customer]
                        temp_customer_plan[orig_customer] = orig_scheme
                        temp_customer_plan[new_customer] = new_scheme
                        temp_rm_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(vehicle_route)
                        if not is_time_feasible(temp_customer_plan, temp_rm_vehicle_arrive_time):
                            print(f"启发式交换方案时间约束不满足，尝试下一个候选方案")
                            continue
                        else:

                            del repaired_state.customer_plan[delete_customer]
                            repaired_state.customer_plan[orig_customer] = orig_scheme
                            repaired_state.customer_plan[new_customer] = new_scheme

                            if orig_drone_id not in repaired_state.uav_assignments:
                                repaired_state.uav_assignments[orig_drone_id] = []
                            repaired_state.uav_assignments[orig_drone_id].append(orig_scheme)
                            if new_drone_id not in repaired_state.uav_assignments:
                                repaired_state.uav_assignments[new_drone_id] = []
                            repaired_state.uav_assignments[new_drone_id].append(new_scheme)

                            del repaired_state.uav_cost[delete_customer]
                            repaired_state.uav_cost[orig_customer] = orig_cost
                            repaired_state.uav_cost[new_customer] = new_cost

                            vehicle_task_data = remove_vehicle_task(vehicle_task_data, delete_task_plan, vehicle_route)
                            orig_launch_time = temp_rm_vehicle_arrive_time[orig_launch_vehicle][orig_launch_node]
                            new_launch_time = temp_rm_vehicle_arrive_time[new_launch_vehicle][new_launch_node]
                            if orig_launch_time <= new_launch_time:
                                vehicle_task_data = update_vehicle_task(vehicle_task_data, orig_scheme, vehicle_route)
                                vehicle_task_data = update_vehicle_task(vehicle_task_data, new_scheme, vehicle_route)
                            else:
                                vehicle_task_data = update_vehicle_task(vehicle_task_data, new_scheme, vehicle_route)
                                vehicle_task_data = update_vehicle_task(vehicle_task_data, orig_scheme, vehicle_route)

                            insert_plan.append((delete_customer, orig_scheme, orig_cost, 'heuristic_swap'))
                            insert_plan.append((customer, new_scheme, new_cost, 'heuristic_swap'))
                            success = True
                            break

                    elif candidate['type'] == 'vtp_expansion':

                        customer = candidate['customer']
                        vtp_node = candidate['vtp_node']
                        vtp_insert_index = candidate['vtp_insert_index']
                        vtp_insert_vehicle_id = candidate['vtp_insert_vehicle_id']
                        best_scheme = candidate['scheme']

                        drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id = best_scheme
                        original_cost = self.drone_insert_cost(drone_id, customer_node, launch_node, recovery_node)

                        temp_customer_plan = {k: v for k, v in repaired_state.customer_plan.items()}
                        temp_customer_plan[customer_node] = best_scheme

                        temp_vehicle_route = [route[:] for route in vehicle_route]
                        temp_route = temp_vehicle_route[vtp_insert_vehicle_id - 1]
                        temp_route.insert(vtp_insert_index, vtp_node)
                        temp_vehicle_route[vtp_insert_vehicle_id - 1] = temp_route
                        repaired_state.temp_vehicle_routes = temp_vehicle_route

                        temp_rm_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(temp_vehicle_route)

                        time_feasible = is_time_feasible(temp_customer_plan, temp_rm_vehicle_arrive_time)

                        if not time_feasible:

                            print(f"VTP扩展方案时间约束不满足，尝试下一个候选方案")
                            continue
                        else:

                            route = vehicle_route[vtp_insert_vehicle_id - 1]

                            route.insert(vtp_insert_index, vtp_node)

                            last_customer_node = route[vtp_insert_index - 1]

                            if vtp_insert_index == 1 or last_customer_node == self.DEPOT_nodeID:
                                last_drone_list = self.base_drone_assignment[vtp_insert_vehicle_id][:]
                            else:
                                last_drone_list = vehicle_task_data[vtp_insert_vehicle_id][last_customer_node].drone_list[:]

                            vehicle_task_data[vtp_insert_vehicle_id][vtp_node].drone_list = last_drone_list
                            vehicle_task_data[vtp_insert_vehicle_id][vtp_node].launch_drone_list = []
                            vehicle_task_data[vtp_insert_vehicle_id][vtp_node].recovery_drone_list = []

                            vehicle_task_data = update_vehicle_task(vehicle_task_data, best_scheme, vehicle_route)

                            repaired_state.customer_plan[customer_node] = best_scheme

                            if drone_id not in repaired_state.uav_assignments:
                                repaired_state.uav_assignments[drone_id] = []
                            repaired_state.uav_assignments[drone_id].append(best_scheme)

                            if repaired_state.uav_cost is None:
                                repaired_state.uav_cost = {}
                            repaired_state.uav_cost[customer_node] = original_cost

                            repaired_state.rm_empty_vehicle_route = [route[:] for route in repaired_state.vehicle_routes]
                            repaired_state.rm_empty_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(repaired_state.rm_empty_vehicle_route)

                            insert_plan.append((customer, best_scheme, original_cost, 'vtp_expansion'))

                            success = True
                            break

                if not success:
                    print(f"客户点 {customer} 的所有候选方案都不满足约束，跳过")
                    repaired_state.repair_objective = float('inf')

                    repaired_state.destroyed_customers_info = {}
                    return repaired_state, insert_plan

                if customer in destroy_node:
                    destroy_node.remove(customer)

                num_repaired += 1

        repaired_state._total_cost = repaired_state.win_total_objective()

        repaired_state.destroyed_customers_info = {}
        print(f'修复策略完成，修复后总成本计算完成')
        print(f"修复后总成本: {repaired_state._total_cost}")

        return repaired_state, insert_plan
    def weighted_choice_sub(self, candidates, k_limit):

        if not candidates:
            return None, []

        limit = min(len(candidates), k_limit)
        pool = candidates[:limit]
        backup = candidates[limit:]

        weights = [math.exp(-0.5 * i) for i in range(limit)]

        total_w = sum(weights)
        probs = [w / total_w for w in weights]

        r = float(self.rng.random())
        cumulative_p = 0.0
        selected_index = 0
        for i, p in enumerate(probs):
            cumulative_p += p
            if r <= cumulative_p:
                selected_index = i
                break

        chosen_one = pool[selected_index]

        remaining_pool = [c for i, c in enumerate(pool) if i != selected_index]

        execution_queue = [chosen_one] + remaining_pool + backup

        return execution_queue
    def _find_k_best_vehicle_for_new_vtp(self, vtp_new, state, k):

        all_insertion_options = []

        for vehicle_idx, route in enumerate(state.vehicle_routes):
            vehicle_id = vehicle_idx + 1
            if len(route) < 2: continue

            for i in range(1, len(route)):
                prev_node = route[i - 1]
                next_node = route[i]
                try:
                    delta_cost = self.veh_distance[vehicle_id][prev_node][vtp_new] + \
                                self.veh_distance[vehicle_id][vtp_new][next_node] - \
                                self.veh_distance[vehicle_id][prev_node][next_node]

                    all_insertion_options.append((vehicle_id, i, delta_cost))

                except KeyError:
                    continue

        all_insertion_options.sort(key=lambda x: x[2])

        return all_insertion_options[:k]

    def _calculate_synergy_score(self, opt, neighbors, all_vtps, state, vehicle_task_data, k_route_nodes=5):

        synergy_score = 0.0
        vtp_insert_vehicle_id = opt.get('vtp_insert_vehicle_id')
        vtp_insert_index = opt.get('vtp_insert_index')
        vehicle_routes = [route[:] for route in state.vehicle_routes]
        try:

            if opt.get('type') != 'vtp_expansion':
                return 0.0

            customer_id = opt.get('customer')
            if customer_id is None:
                scheme = opt.get('scheme')
                if scheme and len(scheme) >= 3:
                    customer_id = scheme[2]

            if customer_id is None or customer_id not in self.node:
                return 0.0

            customer_node = self.node[customer_id]
            customer_lat = customer_node.latDeg
            customer_lon = customer_node.lonDeg
            customer_alt = customer_node.altMeters

            vtp_node = opt.get('vtp_node')
            vtp_insert_vehicle_id = opt.get('vtp_insert_vehicle_id')

            if vtp_node is None or vtp_insert_vehicle_id is None:
                return 0.0

            if vtp_node not in self.node:
                return 0.0

            vtp_node_obj = self.node[vtp_node]
            vtp_lat = vtp_node_obj.latDeg
            vtp_lon = vtp_node_obj.lonDeg
            vtp_alt = vtp_node_obj.altMeters

            drone_count = 0
            temp_vehicle_task_data = deep_copy_vehicle_task_data(vehicle_task_data)

            last_customer_node = vehicle_routes[vtp_insert_vehicle_id - 1][vtp_insert_index - 1]
            if vtp_insert_index == 1 or last_customer_node == self.DEPOT_nodeID:
                drone_count = len(self.base_drone_assignment[vtp_insert_vehicle_id])
            else:
                drone_count = len(temp_vehicle_task_data[vtp_insert_vehicle_id][last_customer_node].drone_list)

            if drone_count == 0:
                print('无人机数量获取失败，使用默认值')

            route_proximity_score = 0.0
            route_node_count = 0
            if isinstance(state.vehicle_routes, list):
                route_idx = vtp_insert_vehicle_id - 1
                if 0 <= route_idx < len(state.vehicle_routes):
                    target_route = state.vehicle_routes[route_idx]
                    vtp_insert_idx = opt.get('vtp_insert_index', 0)

                    start_idx = max(0, vtp_insert_idx - k_route_nodes // 2)
                    end_idx = min(len(target_route), vtp_insert_idx + k_route_nodes // 2 + 1)

                    for i in range(start_idx, end_idx):
                        if i == vtp_insert_idx:
                            continue

                        route_node_id = target_route[i]
                        if route_node_id in self.node:
                            route_node = self.node[route_node_id]
                            route_node_lat = route_node.latDeg
                            route_node_lon = route_node.lonDeg
                            route_node_alt = route_node.altMeters

                            lat_diff = vtp_lat - route_node_lat
                            lon_diff = vtp_lon - route_node_lon
                            alt_diff = vtp_alt - route_node_alt
                            route_distance = np.sqrt(lat_diff**2 + lon_diff**2 + alt_diff**2)

                            position_weight = 1.0 / (1.0 + abs(i - vtp_insert_idx))
                            route_proximity_score += position_weight / (1.0 + route_distance)
                            route_node_count += 1

                    if route_node_count > 0:
                        route_proximity_score = route_proximity_score / route_node_count

            customer_proximity_score = 0.0
            if customer_id in self.node:
                lat_diff = vtp_lat - customer_lat
                lon_diff = vtp_lon - customer_lon
                alt_diff = vtp_alt - customer_alt
                customer_distance = np.sqrt(lat_diff**2 + lon_diff**2 + alt_diff**2)

                customer_proximity_score = 1.0 / (1.0 + customer_distance)

            destroyed_node_score = 0.0
            destroyed_node_count = 0
            if hasattr(state, 'destroyed_customers_info') and state.destroyed_customers_info:
                destroyed_node_list = list(state.destroyed_customers_info.keys())
                for destroyed_node_id in destroyed_node_list:
                    if destroyed_node_id == customer_id:
                        continue
                    if destroyed_node_id not in self.node:
                        continue

                    destroyed_node = self.node[destroyed_node_id]
                    destroyed_lat = destroyed_node.latDeg
                    destroyed_lon = destroyed_node.lonDeg
                    destroyed_alt = destroyed_node.altMeters

                    lat_diff = vtp_lat - destroyed_lat
                    lon_diff = vtp_lon - destroyed_lon
                    alt_diff = vtp_alt - destroyed_alt
                    destroyed_distance = np.sqrt(lat_diff**2 + lon_diff**2 + alt_diff**2)

                    destroyed_node_score += 1.0 / (1.0 + destroyed_distance)
                    destroyed_node_count += 1

                if destroyed_node_count > 0:
                    destroyed_node_score = destroyed_node_score / destroyed_node_count

            vtp_network_score = 0.0
            k_vtp_nodes = 5

            scheme = opt.get('scheme')
            is_launch_point = False
            is_recovery_point = False

            if scheme and len(scheme) >= 4:

                launch_node = scheme[1] if len(scheme) > 1 else None
                recovery_node = scheme[3] if len(scheme) > 3 else None

                if vtp_node == launch_node:
                    is_launch_point = True
                    is_vehicle_id = scheme[4]
                elif vtp_node == recovery_node:
                    is_recovery_point = True
                    is_vehicle_id = scheme[5]

            temp_vehicle_routes = [route[:] for route in state.vehicle_routes]
            if vtp_insert_index is not None and vtp_insert_vehicle_id is not None:
                route_idx = vtp_insert_vehicle_id - 1
                if 0 <= route_idx < len(temp_vehicle_routes):

                    if vtp_insert_index < len(temp_vehicle_routes[route_idx]):
                        if temp_vehicle_routes[route_idx][vtp_insert_index] != vtp_node:
                            temp_vehicle_routes[route_idx].insert(vtp_insert_index, vtp_node)
                    else:
                        temp_vehicle_routes[route_idx].append(vtp_node)

            temp_vehicle_arrive_time = state.calculate_rm_empty_vehicle_arrive_time(temp_vehicle_routes)

            vtp_arrive_time = None
            if vtp_insert_vehicle_id in temp_vehicle_arrive_time:
                if vtp_node in temp_vehicle_arrive_time[vtp_insert_vehicle_id]:
                    vtp_arrive_time = temp_vehicle_arrive_time[vtp_insert_vehicle_id][vtp_node]

            candidate_nodes = []

            for vehicle_id, route in enumerate(state.vehicle_routes):
                vehicle_id = vehicle_id + 1
                if vehicle_id == is_vehicle_id:
                    continue
                if vehicle_id not in temp_vehicle_arrive_time:
                    continue

                vehicle_arrive_time_dict = temp_vehicle_arrive_time[vehicle_id]

                for node_id in route:

                    if node_id == self.DEPOT_nodeID:
                        continue
                    if node_id == vtp_node:
                        continue

                    if node_id not in vehicle_arrive_time_dict:
                        continue

                    node_arrive_time = vehicle_arrive_time_dict[node_id]

                    time_valid = False
                    if is_launch_point:

                        time_valid = node_arrive_time > vtp_arrive_time
                    elif is_recovery_point:

                        time_valid = node_arrive_time < vtp_arrive_time
                    else:

                        time_valid = True

                    if not time_valid:
                        continue

                    if node_id in self.node:

                        node_obj = self.node[node_id]
                        node_lat = node_obj.latDeg
                        node_lon = node_obj.lonDeg
                        node_alt = node_obj.altMeters

                        lat_diff = vtp_lat - node_lat
                        lon_diff = vtp_lon - node_lon
                        alt_diff = vtp_alt - node_alt
                        distance = np.sqrt(lat_diff**2 + lon_diff**2 + alt_diff**2)

                        candidate_nodes.append({
                            'node_id': node_id,
                            'distance': distance,
                            'arrive_time': node_arrive_time,
                            'vehicle_id': vehicle_id
                        })

            unique_nodes = {}
            for node_info in candidate_nodes:
                if node_info['node_id'] == self.DEPOT_nodeID:
                    continue
                node_id = node_info['node_id']
                if node_id not in unique_nodes:
                    unique_nodes[node_id] = node_info

            unique_candidate_nodes = list(unique_nodes.values())
            unique_candidate_nodes.sort(key=lambda x: x['distance'])
            selected_nodes = unique_candidate_nodes[:k_vtp_nodes]

            if selected_nodes:
                for node_info in selected_nodes:
                    distance = node_info['distance']

                    time_diff = abs(node_info['arrive_time'] - vtp_arrive_time)
                    time_factor = 1.0 / (1.0 + time_diff)
                    distance_factor = 1.0 / (1.0 + distance)
                    vtp_network_score += distance_factor * time_factor

                vtp_network_score = vtp_network_score / len(selected_nodes)

            base_score = 0.5 * (1.0 + np.log(1.0 + drone_count))
            synergy_score = (base_score +
                            0.20 * route_proximity_score +
                            0.25 * customer_proximity_score +
                            0.20 * destroyed_node_score +
                            0.15 * vtp_network_score)
            return synergy_score

        except Exception as e:
            print(f"Error calculating synergy score: {e}")
            import traceback
            traceback.print_exc()
            return 0.0

        return synergy_score

    def _calculate_partner_score(self, opt, all_vtps, state):

        return 0.0

    def _find_k_nearest_unassigned(self, customer_id, k, destroy_node):

        if customer_id not in self.node:
            return []

        target_node = self.node[customer_id]
        target_lat = target_node.latDeg
        target_lon = target_node.lonDeg
        target_alt = target_node.altMeters

        def calculate_distance(other_id):
            if other_id not in self.node:
                return float('inf')
            other_node = self.node[other_id]

            lat_diff = target_lat - other_node.latDeg
            lon_diff = target_lon - other_node.lonDeg
            alt_diff = target_alt - other_node.altMeters
            distance = np.sqrt(lat_diff**2 + lon_diff**2 + alt_diff**2)
            return distance

        candidates = [x for x in destroy_node if x != customer_id]

        if not candidates:
            return []

        sorted_candidates = sorted(candidates, key=calculate_distance)
        return sorted_candidates[:k]

    def _find_k_best_vehicles_for_shared_vtp(self, vtp_shared, state, k):

        all_insertion_options = []

        for vehicle_idx, route in enumerate(state.vehicle_routes):
            vehicle_id = vehicle_idx + 1

            if vtp_shared not in route:
                if len(route) < 2: continue

                best_idx_for_this_vehicle = -1
                min_delta_for_this_vehicle = float('inf')

                for i in range(1, len(route)):
                    prev_node = route[i - 1]
                    next_node = route[i]
                    try:
                        delta_cost = self.veh_distance[vehicle_id][prev_node][vtp_shared] + \
                                    self.veh_distance[vehicle_id][vtp_shared][next_node] - \
                                    self.veh_distance[vehicle_id][prev_node][next_node]
                        delta_cost = delta_cost * self.vehicle[vehicle_id].per_cost
                        all_insertion_options.append((vehicle_id, i, delta_cost))

                    except KeyError:
                        continue

        all_insertion_options.sort(key=lambda x: x[2])

        return all_insertion_options[:k]

    def _create_temp_state_with_new_vtp(self, state, vtp_new, veh_id, insert_idx):

        temp_state = state.fast_copy()
        temp_route = temp_state.vehicle_routes[veh_id - 1]
        temp_route.insert(insert_idx, vtp_new)

        try:
            last_customer_node = temp_route[insert_idx - 1]
            if insert_idx == 1 or last_customer_node == self.DEPOT_nodeID:
                last_drone_list = self.base_drone_assignment[veh_id][:]
            else:
                last_drone_list = temp_state.vehicle_task_data[veh_id][last_customer_node].drone_list[:]

            from task_data import TaskData
            temp_state.vehicle_task_data[veh_id][vtp_new] = TaskData(drone_list=last_drone_list, launch_drone_list=[], recovery_drone_list=[])

            return temp_state, True
        except Exception as e:
            print(f"  > 警告: _create_temp_state_with_new_vtp 初始化 {vtp_new} 失败: {e}")
            return temp_state, False

    def _evaluate_insertion_with_specific_vtp(self, customer, vtp_new, vtp_info, temp_state, temp_arrive_time):

        min_cost = float('inf')
        best_scheme = None
        (launch_veh_id, launch_idx) = vtp_info

        new_launch_time = temp_arrive_time[launch_veh_id][vtp_new]
        for rec_veh_id_idx, rec_route in enumerate(temp_state.vehicle_routes):
            rec_veh_id = rec_veh_id_idx + 1
            for rec_vtp in rec_route[1:-1]:
                if rec_vtp == vtp_new and launch_veh_id == rec_veh_id: continue

                if temp_arrive_time[rec_veh_id][rec_vtp] > new_launch_time:
                    for drone_id in self.V:

                        cost = self.drone_insert_cost(drone_id, customer, vtp_new, rec_vtp)
                        if cost < min_cost:
                            min_cost = cost
                            best_scheme = (drone_id, vtp_new, customer, rec_vtp, launch_veh_id, rec_veh_id)

        new_rec_time = temp_arrive_time[launch_veh_id][vtp_new]
        for launch_veh_id_idx, launch_route in enumerate(temp_state.vehicle_routes):
            launch_veh_id_old = launch_veh_id_idx + 1
            for launch_vtp in launch_route[1:-1]:
                if launch_vtp == vtp_new and launch_veh_id_old == launch_veh_id: continue

                if new_rec_time > temp_arrive_time[launch_veh_id_old][launch_vtp]:
                    for drone_id in self.V:

                        cost = self.drone_insert_cost(drone_id, customer, launch_vtp, vtp_new)
                        if cost < min_cost:
                            min_cost = cost
                            best_scheme = (drone_id, launch_vtp, customer, vtp_new, launch_veh_id_old, launch_veh_id)

        if best_scheme:
            return min_cost, best_scheme
        else:
            return float('inf'), None

    def _get_all_candidate_new_vtps(self, customers, state):

        used_vtps = {node for route in state.vehicle_routes for node in route[1:-1]}
        candidate_vtps = set()
        for customer in customers:
            K_NEIGHBORS = 20
            neighbors = self.map_cluster_vtp_dict.get(customer, [])[:K_NEIGHBORS]

            for vtp in neighbors:
                if vtp not in used_vtps:
                    candidate_vtps.add(vtp)
        return list(candidate_vtps)

    def _find_best_vehicle_for_new_vtp(self, vtp_new, repaired_state):

        best_vehicle_id = -1
        best_insert_idx = -1
        min_overall_delta = float('inf')
        epsilon = 1e-6

        for vehicle_idx, route in enumerate(repaired_state.vehicle_routes):
            vehicle_id = vehicle_idx + 1

            if len(route) < 2:
                continue

            for i in range(1, len(route)):
                prev_node = route[i - 1]
                next_node = route[i]

                try:

                    delta_cost = self.veh_distance[vehicle_id][prev_node][vtp_new] + \
                                self.veh_distance[vehicle_id][vtp_new][next_node] - \
                                self.veh_distance[vehicle_id][prev_node][next_node]
                    delta_cost = delta_cost * self.vehicle[vehicle_id].per_cost

                    if delta_cost < min_overall_delta:
                        min_overall_delta = delta_cost
                        best_vehicle_id = vehicle_id
                        best_insert_idx = i

                except (KeyError, IndexError) as e:

                    continue

        if min_overall_delta == float('inf'):
            print(f"  > 警告: 无法为新VTP {vtp_new} 找到任何可插入的车辆路径。")
            return -1, -1, float('inf')

        return best_vehicle_id, best_insert_idx, min_overall_delta

    def _create_temp_state_with_new_vtp(self, state, vtp_new, veh_id, insert_idx):

        temp_state = state.fast_copy()

        try:

            route_index = veh_id - 1

            if route_index < 0 or route_index >= len(temp_state.vehicle_routes):
                print(f"  > 警告: _create_temp_state - 车辆索引 {route_index} (ID: {veh_id}) 无效。")
                return state, False

            temp_route = temp_state.vehicle_routes[route_index]

            if insert_idx < 1 or insert_idx > len(temp_route):
                print(f"  > 警告: _create_temp_state - 插入索引 {insert_idx} 在路线 {route} 中无效。")
                return state, False

            temp_route.insert(insert_idx, vtp_new)

            prev_node = temp_route[insert_idx - 1]

            initial_drone_list = []
            if insert_idx == 1 or prev_node == self.DEPOT_nodeID:

                initial_drone_list = self.base_drone_assignment.get(veh_id, [])[:]
            else:

                prev_task_data = temp_state.vehicle_task_data.get(veh_id, {}).get(prev_node)
                if prev_task_data and hasattr(prev_task_data, 'drone_list'):
                    initial_drone_list = prev_task_data.drone_list[:]
                else:
                    print(f"  > 警告: _create_temp_state - 无法找到前序节点 {prev_node} 的 drone_list。")

                    initial_drone_list = []

            if veh_id not in temp_state.vehicle_task_data:
                temp_state.vehicle_task_data[veh_id] = {}

            temp_state.vehicle_task_data[veh_id][vtp_new].drone_list = initial_drone_list
            temp_state.vehicle_task_data[veh_id][vtp_new].launch_drone_list = []
            temp_state.vehicle_task_data[veh_id][vtp_new].recovery_drone_list = []
            temp_vehicle_time = temp_state.calculate_rm_empty_vehicle_arrive_time(temp_state.vehicle_routes)

            if is_time_feasible(temp_state.customer_plan, temp_vehicle_time):
                return temp_state, True
            else:
                return state, False

        except Exception as e:
            import traceback
            print(f"  > 严重错误: _create_temp_state_with_new_vtp 失败: {e}")
            traceback.print_exc()
            return state, False

    def _evaluate_insertion_with_specific_vtp(self, customer,
                                            vtp_new, vtp_info,
                                            temp_state, temp_arrive_time):

        min_cost = float('inf')
        best_scheme = None

        (new_vtp_veh_id, new_vtp_idx) = vtp_info

        try:
            launch_node = vtp_new
            launch_veh_id = new_vtp_veh_id
            launch_route = temp_state.vehicle_routes[launch_veh_id - 1]
            launch_idx = new_vtp_idx
            launch_time = temp_arrive_time[launch_veh_id][launch_node]
            n_launch = len(launch_route)
        except Exception as e:

            pass
        else:

            for rec_veh_idx, rec_route in enumerate(temp_state.vehicle_routes):
                rec_veh_id = rec_veh_idx + 1
                n_rec = len(rec_route)

                for k in range(1, n_rec - 1):
                    recovery_node = rec_route[k]

                    if launch_veh_id == rec_veh_id and launch_node == recovery_node:
                        continue

                    try:
                        recovery_time = temp_arrive_time[rec_veh_id][recovery_node]
                    except KeyError:
                        continue

                    if recovery_time <= launch_time:
                        continue

                    for drone_id in self.V:

                        if drone_id not in temp_state.vehicle_task_data[launch_veh_id][launch_node].drone_list:
                            continue

                        conflict = False
                        if launch_veh_id == rec_veh_id:
                            rec_idx = k
                            for m in range(launch_idx + 1, rec_idx):
                                if drone_id in temp_state.vehicle_task_data[launch_veh_id][launch_route[m]].launch_drone_list:
                                    conflict = True; break
                        else:

                            for m in range(launch_idx + 1, n_launch - 1):
                                if drone_id in temp_state.vehicle_task_data[launch_veh_id][launch_route[m]].launch_drone_list:
                                    conflict = True; break
                            if conflict: continue

                            for m in range(1, k):
                                if drone_id in temp_state.vehicle_task_data[rec_veh_id][rec_route[m]].launch_drone_list:
                                    conflict = True; break

                        if conflict: continue

                        cost = self.drone_insert_cost(drone_id, customer, launch_node, recovery_node)
                        if cost is not None and cost < min_cost:
                            min_cost = cost
                            best_scheme = (drone_id, launch_node, customer, recovery_node, launch_veh_id, rec_veh_id)

        try:
            recovery_node = vtp_new
            rec_veh_id = new_vtp_veh_id
            rec_route = temp_state.vehicle_routes[rec_veh_id - 1]
            rec_idx = new_vtp_idx
            recovery_time = temp_arrive_time[rec_veh_id][recovery_node]
            n_recovery = len(rec_route)
        except Exception as e:

            pass
        else:

            for launch_veh_idx, launch_route in enumerate(temp_state.vehicle_routes):
                launch_veh_id = launch_veh_idx + 1
                n_launch = len(launch_route)

                for i in range(1, n_launch - 1):
                    launch_node = launch_route[i]

                    if launch_veh_id == rec_veh_id and launch_node == recovery_node:
                        continue

                    try:
                        launch_time = temp_arrive_time[launch_veh_id][launch_node]
                    except KeyError:
                        continue

                    if recovery_time <= launch_time:
                        continue

                    for drone_id in self.V:

                        if drone_id not in temp_state.vehicle_task_data[launch_veh_id][launch_node].drone_list:
                            continue

                        conflict = False
                        if launch_veh_id == rec_veh_id:
                            for m in range(i + 1, rec_idx):
                                if drone_id in temp_state.vehicle_task_data[launch_veh_id][launch_route[m]].launch_drone_list:
                                    conflict = True; break
                        else:

                            for m in range(i + 1, n_launch - 1):
                                if drone_id in temp_state.vehicle_task_data[launch_veh_id][launch_route[m]].launch_drone_list:
                                    conflict = True; break
                            if conflict: continue

                            for m in range(1, rec_idx):
                                if drone_id in temp_state.vehicle_task_data[rec_veh_id][rec_route[m]].launch_drone_list:
                                    conflict = True; break

                        if conflict: continue

                        cost = self.drone_insert_cost(drone_id, customer, launch_node, recovery_node)
                        if cost is not None and cost < min_cost:
                            min_cost = cost
                            best_scheme = (drone_id, launch_node, customer, recovery_node, launch_veh_id, rec_veh_id)

        if best_scheme:
            return min_cost, best_scheme
        else:
            return float('inf'), None

    def _calculate_vtp_benefits(self, vtp_new, vtp_info, state, customers_to_repair,temp_vtp_task_data):

        total_benefit = {}
        affected_customers = {}
        (veh_id, insert_idx) = vtp_info
        epsilon = 1e-6

        vehicle_route = [route[:] for route in state.vehicle_routes]

        vtp_task_data = temp_vtp_task_data

        temp_vehicle_route = [route[:] for route in vehicle_route]
        temp_route = temp_vehicle_route[veh_id - 1]
        temp_route.insert(insert_idx, vtp_new)
        temp_vehicle_route[veh_id - 1] = temp_route
        temp_rm_vehicle_arrive_time = state.calculate_rm_empty_vehicle_arrive_time(temp_vehicle_route)
        if not is_time_feasible(state.customer_plan, temp_rm_vehicle_arrive_time):
            return float('inf'), {}

        customer = customers_to_repair

        vehicle_id = vtp_info[0]
        insert_pos = vtp_info[1]
        vehicle_idx = vehicle_id - 1
        new_cost, new_scheme = self._calculate_vtp_expansion_cost(customer, vehicle_idx, insert_pos, vehicle_route, vtp_task_data, state, vtp_new)

        if new_scheme:
            total_benefit[customer] = new_cost
            affected_customers[customer] = (new_cost, new_scheme)

        affected_customers = sorted(affected_customers.items(), key=lambda x: x[1])
        return total_benefit, affected_customers

    def repair_vtp_centric(self, state, strategic_bonus=0, num_destroyed=1, force_vtp_mode=False):

        repaired_state = state.fast_copy()
        repaired_state.repair_objective = 0
        destroy_node = list(state.destroyed_customers_info.keys())
        insert_plan = []

        force_vtp_mode = True
        if force_vtp_mode:
            num_repaired = 0

            baseline_costs = {}
            vehicle_route = repaired_state.vehicle_routes
            vehicle_task_data = repaired_state.vehicle_task_data
            vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(vehicle_route)
            for customer in destroy_node:

                traditional_result, is_swap = self._evaluate_traditional_insertion(
                    customer, vehicle_route, vehicle_task_data,
                    vehicle_arrive_time, repaired_state,
                )
                if traditional_result and not is_swap:
                    baseline_costs[customer] = (traditional_result[0], traditional_result[1])
                else:
                    baseline_costs[customer] = (float('inf'), None)

            best_modification = None
            best_vtp_score = 0.0
            best_vtp_affected_customers = {}

            final_bonus = 0.0
            if num_destroyed > 0 and strategic_bonus > 0:
                tactical_multiplier = (num_destroyed - num_repaired) / max(num_destroyed, 1)
                final_bonus = strategic_bonus * tactical_multiplier

            print(f"  > [VTP中心决策]: (Bonus: {final_bonus:.2f})")

            candidate_new_vtps = self._get_all_candidate_new_vtps(destroy_node, repaired_state)
            print(f"  > [VTP中心-投资]: 评估 {len(candidate_new_vtps)} 个全新VTP...")

            K_position = 10
            for vtp_new in candidate_new_vtps:

                best_positions = self._find_k_best_vehicle_for_new_vtp(vtp_new, repaired_state,K_position)
                if not best_positions: continue

                for (veh_id, insert_idx, veh_delta_cost) in best_positions:

                    total_benefit, affected_customers = self._calculate_vtp_benefits(
                        vtp_new, (veh_id, insert_idx), repaired_state, destroy_node, baseline_costs
                    )

                    score = (total_benefit * (1 + final_bonus)) - veh_delta_cost

                    if score > best_vtp_score:
                        best_vtp_score = score
                        best_modification = {'vtp_node': vtp_new, 'veh_id': veh_id, 'idx': insert_idx, 'type': 'investment'}
                        best_vtp_affected_customers = affected_customers

            used_vtps_set = {node for route in repaired_state.vehicle_routes for node in route[1:-1]}
            print(f"  > [VTP中心-共享]: 评估 {len(used_vtps_set)} 个现有VTP的共享潜力...")
            K_BEST_POSITIONS = len(used_vtps_set)

            for vtp_shared in used_vtps_set:

                best_shared_positions = self._find_k_best_vehicles_for_shared_vtp(vtp_shared, repaired_state, K_BEST_POSITIONS)

                if not best_shared_positions: continue

                for (veh_id, insert_idx, veh_delta_cost) in best_shared_positions:

                    total_benefit, affected_customers = self._calculate_vtp_benefits(
                        vtp_shared, (veh_id, insert_idx), repaired_state, destroy_node, baseline_costs
                    )

                    score = (total_benefit * (1 + final_bonus)) - veh_delta_cost

                    if score > best_vtp_score:
                        best_vtp_score = score
                        best_modification = {'vtp_node': vtp_shared, 'veh_id': veh_id, 'idx': insert_idx, 'type': 'sharing'}
                        best_vtp_affected_customers = affected_customers

            if best_modification:
                vtp_node = best_modification['vtp_node']
                vtp_insert_vehicle_id = best_modification['veh_id']
                vtp_insert_index = best_modification['idx']

                print(f"  > [VTP中心决策]: {best_modification['type']} VTP {vtp_node} (车辆 {vtp_insert_vehicle_id}), 潜力分数: {best_vtp_score:.2f}, 批量修复 {len(best_vtp_affected_customers)} 个客户。")

                route = repaired_state.vehicle_routes[vtp_insert_vehicle_id - 1]
                route.insert(vtp_insert_index, vtp_node)

                last_customer_node = route[vtp_insert_index - 1]
                if vtp_insert_index == 1 or last_customer_node == self.DEPOT_nodeID:
                    last_drone_list = self.base_drone_assignment[vtp_insert_vehicle_id][:]
                else:
                    last_drone_list = vehicle_task_data[vtp_insert_vehicle_id][last_customer_node].drone_list[:]

                vehicle_task_data[vtp_insert_vehicle_id][vtp_node].drone_list = last_drone_list
                vehicle_task_data[vtp_insert_vehicle_id][vtp_node].launch_drone_list = []
                vehicle_task_data[vtp_insert_vehicle_id][vtp_node].recovery_drone_list = []

                sorted_affected_customers = sorted(
                    best_vtp_affected_customers.items(),
                    key=lambda item: baseline_costs.get(item[0], (float('inf'), None))[0] - item[1][0],
                    reverse=True
                )

                for customer, (real_cost, scheme) in sorted_affected_customers:
                    if customer in destroy_node:

                        if not is_constraints_satisfied(repaired_state, vehicle_task_data, scheme):
                            continue
                        vehicle_task_data = update_vehicle_task(vehicle_task_data, scheme, vehicle_route)
                        repaired_state.customer_plan[customer] = scheme
                        drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id = scheme
                        state.destroyed_customers_info.pop(customer)
                        if drone_id not in repaired_state.uav_assignments:
                            repaired_state.uav_assignments[drone_id] = []
                        repaired_state.uav_assignments[drone_id].append(scheme)
                        if repaired_state.uav_cost is None:
                            repaired_state.uav_cost = {}
                        repaired_state.uav_cost[customer] = self.drone_insert_cost(drone_id, customer_node, launch_node, recovery_node)
                        destroy_node.remove(customer)
                        num_repaired += 1
                        insert_plan.append((customer, scheme, real_cost, 'vtp_centric_batch'))

                if destroy_node:
                    print(f"  > [VTP中心收尾]: 仍有 {len(destroy_node)} 个客户待修复，转交标准贪婪或者后悔之策略插入修复...")

                    random_choice = self.rng.integers(1, 3)
                    if random_choice == 1:
                        repaired_state, insert_plan = self.repair_regret_insertion(repaired_state,strategic_bonus=0, num_destroyed=len(destroy_node), force_vtp_mode=True)
                    else:
                        repaired_state, insert_plan = self.repair_greedy_insertion(repaired_state,strategic_bonus=0, num_destroyed=len(destroy_node), force_vtp_mode=True)
                    return repaired_state, insert_plan
            else:

                repaired_state, insert_plan = self.repair_regret_insertion(repaired_state,strategic_bonus=0, num_destroyed=len(destroy_node), force_vtp_mode=True)

                return repaired_state, insert_plan

            return repaired_state, insert_plan

    def repair_k_insertion(self, state):

        repaired_state = state.fast_copy()
        destroy_node = list(state.destroyed_customers_info.keys())
        insert_plan = []

        print(f"快速K步修复：需要插入 {len(destroy_node)} 个客户点: {destroy_node}")

        k_steps = 3
        max_samples = 15
        candidate_limit = 6

        while len(destroy_node) > 0:
            print(f"当前剩余待插入节点: {destroy_node}")

            vehicle_route = repaired_state.vehicle_routes
            vehicle_task_data = repaired_state.vehicle_task_data
            vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(repaired_state.rm_empty_vehicle_route)

            if len(destroy_node) <= 3:
                print(f"剩余节点数({len(destroy_node)}) <= 3，使用贪婪策略")
                best_customer, best_scheme, best_cost = self._greedy_select_best_insertion(
                    destroy_node, vehicle_route, vehicle_task_data, vehicle_arrive_time, repaired_state
                )
                if best_customer is not None:

                    self._apply_insertion(repaired_state, best_customer, best_scheme, best_cost)
                    insert_plan.append((best_customer, best_scheme, best_cost))
                    destroy_node.remove(best_customer)
                    print(f"快速K步修复：成功插入客户点 {best_customer}，成本: {best_cost:.2f}")
                else:
                    print("快速K步修复：没有找到可行的插入方案")
                    break
            else:

                best_customer, best_scheme, best_cost = self._balanced_k_step_selection(
                    destroy_node, k_steps, max_samples, candidate_limit, vehicle_route, vehicle_task_data, vehicle_arrive_time, repaired_state
                )

                if best_customer is not None:

                    self._apply_insertion(repaired_state, best_customer, best_scheme, best_cost)
                    insert_plan.append((best_customer, best_scheme, best_cost))
                    destroy_node.remove(best_customer)
                    print(f"快速K步修复：成功插入客户点 {best_customer}，成本: {best_cost:.2f}")
                else:

                    print("快速K步策略失败，回退到贪婪策略")
                    best_customer, best_scheme, best_cost = self._greedy_select_best_insertion(
                        destroy_node, vehicle_route, vehicle_task_data, vehicle_arrive_time, repaired_state
                    )
                    if best_customer is not None:
                        self._apply_insertion(repaired_state, best_customer, best_scheme, best_cost)
                        insert_plan.append((best_customer, best_scheme, best_cost))
                        destroy_node.remove(best_customer)
                        print(f"快速K步修复：贪婪回退插入客户点 {best_customer}，成本: {best_cost:.2f}")
                    else:
                        print("快速K步修复：没有找到可行的插入方案")
                        break

        repaired_state._total_cost = repaired_state.update_calculate_plan_cost(repaired_state.uav_cost, repaired_state.vehicle_routes)
        print(f"快速K步修复完成：成功插入 {len(insert_plan)} 个客户点")
        return repaired_state, insert_plan

    def repair_vtp_insertion(self, state):

        repaired_state = state.fast_copy()
        destroy_node = list(state.destroyed_customers_info.keys())
        insert_plan = []

        print(f"VTP插入修复：需要插入 {len(destroy_node)} 个客户点: {destroy_node}")

        k_vtp_candidates = 10

        while len(destroy_node) > 0:
            print(f"当前剩余待插入节点: {destroy_node}")

            vehicle_route = repaired_state.vehicle_routes
            vehicle_task_data = repaired_state.vehicle_task_data
            vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(repaired_state.rm_empty_vehicle_route)

            best_customer = None
            best_scheme = None
            best_cost = float('inf')
            best_vtp_insertion = None

            for customer in destroy_node:

                candidate_vtps = self._get_nearest_vtp_candidates(customer, k_vtp_candidates, vehicle_route)

                for vtp_candidate in candidate_vtps:

                    insertion_costs = self._evaluate_vtp_insertion_costs(
                        vtp_candidate, customer, vehicle_route, vehicle_task_data, vehicle_arrive_time, repaired_state
                    )

                    for (vehicle_id, insert_position, total_cost, scheme) in insertion_costs:
                        if total_cost < best_cost:
                            best_cost = total_cost
                            best_scheme = scheme
                            best_customer = customer
                            best_vtp_insertion = (vtp_candidate, vehicle_id, insert_position)

            if best_customer is not None and best_vtp_insertion is not None:

                vtp_node, vehicle_id, insert_position = best_vtp_insertion

                route = repaired_state.vehicle_routes[vehicle_id]
                route.insert(insert_position, vtp_node)
                repaired_state.vehicle_routes[vehicle_id] = route

                self._apply_insertion(repaired_state, best_customer, best_scheme, best_cost)
                insert_plan.append((best_customer, best_scheme, best_cost))
                destroy_node.remove(best_customer)

                print(f"VTP插入修复：成功插入VTP节点 {vtp_node} 到车辆 {vehicle_id} 位置 {insert_position}")
                print(f"VTP插入修复：成功插入客户点 {best_customer}，总成本: {best_cost:.2f}")
            else:

                print("VTP插入策略失败，回退到传统贪婪策略")
                best_customer, best_scheme, best_cost = self._greedy_select_best_insertion(
                    destroy_node, vehicle_route, vehicle_task_data, vehicle_arrive_time, repaired_state, 0
                )
                if best_customer is not None:
                    self._apply_insertion(repaired_state, best_customer, best_scheme, best_cost)
                    insert_plan.append((best_customer, best_scheme, best_cost))
                    destroy_node.remove(best_customer)
                    print(f"VTP插入修复：贪婪回退插入客户点 {best_customer}，成本: {best_cost:.2f}")
                else:
                    print("VTP插入修复：没有找到可行的插入方案")
                    break

        repaired_state._total_cost = repaired_state.update_calculate_plan_cost(repaired_state.uav_cost, repaired_state.vehicle_routes)
        print(f"VTP插入修复完成：成功插入 {len(insert_plan)} 个客户点")
        return repaired_state, insert_plan

    def _get_nearest_vtp_candidates(self, customer, k, vehicle_route):

        customer_pos = np.array([
            self.node[customer].latDeg,
            self.node[customer].lonDeg,
            self.node[customer].altMeters
        ])

        vtp_candidates = []
        for vtp_id in self.A_vtp:

            vtp_in_route = False
            for route in vehicle_route:
                if vtp_id in route:
                    vtp_in_route = True
                    break

            if not vtp_in_route:
                vtp_pos = np.array([
                    self.node[vtp_id].latDeg,
                    self.node[vtp_id].lonDeg,
                    self.node[vtp_id].altMeters
                ])
                distance = np.linalg.norm(vtp_pos - customer_pos)
                vtp_candidates.append((vtp_id, distance))

        vtp_candidates.sort(key=lambda x: x[1])
        return [vtp_id for vtp_id, _ in vtp_candidates[:k]]

    def _evaluate_vtp_insertion_costs(self, vtp_node, customer, vehicle_route, vehicle_task_data, vehicle_arrive_time, repaired_state):

        insertion_costs = []

        for vehicle_id, route in vehicle_route.items():

            for insert_position in range(1, len(route)):

                vehicle_cost_increase = self._calculate_vehicle_cost_increase(
                    vehicle_id, route, insert_position, vtp_node
                )

                uav_cost = self._calculate_uav_cost_from_vtp(vtp_node, customer)

                total_cost = vehicle_cost_increase + uav_cost

                scheme = self._generate_vtp_insertion_scheme(
                    vtp_node, customer, vehicle_id, vehicle_route, vehicle_task_data
                )

                if scheme is not None:
                    insertion_costs.append((vehicle_id, insert_position, total_cost, scheme))

        return insertion_costs

    def _calculate_vehicle_cost_increase(self, vehicle_id, route, insert_position, vtp_node):

        try:
            if insert_position <= 0 or insert_position >= len(route):
                return float('inf')

            vehicle_id = vehicle_id + 1
            prev_node = route[insert_position - 1]
            next_node = route[insert_position]

            original_distance = self.veh_distance[vehicle_id][prev_node][next_node]

            new_distance = (self.veh_distance[vehicle_id][prev_node][vtp_node] +
                           self.veh_distance[vehicle_id][vtp_node][next_node])

            cost_increase = (new_distance - original_distance) * self.vehicle[vehicle_id].per_cost

            return cost_increase

        except Exception as e:
            return float('inf')

    def _calculate_uav_cost_from_vtp(self, vtp_node, customer):

        min_cost = float('inf')

        for drone_id in self.V:

            vtp_map_index = self.node[vtp_node].map_key
            customer_map_index = self.node[customer].map_key

            cost = self.uav_travel[drone_id][vtp_map_index][customer].totalDistance * self.vehicle[drone_id].per_cost
            min_cost = min(min_cost, cost)

        return min_cost

    def _update_vehicle_task_data_for_vtp(self, repaired_state, vtp_node, vehicle_id, insert_position):

        route = repaired_state.vehicle_routes[vehicle_id - 1]

        from task_data import VehicleTaskData

        vtp_task_data = VehicleTaskData()
        vtp_task_data.drone_list = list(self.V)
        vtp_task_data.launch_drone_list = list(self.V)

        if vehicle_id not in repaired_state.vehicle_task_data:
            repaired_state.vehicle_task_data[vehicle_id] = {}

        repaired_state.vehicle_task_data[vehicle_id][vtp_node] = vtp_task_data

    def _generate_vtp_insertion_scheme(self, vtp_node, customer, vehicle_id, vehicle_route, vehicle_task_data):

        if not self.V:
            return None

        drone_id = self.V[0]
        launch_node = vtp_node
        customer_node = customer
        recovery_node = vtp_node
        launch_vehicle_id = vehicle_id
        recovery_vehicle_id = vehicle_id

        return (drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id)

    def _greedy_select_best_insertion(self, destroy_node, vehicle_route, vehicle_task_data, vehicle_arrive_time, repaired_state, base_cost):

        best_customer = None
        best_scheme = None
        best_cost = float('inf')
        base_cost = 0

        for customer in destroy_node:

            traditional_result = self._evaluate_traditional_insertion(customer, vehicle_route, vehicle_task_data, vehicle_arrive_time, repaired_state)
            if traditional_result is not None:
                traditional_cost, traditional_scheme = traditional_result
                if traditional_cost < best_cost:
                    best_cost = traditional_cost
                    best_scheme = traditional_scheme
                    best_customer = customer

            vtp_cost, vtp_scheme = self._evaluate_vtp_expansion_insertion(customer, vehicle_route, vehicle_task_data, vehicle_arrive_time, repaired_state)
            if vtp_scheme is not None:
                if vtp_cost-base_cost < best_cost:
                    best_cost = vtp_cost
                    best_scheme = vtp_scheme
                    best_customer = customer

        return best_customer, best_scheme, best_cost

    def _regret_evaluate_traditional_insertion(self, customer, vehicle_route, vehicle_task_data, vehicle_arrive_time, base_total_cost, uav_tw_violation_cost, total_cost_dict, repaired_state=None):

        options = []
        is_heuristic_swap = False

        try:
            all_insert_position = self.get_all_insert_position(vehicle_route, vehicle_task_data, customer, vehicle_arrive_time)
            insert_plan = {}
            if all_insert_position:
                for drone_id, inert_positions in all_insert_position.items():
                    for inert_position in inert_positions:
                        temp_customer_plan = {k: v for k, v in repaired_state.customer_plan.items()}
                        temp_customer_cost = {k: v for k, v in repaired_state.uav_cost.items()}
                        launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id = inert_position
                        if customer_node in temp_customer_plan:
                            del temp_customer_plan[customer_node]
                        temp_customer_plan[customer_node] = (drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id)
                        if customer_node in temp_customer_cost:
                            del temp_customer_cost[customer_node]
                        temp_customer_cost[customer_node] = self.drone_insert_cost(drone_id, customer_node, launch_node, recovery_node)
                        temp_total_cost = sum(temp_customer_cost.values())

                        insert_plan[customer_node] = (drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id)
                        real_cost = self.drone_insert_cost(drone_id, customer_node, launch_node, recovery_node)
                        real_cost += calculate_customer_window_cost(insert_plan, self.vehicle, vehicle_arrive_time, self.customer_time_windows_h, self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node)
                        total_cost = real_cost + base_total_cost

                        insert_plan.pop(customer_node)
                        if real_cost is not None:
                            plan = (drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id)

                            options.append({
                                'customer': customer,
                                'eval_cost': real_cost,
                                'real_cost': real_cost,
                                'total_cost': total_cost,
                                'scheme': plan,
                                'type': 'traditional',
                                'extra_info': None
                            })

        except Exception as e:
            print(f"  > 警告: 在评估客户 {customer} 的直接插入方案时发生错误: {e}")

        initiate_heuristic_swap = (len(options) < 2)

        if initiate_heuristic_swap:
            print(f"  > 客户点 {customer} 直接插入方案不足，尝试启发式交换策略...")
            try:

                generator = DiverseRouteGenerator(self.node, self.DEPOT_nodeID, self.A_vtp, self.V, self.T, self.vehicle, self.uav_travel, self.veh_distance, self.veh_travel, self.vtp_coords, self.num_clusters, self.G_air, self.G_ground, self.air_matrix, self.ground_matrix, self.air_node_types, self.ground_node_types, self.A_c, self.xeee)
                is_heuristic_swap = True
                best_orig_y, best_new_y, best_orig_cost, best_new_cost, best_orig_y_cijkdu_plan, best_new_y_cijkdu_plan = generator.greedy_insert_feasible_plan(
                    customer, vehicle_route, vehicle_arrive_time, vehicle_task_data, repaired_state.customer_plan
                )
                orig_customer = best_orig_y[2]
                new_customer = best_new_y[2]
                temp_delta_cost = repaired_state.uav_cost[orig_customer]
                delta_cost = best_orig_cost + best_new_cost - temp_delta_cost
                if best_orig_y is not None and best_new_y is not None:

                    real_cost = best_orig_cost + best_new_cost
                    orig_drone_id, orig_launch_node, orig_customer, orig_recovery_node, orig_launch_vehicle, orig_recovery_vehicle = best_orig_y
                    new_drone_id, new_launch_node, new_customer, new_recovery_node, new_launch_vehicle, new_recovery_vehicle = best_new_y

                    temp_customer_cost = {k: v for k, v in repaired_state.uav_cost.items()}

                    if orig_customer in temp_customer_cost:
                        temp_delta_cost = temp_customer_cost[orig_customer]
                        del temp_customer_cost[orig_customer]
                    temp_customer_cost[orig_customer] = best_orig_cost
                    temp_customer_cost[new_customer] = best_new_cost
                    temp_total_cost = sum(temp_customer_cost.values())

                    temp_orig_scheme = {}
                    temp_new_scheme = {}
                    total_swap_cost = best_orig_cost + best_new_cost
                    delete_customer = orig_customer
                    delete_traditional_cost = total_cost_dict.get(delete_customer, 0.0)
                    traditional_orig_scheme = best_orig_y
                    temp_orig_scheme[orig_customer] = traditional_orig_scheme
                    traditional_new_scheme = best_new_y
                    temp_new_scheme[new_customer] = traditional_new_scheme
                    traditional_orig_win_cost = total_cost_dict.get(traditional_orig_scheme, 0.0)
                    orig_cost = calculate_customer_window_cost(temp_orig_scheme, self.vehicle, vehicle_arrive_time, self.customer_time_windows_h, self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node)
                    new_cost = calculate_customer_window_cost(temp_new_scheme, self.vehicle, vehicle_arrive_time, self.customer_time_windows_h, self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node)
                    trad_total_cost = orig_cost + new_cost
                    current_total_cost = base_total_cost - traditional_orig_win_cost + trad_total_cost + best_orig_cost + best_new_cost
                    deta_total_cost = current_total_cost - base_total_cost
                    options.append(
                        {
                        'customer': customer,
                        'orig_scheme': best_orig_y,
                        'new_scheme': best_new_y,
                        'orig_cost': best_orig_cost,
                        'new_cost': best_new_cost,
                        'eval_cost': deta_total_cost,
                        'real_cost': deta_total_cost,
                        'delta_cost': deta_total_cost,
                        'total_cost': current_total_cost,
                        'type': 'heuristic_swap',
                        'extra_info': None,
                        'orig_plan_details': best_orig_y_cijkdu_plan,
                        'new_plan_details': best_new_y_cijkdu_plan
                    }
                    )
                    print(f"    - 找到启发式交换方案，总成本: {real_cost:.2f}")

            except Exception as e:
                print(f"  > 警告: 客户点 {customer} 启发式交换失败: {e}")

        return options, is_heuristic_swap

    def _evaluate_traditional_insertion(self, customer, vehicle_route, vehicle_task_data, vehicle_arrive_time, base_total_cost, uav_tw_violation_cost, total_cost_dict, repaired_state=None):

        is_heuristic_swap = False
        all_insert_position = self.get_all_insert_position(vehicle_route, vehicle_task_data, customer, vehicle_arrive_time)
        insert_plan = {}
        if all_insert_position is not None:
            best_scheme = None
            min_cost = float('inf')
            for drone_id, inert_positions in all_insert_position.items():
                for inert_position in inert_positions:
                    launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id = inert_position
                    insert_plan[customer_node] = (drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id)

                    if launch_node == recovery_node:
                        continue
                    insert_cost = self.drone_insert_cost(drone_id, customer_node, launch_node, recovery_node)
                    insert_cost += calculate_customer_window_cost(insert_plan, self.vehicle, vehicle_arrive_time, self.customer_time_windows_h, self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node)
                    if insert_cost < min_cost:
                        min_cost = insert_cost
                        best_scheme = (drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id)

                    insert_plan.pop(customer_node)

            if min_cost != float('inf'):
                return (min_cost, best_scheme), is_heuristic_swap

        if repaired_state is not None:
            try:

                generator = DiverseRouteGenerator(self.node, self.DEPOT_nodeID, self.A_vtp, self.V, self.T, self.vehicle, self.uav_travel, self.veh_distance, self.veh_travel, self.vtp_coords, self.num_clusters, self.G_air, self.G_ground, self.air_matrix, self.ground_matrix, self.air_node_types, self.ground_node_types, self.A_c, self.xeee)
                best_orig_y, best_new_y, best_orig_cost, best_new_cost, best_orig_y_cijkdu_plan, best_new_y_cijkdu_plan = generator.greedy_insert_feasible_plan(
                    customer, vehicle_route, vehicle_arrive_time, vehicle_task_data, repaired_state.customer_plan
                )
                orig_customer = best_orig_y[2]
                new_customer = best_new_y[2]
                temp_delta_cost = repaired_state.uav_cost[orig_customer]
                delta_cost = best_orig_cost + best_new_cost - temp_delta_cost
                if best_orig_y is not None and best_new_y is not None:

                    temp_orig_scheme = {}
                    temp_new_scheme = {}
                    total_swap_cost = best_orig_cost + best_new_cost
                    delete_customer = orig_customer
                    delete_traditional_cost = total_cost_dict.get(delete_customer, 0.0)
                    traditional_orig_scheme = best_orig_y
                    temp_orig_scheme[orig_customer] = traditional_orig_scheme
                    traditional_new_scheme = best_new_y
                    temp_new_scheme[new_customer] = traditional_new_scheme
                    traditional_orig_win_cost = total_cost_dict.get(traditional_orig_scheme, 0.0)
                    orig_cost = calculate_customer_window_cost(temp_orig_scheme, self.vehicle, vehicle_arrive_time, self.customer_time_windows_h, self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node)
                    new_cost = calculate_customer_window_cost(temp_new_scheme, self.vehicle, vehicle_arrive_time, self.customer_time_windows_h, self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node)
                    trad_total_cost = orig_cost + new_cost
                    current_total_cost = base_total_cost - traditional_orig_win_cost + trad_total_cost + best_orig_cost + best_new_cost
                    deta_total_cost = current_total_cost - base_total_cost

                    heuristic_scheme = {
                                'customer': customer,
                                'orig_scheme': best_orig_y,
                                'new_scheme': best_new_y,
                                'orig_cost': best_orig_cost,
                                'new_cost': best_new_cost,
                                'orig_win_cost': orig_cost,
                                'new_win_cost': new_cost,
                                'total_cost': current_total_cost,
                                'orig_plan': best_orig_y_cijkdu_plan,
                                'new_plan': best_new_y_cijkdu_plan,
                                'win_cost': deta_total_cost,
                                'delta_cost': delta_cost,
                                'type': 'heuristic_swap'
                    }

                    return (deta_total_cost, heuristic_scheme), True
            except Exception as e:
                print(f"客户点 {customer} 启发式插入失败: {e}")

        return (None,None),False

    def _regret_evaluate_vtp_expansion_insertion(self, customer, vehicle_route, vehicle_task_data, vehicle_arrive_time, repaired_state):

            best_scheme = None
            best_vtp_infor = [None, None, None]
            vtp_infor = [None, None, None]
            min_cost = float('inf')
            vtp_task_data = deep_copy_vehicle_task_data(repaired_state.vehicle_task_data)
            options_result = []
            options_vtp_infor = []
            options_scheme = []
            total_options = []

            for vehicle_id in range(len(vehicle_route)):

                route = vehicle_route[vehicle_id]
                if len(route) < 2:
                    continue

                available_nodes = self._get_available_nodes_for_vehicle(vehicle_id, route, repaired_state)

                candidate_positions = self._get_best_insertion_positions(
                    customer, vehicle_id, route, available_nodes, vehicle_route, vehicle_task_data, repaired_state
                )

                for node, insert_pos in candidate_positions:
                    result, scheme = self._calculate_vtp_expansion_cost(customer, vehicle_id, insert_pos, vehicle_route, vtp_task_data, repaired_state, node)
                    options_result.append(result)
                    vtp_infor[0] = node
                    vtp_infor[1] = vehicle_id+1
                    vtp_infor[2] = insert_pos
                    options_vtp_infor.append(vtp_infor)
                    options_scheme.append(scheme)

                    total_options.append({
                        'customer': customer,
                        'scheme': scheme,
                        'eval_cost': result,
                        'real_cost': result,
                        'total_cost': result,
                        'type': 'vtp_expansion',
                        'vtp_node': node,
                        'vtp_insert_vehicle_id': vehicle_id+1,
                        'vtp_insert_index': insert_pos,
                        'infor': vtp_infor
                    })

            return total_options

    def _evaluate_vtp_expansion_insertion(self, customer, vehicle_route, vehicle_task_data, vehicle_arrive_time, repaired_state):

        best_scheme = None
        best_vtp_infor = [None, None, None]
        min_cost = float('inf')
        vtp_task_data = deep_copy_vehicle_task_data(repaired_state.vehicle_task_data)

        for vehicle_id in range(len(vehicle_route)):

            route = vehicle_route[vehicle_id]
            if len(route) < 2:
                continue

            available_nodes = self._get_available_nodes_for_vehicle(vehicle_id, route, repaired_state)

            candidate_positions = self._get_best_insertion_positions(
                customer, vehicle_id, route, available_nodes, vehicle_route, vehicle_task_data, repaired_state
            )

            for node, insert_pos in candidate_positions:

                result, scheme = self._calculate_vtp_expansion_cost(customer, vehicle_id, insert_pos, vehicle_route, vtp_task_data, repaired_state, node)
                if result is not None:
                    if result < min_cost:
                        min_cost = result
                        best_scheme = scheme
                        best_vtp_infor[0] = node
                        best_vtp_infor[1] = vehicle_id+1
                        best_vtp_infor[2] = insert_pos
        return (min_cost, best_scheme), best_vtp_infor if min_cost != float('inf') else None

    def _get_available_nodes_for_vehicle(self, vehicle_id, route, repaired_state):

        available_nodes = []

        existing_nodes = set(route)

        all_nodes = repaired_state.N

        for node_id in all_nodes:
            if node_id not in existing_nodes:
                available_nodes.append(node_id)

        return available_nodes

    def _get_best_insertion_positions(self, customer, vehicle_id, route, available_nodes, vehicle_route, vehicle_task_data, repaired_state):

        position_costs = []

        for node in available_nodes:
            for insert_pos in range(1, len(route)):
                cost = self._calculate_insertion_cost_estimate(customer, vehicle_id, insert_pos, route, node, vehicle_route, vehicle_task_data, repaired_state)
                if cost is not None:
                    position_costs.append((node, insert_pos, cost))

        position_costs.sort(key=lambda x: x[2])
        max_candidates = min(50, len(position_costs))

        return [(node, pos) for node, pos, _ in position_costs[:max_candidates]]

    def _calculate_insertion_cost_estimate(self, customer, vehicle_id, insert_pos, route, node, vehicle_route, vehicle_task_data, repaired_state):

        try:

            vehicle_cost_increase = self._calculate_vehicle_cost_increase(vehicle_id, route, insert_pos, node)

            uav_cost = float('inf')

            drone_id = self.V[0]

            uav_cost = self._calculate_uav_mission_cost_estimate(drone_id, customer, node)

            total_cost = vehicle_cost_increase + uav_cost

            return total_cost

        except Exception as e:
            return None

    def _calculate_uav_mission_cost_estimate(self, drone_id, customer, vtp_node):

        try:

            map_vtp_node = self.node[vtp_node].map_key

            distance = self.uav_travel[drone_id][map_vtp_node][customer].totalDistance * 1
            cost = distance * self.vehicle[drone_id].per_cost

            return cost
        except:
            return None

    def _is_customer_node(self, node_id):

        return hasattr(self.node[node_id], 'customer') and self.node[node_id].customer

    def _is_vehicle_node(self, node_id):

        return node_id in self.vehicle

    def _calculate_vtp_expansion_cost(self, customer, vehicle_id, insert_pos, vehicle_route, vtp_vehicle_task_data, repaired_state, vtp_node):

        try:
            vehicle_idx = vehicle_id
            vehicle_id = vehicle_id + 1
            all_route = [sub_route[:] for sub_route in vehicle_route]
            route = all_route[vehicle_idx]
            in_route = vehicle_route[vehicle_idx]

            route.insert(insert_pos, vtp_node)
            all_route[vehicle_idx] = route

            temp_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(all_route)

            temp_total_cost, temp_uav_tw_violation_cost, temp_total_cost_dict = calculate_window_cost(
                repaired_state.customer_plan, repaired_state.uav_cost, temp_vehicle_arrive_time,
                self.vehicle, self.customer_time_windows_h,
                self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node
            )

            prev_node = route[insert_pos - 1]
            if prev_node == self.DEPOT_nodeID or insert_pos == 1:
                drone_list = self.base_drone_assignment[vehicle_id][:]
            else:
                drone_list = vtp_vehicle_task_data[vehicle_id][prev_node].drone_list[:]
            launch_drone_list = vtp_vehicle_task_data[vehicle_id][prev_node].launch_drone_list[:]
            recovery_drone_list = vtp_vehicle_task_data[vehicle_id][prev_node].recovery_drone_list[:]
            vtp_vehicle_task_data[vehicle_id][vtp_node].drone_list = drone_list
            vtp_vehicle_task_data[vehicle_id][vtp_node].launch_drone_list = []
            vtp_vehicle_task_data[vehicle_id][vtp_node].recovery_drone_list = []

            vehicle_cost_increase = self._calculate_vehicle_cost_increase(vehicle_id, in_route, insert_pos, vtp_node)

            min_uav_cost = float('inf')
            best_drone_scheme = None
            best_scheme = None

            for drone_id in self.V:

                if drone_id not in drone_list:
                    continue
                launch_cost,scheme = self._calculate_uav_mission_cost(drone_id, customer, vehicle_id, insert_pos, all_route, vtp_node, vtp_vehicle_task_data, repaired_state,'launch', temp_vehicle_arrive_time)
                if launch_cost is not None and launch_cost < min_uav_cost:
                    min_uav_cost = launch_cost
                    best_drone_scheme = (drone_id, customer, vehicle_id, insert_pos, 'launch')
                    best_scheme = scheme

            for drone_id in self.V:
                recovery_cost,scheme = self._calculate_uav_mission_cost(drone_id, customer, vehicle_id, insert_pos, all_route, vtp_node, vtp_vehicle_task_data, repaired_state,'recovery', temp_vehicle_arrive_time)
                if recovery_cost is not None and recovery_cost < min_uav_cost:
                    min_uav_cost = recovery_cost
                    best_drone_scheme = (drone_id, customer, vehicle_id, insert_pos, 'recovery')
                    best_scheme = scheme
            if min_uav_cost == float('inf'):
                return float('inf'), None

            total_cost = vehicle_cost_increase + min_uav_cost + temp_total_cost

            return total_cost, best_scheme

        except Exception as e:
            return float('inf'), None

    def _calculate_uav_mission_cost(self, drone_id, customer, vehicle_id, insert_pos, route, vtp_node, vtp_vehicle_task_data, repaired_state, mission_type, temp_vehicle_arrive_time):

        if vtp_node is None:
            vtp_node = f"vtp_{vehicle_id}_{insert_pos}_{customer}"
        repaired_state.add_vehicle_route = route
        vtp_vehicle_arrive_time = repaired_state.fast_arrive_time_memo(repaired_state.add_vehicle_route)

        if mission_type == 'launch':

            cost,scheme = self._calculate_launch_mission_cost(drone_id, vtp_node, customer, route, vtp_vehicle_arrive_time, vtp_vehicle_task_data, repaired_state, vehicle_id, temp_vehicle_arrive_time)
        elif mission_type == 'recovery':

            cost,scheme = self._calculate_recovery_mission_cost(drone_id, vtp_node, customer, route, vtp_vehicle_arrive_time, vtp_vehicle_task_data, repaired_state, vehicle_id, temp_vehicle_arrive_time)
        else:
            return None

        return cost,scheme

    def _calculate_launch_mission_cost(self, drone_id, vtp_node, customer, route, vtp_vehicle_arrive_time, vtp_vehicle_task_data, repaired_state, vehicle_id, temp_vehicle_arrive_time):

        customer_vtp_candidates = self.map_cluster_vtp_dict[customer]

        temp_route = [in_route[:] for in_route in route]
        vehicle_idx = vehicle_id -1

        route = route[vehicle_idx]
        vtp_node_idx = route.index(vtp_node)
        launch_vehicle_id = vehicle_id

        min_cost = float('inf')
        best_scheme = None
        temp_scheme = {}

        n = len(route)
        next_launch_idx = n - 1

        if drone_id not in vtp_vehicle_task_data[launch_vehicle_id][vtp_node].drone_list:
            return None, None

        for k in range(vtp_node_idx + 1, n - 1):
            if drone_id in vtp_vehicle_task_data[launch_vehicle_id][route[k]].launch_drone_list:
                next_launch_idx = k
                break

        for k in range(vtp_node_idx + 1, next_launch_idx+1):
            recovery_node = route[k]
            if drone_id not in vtp_vehicle_task_data[launch_vehicle_id][recovery_node].drone_list or \
                drone_id not in vtp_vehicle_task_data[launch_vehicle_id][recovery_node].launch_drone_list:
                continue

            has_conflict = False
            for m in range(vtp_node_idx + 1, k):
                if drone_id not in vtp_vehicle_task_data[launch_vehicle_id][route[m]].drone_list:
                    has_conflict = True
                    break
            if has_conflict:
                continue

            if vtp_node == recovery_node:
                continue

            route_cost = self.drone_insert_cost(drone_id, customer, vtp_node, recovery_node)

            temp_scheme[customer] = (drone_id, vtp_node, customer, recovery_node, launch_vehicle_id, launch_vehicle_id)
            win_cost = calculate_customer_window_cost(temp_scheme, self.vehicle, temp_vehicle_arrive_time, self.customer_time_windows_h, self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node)
            total_cost = route_cost + win_cost
            if total_cost is not None and total_cost < min_cost:

                min_cost = total_cost
                best_scheme = (drone_id, vtp_node, customer, recovery_node, launch_vehicle_id, launch_vehicle_id)

        for recovery_vehicle_idx, other_route in enumerate(repaired_state.vehicle_routes):
            recovery_vehicle_id = recovery_vehicle_idx + 1
            if recovery_vehicle_id == launch_vehicle_id:
                continue

            launch_time = vtp_vehicle_arrive_time[launch_vehicle_id][vtp_node]

            for recovery_node in other_route[1:-1]:

                if vtp_node == recovery_node:
                    continue

                recovery_time = vtp_vehicle_arrive_time[recovery_vehicle_id][recovery_node]
                if recovery_time <= launch_time:
                    continue

                if drone_id in vtp_vehicle_task_data[recovery_vehicle_id][recovery_node].launch_drone_list or \
                    drone_id in vtp_vehicle_task_data[recovery_vehicle_id][recovery_node].recovery_drone_list:
                    continue

                conflict = False
                for m in range(vtp_node_idx + 1, len(route)):
                    if drone_id in vtp_vehicle_task_data[launch_vehicle_id][route[m]].launch_drone_list or \
                        drone_id not in vtp_vehicle_task_data[launch_vehicle_id][route[m]].drone_list:
                        conflict = True
                        break

                if not conflict:

                    temp_scheme[customer] = (drone_id, vtp_node, customer, recovery_node, launch_vehicle_id, recovery_vehicle_id)
                    win_cost = calculate_customer_window_cost(temp_scheme, self.vehicle, temp_vehicle_arrive_time, self.customer_time_windows_h, self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node)
                    route_cost = self.drone_insert_cost(drone_id, customer, vtp_node, recovery_node)
                    total_cost = route_cost + win_cost
                    if total_cost is not None and total_cost < min_cost:

                        min_cost = total_cost
                        best_scheme = (drone_id, vtp_node, customer, recovery_node, launch_vehicle_id, recovery_vehicle_id)

        return min_cost if min_cost != float('inf') else None, best_scheme

    def _calculate_recovery_mission_cost(self, drone_id, vtp_node, customer, route, vtp_vehicle_arrive_time, vtp_vehicle_task_data, repaired_state, vehicle_id, temp_vehicle_arrive_time):

        vehicle_idx = vehicle_id - 1

        route = route[vehicle_idx]
        vtp_node_idx = route.index(vtp_node)
        recovery_vehicle_id = vehicle_id

        min_cost = float('inf')
        best_scheme = None
        temp_scheme = {}

        for launch_vehicle_idx, launch_route in enumerate(repaired_state.vehicle_routes):
            launch_vehicle_id = launch_vehicle_idx + 1

            if launch_vehicle_id == recovery_vehicle_id:

                has_conflict = False
                for index,node in enumerate(launch_route[1:-1]):
                    if drone_id not in vtp_vehicle_task_data[launch_vehicle_id][node].drone_list:
                        has_conflict = True
                        break
                if has_conflict:
                    continue

                nearest_recovery_idx = -1
                for k in range(vtp_node_idx - 1, 0, -1):
                    if drone_id in vtp_vehicle_task_data[launch_vehicle_id][route[k]].recovery_drone_list:
                        nearest_recovery_idx = k
                        break

                if nearest_recovery_idx == -1:
                    nearest_recovery_idx = 1

                    for k in range(nearest_recovery_idx, vtp_node_idx):
                        has_conflict = False
                        for m in range(k, vtp_node_idx+1):
                            if drone_id not in vtp_vehicle_task_data[launch_vehicle_id][route[m]].drone_list:
                                has_conflict = True
                                break
                        if has_conflict:
                            continue
                        launch_node = route[k]
                        if launch_node == vtp_node:
                            continue
                        if drone_id in vtp_vehicle_task_data[launch_vehicle_id][launch_node].drone_list:

                            temp_scheme[customer] = (drone_id, launch_node, customer, vtp_node, launch_vehicle_id, recovery_vehicle_id)
                            route_cost = self.drone_insert_cost(drone_id, customer, launch_node, vtp_node)
                            win_cost = calculate_customer_window_cost(temp_scheme, self.vehicle, temp_vehicle_arrive_time, self.customer_time_windows_h, self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node)
                            total_cost = route_cost + win_cost
                            if total_cost is not None and total_cost < min_cost:

                                min_cost = total_cost
                                best_scheme = (drone_id, launch_node, customer, vtp_node, launch_vehicle_id, recovery_vehicle_id)
                else:

                    for k in range(nearest_recovery_idx, vtp_node_idx + 1):
                        has_conflict = False
                        for m in range(k, vtp_node_idx):
                            if drone_id not in vtp_vehicle_task_data[launch_vehicle_id][route[m]].drone_list:
                                has_conflict = True
                                break
                        if has_conflict:
                            continue
                        launch_node = route[k]

                        if launch_node == vtp_node:
                            continue

                        temp_scheme[customer] = (drone_id, launch_node, customer, vtp_node, launch_vehicle_id, recovery_vehicle_id)
                        route_cost = self.drone_insert_cost(drone_id, customer, launch_node, vtp_node)
                        win_cost = calculate_customer_window_cost(temp_scheme, self.vehicle, temp_vehicle_arrive_time, self.customer_time_windows_h, self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node)
                        total_cost = route_cost + win_cost
                        if total_cost is not None and total_cost < min_cost:

                            min_cost = total_cost
                            best_scheme = (drone_id, launch_node, customer, vtp_node, launch_vehicle_id, recovery_vehicle_id)

            else:

                for launch_node in launch_route[1:-1]:

                    launch_node_idx = launch_route.index(launch_node)
                    if launch_node == vtp_node:
                        continue
                    if vtp_vehicle_arrive_time[launch_vehicle_id][launch_node] >= vtp_vehicle_arrive_time[recovery_vehicle_id][vtp_node]:
                        continue

                    if drone_id not in vtp_vehicle_task_data[launch_vehicle_id][launch_node].drone_list:
                        continue

                    has_conflict = False
                    for m in range(launch_node_idx + 1, len(launch_route) - 1):
                        if drone_id in vtp_vehicle_task_data[launch_vehicle_id][launch_route[m]].launch_drone_list or \
                            drone_id not in vtp_vehicle_task_data[launch_vehicle_id][launch_route[m]].drone_list:
                            has_conflict = True
                            break
                    if has_conflict:
                        continue

                    temp_scheme[customer] = (drone_id, launch_node, customer, vtp_node, launch_vehicle_id, recovery_vehicle_id)
                    route_cost = self.drone_insert_cost(drone_id, customer, launch_node, vtp_node)
                    win_cost = calculate_customer_window_cost(temp_scheme, self.vehicle, temp_vehicle_arrive_time, self.customer_time_windows_h, self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node)
                    total_cost = route_cost + win_cost
                    if total_cost is not None and total_cost < min_cost:

                        min_cost = total_cost
                        best_scheme = (drone_id, launch_node, customer, vtp_node, launch_vehicle_id, recovery_vehicle_id)

        return min_cost if min_cost != float('inf') else None, best_scheme

    def _estimate_uav_cost(self, drone_id, vtp_node, customer):

        try:

            vtp_x, vtp_y = self._get_node_coordinates(vtp_node)
            customer_x, customer_y = self._get_node_coordinates(customer)

            distance = ((vtp_x - customer_x) ** 2 + (vtp_y - customer_y) ** 2) ** 0.5
            cost = distance * self.vehicle[drone_id].per_cost

            return cost
        except:
            return float('inf')

    def _get_node_coordinates(self, node_id):

        try:
            if hasattr(self.node[node_id], 'x') and hasattr(self.node[node_id], 'y'):
                return self.node[node_id].x, self.node[node_id].y
            else:

                return 0, 0
        except:
            return 0, 0

    def _calculate_landing_cost(self, customer, vehicle_id, insert_pos, route, drone_scheme):

        try:

            base_landing_cost = 10.0

            route_length_factor = len(route) / 10.0
            customer_priority = 1.0

            total_landing_cost = base_landing_cost * route_length_factor * customer_priority

            return total_landing_cost

        except:
            return 0.0

    def _find_vtp_insert_position(self, route, vtp_node, customer):

        try:

            parts = vtp_node.split('_')
            if len(parts) >= 3:
                insert_pos = int(parts[2])
                return insert_pos

            return len(route) // 2 if len(route) > 1 else 1

        except:

            return len(route) // 2 if len(route) > 1 else 1

    def _balanced_k_step_selection(self, destroy_node, k_steps, max_samples, candidate_limit, vehicle_route, vehicle_task_data, vehicle_arrive_time, repaired_state):

        import random

        if len(destroy_node) > candidate_limit:

            node_costs = []
            for customer in destroy_node:
                all_insert_position = self.get_all_insert_position(vehicle_route, vehicle_task_data, customer, vehicle_arrive_time)
                if all_insert_position is not None:
                    min_cost = float('inf')
                    for drone_id, inert_positions in all_insert_position.items():
                        for inert_position in inert_positions:
                            launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id = inert_position
                            insert_cost = self.drone_insert_cost(drone_id, customer_node, launch_node, recovery_node)
                            min_cost = min(min_cost, insert_cost)
                    if min_cost != float('inf'):
                        node_costs.append((customer, min_cost))

            node_costs.sort(key=lambda x: x[1])
            candidate_nodes = [customer for customer, _ in node_costs[:candidate_limit]]
        else:
            candidate_nodes = destroy_node

        candidate_sequences = []

        greedy_sequence = candidate_nodes[:k_steps] if len(candidate_nodes) >= k_steps else candidate_nodes
        candidate_sequences.append(greedy_sequence)

        sample_size = min(max_samples - 1, len(candidate_nodes))
        for _ in range(sample_size):

            if len(candidate_nodes) >= k_steps:
                idx = self.rng.choice(len(candidate_nodes), size=k_steps, replace=False)
                sequence = [candidate_nodes[i] for i in idx]
            else:
                sequence = list(candidate_nodes)
            candidate_sequences.append(sequence)

        best_customer = None
        best_scheme = None
        best_cost = float('inf')

        for sequence in candidate_sequences:

            sequence_cost = self._evaluate_k_step_sequence_cost(
                sequence, vehicle_route, vehicle_task_data, vehicle_arrive_time, repaired_state
            )

            if sequence_cost < best_cost:
                best_cost = sequence_cost

                first_customer = sequence[0]
                best_scheme = self._get_best_insertion_scheme(
                    first_customer, vehicle_route, vehicle_task_data, vehicle_arrive_time
                )
                best_customer = first_customer

        return best_customer, best_scheme, best_cost

    def _evaluate_k_step_sequence_cost(self, sequence, vehicle_route, vehicle_task_data, vehicle_arrive_time, repaired_state):

        if len(sequence) == 1:

            customer = sequence[0]
            all_insert_position = self.get_all_insert_position(vehicle_route, vehicle_task_data, customer, vehicle_arrive_time)
            if all_insert_position is not None:
                min_cost = float('inf')
                for drone_id, inert_positions in all_insert_position.items():
                    for inert_position in inert_positions:
                        launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id = inert_position
                        insert_cost = self.drone_insert_cost(drone_id, customer_node, launch_node, recovery_node)
                        min_cost = min(min_cost, insert_cost)
                return min_cost if min_cost != float('inf') else float('inf')

        total_cost = 0
        temp_state = repaired_state.fast_copy()

        for i, customer in enumerate(sequence[:2]):
            all_insert_position = self.get_all_insert_position(
                temp_state.vehicle_routes, temp_state.vehicle_task_data, customer, vehicle_arrive_time
            )

            if all_insert_position is not None:
                min_cost = float('inf')
                best_scheme = None

                for drone_id, inert_positions in all_insert_position.items():
                    for inert_position in inert_positions:
                        launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id = inert_position
                        insert_cost = self.drone_insert_cost(drone_id, customer_node, launch_node, recovery_node)
                        if insert_cost < min_cost:
                            min_cost = insert_cost
                            best_scheme = (drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id)

                if best_scheme is not None:
                    total_cost += min_cost

                    self._simulate_insertion_simple(temp_state, customer, best_scheme)
                else:
                    return float('inf')
            else:
                return float('inf')

        if len(sequence) > 2:
            remaining_customers = sequence[2:]
            avg_cost_estimate = self._estimate_average_cost(remaining_customers, vehicle_route, vehicle_task_data, vehicle_arrive_time)
            total_cost += avg_cost_estimate * len(remaining_customers) * 0.9

        return total_cost

    def _get_best_insertion_scheme(self, customer, vehicle_route, vehicle_task_data, vehicle_arrive_time):

        all_insert_position = self.get_all_insert_position(vehicle_route, vehicle_task_data, customer, vehicle_arrive_time)
        if all_insert_position is not None:
            min_cost = float('inf')
            best_scheme = None

            for drone_id, inert_positions in all_insert_position.items():
                for inert_position in inert_positions:
                    launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id = inert_position
                    insert_cost = self.drone_insert_cost(drone_id, customer_node, launch_node, recovery_node)
                    if insert_cost < min_cost:
                        min_cost = insert_cost
                        best_scheme = (drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id)

            return best_scheme
        return None

    def _simulate_insertion_simple(self, temp_state, customer, scheme):

        drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id = scheme

        temp_state.customer_plan[customer_node] = scheme

        if drone_id not in temp_state.uav_assignments:
            temp_state.uav_assignments[drone_id] = []
        temp_state.uav_assignments[drone_id].append(scheme)

        if temp_state.uav_cost is None:
            temp_state.uav_cost = {}
        temp_state.uav_cost[customer_node] = self.drone_insert_cost(drone_id, customer_node, launch_node, recovery_node)

        temp_state.vehicle_task_data = update_vehicle_task(
            temp_state.vehicle_task_data, scheme, temp_state.vehicle_routes
        )

    def _estimate_average_cost(self, customers, vehicle_route, vehicle_task_data, vehicle_arrive_time):

        if not customers:
            return 0

        total_cost = 0
        valid_customers = 0

        sample_customers = customers[:3]

        for customer in sample_customers:
            all_insert_position = self.get_all_insert_position(vehicle_route, vehicle_task_data, customer, vehicle_arrive_time)
            if all_insert_position is not None:
                min_cost = float('inf')
                for drone_id, inert_positions in all_insert_position.items():
                    for inert_position in inert_positions:
                        launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id = inert_position
                        insert_cost = self.drone_insert_cost(drone_id, customer_node, launch_node, recovery_node)
                        min_cost = min(min_cost, insert_cost)
                if min_cost != float('inf'):
                    total_cost += min_cost
                    valid_customers += 1

        return total_cost / max(valid_customers, 1)

    def _apply_insertion(self, repaired_state, customer, scheme, cost):

        drone_id, launch_node, customer_node, recovery_node, launch_vehicle_id, recovery_vehicle_id = scheme

        repaired_state.customer_plan[customer_node] = scheme

        if drone_id not in repaired_state.uav_assignments:
            repaired_state.uav_assignments[drone_id] = []
        repaired_state.uav_assignments[drone_id].append(scheme)

        if repaired_state.uav_cost is None:
            repaired_state.uav_cost = {}
        repaired_state.uav_cost[customer_node] = cost

        repaired_state.vehicle_task_data = update_vehicle_task(
            repaired_state.vehicle_task_data, scheme, repaired_state.vehicle_routes
        )

    def get_all_insert_position(self, vehicle_route, vehicle_task_data, customer, vehicle_arrive_time):

            all_insert_position = {drone_id: [] for drone_id in self.V}

            customer_vtp_candidates = self.map_cluster_vtp_dict[customer]

            for drone_id in self.V:
                for vehicle_idx, route in enumerate(vehicle_route):
                    v_id = vehicle_idx + 1
                    n = len(route)

                    for i in range(1, n - 1):
                        launch_node = route[i]

                        if drone_id not in vehicle_task_data[v_id][launch_node].drone_list:
                            continue

                        for k in range(i + 1, n - 1):
                            recovery_node = route[k]

                            if drone_id not in vehicle_task_data[v_id][recovery_node].drone_list:
                                continue

                            has_conflict = False
                            for m in range(i + 1, k):
                                intermediate_node = route[m]
                                if drone_id in vehicle_task_data[v_id][intermediate_node].launch_drone_list:
                                    has_conflict = True
                                    break

                            if not has_conflict:

                                all_insert_position[drone_id].append(
                                    (launch_node, customer, recovery_node, v_id, v_id)
                                )

                for launch_veh_idx, launch_route in enumerate(vehicle_route):
                    launch_v_id = launch_veh_idx + 1

                    for i in range(1, len(launch_route)-1):
                        launch_node = launch_route[i]

                        if drone_id not in vehicle_task_data[launch_v_id][launch_node].drone_list:
                            continue

                        launch_time = vehicle_arrive_time[launch_v_id][launch_node]

                        for rec_veh_idx, rec_route in enumerate(vehicle_route):
                            rec_v_id = rec_veh_idx + 1
                            if launch_v_id == rec_v_id:
                                continue

                            for k in range(1, len(rec_route) - 1):
                                recovery_node = rec_route[k]

                                if drone_id not in vehicle_task_data[rec_v_id][recovery_node].drone_list:
                                    continue

                                recovery_time = vehicle_arrive_time[rec_v_id][recovery_node]

                                if recovery_time <= launch_time:
                                    continue

                                conflict = False

                                for m in range(i + 1, len(launch_route) - 1):
                                    if drone_id in vehicle_task_data[launch_v_id][launch_route[m]].launch_drone_list:
                                        conflict = True
                                        break
                                if conflict: continue

                                all_insert_position[drone_id].append(
                                    (launch_node, customer, recovery_node, launch_v_id, rec_v_id)
                                )

            total_positions = sum(len(positions) for positions in all_insert_position.values())

            return all_insert_position

    def calculate_multiopt_cost(self, repair_state, best_scheme):

        drone_id, launch_node, customer, recovery_node, launch_vehicle_id, recovery_vehicle_id = best_scheme
        total_cost = 0
        for drone_id in self.V:
            total_cost += self.drone_insert_cost(drone_id, repair_state.vehicle_routes, repair_state.vehicle_task_data, customer, launch_node, recovery_node, launch_vehicle_id, recovery_vehicle_id)
        return total_cost

    def multiopt_update_best_scheme(self, best_scheme, near_node_list, vehicle_route, vehicle_task_data, repair_state, sample_size=30):

            init_multiopt_cost = self.calculate_multiopt_cost(repair_state, best_scheme)

            import random
            drone_id, launch_node, customer, recovery_node, launch_vehicle_id, recovery_vehicle_id = best_scheme
            best = best_scheme
            best_cost = float('inf')

            def get_related_drones(vehicle_id, node, task_data):
                related = set()
                if hasattr(task_data[vehicle_id][node], 'drone_list'):
                    related.update(task_data[vehicle_id][node].drone_list)
                if hasattr(task_data[vehicle_id][node], 'launch_drone_list'):
                    related.update(task_data[vehicle_id][node].launch_drone_list)
                if hasattr(task_data[vehicle_id][node], 'recovery_drone_list'):
                    related.update(task_data[vehicle_id][node].recovery_drone_list)
                return related

            def get_greedy_cost(vehicle_id, l_n, r_n):
                total = 0

                total += self.drone_insert_cost(drone_id, customer, l_n, r_n)

                related = get_related_drones(vehicle_id, l_n, vehicle_task_data) | get_related_drones(vehicle_id, r_n, vehicle_task_data)
                related.discard(drone_id)
                for d_id in related:

                    launch_n, recovery_n = None, None
                    route = vehicle_route[vehicle_id - 1]
                    for n2 in route:
                        if hasattr(vehicle_task_data[vehicle_id][n2], 'launch_drone_list') and d_id in vehicle_task_data[vehicle_id][n2].launch_drone_list:
                            launch_n = n2
                        if hasattr(vehicle_task_data[vehicle_id][n2], 'recovery_drone_list') and d_id in vehicle_task_data[vehicle_id][n2].recovery_drone_list:
                            recovery_n = n2
                    if launch_n and recovery_n:
                        total += self.drone_insert_cost(d_id, customer, launch_n, recovery_n)
                return total

            if launch_vehicle_id == recovery_vehicle_id:
                node_list = near_node_list

                candidates = set()

                n = len(node_list)
                while len(candidates) < sample_size:
                    l_n = node_list[int(self.rng.integers(0, n))]
                    r_n = node_list[int(self.rng.integers(0, n))]
                    if l_n != r_n:
                        candidates.add((l_n, r_n))
                for l_n, r_n in candidates:
                    cost = get_greedy_cost(launch_vehicle_id, l_n, r_n)
                    if cost < best_cost:
                        best = (drone_id, l_n, customer, r_n, launch_vehicle_id, recovery_vehicle_id)
                        best_cost = cost
                return best, best_cost
            else:

                launch_list = near_node_list[launch_vehicle_id]
                recovery_list = near_node_list[recovery_vehicle_id]
                candidates = set()

                nl = len(launch_list)
                nr = len(recovery_list)
                while len(candidates) < sample_size:
                    l_n = launch_list[int(self.rng.integers(0, nl))]
                    r_n = recovery_list[int(self.rng.integers(0, nr))]
                    if l_n != r_n:
                        candidates.add((l_n, r_n))
                for l_n, r_n in candidates:
                    cost = get_greedy_cost(launch_vehicle_id, l_n, r_n) + get_greedy_cost(recovery_vehicle_id, l_n, r_n)
                    if cost < best_cost:
                        best = (drone_id, l_n, customer, r_n, launch_vehicle_id, recovery_vehicle_id)
                        best_cost = cost
                return best, best_cost

    def _r12_safe_repair(self, repair_op, destroyed_state, *args, **kwargs):

        if getattr(destroyed_state, '_r12_destroy_failed', False):
            failed_state = destroyed_state.fast_copy()
            failed_state.__dict__.pop('_r12_destroy_failed', None)
            failed_state.repair_objective = float('inf')
            return failed_state, []
        try:
            return repair_op(destroyed_state, *args, **kwargs)
        except ValueError as e:
            if 'x not in list' not in str(e):
                raise
            self.r12_hfix_events = getattr(self, 'r12_hfix_events', 0) + 1
            print(f"  > [r12-hfix] repair raised ValueError({e}); repair treated as infeasible")
            failed_state = destroyed_state.fast_copy()
            failed_state.__dict__.pop('_r12_destroy_failed', None)
            failed_state.repair_objective = float('inf')
            return failed_state, []

    def _r12_safe_destroy(self, destroy_op, state, *args, **kwargs):

        try:
            return destroy_op(state, *args, **kwargs)
        except ValueError as e:
            if 'x not in list' not in str(e):
                raise
            self.r12_hfix_events = getattr(self, 'r12_hfix_events', 0) + 1
            print(f"  > [r12-hfix] destroy raised ValueError({e}); iteration treated as infeasible")
            failed = state.fast_copy()
            failed.destroyed_customers_info = {}
            failed._r12_destroy_failed = True
            return failed

    def solve(self, initial_state):

        y_best = []
        y_cost = _R12TimedList()
        win_cost = []
        uav_route_cost = []
        vehicle_route_cost = []
        final_uav_cost = []
        final_total_list = []
        final_win_cost = []
        final_vehicle_route_cost = []
        final_total_objective = []
        work_time = []
        final_work_time = []
        current_state = initial_state.fast_copy()

        decay_factor = 0.95

        current_state.rm_empty_vehicle_route, current_state.empty_nodes_by_vehicle = current_state.update_rm_empty_task()

        current_state.vehicle_routes = [route[:] for route in current_state.rm_empty_vehicle_route]

        current_state.destroyed_node_cost = current_state.win_total_objective()
        print(f"初始解总成本: {current_state.destroyed_node_cost}")
        current_state.rm_empty_vehicle_arrive_time = current_state.calculate_rm_empty_vehicle_arrive_time(current_state.rm_empty_vehicle_route)
        current_state.vehicle_arrive_time = current_state.calculate_rm_empty_vehicle_arrive_time(current_state.vehicle_routes)

        current_state.final_uav_plan, current_state.final_uav_cost, current_state.final_vehicle_plan_time, current_state.final_vehicle_task_data, current_state.final_global_reservation_table = current_state.re_update_time(current_state.vehicle_routes, current_state.vehicle_arrive_time, current_state.vehicle_task_data, current_state)
        final_vehicle_arrive_time = extract_arrive_time_from_plan(current_state.final_vehicle_plan_time)
        final_vehicle_max_times, final_global_max_time = get_max_completion_time(final_vehicle_arrive_time)
        final_work_time.append(final_global_max_time)
        final_window_total_cost, final_uav_tw_violation_cost, final_total_cost_dict  = calculate_window_cost(current_state.customer_plan,
                    current_state.final_uav_cost,
                    final_vehicle_arrive_time,
                    self.vehicle,
                    self.customer_time_windows_h,
                    self.early_arrival_cost,
                    self.late_arrival_cost,
                    self.uav_travel,
                    self.node)
        final_total_list.append(final_window_total_cost)
        final_total_objective_value = current_state.update_calculate_plan_cost(final_total_cost_dict, current_state.vehicle_routes)
        final_total_objective.append(final_total_objective_value)
        final_vehicle_route_cost.append(final_total_objective_value - final_window_total_cost)
        best_final_objective = final_total_objective_value
        final_current_objective = final_total_objective_value

        best_final_state = current_state.fast_copy()
        final_best_objective = final_total_objective_value
        best_final_state.final_best_objective = final_best_objective
        best_final_uav_cost = sum(current_state.final_uav_cost.values())
        best_final_win_cost = sum(final_uav_tw_violation_cost.values())
        best_final_vehicle_max_times = final_vehicle_max_times
        best_final_global_max_time = final_global_max_time
        best_total_win_cost = final_window_total_cost
        best_final_vehicle_route_cost = final_total_objective_value - final_window_total_cost

        best_state = current_state.fast_copy()
        total_customer_num=len(best_state.customer_plan)
        best_objective = current_state.destroyed_node_cost

        current_objective = best_objective
        initial_delta_estimate = best_objective * 0.05
        initial_accept_prob = 0.20
        self.temperature = -initial_delta_estimate / np.log(initial_accept_prob)
        self.initial_temperature = self.temperature
        no_improve_count = 0
        _r12_sched = int(os.environ.get('R12_SCHEDULE_ITERS') or self.max_iterations)
        no_improve_threshold = max(50, _r12_sched // 10)
        print(f"自适应初始温度: {self.temperature:.4f} (基于初始成本 {best_objective:.2f})")

        y_best.append(best_objective)
        y_cost.append(best_objective)
        current_window_total_cost, current_uav_tw_violation_cost, current_total_cost_dict  = calculate_window_cost(current_state.customer_plan,
                    current_state.uav_cost,
                    current_state.rm_empty_vehicle_arrive_time,
                    self.vehicle,
                    self.customer_time_windows_h,
                    self.early_arrival_cost,
                    self.late_arrival_cost,
                    self.uav_travel,
                    self.node)
        current_vehicle_max_times, current_global_max_time = get_max_completion_time(current_state.vehicle_arrive_time)
        work_time.append(current_global_max_time)
        current_total_violation_cost = sum(current_uav_tw_violation_cost.values())
        win_cost.append(current_total_violation_cost)
        current_state._total_cost = current_state.update_calculate_plan_cost(current_total_cost_dict, current_state.vehicle_routes)
        uav_route_cost.append(current_window_total_cost - current_total_violation_cost)
        vehicle_route_cost.append(current_objective - current_window_total_cost)

        final_best_objective = best_final_objective

        start_time = time.time()

        init_uav_cost = list(current_state.uav_cost.values())
        base_flexibility_bonus = sum(init_uav_cost) / len(init_uav_cost)

        T_final = 0.01
        cooling_rate = (T_final / self.temperature) ** (1.0 / _r12_sched)
        cooling_rate = max(0.95, min(0.9999, cooling_rate))
        print(f"自适应降温系数: {cooling_rate:.6f}")
        print(f"开始ALNS求解，初始成本: {best_objective:.2f}")

        destroy_names = [op.__name__ for op in self.destroy_operators]
        repair_names  = [op.__name__ for op in self.repair_operators]
        global_destroy_stats = {n: {
            'total_uses': 0, 'total_score': 0.0, 'improvements': 0,
            'new_best_count': 0, 'better_current_count': 0,
            'total_improvement': 0.0,
        } for n in destroy_names}
        global_repair_stats = {n: {
            'total_uses': 0, 'total_score': 0.0, 'improvements': 0,
            'new_best_count': 0, 'better_current_count': 0,
            'total_improvement': 0.0,
        } for n in repair_names}

        global_combo_stats = {}

        iteration_operator_log = []

        operator_history = []

        best_operator_combo = {'destroy': None, 'repair': None,
                               'best_improvement': 0.0, 'iteration': None,
                               'objective_before': None, 'objective_after': None}

        for iteration in range(self.max_iterations):

            strategy_names = list(self.strategy_weights.keys())

            strategy_w = np.array(list(self.strategy_weights.values()))

            chosen_strategy = self.rng.choice(strategy_names, p=strategy_w / np.sum(strategy_w))

            destroy_op_weights = self.operator_weights[chosen_strategy]['destroy']
            repair_op_weights = self.operator_weights[chosen_strategy]['repair']

            destroy_op_names = list(destroy_op_weights.keys())
            destroy_w = np.array(list(destroy_op_weights.values()))
            chosen_destroy_op_name = self.rng.choice(destroy_op_names, p=destroy_w / np.sum(destroy_w))
            destroy_op = getattr(self, chosen_destroy_op_name)

            repair_op_names = list(repair_op_weights.keys())
            repair_w = np.array(list(repair_op_weights.values()))
            chosen_repair_op_name = self.rng.choice(repair_op_names, p=repair_w / np.sum(repair_w))
            repair_op = getattr(self, chosen_repair_op_name)

            print(f"\n--- 迭代 {iteration} | 温度: {self.temperature:.2f} | 选择策略: {chosen_strategy.upper()} ---")
            print(f"  > 战术组合: {chosen_destroy_op_name} + {chosen_repair_op_name}")

            prev_state = current_state.fast_copy()
            print(f'当前的任务客户点数量为:{len(current_state.customer_plan.keys())}')
            print(f'当前uav_cost个数为:{len(current_state.uav_cost.keys())}')
            strategic_bonus = base_flexibility_bonus * (self.temperature / self.initial_temperature)

            if chosen_strategy == 'structural':

                destroyed_state = self._r12_safe_destroy(destroy_op, prev_state, force_vtp_mode=True)

                strategic_bonus = base_flexibility_bonus * (self.temperature / self.initial_temperature)
                num_destroyed = len(destroyed_state.destroyed_customers_info)

                repaired_state, _ = self._r12_safe_repair(repair_op, destroyed_state, strategic_bonus, num_destroyed, force_vtp_mode=True)
                if len(repaired_state.customer_plan) < total_customer_num:
                    repaired_state.repair_objective = float('inf')
                if repaired_state.repair_objective == float('inf'):
                    print("  > 修复后方案为空，跳过此次迭代。")
                    iteration += 1

                    repaired_state.destroyed_customers_info = {}

                    self.operator_weights[chosen_strategy]['destroy'][chosen_destroy_op_name] *= decay_factor

                    if self.operator_weights[chosen_strategy]['destroy'][chosen_destroy_op_name] < 0.1:
                        self.operator_weights[chosen_strategy]['destroy'][chosen_destroy_op_name] = 0.1

                    self.operator_weights[chosen_strategy]['repair'][chosen_repair_op_name] *= decay_factor
                    if self.operator_weights[chosen_strategy]['repair'][chosen_repair_op_name] < 0.1:
                        self.operator_weights[chosen_strategy]['repair'][chosen_repair_op_name] = 0.1

                    self.temperature *= cooling_rate

                    y_cost.append(current_objective)

                    repaired_state.repair_objective = 0
                    current_state = best_state.fast_copy()

                    global_destroy_stats[chosen_destroy_op_name]['total_uses'] += 1
                    global_repair_stats[chosen_repair_op_name]['total_uses'] += 1
                    _combo_key = (chosen_destroy_op_name, chosen_repair_op_name)
                    if _combo_key not in global_combo_stats:
                        global_combo_stats[_combo_key] = {
                            'uses': 0, 'total_score': 0.0, 'improvements': 0,
                            'new_best_count': 0, 'better_current_count': 0,
                            'total_improvement': 0.0, 'best_improvement': 0.0,
                        }
                    global_combo_stats[_combo_key]['uses'] += 1
                    iteration_operator_log.append({
                        'iteration': iteration,
                        'strategy': chosen_strategy,
                        'destroy_operator': chosen_destroy_op_name,
                        'repair_operator': chosen_repair_op_name,
                        'score': 0.0,
                        'accepted': False,
                        'found_new_best': False,
                        'found_improvement': False,
                        'current_objective': current_objective,
                        'new_objective': float('inf'),
                        'best_objective': best_objective,
                        'temperature': self.temperature,
                        'improvement_delta': 0.0,
                    })
                    continue
            else:

                destroyed_state = self._r12_safe_destroy(destroy_op, prev_state, force_vtp_mode=False)

                num_destroyed = len(destroyed_state.destroyed_customers_info)

                repaired_state, _ = self._r12_safe_repair(repair_op, destroyed_state, strategic_bonus=0, num_destroyed=num_destroyed, force_vtp_mode=False)

                repaired_state, _ = self._r12_safe_repair(repair_op, destroyed_state, strategic_bonus, num_destroyed, force_vtp_mode=True)
                if len(repaired_state.customer_plan) < total_customer_num:
                    repaired_state.repair_objective = float('inf')
                if repaired_state.repair_objective == float('inf'):
                    print("  > 修复后方案为空，跳过此次迭代。")
                    iteration += 1

                    repaired_state.destroyed_customers_info = {}

                    self.operator_weights[chosen_strategy]['destroy'][chosen_destroy_op_name] *= decay_factor

                    if self.operator_weights[chosen_strategy]['destroy'][chosen_destroy_op_name] < 0.1:
                        self.operator_weights[chosen_strategy]['destroy'][chosen_destroy_op_name] = 0.1

                    self.operator_weights[chosen_strategy]['repair'][chosen_repair_op_name] *= decay_factor
                    if self.operator_weights[chosen_strategy]['repair'][chosen_repair_op_name] < 0.1:
                        self.operator_weights[chosen_strategy]['repair'][chosen_repair_op_name] = 0.1

                    self.temperature *= cooling_rate

                    y_cost.append(current_objective)

                    repaired_state.repair_objective = 0
                    current_state = best_state.fast_copy()

                    global_destroy_stats[chosen_destroy_op_name]['total_uses'] += 1
                    global_repair_stats[chosen_repair_op_name]['total_uses'] += 1
                    _combo_key = (chosen_destroy_op_name, chosen_repair_op_name)
                    if _combo_key not in global_combo_stats:
                        global_combo_stats[_combo_key] = {
                            'uses': 0, 'total_score': 0.0, 'improvements': 0,
                            'new_best_count': 0, 'better_current_count': 0,
                            'total_improvement': 0.0, 'best_improvement': 0.0,
                        }
                    global_combo_stats[_combo_key]['uses'] += 1
                    iteration_operator_log.append({
                        'iteration': iteration,
                        'strategy': chosen_strategy,
                        'destroy_operator': chosen_destroy_op_name,
                        'repair_operator': chosen_repair_op_name,
                        'score': 0.0,
                        'accepted': False,
                        'found_new_best': False,
                        'found_improvement': False,
                        'current_objective': current_objective,
                        'new_objective': float('inf'),
                        'best_objective': best_objective,
                        'temperature': self.temperature,
                        'improvement_delta': 0.0,
                    })
                    continue
            if not destroyed_state.customer_plan or not repaired_state.customer_plan:
                print("  > 破坏或修复后方案为空，跳过此次迭代。")
                iteration += 1
                continue

            new_objective = repaired_state.win_total_objective()
            score = 0
            accepted = False

            current_vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(repaired_state.vehicle_routes)
            current_window_total_cost, current_uav_tw_violation_cost, current_total_cost_dict  = calculate_window_cost(repaired_state.customer_plan,
                    repaired_state.uav_cost,
                    current_vehicle_arrive_time,
                    self.vehicle,
                    self.customer_time_windows_h,
                    self.early_arrival_cost,
                    self.late_arrival_cost,
                    self.uav_travel,
                    self.node)
            current_vehicle_max_times, current_global_max_time = get_max_completion_time(current_vehicle_arrive_time)
            current_total_violation_cost = sum(current_uav_tw_violation_cost.values())
            win_cost.append(current_total_violation_cost)
            uav_route_cost.append(current_window_total_cost - current_total_violation_cost)
            vehicle_route_cost.append(new_objective - current_window_total_cost)
            work_time.append(current_global_max_time)

            repaired_state.vehicle_arrive_time = repaired_state.calculate_rm_empty_vehicle_arrive_time(repaired_state.vehicle_routes)
            repaired_state.final_uav_plan, repaired_state.final_uav_cost, repaired_state.final_vehicle_plan_time, repaired_state.final_vehicle_task_data, repaired_state.final_global_reservation_table = repaired_state.re_update_time(repaired_state.vehicle_routes, repaired_state.vehicle_arrive_time, repaired_state.vehicle_task_data, repaired_state)
            final_vehicle_arrive_time = extract_arrive_time_from_plan(repaired_state.final_vehicle_plan_time)
            finial_window_total_cost, finial_uav_tw_violation_cost, finial_total_cost_dict  = calculate_window_cost(repaired_state.customer_plan,
            repaired_state.final_uav_cost,
            final_vehicle_arrive_time,
            self.vehicle,
            self.customer_time_windows_h,
            self.early_arrival_cost,
            self.late_arrival_cost,
            self.uav_travel,
            self.node)
            final_uav_cost.append(sum(repaired_state.final_uav_cost.values()))
            final_total_list.append(finial_window_total_cost)
            final_win_cost.append(sum(finial_uav_tw_violation_cost.values()))
            final_total_objective_value = repaired_state.update_calculate_plan_cost(finial_total_cost_dict, repaired_state.vehicle_routes)
            final_total_objective.append(final_total_objective_value)
            final_vehicle_max_times, final_global_max_time = get_max_completion_time(final_vehicle_arrive_time)
            final_work_time.append(final_global_max_time)
            final_vehicle_route_cost.append(final_total_objective_value - finial_window_total_cost)
            final_new_objective = final_total_objective_value

            print(f"  > 成本变化: {current_objective:.2f} -> {new_objective:.2f}")

            if new_objective < best_objective:
                accepted = True
                score = self.reward_scores['new_best']
                print(f"  > 结果: 发现新的全局最优解! 奖励 {score} 分。")
            elif new_objective < current_objective:
                accepted = True
                score = self.reward_scores['better_than_current']
                print(f"  > 结果: 找到更优解。奖励 {score} 分。")
            elif self._simulated_annealing_accept(current_objective, new_objective, self.temperature):
                accepted = True
                score = self.reward_scores['accepted']
                print(f"  > 结果: 接受一个较差解（探索成功）。奖励 {score} 分。")
            else:
                accepted = False
                score = 0
                print(f"  > 结果: 拒绝劣解。")

            found_new_best = (accepted and new_objective < best_objective)
            found_improvement = (accepted and new_objective < current_objective)
            improvement_delta = current_objective - new_objective if accepted else 0.0

            ds = global_destroy_stats[chosen_destroy_op_name]
            ds['total_uses'] += 1
            ds['total_score'] += score
            if found_new_best:
                ds['new_best_count'] += 1
            if found_improvement:
                ds['better_current_count'] += 1
                ds['improvements'] += 1
                ds['total_improvement'] += improvement_delta

            rs = global_repair_stats[chosen_repair_op_name]
            rs['total_uses'] += 1
            rs['total_score'] += score
            if found_new_best:
                rs['new_best_count'] += 1
            if found_improvement:
                rs['better_current_count'] += 1
                rs['improvements'] += 1
                rs['total_improvement'] += improvement_delta

            combo_key = (chosen_destroy_op_name, chosen_repair_op_name)
            if combo_key not in global_combo_stats:
                global_combo_stats[combo_key] = {
                    'uses': 0, 'total_score': 0.0, 'improvements': 0,
                    'new_best_count': 0, 'better_current_count': 0,
                    'total_improvement': 0.0, 'best_improvement': 0.0,
                }
            cs = global_combo_stats[combo_key]
            cs['uses'] += 1
            cs['total_score'] += score
            if found_new_best:
                cs['new_best_count'] += 1
            if found_improvement:
                cs['better_current_count'] += 1
                cs['improvements'] += 1
                cs['total_improvement'] += improvement_delta
                if improvement_delta > cs['best_improvement']:
                    cs['best_improvement'] = improvement_delta

            if found_improvement and improvement_delta > best_operator_combo['best_improvement']:
                best_operator_combo.update({
                    'destroy': chosen_destroy_op_name, 'repair': chosen_repair_op_name,
                    'best_improvement': improvement_delta,
                    'iteration': iteration,
                    'objective_before': current_objective,
                    'objective_after': new_objective,
                })

            iteration_operator_log.append({
                'iteration': iteration,
                'strategy': chosen_strategy,
                'destroy_operator': chosen_destroy_op_name,
                'repair_operator': chosen_repair_op_name,
                'score': score,
                'accepted': accepted,
                'found_new_best': found_new_best,
                'found_improvement': found_improvement,
                'current_objective': current_objective,
                'new_objective': new_objective,
                'best_objective': best_objective,
                'temperature': self.temperature,
                'improvement_delta': improvement_delta,
            })

            if score > 0:

                self.strategy_weights[chosen_strategy] = \
                    (1 - self.reaction_factor) * self.strategy_weights[chosen_strategy] + \
                    self.reaction_factor * score

                self.operator_weights[chosen_strategy]['destroy'][chosen_destroy_op_name] = \
                    (1 - self.reaction_factor) * self.operator_weights[chosen_strategy]['destroy'][chosen_destroy_op_name] + \
                    self.reaction_factor * score

                self.operator_weights[chosen_strategy]['repair'][chosen_repair_op_name] = \
                    (1 - self.reaction_factor) * self.operator_weights[chosen_strategy]['repair'][chosen_repair_op_name] + \
                    self.reaction_factor * score
            else:

                decay = 0.95
                self.operator_weights[chosen_strategy]['destroy'][chosen_destroy_op_name] = max(
                    0.1,
                    self.operator_weights[chosen_strategy]['destroy'][chosen_destroy_op_name] * decay
                )
                self.operator_weights[chosen_strategy]['repair'][chosen_repair_op_name] = max(
                    0.1,
                    self.operator_weights[chosen_strategy]['repair'][chosen_repair_op_name] * decay
                )

                self.strategy_weights[chosen_strategy] = max(
                    0.1,
                    self.strategy_weights[chosen_strategy] * decay
                )

            history_entry = {'iteration': iteration, 'strategy': chosen_strategy}
            for strat in self.strategy_weights:
                history_entry[f'strategy_{strat}'] = self.strategy_weights[strat]
                for n in destroy_names:
                    history_entry[f'{strat}_destroy_{n}'] = self.operator_weights[strat]['destroy'].get(n, 0)
                for n in repair_names:
                    history_entry[f'{strat}_repair_{n}'] = self.operator_weights[strat]['repair'].get(n, 0)
            operator_history.append(history_entry)

            improved_this_iter = False
            if accepted:

                repaired_state.destroyed_customers_info = {}
                current_state = repaired_state.fast_copy()
                current_objective = new_objective
                if new_objective < best_objective:
                    best_state = repaired_state.fast_copy()
                    best_objective = new_objective
                    self._best_objective_for_accept = best_objective
                    y_best.append(best_objective)
                    improved_this_iter = True
            else:

                pass

            if final_new_objective < final_best_objective:
                best_final_state = repaired_state.fast_copy()
                final_best_objective = final_new_objective
                best_final_state.final_best_objective = final_best_objective
                best_final_uav_cost = sum(repaired_state.final_uav_cost.values())
                best_final_objective = final_best_objective
                best_final_win_cost = sum(finial_uav_tw_violation_cost.values())
                best_final_vehicle_max_times = final_vehicle_max_times
                best_final_global_max_time = final_global_max_time
                best_total_win_cost = finial_window_total_cost
                best_final_vehicle_route_cost = final_total_objective_value - finial_window_total_cost
            else:

                pass

            self.temperature *= cooling_rate
            y_cost.append(current_objective)

            if improved_this_iter:
                no_improve_count = 0
            else:
                no_improve_count += 1

            if no_improve_count >= no_improve_threshold // 3:
                result = self.enhanced_stagnation_handler(
                    no_improve_count, no_improve_threshold,
                    current_state, best_state, best_objective, iteration)
                new_state, new_obj, new_temp, no_improve_count = result
                current_state = new_state
                if new_obj is not None:
                    current_objective = new_obj
                self.temperature = new_temp

            if iteration % 10 == 0:
                elapsed_time = time.time() - start_time
                print(f"  > 进度: 迭代 {iteration}, 当前成本: {current_objective:.2f}, 最优成本: {best_objective:.2f}, 运行时间: {elapsed_time:.2f}秒")

                print(f"  > 策略权重: {self.strategy_weights}")
                print(f"  > 算子权重: {self.operator_weights}")

            iteration += 1

        elapsed_time = time.time() - start_time

        best_arrive_time = best_state.calculate_rm_empty_vehicle_arrive_time(best_state.vehicle_routes)
        best_window_total_cost, best_uav_tw_violation_cost, best_total_cost_dict  = calculate_window_cost(best_state.customer_plan,
                          best_state.uav_cost,
                          best_state.rm_empty_vehicle_arrive_time,
                          self.vehicle,
                          self.customer_time_windows_h,
                          self.early_arrival_cost,
                          self.late_arrival_cost,
                          self.uav_travel,
                          self.node)

        best_vehicle_max_times, best_global_max_time = get_max_completion_time(best_arrive_time)
        best_total_uav_tw_violation_cost = sum(best_uav_tw_violation_cost.values())
        best_total_vehicle_cost = best_objective - best_window_total_cost

        for n, ds in global_destroy_stats.items():
            ds['avg_score'] = ds['total_score'] / ds['total_uses'] if ds['total_uses'] > 0 else 0.0
            ds['avg_improvement'] = ds['total_improvement'] / ds['improvements'] if ds['improvements'] > 0 else 0.0
        for n, rs in global_repair_stats.items():
            rs['avg_score'] = rs['total_score'] / rs['total_uses'] if rs['total_uses'] > 0 else 0.0
            rs['avg_improvement'] = rs['total_improvement'] / rs['improvements'] if rs['improvements'] > 0 else 0.0

        operator_statistics = {
            'destroy': global_destroy_stats,
            'repair': global_repair_stats,
        }

        best_destroy_by_avg_score = max(global_destroy_stats.items(), key=lambda x: x[1]['avg_score']) if global_destroy_stats else None
        best_repair_by_avg_score  = max(global_repair_stats.items(),  key=lambda x: x[1]['avg_score']) if global_repair_stats else None

        best_combo_by_avg_score = None
        best_combo_by_new_best  = None
        if global_combo_stats:
            best_combo_by_avg_score = max(global_combo_stats.keys(),
                key=lambda k: global_combo_stats[k]['total_score'] / max(global_combo_stats[k]['uses'], 1))
            best_combo_by_new_best  = max(global_combo_stats.keys(),
                key=lambda k: global_combo_stats[k]['new_best_count'])

        save_alns_results(
            instance_name=self.problemName + "HA" + str(self.iter),
            y_best=y_best,
            y_cost=y_cost,
            win_cost=win_cost,
            uav_route_cost=uav_route_cost,
            vehicle_route_cost=vehicle_route_cost,
            strategy_weights=self.strategy_weights,
            operator_weights=self.operator_weights,
            elapsed_time=elapsed_time,
            best_objective=best_objective,
            best_vehicle_max_times=best_vehicle_max_times,
            best_global_max_time=best_global_max_time,
            best_arrive_time=best_arrive_time,
            best_window_total_cost=best_window_total_cost,
            best_uav_tw_violation_cost=best_uav_tw_violation_cost,
            best_total_cost_dict=best_total_cost_dict,
            best_state=best_state,

            best_final_uav_cost=best_final_uav_cost,
            best_final_objective=best_final_objective,
            best_final_win_cost=best_final_win_cost,
            best_total_win_cost=best_total_win_cost,
            best_final_vehicle_route_cost=best_final_vehicle_route_cost,
            final_uav_cost=final_uav_cost,
            final_total_list=final_total_list,
            final_win_cost=final_win_cost,
            final_total_objective=final_total_objective,
            final_vehicle_route_cost=final_vehicle_route_cost,

            best_final_vehicle_max_times=best_final_vehicle_max_times,
            best_final_global_max_time=best_final_global_max_time,
            work_time=work_time,
            final_work_time=final_work_time,
            best_final_state=best_final_state,

            operator_statistics=operator_statistics,
            operator_combinations=global_combo_stats,
            best_operator_combo=best_operator_combo,
            iteration_operator_log=iteration_operator_log,
            best_destroy_by_avg_score=best_destroy_by_avg_score,
            best_repair_by_avg_score=best_repair_by_avg_score,
            best_combo_by_avg_score=best_combo_by_avg_score,
            best_combo_by_new_best=best_combo_by_new_best,
            operator_history=operator_history,
            base_dir=self.summary_dir,
        )
        print(f"ALNS求解完成，最终成本: {best_objective}, 迭代次数: {iteration}, 运行时间: {elapsed_time:.2f}秒")
        return best_state, best_final_state, best_objective, best_final_objective, best_final_uav_cost, best_final_win_cost, best_total_win_cost, best_final_global_max_time, best_global_max_time, best_window_total_cost, best_total_uav_tw_violation_cost, best_total_vehicle_cost, elapsed_time, win_cost, uav_route_cost, vehicle_route_cost, final_uav_cost, final_total_list, final_win_cost, final_total_objective, y_cost, y_best, work_time, final_work_time

    def _roulette_wheel_select(self, weights):

        total_weight = sum(weights)
        if total_weight == 0:
            return self.rng.integers(0, len(weights))

        r = self.rng.random() * total_weight
        cumulative_weight = 0
        for i, weight in enumerate(weights):
            cumulative_weight += weight
            if r <= cumulative_weight:
                return i
        return len(weights) - 1

    def _simulated_annealing_accept(self, current_cost, new_cost, temperature):

        if new_cost < current_cost:
            return True
        else:
            delta = new_cost - current_cost
            probability = np.exp(-delta / temperature)
            return self.rng.random() < probability

    def _sync_cost_with_plan(self, state, context_label=""):

        if state is None:
            return
        if state.uav_cost is None:
            state.uav_cost = {}
        try:
            plan_keys = set(state.customer_plan.keys())
            cost_keys = set(state.uav_cost.keys())
        except Exception:
            return

        extra_cost = cost_keys - plan_keys
        for c in extra_cost:
            state.uav_cost.pop(c, None)

        missing_cost = plan_keys - cost_keys
        for c in missing_cost:
            try:
                assignment = state.customer_plan[c]
                drone_id, launch_node, customer_node, recovery_node, _, _ = assignment
                est = self.drone_insert_cost(drone_id, customer_node, launch_node, recovery_node)
                state.uav_cost[c] = est
            except Exception:
                state.uav_cost[c] = 0

    def destroy_random_removal(self, state, force_vtp_mode = None):

        """
        随机破坏算子，实现了双重模式以适应自适应策略选择框架。
        它既可以随机移除少量VTP节点以重构路径，也可以随机移除大量客户以重组任务。
        Args:
            state (FastMfstspState): 当前解的状态。
            force_vtp_mode (bool, optional):
                - True:  强制执行VTP破坏模式 (用于“结构性重组”策略)。
                - False: 强制执行客户破坏模式 (用于“内部精细优化”策略)。
                - None: (默认) 按预设概率随机选择一种模式 (此模式在当前框架下不会被触发，但保留以备后用)。

        Returns:
            FastMfstspState: 被部分破坏后的解的状态。
        """

        new_state = state.fast_copy()

        new_state.destroyed_customers_info = {}

        current_customers = list(new_state.customer_plan.keys())
        if not current_customers:
            print("没有客户点需要移除")
            return new_state

        mode = 'vtp' if force_vtp_mode else 'customer'
        print(f"  > [破坏模式]: 随机破坏 ({'VTP模式' if mode == 'vtp' else '客户模式'})")
        vehicle_task_data = new_state.vehicle_task_data
        if mode == 'vtp':

            active_vtps = []
            destroyed_vts_info = {}
            for vehicle_id, route in enumerate(new_state.vehicle_routes):
                v_id = vehicle_id + 1
                for vtp_node in route[1:-1]:
                    active_vtps.append((v_id, vtp_node))

            low, high = self.vtp_destroy_quantity['random']
            num_to_remove = self.rng.integers(low, min(len(active_vtps), high) + 1)
            print(f"VTP破坏策略：目标破坏 {num_to_remove} 个VTP节点，候选池共有 {len(active_vtps)} 个节点")

            destroyed_customers_info = {}
            destroyed_vtp_count = 0
            max_attempts = len(active_vtps) * 2
            attempt_count = 0

            candidate_vtps = active_vtps.copy()

            while destroyed_vtp_count < num_to_remove and candidate_vtps and attempt_count < max_attempts:
                attempt_count += 1

                if not candidate_vtps:
                    print(f"候选池已空，无法继续破坏VTP节点")
                    break

                selected_index = self.rng.integers(0, len(candidate_vtps))
                vehicle_id, vtp_node = candidate_vtps.pop(selected_index)
                if vtp_node not in new_state.rm_empty_vehicle_route[vehicle_id-1]:
                    continue

                customers_to_remove = []
                for customer, assignment in list(new_state.customer_plan.items()):
                    uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle = assignment

                    if (launch_node == vtp_node and launch_vehicle == vehicle_id) or \
                       (recovery_node == vtp_node and recovery_vehicle == vehicle_id):
                        customers_to_remove.append(customer)

                all_tasks_to_remove = []
                temp_vehicle_task_data = deep_copy_vehicle_task_data(vehicle_task_data)

                temp_customer_plan = {k: v for k, v in new_state.customer_plan.items()}
                temp_rm_empty_vehicle_route = [route[:] for route in new_state.rm_empty_vehicle_route]

                for customer in customers_to_remove:
                    if customer in temp_customer_plan:
                        assignment = temp_customer_plan[customer]
                        all_tasks_to_remove.append((customer, assignment))

                        need_to_remove_tasks = find_chain_tasks(assignment, temp_customer_plan, new_state.vehicle_routes, temp_vehicle_task_data)
                        all_tasks_to_remove.extend(need_to_remove_tasks)

                for customer, assignment in all_tasks_to_remove:
                    if customer in temp_customer_plan:
                        temp_customer_plan.pop(customer, None)

                if vtp_node in temp_rm_empty_vehicle_route[vehicle_id-1]:
                    temp_rm_empty_vehicle_route[vehicle_id-1].remove(vtp_node)

                temp_rm_vehicle_arrive_time = new_state.calculate_rm_empty_vehicle_arrive_time(temp_rm_empty_vehicle_route)

                if not is_time_feasible(temp_customer_plan, temp_rm_vehicle_arrive_time):

                    continue

                new_state.rm_empty_vehicle_route[vehicle_id-1].remove(vtp_node)
                destroyed_vts_info[(vehicle_id, vtp_node)] = True
                destroyed_vtp_count += 1

                for customer, assignment in all_tasks_to_remove:
                    if customer in new_state.customer_plan:
                        uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle = assignment

                        customer_info = [uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle, new_state.uav_cost.get(customer, 0) if new_state.uav_cost else 0]
                        destroyed_customers_info[customer] = customer_info

                        new_state.customer_plan.pop(customer, None)

                        if uav_id in new_state.uav_assignments:
                            new_state.uav_assignments[uav_id] = [
                                task for task in new_state.uav_assignments[uav_id]
                                if task[2] != customer_node
                            ]

                        if new_state.uav_cost and customer_node in new_state.uav_cost:
                            new_state.uav_cost.pop(customer_node, None)

                        vehicle_task_data = remove_vehicle_task(vehicle_task_data, assignment, new_state.vehicle_routes)
                        orig_vehicle_id = assignment[4]

                        from task_data import deep_remove_vehicle_task
                        need_to_remove_tasks = find_chain_tasks(assignment, new_state.customer_plan, new_state.vehicle_routes, vehicle_task_data)
                        for chain_customer, chain_assignment in need_to_remove_tasks:

                            if chain_customer in new_state.customer_plan:
                                chain_uav_id, chain_launch_node, chain_customer_node, chain_recovery_node, chain_launch_vehicle, chain_recovery_vehicle = chain_assignment

                                chain_customer_info = [chain_uav_id, chain_launch_node, chain_customer_node, chain_recovery_node, chain_launch_vehicle, chain_recovery_vehicle, new_state.uav_cost.get(chain_customer, 0) if new_state.uav_cost else 0]
                                destroyed_customers_info[chain_customer] = chain_customer_info

                                new_state.customer_plan.pop(chain_customer, None)

                                if chain_uav_id in new_state.uav_assignments:
                                    new_state.uav_assignments[chain_uav_id] = [
                                        task for task in new_state.uav_assignments[chain_uav_id]
                                        if task[2] != chain_customer_node
                                    ]

                                if new_state.uav_cost and chain_customer_node in new_state.uav_cost:
                                    new_state.uav_cost.pop(chain_customer_node, None)

                                vehicle_task_data = deep_remove_vehicle_task(vehicle_task_data, chain_assignment, new_state.vehicle_routes, orig_vehicle_id)

            if destroyed_vtp_count == num_to_remove:
                print(f"VTP破坏策略成功完成：目标 {num_to_remove} 个，实际破坏 {destroyed_vtp_count} 个VTP节点，共删除 {len(destroyed_customers_info)} 个客户点")
            elif destroyed_vtp_count > 0:
                print(f"VTP破坏策略部分完成：目标 {num_to_remove} 个，实际破坏 {destroyed_vtp_count} 个VTP节点，共删除 {len(destroyed_customers_info)} 个客户点")
            else:
                print(f"VTP破坏策略失败：目标 {num_to_remove} 个，实际破坏 {destroyed_vtp_count} 个VTP节点，共删除 {len(destroyed_customers_info)} 个客户点")
                print(f"警告：VTP破坏失败，destroyed_customers_info为空: {destroyed_customers_info}")

                print("VTP破坏失败，回退到客户破坏模式...")

                return state

                if current_customers:
                    fallback_customer = self.rng.choice(current_customers)
                    if fallback_customer in new_state.customer_plan:
                        assignment = new_state.customer_plan.pop(fallback_customer)
                        uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle = assignment

                        customer_info = [uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle, new_state.uav_cost.get(fallback_customer, 0) if new_state.uav_cost else 0]
                        destroyed_customers_info[fallback_customer] = customer_info

                        if uav_id in new_state.uav_assignments:
                            new_state.uav_assignments[uav_id] = [
                                task for task in new_state.uav_assignments[uav_id]
                                if task[2] != customer_node
                            ]

                        if new_state.uav_cost and customer_node in new_state.uav_cost:
                            new_state.uav_cost.pop(customer_node, None)

                        vehicle_task_data = remove_vehicle_task(vehicle_task_data, assignment, new_state.vehicle_routes)

                        print(f"回退破坏：成功破坏客户点 {fallback_customer}")
                    else:
                        print("回退破坏：无法找到可破坏的客户点")
                else:
                    print("回退破坏：没有可用的客户点")

            new_state.destroyed_vts_info = destroyed_vts_info
            new_state.destroyed_customers_info = destroyed_customers_info
            new_state.vehicle_task_data = vehicle_task_data

            new_state.vehicle_routes = [route[:] for route in new_state.rm_empty_vehicle_route]

            new_state.rm_vehicle_arrive_time = new_state.calculate_rm_empty_vehicle_arrive_time(new_state.vehicle_routes)
            new_state.destroyed_node_cost = new_state.update_calculate_plan_cost(new_state.uav_cost, new_state.vehicle_routes)

            print("=== VTP破坏阶段完成 ===\n")
        else:

            n = len(current_customers)
            num_to_remove = self.rng.integers(
                max(1, int(n * 0.2)),
                max(2, int(n * 0.3)) + 1
            )
            customers_to_remove = self.rng.choice(current_customers, num_to_remove, replace=False)

            print(f"随机破坏：移除 {len(customers_to_remove)} 个客户点: {customers_to_remove}")
            destroyed_customers_info = {}

            for customer in customers_to_remove:
                if customer in new_state.customer_plan:
                    assignment = new_state.customer_plan.pop(customer)
                    uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle = assignment

                    customer_info = [uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle, new_state.uav_cost.get(customer, 0) if new_state.uav_cost else 0]
                    destroyed_customers_info[customer] = customer_info

                    if uav_id in new_state.uav_assignments:
                        new_state.uav_assignments[uav_id] = [
                            task for task in new_state.uav_assignments[uav_id]
                            if task[2] != customer_node
                        ]

                    if new_state.uav_cost and customer_node in new_state.uav_cost:
                        new_state.uav_cost.pop(customer_node, None)
                    orig_vehicle_id = assignment[4]

                    vehicle_task_data = remove_vehicle_task(vehicle_task_data, assignment, new_state.vehicle_routes)

                    need_to_remove_tasks = find_chain_tasks(assignment, new_state.customer_plan, new_state.vehicle_routes, new_state.vehicle_task_data)

                    for chain_customer, chain_assignment in need_to_remove_tasks:
                        if chain_customer in new_state.customer_plan:
                            chain_uav_id, chain_launch_node, chain_customer_node, chain_recovery_node, chain_launch_vehicle, chain_recovery_vehicle = chain_assignment

                            chain_customer_info = [chain_uav_id, chain_launch_node, chain_customer_node, chain_recovery_node, chain_launch_vehicle, chain_recovery_vehicle, new_state.uav_cost.get(chain_customer, 0) if new_state.uav_cost else 0]
                            destroyed_customers_info[chain_customer] = chain_customer_info

                            new_state.customer_plan.pop(chain_customer, None)

                            if chain_uav_id in new_state.uav_assignments:
                                new_state.uav_assignments[chain_uav_id] = [
                                    task for task in new_state.uav_assignments[chain_uav_id]
                                    if task[2] != chain_customer_node
                                ]

                            if new_state.uav_cost and chain_customer_node in new_state.uav_cost:
                                new_state.uav_cost.pop(chain_customer_node, None)
                            from task_data import deep_remove_vehicle_task

                            vehicle_task_data = deep_remove_vehicle_task(vehicle_task_data, chain_assignment, new_state.vehicle_routes, orig_vehicle_id)

            new_state.destroyed_node_cost = new_state.update_calculate_plan_cost(new_state.uav_cost, new_state.vehicle_routes)

            new_state.destroyed_customers_info = destroyed_customers_info
            new_state.vehicle_task_data = vehicle_task_data
            print(f"破坏后剩余客户点: {len(new_state.customer_plan)}")
            print("=== 破坏阶段完成 ===\n")
        return new_state

    def destroy_comprehensive_removal(self, state, force_vtp_mode = None):
        new_state = state.fast_copy()

        new_state.destroyed_customers_info = {}
        current_customers = list(new_state.customer_plan.keys())
        temp_vehicle_route = [route[:] for route in new_state.vehicle_routes]
        temp_vehicle_arrive_time = new_state.calculate_rm_empty_vehicle_arrive_time(temp_vehicle_route)
        window_total_cost, uav_tw_violation_cost, total_cost_dict = calculate_window_cost(new_state.customer_plan, new_state.uav_cost, temp_vehicle_arrive_time, self.vehicle, self.customer_time_windows_h,
            self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node)
        vehicle_task_data = new_state.vehicle_task_data
        mode = 'vtp' if force_vtp_mode else 'customer'

        if mode == 'vtp':

            vtp_metrics = {}

            vtp_drone_performance = defaultdict(lambda: {'total_cost': 0.0, 'task_count': 0})

            for customer, assignment in new_state.customer_plan.items():
                try:
                    uav_id, launch_node, _, recovery_node, launch_veh, recovery_veh = assignment

                    task_total_cost = total_cost_dict.get(customer, 0.0)

                    launch_node_map_key = self.node[launch_node].map_key
                    recovery_node_map_key = self.node[recovery_node].map_key
                    customer_map_key = customer
                    per_cost = self.vehicle[uav_id].per_cost

                    physical_cost_leg1 = self.uav_travel[uav_id][launch_node_map_key][customer_map_key].totalDistance * per_cost
                    physical_cost_leg2 = self.uav_travel[uav_id][customer_map_key][recovery_node_map_key].totalDistance * per_cost
                    total_physical = physical_cost_leg1 + physical_cost_leg2
                    penalty_cost = uav_tw_violation_cost.get(customer, 0.0)
                    cost_allocated_launch = physical_cost_leg1 + penalty_cost
                    cost_allocated_recovery = physical_cost_leg2

                except Exception as e:
                    continue

                launch_key = (launch_veh, launch_node)
                vtp_drone_performance[launch_key]['total_cost'] += cost_allocated_launch
                vtp_drone_performance[launch_key]['task_count'] += 1

                recovery_key = (recovery_veh, recovery_node)
                vtp_drone_performance[recovery_key]['total_cost'] += cost_allocated_recovery
                vtp_drone_performance[recovery_key]['task_count'] += 1

            epsilon = 1e-6
            active_vtps_keys = set()
            for vehicle_id_minus_1, route in enumerate(new_state.vehicle_routes):
                vehicle_id = vehicle_id_minus_1 + 1
                if len(route) <= 2: continue

                for node_idx in range(1, len(route) - 1):
                    vtp_node = route[node_idx]
                    vtp_key = (vehicle_id, vtp_node)
                    active_vtps_keys.add(vtp_key)

                    drone_data = vtp_drone_performance.get(vtp_key, {'total_cost': 0.0, 'task_count': 0})
                    task_count = drone_data['task_count']
                    score_drone = drone_data['total_cost'] / (task_count + epsilon)

                    prev_node = route[node_idx - 1]
                    next_node = route[node_idx + 1]

                    try:
                        if prev_node not in self.veh_distance[vehicle_id] or \
                        vtp_node not in self.veh_distance[vehicle_id][prev_node] or \
                        next_node not in self.veh_distance[vehicle_id][vtp_node] or \
                        next_node not in self.veh_distance[vehicle_id][prev_node]:
                            raise KeyError("Missing distance data")

                        detour_cost = self.veh_distance[vehicle_id][prev_node][vtp_node] + \
                                    self.veh_distance[vehicle_id][vtp_node][next_node] - \
                                    self.veh_distance[vehicle_id][prev_node][next_node]
                    except (KeyError, IndexError) as e:
                        print(f"  > 警告: 无法计算VTP {vtp_key} 的绕路成本 ({prev_node}->{vtp_node}->{next_node})。设为0。错误: {e}")
                        detour_cost = 0.0

                    score_vehicle = detour_cost / (task_count + epsilon)

                    vtp_metrics[vtp_key] = {
                        'score_drone': score_drone,
                        'score_vehicle': score_vehicle,
                        'task_count': task_count
                    }

            active_vtp_list = list(vtp_metrics.keys())
            N = len(active_vtp_list)

            if N == 0:
                print("  > 警告: 没有可评估的活动VTP节点，本次破坏无操作。")
                return new_state

            P_thresh = 0.3

            T = max(1, math.ceil(N * P_thresh))

            sorted_by_drone = sorted(active_vtp_list, key=lambda k: vtp_metrics[k]['score_drone'], reverse=True)

            sorted_by_vehicle = sorted(active_vtp_list, key=lambda k: vtp_metrics[k]['score_vehicle'], reverse=True)

            P_worst_drone = set(sorted_by_drone[:T])
            P_worst_vehicle = set(sorted_by_vehicle[:T])

            P_pareto = P_worst_drone.intersection(P_worst_vehicle)

            candidate_keys_sorted = []

            if P_pareto:
                print(f"  > Pareto筛选: 找到 {len(P_pareto)} 个双差生VTP。")

                candidate_keys_sorted = sorted(list(P_pareto), key=lambda k: vtp_metrics[k]['score_drone'], reverse=True)
            elif P_worst_drone:
                print("  > Pareto筛选: 未找到双差生，仅基于无人机效率选择。")

                candidate_keys_sorted = sorted_by_drone[:T]
            else:

                print("  > 警告: 无法确定候选VTP池，本次破坏无操作。")
                return new_state

            if not candidate_keys_sorted:
                print("  > 警告: 最终候选VTP池为空，本次破坏无操作。")
                return new_state

            num_to_remove = 5

            num_to_remove = min(num_to_remove, len(candidate_keys_sorted))

            K = 5

            top_k_candidates_keys = candidate_keys_sorted[:K]

            vtps_to_destroy = []

            if not top_k_candidates_keys:
                print("  > 警告: Top-K 候选池为空，本次破坏无操作。")
                return new_state
            elif len(top_k_candidates_keys) == 1 or num_to_remove == 0:

                vtps_to_destroy = top_k_candidates_keys[:num_to_remove]
            else:

                weights = np.arange(len(top_k_candidates_keys), 0, -1)

                weight_sum = np.sum(weights)
                probabilities = weights / weight_sum if weight_sum > 0 else None

                if probabilities is None:
                    print("  > 警告: 无法计算选择概率，将选择Top-N。")

                    vtps_to_destroy = top_k_candidates_keys[:num_to_remove]
                else:

                    chosen_indices = self.rng.choice(len(top_k_candidates_keys),
                                                    size=num_to_remove,
                                                    p=probabilities,
                                                    replace=False)

                    vtps_to_destroy = [top_k_candidates_keys[i] for i in chosen_indices]

            destroyed_customers_info = new_state.destroyed_customers_info

            destroyed_vts_info = getattr(new_state, 'destroyed_vts_info', {})

            destroyed_vtp_count = 0
            actual_destroyed_vtps = []

            destroyed_customers_info = {}
            destroyed_vtp_count = 0
            max_attempts = len(vtps_to_destroy) * 2
            attempt_count = 0

            candidate_vtps = vtps_to_destroy.copy()

            while destroyed_vtp_count < self.vtp_destroy_quantity['worst'] and candidate_vtps and attempt_count < max_attempts:
                attempt_count += 1

                if not candidate_vtps:
                    print(f"候选池已空，无法继续破坏VTP节点")
                    break

                vehicle_id, vtp_node = candidate_vtps.pop(0)
                if vtp_node not in new_state.rm_empty_vehicle_route[vehicle_id-1]:

                    continue

                customers_to_remove = []
                for customer, assignment in list(new_state.customer_plan.items()):
                    uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle = assignment

                    if (launch_node == vtp_node and launch_vehicle == vehicle_id) or \
                       (recovery_node == vtp_node and recovery_vehicle == vehicle_id):
                        customers_to_remove.append(customer)

                all_tasks_to_remove = []
                temp_vehicle_task_data = deep_copy_vehicle_task_data(vehicle_task_data)

                temp_customer_plan = {k: v for k, v in new_state.customer_plan.items()}
                temp_rm_empty_vehicle_route = [route[:] for route in new_state.rm_empty_vehicle_route]

                for customer in customers_to_remove:
                    if customer in temp_customer_plan:
                        assignment = temp_customer_plan[customer]
                        all_tasks_to_remove.append((customer, assignment))

                        need_to_remove_tasks = find_chain_tasks(assignment, temp_customer_plan, new_state.vehicle_routes, temp_vehicle_task_data)
                        all_tasks_to_remove.extend(need_to_remove_tasks)

                for customer, assignment in all_tasks_to_remove:
                    if customer in temp_customer_plan:
                        temp_customer_plan.pop(customer, None)

                if vtp_node in temp_rm_empty_vehicle_route[vehicle_id-1]:
                    temp_rm_empty_vehicle_route[vehicle_id-1].remove(vtp_node)

                temp_rm_vehicle_arrive_time = new_state.calculate_rm_empty_vehicle_arrive_time(temp_rm_empty_vehicle_route)

                if not is_time_feasible(temp_customer_plan, temp_rm_vehicle_arrive_time):
                    print(f"VTP节点 {vtp_node} 删除后不满足时间约束，跳过删除 (尝试 {attempt_count}/{max_attempts})")

                    continue

                print(f"成功破坏VTP节点: 车辆{vehicle_id}的节点{vtp_node} (进度: {destroyed_vtp_count + 1}/{len(vtps_to_destroy)})")

                new_state.rm_empty_vehicle_route[vehicle_id-1].remove(vtp_node)
                destroyed_vts_info[(vehicle_id-1, vtp_node)] = True
                destroyed_vtp_count += 1

                for customer, assignment in all_tasks_to_remove:
                    if customer in new_state.customer_plan:
                        uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle = assignment

                        customer_info = [uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle, new_state.uav_cost.get(customer, 0) if new_state.uav_cost else 0]
                        destroyed_customers_info[customer] = customer_info

                        new_state.customer_plan.pop(customer, None)

                        if uav_id in new_state.uav_assignments:
                            new_state.uav_assignments[uav_id] = [
                                task for task in new_state.uav_assignments[uav_id]
                                if task[2] != customer_node
                            ]

                        if new_state.uav_cost and customer_node in new_state.uav_cost:
                            new_state.uav_cost.pop(customer_node, None)

                        vehicle_task_data = remove_vehicle_task(vehicle_task_data, assignment, new_state.vehicle_routes)
                        orig_vehicle_id = assignment[4]

                        from task_data import deep_remove_vehicle_task
                        need_to_remove_tasks = find_chain_tasks(assignment, new_state.customer_plan, new_state.vehicle_routes, vehicle_task_data)
                        for chain_customer, chain_assignment in need_to_remove_tasks:
                            if chain_customer in new_state.customer_plan:
                                chain_uav_id, chain_launch_node, chain_customer_node, chain_recovery_node, chain_launch_vehicle, chain_recovery_vehicle = chain_assignment

                                chain_customer_info = [chain_uav_id, chain_launch_node, chain_customer_node, chain_recovery_node, chain_launch_vehicle, chain_recovery_vehicle, new_state.uav_cost.get(chain_customer, 0) if new_state.uav_cost else 0]
                                destroyed_customers_info[chain_customer] = chain_customer_info

                                new_state.customer_plan.pop(chain_customer, None)

                                if chain_uav_id in new_state.uav_assignments:
                                    new_state.uav_assignments[chain_uav_id] = [
                                        task for task in new_state.uav_assignments[chain_uav_id]
                                        if task[2] != chain_customer_node
                                    ]

                                if new_state.uav_cost and chain_customer_node in new_state.uav_cost:
                                    new_state.uav_cost.pop(chain_customer_node, None)

                                vehicle_task_data = deep_remove_vehicle_task(vehicle_task_data, chain_assignment, new_state.vehicle_routes, orig_vehicle_id)

            print(f"VTP最差破坏策略完成：成功破坏 {destroyed_vtp_count}/{len(vtps_to_destroy)} 个VTP节点")

            new_state.destroyed_vts_info = destroyed_vts_info
            new_state.destroyed_customers_info = destroyed_customers_info
            new_state.vehicle_task_data = vehicle_task_data

            new_state.vehicle_routes = [route[:] for route in new_state.rm_empty_vehicle_route]
            new_state.rm_vehicle_arrive_time = new_state.calculate_rm_empty_vehicle_arrive_time(new_state.vehicle_routes)
            new_state.destroyed_node_cost = new_state.update_calculate_plan_cost(new_state.uav_cost, new_state.vehicle_routes)

            print("=== VTP破坏阶段完成 ===\n")
        else:

            print("  > [破坏模式]: 综合最差破坏 (客户模式 - Pareto)")

            current_customers = list(new_state.customer_plan.keys())
            if not current_customers:
                print("  > 警告: 没有已服务的客户可供破坏。")
                return new_state

            customer_metrics = {}

            for customer in current_customers:

                cost = total_cost_dict.get(customer, 0.0)

                score_cost = cost

                score_duration = 0.0
                try:
                    assignment = new_state.customer_plan[customer]
                    uav_id, launch_node, _, recovery_node, launch_veh, recovery_veh = assignment

                    launch_node_map_key = self.node[launch_node].map_key
                    recovery_node_map_key = self.node[recovery_node].map_key

                    time_leg1 = self.uav_travel[uav_id][launch_node_map_key][customer].totalTime
                    time_leg2 = self.uav_travel[uav_id][customer][recovery_node_map_key].totalTime
                    score_duration = time_leg1 + time_leg2

                except Exception as e:

                    score_duration = 0.0

                customer_metrics[customer] = {
                    'score_cost': score_cost,
                    'score_duration': score_duration
                }

            if not customer_metrics:
                return new_state

            active_customer_list = list(customer_metrics.keys())
            N = len(active_customer_list)
            P_thresh = 0.5
            T = max(1, math.ceil(N * P_thresh))

            sorted_by_cost = sorted(active_customer_list, key=lambda k: customer_metrics[k]['score_cost'], reverse=True)

            sorted_by_duration = sorted(active_customer_list, key=lambda k: customer_metrics[k]['score_duration'], reverse=True)

            K_worst_cost = set(sorted_by_cost[:T])
            K_worst_duration = set(sorted_by_duration[:T])

            K_pareto = K_worst_cost.intersection(K_worst_duration)

            candidate_keys_sorted = []
            if K_pareto:
                print(f"  > Pareto筛选: 找到 {len(K_pareto)} 个双差生客户。")
                candidate_keys_sorted = sorted(list(K_pareto), key=lambda k: customer_metrics[k]['score_cost'], reverse=True)
            elif K_worst_cost:
                print("  > Pareto筛选: 未找到双差生，仅基于任务成本选择。")
                candidate_keys_sorted = sorted_by_cost[:T]
            else:
                print("  > 警告: 无法确定客户候选池，本次破坏无操作。")
                return new_state

            if not candidate_keys_sorted:
                print("  > 警告: 最终客户候选池为空，本次破坏无操作。")
                return new_state

            n = len(current_customers)
            num_to_remove = self.rng.integers(
                max(1, int(n * self.customer_destroy_ratio[0])),
                max(2, int(n * self.customer_destroy_ratio[1])) + 1
            )
            num_to_remove = min(num_to_remove, n)

            K = max(10, 2 * num_to_remove)
            top_k_candidates_keys = candidate_keys_sorted[:K]

            customers_to_destroy = []

            if not top_k_candidates_keys:
                print("  > 警告: Top-K 客户候选池为空，本次破坏无操作。")
                return new_state
            elif len(top_k_candidates_keys) == 1 or num_to_remove == 0:
                customers_to_destroy = top_k_candidates_keys[:num_to_remove]
            else:
                weights = np.arange(len(top_k_candidates_keys), 0, -1)
                weight_sum = np.sum(weights)
                probabilities = weights / weight_sum if weight_sum > 0 else None

                if probabilities is None:
                    print("  > 警告: 无法计算客户选择概率，将选择成本最高的。")
                    customers_to_destroy = top_k_candidates_keys[:num_to_remove]
                else:
                    num_to_select = min(num_to_remove, len(top_k_candidates_keys))
                    chosen_indices = self.rng.choice(len(top_k_candidates_keys), size=num_to_select, p=probabilities, replace=False)
                    customers_to_destroy = [top_k_candidates_keys[i] for i in chosen_indices]

            print(f"  > 计划移除 {len(customers_to_destroy)} 个综合最差客户 (Top-{min(K, n)}随机): {customers_to_destroy}")

            destroyed_customers_info = new_state.destroyed_customers_info
            vehicle_task_data = new_state.vehicle_task_data

            for customer in customers_to_destroy:
                if customer in new_state.customer_plan:

                    temp_customer_plan = {k: v for k, v in new_state.customer_plan.items()}
                    temp_vehicle_routes = [route[:] for route in new_state.vehicle_routes]
                    temp_vehicle_task_data = deep_copy_vehicle_task_data(new_state.vehicle_task_data)
                    temp_assignment = new_state.customer_plan[customer]
                    temp_chain_tasks = find_chain_tasks(temp_assignment, temp_customer_plan, temp_vehicle_routes, temp_vehicle_task_data)
                    temp_customer_plan.pop(customer, None)
                    for chain_customer, chain_assignment in temp_chain_tasks:
                        temp_customer_plan.pop(chain_customer, None)
                    temp_rm_vehicle_arrive_time = new_state.calculate_rm_empty_vehicle_arrive_time(temp_vehicle_routes)
                    if not is_time_feasible(temp_customer_plan, temp_rm_vehicle_arrive_time):
                        continue

                    assignment = new_state.customer_plan.pop(customer)
                    uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle = assignment

                    customer_info = [uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle, new_state.uav_cost.get(customer, 0) if new_state.uav_cost else 0]
                    destroyed_customers_info[customer] = customer_info

                    if uav_id in new_state.uav_assignments:
                        new_state.uav_assignments[uav_id] = [
                            task for task in new_state.uav_assignments[uav_id]
                            if task[2] != customer_node
                        ]

                    if new_state.uav_cost and customer_node in new_state.uav_cost:
                        new_state.uav_cost.pop(customer_node, None)

                    vehicle_task_data = remove_vehicle_task(vehicle_task_data, assignment, new_state.vehicle_routes)

                    need_to_remove_tasks = find_chain_tasks(assignment, new_state.customer_plan, new_state.vehicle_routes, new_state.vehicle_task_data)
                    orig_vehicle_id = assignment[4]

                    for chain_customer, chain_assignment in need_to_remove_tasks:
                        if chain_customer in new_state.customer_plan:
                            chain_uav_id, chain_launch_node, chain_customer_node, chain_recovery_node, chain_launch_vehicle, chain_recovery_vehicle = chain_assignment

                            chain_customer_info = [chain_uav_id, chain_launch_node, chain_customer_node, chain_recovery_node, chain_launch_vehicle, chain_recovery_vehicle, new_state.uav_cost.get(chain_customer, 0) if new_state.uav_cost else 0]
                            destroyed_customers_info[chain_customer] = chain_customer_info

                            new_state.customer_plan.pop(chain_customer, None)

                            if chain_uav_id in new_state.uav_assignments:
                                new_state.uav_assignments[chain_uav_id] = [
                                    task for task in new_state.uav_assignments[chain_uav_id]
                                    if task[2] != chain_customer_node
                                ]

                            if new_state.uav_cost and chain_customer_node in new_state.uav_cost:
                                new_state.uav_cost.pop(chain_customer_node, None)
                            from task_data import deep_remove_vehicle_task

                            vehicle_task_data = deep_remove_vehicle_task(vehicle_task_data, chain_assignment, new_state.vehicle_routes, orig_vehicle_id)

            new_state.destroyed_node_cost = new_state.update_calculate_plan_cost(new_state.uav_cost, new_state.vehicle_routes)

            new_state.destroyed_customers_info = destroyed_customers_info
            new_state.vehicle_task_data = vehicle_task_data
            print(f"破坏后剩余客户点: {len(new_state.customer_plan)}")
            print("=== 破坏阶段完成 ===\n")
        return new_state

    def _calculate_vehicle_load(self, state):

        vehicle_load = {}
        costs = {}
        tasks = {}
        epsilon = 1e-6

        for vehicle_id_minus_1, route in enumerate(state.vehicle_routes):
            vehicle_id = vehicle_id_minus_1 + 1
            route_cost = 0
            task_count = 0

            if len(route) > 1:
                for i in range(len(route) - 1):
                    try:
                        route_cost += self.veh_distance[vehicle_id][route[i]][route[i+1]]
                    except KeyError:
                        pass

            for node in route[1:-1]:
                key = (vehicle_id, node)
                task_count += len(state.customer_plan.get(key, {}).get('launch_drone_list', []))
                task_count += len(state.customer_plan.get(key, {}).get('recovery_drone_list', []))

            costs[vehicle_id] = route_cost
            tasks[vehicle_id] = task_count

        max_cost = max(costs.values()) if costs else 1
        max_tasks = max(tasks.values()) if tasks else 1

        w_route = 0.5
        w_task = 0.5

        for v_id in costs.keys():
            norm_cost = costs[v_id] / (max_cost + epsilon)
            norm_task = tasks[v_id] / (max_tasks + epsilon)
            score_load = w_route * norm_cost + w_task * norm_task
            vehicle_load[v_id] = score_load

        return vehicle_load

    def destroy_shaw_rebalance_removal(self, state, force_vtp_mode = None):
        new_state = state.fast_copy()

        new_state.destroyed_customers_info = {}
        current_customers = list(new_state.customer_plan.keys())
        vehicle_task_data = new_state.vehicle_task_data
        mode = 'vtp' if force_vtp_mode else 'customer'

        vehicle_load_scores = self._calculate_vehicle_load(new_state)
        if not vehicle_load_scores or len(vehicle_load_scores) < 2:
            print("  > 警告: 无法计算车辆负载或车辆数不足，退化为随机破坏。")
            return self.destroy_random_removal(state, force_vtp_mode)

        sorted_vehicles = sorted(vehicle_load_scores.items(), key=lambda item: item[1])
        v_min_id = sorted_vehicles[0][0]
        v_max_id = sorted_vehicles[-1][0]

        if v_min_id == v_max_id:
            print("  > 警告: 车辆负载相同，退化为随机破坏。")
            return self.destroy_random_removal(state, force_vtp_mode)

        print(f"  > 负载分析: 最忙车辆 V{v_max_id} (Score: {sorted_vehicles[-1][1]:.2f}), 最闲车辆 V{v_min_id} (Score: {sorted_vehicles[0][1]:.2f})")

        v_min_route_nodes = new_state.vehicle_routes[v_min_id - 1][1:-1]
        v_min_positions = [(self.node[node].latDeg, self.node[node].lonDeg) for node in v_min_route_nodes if node in self.node]

        epsilon = 1e-6

        if mode == 'vtp':
            temp_vehicle_route = [route[:] for route in new_state.vehicle_routes]
            temp_vehicle_arrive_time = new_state.calculate_rm_empty_vehicle_arrive_time(temp_vehicle_route)
            window_total_cost, uav_tw_violation_cost, total_cost_dict = calculate_window_cost(new_state.customer_plan, new_state.uav_cost, temp_vehicle_arrive_time, self.vehicle, self.customer_time_windows_h,
            self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node)

            vtp_performance = defaultdict(float)
            for customer, assignment in new_state.customer_plan.items():
                uav_id, launch_node, _, recovery_node, launch_veh, recovery_veh = assignment

                if launch_veh != v_max_id or recovery_veh != v_max_id:
                    continue

                task_cost = total_cost_dict.get(customer, 0.0)

                if launch_veh == v_max_id:
                    vtp_performance[(launch_veh, launch_node)] += task_cost * 1.0

            v_max_route_nodes = new_state.vehicle_routes[v_max_id - 1][1:-1]
            if not v_max_route_nodes:
                print("  > 警告: 最忙车辆没有可破坏的VTP节点。")
                return new_state

            weights = []
            candidates = []
            for node in v_max_route_nodes:
                key = (v_max_id, node)

                w = vtp_performance.get(key, 0) ** 2
                weights.append(w + 1.0)
                candidates.append(node)
            probs = np.array(weights) / sum(weights)
            seed_vtp_node = self.rng.choice(candidates, p=probs)
            seed_key = (v_max_id, seed_vtp_node)

            seed_node_obj = self.node[seed_vtp_node]
            seed_pos = (seed_node_obj.latDeg, seed_node_obj.lonDeg)

            print(f"  > Shaw重平衡(VTP): 种子 VTP {seed_vtp_node} (车辆 {v_max_id}), 归因惩罚: {vtp_performance.get(seed_key, 0):.2f}")

            max_dist_seed = 0
            max_dist_idle = 0
            max_badness = 0
            relatedness_scores = []

            all_other_vtps = []
            for v_idx, route in enumerate(new_state.vehicle_routes):
                vid = v_idx + 1
                for node in route[1:-1]:
                    if (vid, node) != seed_key:
                        all_other_vtps.append((vid, node))

            if not all_other_vtps: return new_state

            max_dist_seed = 0.0
            max_dist_idle = 0.0
            max_badness = 0.0

            temp_data = []

            for v_id, vtp_node in all_other_vtps:
                node_obj = self.node[vtp_node]
                pos = (node_obj.latDeg, node_obj.lonDeg)

                d_seed = math.sqrt((pos[0] - seed_pos[0])**2 + (pos[1] - seed_pos[1])**2)

                d_idle = float('inf')
                if not v_min_positions:
                    d_idle = 0.0
                else:
                    for idle_pos in v_min_positions:
                        d_tmp = math.sqrt((pos[0] - idle_pos[0])**2 + (pos[1] - idle_pos[1])**2)
                        if d_tmp < d_idle: d_idle = d_tmp

                badness = vtp_performance.get((v_id, vtp_node), 0.0)

                if d_seed > max_dist_seed: max_dist_seed = d_seed
                if d_idle > max_dist_idle: max_dist_idle = d_idle
                if badness > max_badness: max_badness = badness

                temp_data.append({
                    'key': (v_id, vtp_node),
                    'd_seed': d_seed,
                    'd_idle': d_idle,
                    'badness': badness
                })

            w_seed = 0.4
            w_idle = 0.4
            w_bad = 0.2
            epsilon = 1e-6

            for item in temp_data:

                norm_d_seed = item['d_seed'] / (max_dist_seed + epsilon)
                norm_d_idle = item['d_idle'] / (max_dist_idle + epsilon)
                norm_bad = item['badness'] / (max_badness + epsilon)

                score = w_seed * norm_d_seed + w_idle * norm_d_idle + w_bad * (1.0 - norm_bad)

                relatedness_scores.append({'key': item['key'], 'score': score})

            relatedness_scores.sort(key=lambda x: x['score'])
            total_available_candidates = len(relatedness_scores) + 1
            num_to_remove = min(self.vtp_destroy_quantity['shaw'], total_available_candidates)

            vtps_to_destroy = [seed_key]

            num_neighbors_to_remove = num_to_remove - 1

            if num_neighbors_to_remove > 0 and relatedness_scores:

                K = max(num_neighbors_to_remove + 2, 5)

                top_k_candidates = relatedness_scores[:K]

                if len(top_k_candidates) <= num_neighbors_to_remove:
                    vtps_to_destroy.extend([item['key'] for item in top_k_candidates])
                else:

                    weights = np.arange(len(top_k_candidates), 0, -1)
                    weight_sum = np.sum(weights)
                    probs = weights / weight_sum if weight_sum > 0 else None

                    chosen_indices = self.rng.choice(
                        len(top_k_candidates),
                        size=num_neighbors_to_remove,
                        p=probs,
                        replace=False
                    )

                    vtps_to_destroy.extend([top_k_candidates[i]['key'] for i in chosen_indices])
            print(f"  > Shaw重平衡(VTP): 计划移除集群: {vtps_to_destroy}")

            destroyed_customers_info = new_state.destroyed_customers_info

            destroyed_vts_info = getattr(new_state, 'destroyed_vts_info', {})

            destroyed_vtp_count = 0
            actual_destroyed_vtps = []

            destroyed_customers_info = {}
            destroyed_vtp_count = 0
            max_attempts = len(vtps_to_destroy) * 2
            attempt_count = 0

            candidate_vtps = vtps_to_destroy.copy()

            while destroyed_vtp_count < self.vtp_destroy_quantity['shaw'] and candidate_vtps and attempt_count < max_attempts:
                attempt_count += 1

                if not candidate_vtps:
                    print(f"候选池已空，无法继续破坏VTP节点")
                    break

                vehicle_id, vtp_node = candidate_vtps.pop(0)

                if vtp_node not in new_state.rm_empty_vehicle_route[vehicle_id-1]:

                    continue

                customers_to_remove = []
                for customer, assignment in list(new_state.customer_plan.items()):
                    uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle = assignment

                    if (launch_node == vtp_node and launch_vehicle == vehicle_id) or \
                       (recovery_node == vtp_node and recovery_vehicle == vehicle_id):
                        customers_to_remove.append(customer)

                all_tasks_to_remove = []
                temp_vehicle_task_data = deep_copy_vehicle_task_data(vehicle_task_data)

                temp_customer_plan = {k: v for k, v in new_state.customer_plan.items()}
                temp_rm_empty_vehicle_route = [route[:] for route in new_state.rm_empty_vehicle_route]

                for customer in customers_to_remove:
                    if customer in temp_customer_plan:
                        assignment = temp_customer_plan[customer]
                        all_tasks_to_remove.append((customer, assignment))

                        need_to_remove_tasks = find_chain_tasks(assignment, temp_customer_plan, new_state.vehicle_routes, temp_vehicle_task_data)
                        all_tasks_to_remove.extend(need_to_remove_tasks)

                for customer, assignment in all_tasks_to_remove:
                    if customer in temp_customer_plan:
                        temp_customer_plan.pop(customer, None)

                if vtp_node in temp_rm_empty_vehicle_route[vehicle_id-1]:
                    temp_rm_empty_vehicle_route[vehicle_id-1].remove(vtp_node)

                temp_rm_vehicle_arrive_time = new_state.calculate_rm_empty_vehicle_arrive_time(temp_rm_empty_vehicle_route)

                if not is_time_feasible(temp_customer_plan, temp_rm_vehicle_arrive_time):
                    print(f"VTP节点 {vtp_node} 删除后不满足时间约束，跳过删除 (尝试 {attempt_count}/{max_attempts})")

                    continue

                print(f"成功破坏VTP节点: 车辆{vehicle_id}的节点{vtp_node} (进度: {destroyed_vtp_count + 1}/{len(vtps_to_destroy)})")

                new_state.rm_empty_vehicle_route[vehicle_id-1].remove(vtp_node)
                destroyed_vts_info[(vehicle_id-1, vtp_node)] = True
                destroyed_vtp_count += 1

                for customer, assignment in all_tasks_to_remove:
                    if customer in new_state.customer_plan:
                        uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle = assignment

                        customer_info = [uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle, new_state.uav_cost.get(customer, 0) if new_state.uav_cost else 0]
                        destroyed_customers_info[customer] = customer_info

                        new_state.customer_plan.pop(customer, None)

                        if uav_id in new_state.uav_assignments:
                            new_state.uav_assignments[uav_id] = [
                                task for task in new_state.uav_assignments[uav_id]
                                if task[2] != customer_node
                            ]

                        if new_state.uav_cost and customer_node in new_state.uav_cost:
                            new_state.uav_cost.pop(customer_node, None)

                        vehicle_task_data = remove_vehicle_task(vehicle_task_data, assignment, new_state.vehicle_routes)

                        from task_data import deep_remove_vehicle_task
                        need_to_remove_tasks = find_chain_tasks(assignment, new_state.customer_plan, new_state.vehicle_routes, vehicle_task_data)
                        orig_vehicle_id = assignment[4]
                        for chain_customer, chain_assignment in need_to_remove_tasks:
                            if chain_customer in new_state.customer_plan:
                                chain_uav_id, chain_launch_node, chain_customer_node, chain_recovery_node, chain_launch_vehicle, chain_recovery_vehicle = chain_assignment

                                chain_customer_info = [chain_uav_id, chain_launch_node, chain_customer_node, chain_recovery_node, chain_launch_vehicle, chain_recovery_vehicle, new_state.uav_cost.get(chain_customer, 0) if new_state.uav_cost else 0]
                                destroyed_customers_info[chain_customer] = chain_customer_info

                                new_state.customer_plan.pop(chain_customer, None)

                                if chain_uav_id in new_state.uav_assignments:
                                    new_state.uav_assignments[chain_uav_id] = [
                                        task for task in new_state.uav_assignments[chain_uav_id]
                                        if task[2] != chain_customer_node
                                    ]

                                if new_state.uav_cost and chain_customer_node in new_state.uav_cost:
                                    new_state.uav_cost.pop(chain_customer_node, None)

                                vehicle_task_data = deep_remove_vehicle_task(vehicle_task_data, chain_assignment, new_state.vehicle_routes, orig_vehicle_id)

            print(f"VTP最差破坏策略完成：成功破坏 {destroyed_vtp_count}/{len(vtps_to_destroy)} 个VTP节点")

            new_state.destroyed_vts_info = destroyed_vts_info
            new_state.destroyed_customers_info = destroyed_customers_info
            new_state.vehicle_task_data = vehicle_task_data

            new_state.vehicle_routes = [route[:] for route in new_state.rm_empty_vehicle_route]
            new_state.rm_vehicle_arrive_time = new_state.calculate_rm_empty_vehicle_arrive_time(new_state.vehicle_routes)
            new_state.destroyed_node_cost = new_state.update_calculate_plan_cost(new_state.uav_cost, new_state.vehicle_routes)

            print("=== VTP破坏阶段完成 ===\n")
        else:

            print("  > [破坏模式]: 综合shaw破坏策略")

            if not current_customers:
                print("  > 警告: 没有已服务的客户可供破坏。")
                return new_state

            try:
                temp_vehicle_route = [route[:] for route in new_state.vehicle_routes]
                temp_vehicle_arrive_time = new_state.calculate_rm_empty_vehicle_arrive_time(temp_vehicle_route)
                _, _, total_cost_dict = calculate_window_cost(new_state.customer_plan, new_state.uav_cost, temp_vehicle_arrive_time, self.vehicle, self.customer_time_windows_h,
                self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node)
            except Exception:
                total_cost_dict = new_state.uav_cost

            current_customers = list(new_state.customer_plan.keys())
            if not current_customers: return new_state

            weights = []
            for k in current_customers:

                w = total_cost_dict.get(k, 0) ** 2
                weights.append(w + 1.0)

            probs = np.array(weights) / sum(weights)
            seed_customer = self.rng.choice(current_customers, p=probs)

            seed_node = self.node[seed_customer]
            seed_pos = (seed_node.latDeg, seed_node.lonDeg)
            seed_tw = self.customer_time_windows_h.get(seed_customer)

            s_start = float(seed_tw['ready_h']) if seed_tw else 0.0
            s_end = float(seed_tw['due_h']) if seed_tw else 24.0

            print(f"  > 高冲突Shaw破坏: 种子 {seed_customer}, 惩罚成本: {total_cost_dict.get(seed_customer,0):.2f}")

            relatedness_list = []
            all_other = [c for c in current_customers if c != seed_customer]

            max_dist = 0.0
            max_penalty = 0.0

            temp_data = []

            for k in all_other:

                k_node = self.node[k]
                d = math.sqrt((k_node.latDeg - seed_pos[0])**2 + (k_node.lonDeg - seed_pos[1])**2)
                max_dist = max(max_dist, d)

                p = total_cost_dict.get(k, 0)
                max_penalty = max(max_penalty, p)

                k_tw = self.customer_time_windows_h.get(k)
                k_start = float(k_tw['ready_h']) if k_tw else 0.0
                k_end = float(k_tw['due_h']) if k_tw else 24.0

                overlap = max(0.0, min(s_end, k_end) - max(s_start, k_start))

                min_len = min(s_end - s_start, k_end - k_start) + 0.01
                norm_overlap = overlap / min_len

                temp_data.append({'k': k, 'd': d, 'p': p, 'o': norm_overlap})

            epsilon = 1e-6
            w_dist = 0.4
            w_overlap = 0.4
            w_penalty = 0.2

            for item in temp_data:
                n_dist = item['d'] / (max_dist + epsilon)
                n_penalty = item['p'] / (max_penalty + epsilon)

                score = w_dist * n_dist - w_overlap * item['o'] - w_penalty * n_penalty

                relatedness_list.append({'key': item['k'], 'score': score})

            relatedness_list.sort(key=lambda x: x['score'])

            n = len(current_customers)
            num_to_remove = self.rng.integers(
                max(1, int(n * self.customer_destroy_ratio[0])),
                max(2, int(n * self.customer_destroy_ratio[1])) + 1
            )
            num_to_remove = min(num_to_remove, n)

            customers_to_destroy = [seed_customer]
            if num_to_remove > 1:
                neighbors_needed = num_to_remove - 1

                K_pool = max(neighbors_needed + 5, 10)
                top_candidates = relatedness_list[:K_pool]

                customers_to_destroy.extend([x['key'] for x in top_candidates[:neighbors_needed]])

            print(f"  > 计划移除 {len(customers_to_destroy)} 个高冲突聚类客户: {customers_to_destroy}")

            destroyed_customers_info = new_state.destroyed_customers_info
            vehicle_task_data = new_state.vehicle_task_data

            for customer in customers_to_destroy:
                if customer in new_state.customer_plan:

                    temp_customer_plan = {k: v for k, v in new_state.customer_plan.items()}
                    temp_vehicle_routes = [route[:] for route in new_state.vehicle_routes]
                    temp_vehicle_task_data = deep_copy_vehicle_task_data(new_state.vehicle_task_data)
                    temp_assignment = new_state.customer_plan[customer]
                    temp_chain_tasks = find_chain_tasks(temp_assignment, temp_customer_plan, temp_vehicle_routes, temp_vehicle_task_data)
                    temp_customer_plan.pop(customer, None)
                    for chain_customer, chain_assignment in temp_chain_tasks:
                        temp_customer_plan.pop(chain_customer, None)
                    temp_rm_vehicle_arrive_time = new_state.calculate_rm_empty_vehicle_arrive_time(temp_vehicle_routes)
                    if not is_time_feasible(temp_customer_plan, temp_rm_vehicle_arrive_time):
                        continue

                    assignment = new_state.customer_plan.pop(customer)
                    uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle = assignment

                    customer_info = [uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle, new_state.uav_cost.get(customer, 0) if new_state.uav_cost else 0]
                    destroyed_customers_info[customer] = customer_info

                    if uav_id in new_state.uav_assignments:
                        new_state.uav_assignments[uav_id] = [
                            task for task in new_state.uav_assignments[uav_id]
                            if task[2] != customer_node
                        ]

                    if new_state.uav_cost and customer_node in new_state.uav_cost:
                        new_state.uav_cost.pop(customer_node, None)

                    vehicle_task_data = remove_vehicle_task(vehicle_task_data, assignment, new_state.vehicle_routes)

                    need_to_remove_tasks = find_chain_tasks(assignment, new_state.customer_plan, new_state.vehicle_routes, new_state.vehicle_task_data)
                    orig_vehicle_id = assignment[4]

                    for chain_customer, chain_assignment in need_to_remove_tasks:
                        if chain_customer in new_state.customer_plan:
                            chain_uav_id, chain_launch_node, chain_customer_node, chain_recovery_node, chain_launch_vehicle, chain_recovery_vehicle = chain_assignment

                            chain_customer_info = [chain_uav_id, chain_launch_node, chain_customer_node, chain_recovery_node, chain_launch_vehicle, chain_recovery_vehicle, new_state.uav_cost.get(chain_customer, 0) if new_state.uav_cost else 0]
                            destroyed_customers_info[chain_customer] = chain_customer_info

                            new_state.customer_plan.pop(chain_customer, None)

                            if chain_uav_id in new_state.uav_assignments:
                                new_state.uav_assignments[chain_uav_id] = [
                                    task for task in new_state.uav_assignments[chain_uav_id]
                                    if task[2] != chain_customer_node
                                ]

                            if new_state.uav_cost and chain_customer_node in new_state.uav_cost:
                                new_state.uav_cost.pop(chain_customer_node, None)
                            from task_data import deep_remove_vehicle_task

                            vehicle_task_data = deep_remove_vehicle_task(vehicle_task_data, chain_assignment, new_state.vehicle_routes, orig_vehicle_id)

            new_state.destroyed_node_cost = new_state.update_calculate_plan_cost(new_state.uav_cost, new_state.vehicle_routes)

            new_state.destroyed_customers_info = destroyed_customers_info
            new_state.vehicle_task_data = vehicle_task_data
            print(f"破坏后剩余客户点: {len(new_state.customer_plan)}")
            print("=== 破坏阶段完成 ===\n")
        return new_state

    def destroy_worst_removal(self, state, force_vtp_mode = None):

        """
        最差破坏算子，实现了双重模式以适应自适应策略选择框架。
        它既可以基于成本效益比移除最差的VTP节点以重构路径，也可以移除成本最高的客户以重组任务。
        Args:
            state (FastMfstspState): 当前解的状态。
            force_vtp_mode (bool, optional):
                - True:  强制执行VTP破坏模式 (用于"结构性重组"策略)。
                - False: 强制执行客户破坏模式 (用于"内部精细优化"策略)。
                - None: (默认) 按预设概率随机选择一种模式 (此模式在当前框架下不会被触发，但保留以备后用)。

        Returns:
            FastMfstspState: 被部分破坏后的解的状态。
        """

        new_state = state.fast_copy()

        current_customers = list(new_state.customer_plan.keys())
        if not current_customers:
            print("没有客户点需要移除")
            return new_state

        mode = 'vtp' if force_vtp_mode else 'customer'
        print(f"  > [破坏模式]: 最差破坏 ({'VTP模式' if mode == 'vtp' else '客户模式'})")
        vehicle_task_data = new_state.vehicle_task_data

        if mode == 'vtp':

            active_vtps_with_cost_ratio = []
            destroyed_vts_info = {}

            temp_vehicle_route = [route[:] for route in new_state.vehicle_routes]
            temp_vehicle_arrive_time = new_state.calculate_rm_empty_vehicle_arrive_time(temp_vehicle_route)
            window_total_cost, uav_tw_violation_cost, total_cost_dict = calculate_window_cost(new_state.customer_plan, new_state.uav_cost, temp_vehicle_arrive_time, self.vehicle, self.customer_time_windows_h,
            self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node)

            for vehicle_id, route in enumerate(new_state.vehicle_routes):
                v_id = vehicle_id + 1
                for vtp_node in route[1:-1]:

                    launch_tasks = 0
                    recovery_tasks = 0
                    total_cost = 0.0

                    for customer, assignment in new_state.customer_plan.items():
                        uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle = assignment
                        if launch_node == vtp_node and launch_vehicle == v_id:

                            uav_violate_cost = uav_tw_violation_cost.get(customer, 0) if uav_tw_violation_cost else 0
                            total_cost = total_cost + uav_violate_cost
                            launch_tasks += 1

                            launch_node_map = self.node[launch_node].map_key
                            total_cost += self.uav_travel[uav_id][launch_node_map][customer_node].totalDistance * self.vehicle[uav_id].per_cost

                    for customer, assignment in new_state.customer_plan.items():
                        uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle = assignment
                        if recovery_node == vtp_node and recovery_vehicle == v_id:
                            recovery_tasks += 1

                            recovery_node_map = self.node[recovery_node].map_key
                            total_cost += self.uav_travel[uav_id][customer_node][recovery_node_map].totalDistance * self.vehicle[uav_id].per_cost

                    total_tasks = launch_tasks + recovery_tasks
                    if total_tasks > 0:
                        cost_ratio = total_cost / total_tasks
                    else:

                        cost_ratio = float('inf')

                    active_vtps_with_cost_ratio.append(((v_id, vtp_node), cost_ratio, total_cost, total_tasks))

            active_vtps_with_cost_ratio.sort(key=lambda x: x[1], reverse=True)

            num_to_remove = self.vtp_destroy_quantity['worst']
            num_to_remove = min(num_to_remove, len(active_vtps_with_cost_ratio))

            vtps_to_destroy = [item[0] for item in active_vtps_with_cost_ratio[:num_to_remove]]

            for i, (vehicle_id, vtp_node) in enumerate(vtps_to_destroy):
                vtp_info = active_vtps_with_cost_ratio[i]
                cost_ratio, total_cost, total_tasks = vtp_info[1], vtp_info[2], vtp_info[3]

            destroyed_customers_info = {}
            destroyed_vtp_count = 0
            max_attempts = len(vtps_to_destroy) * 2
            attempt_count = 0

            candidate_vtps = vtps_to_destroy.copy()

            while destroyed_vtp_count < len(vtps_to_destroy) and candidate_vtps and attempt_count < max_attempts:
                attempt_count += 1

                if not candidate_vtps:
                    print(f"候选池已空，无法继续破坏VTP节点")
                    break

                vehicle_id, vtp_node = candidate_vtps.pop(0)
                if vtp_node not in new_state.rm_empty_vehicle_route[vehicle_id-1]:
                    continue

                customers_to_remove = []
                for customer, assignment in list(new_state.customer_plan.items()):
                    uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle = assignment

                    if (launch_node == vtp_node and launch_vehicle == vehicle_id) or \
                       (recovery_node == vtp_node and recovery_vehicle == vehicle_id):
                        customers_to_remove.append(customer)

                all_tasks_to_remove = []
                temp_vehicle_task_data = deep_copy_vehicle_task_data(vehicle_task_data)

                temp_customer_plan = {k: v for k, v in new_state.customer_plan.items()}
                temp_rm_empty_vehicle_route = [route[:] for route in new_state.rm_empty_vehicle_route]

                for customer in customers_to_remove:
                    if customer in temp_customer_plan:
                        assignment = temp_customer_plan[customer]
                        all_tasks_to_remove.append((customer, assignment))

                        need_to_remove_tasks = find_chain_tasks(assignment, temp_customer_plan, new_state.vehicle_routes, temp_vehicle_task_data)
                        all_tasks_to_remove.extend(need_to_remove_tasks)

                for customer, assignment in all_tasks_to_remove:
                    if customer in temp_customer_plan:
                        temp_customer_plan.pop(customer, None)

                if vtp_node in temp_rm_empty_vehicle_route[vehicle_id-1]:
                    temp_rm_empty_vehicle_route[vehicle_id-1].remove(vtp_node)

                temp_rm_vehicle_arrive_time = new_state.calculate_rm_empty_vehicle_arrive_time(temp_rm_empty_vehicle_route)

                if not is_time_feasible(temp_customer_plan, temp_rm_vehicle_arrive_time):
                    print(f"VTP节点 {vtp_node} 删除后不满足时间约束，跳过删除 (尝试 {attempt_count}/{max_attempts})")
                    continue

                print(f"成功破坏VTP节点: 车辆{vehicle_id}的节点{vtp_node} (进度: {destroyed_vtp_count + 1}/{len(vtps_to_destroy)})")

                new_state.rm_empty_vehicle_route[vehicle_id-1].remove(vtp_node)
                destroyed_vts_info[(vehicle_id-1, vtp_node)] = True
                destroyed_vtp_count += 1

                for customer, assignment in all_tasks_to_remove:
                    if customer in new_state.customer_plan:
                        uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle = assignment

                        customer_info = [uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle, new_state.uav_cost.get(customer, 0) if new_state.uav_cost else 0]
                        destroyed_customers_info[customer] = customer_info

                        new_state.customer_plan.pop(customer, None)

                        if uav_id in new_state.uav_assignments:
                            new_state.uav_assignments[uav_id] = [
                                task for task in new_state.uav_assignments[uav_id]
                                if task[2] != customer_node
                            ]

                        if new_state.uav_cost and customer_node in new_state.uav_cost:
                            new_state.uav_cost.pop(customer_node, None)

                        vehicle_task_data = remove_vehicle_task(vehicle_task_data, assignment, new_state.vehicle_routes)
                        orig_vehicle_id = assignment[4]

                        from task_data import deep_remove_vehicle_task
                        need_to_remove_tasks = find_chain_tasks(assignment, new_state.customer_plan, new_state.vehicle_routes, vehicle_task_data)
                        for chain_customer, chain_assignment in need_to_remove_tasks:
                            if chain_customer in new_state.customer_plan:
                                chain_uav_id, chain_launch_node, chain_customer_node, chain_recovery_node, chain_launch_vehicle, chain_recovery_vehicle = chain_assignment

                                chain_customer_info = [chain_uav_id, chain_launch_node, chain_customer_node, chain_recovery_node, chain_launch_vehicle, chain_recovery_vehicle, new_state.uav_cost.get(chain_customer, 0) if new_state.uav_cost else 0]
                                destroyed_customers_info[chain_customer] = chain_customer_info

                                new_state.customer_plan.pop(chain_customer, None)

                                if chain_uav_id in new_state.uav_assignments:
                                    new_state.uav_assignments[chain_uav_id] = [
                                        task for task in new_state.uav_assignments[chain_uav_id]
                                        if task[2] != chain_customer_node
                                    ]

                                if new_state.uav_cost and chain_customer_node in new_state.uav_cost:
                                    new_state.uav_cost.pop(chain_customer_node, None)

                                vehicle_task_data = deep_remove_vehicle_task(vehicle_task_data, chain_assignment, new_state.vehicle_routes, orig_vehicle_id)

            print(f"VTP最差破坏策略完成：成功破坏 {destroyed_vtp_count}/{len(vtps_to_destroy)} 个VTP节点")

            new_state.destroyed_vts_info = destroyed_vts_info
            new_state.destroyed_customers_info = destroyed_customers_info
            new_state.vehicle_task_data = vehicle_task_data

            new_state.vehicle_routes = [route.copy() for route in new_state.rm_empty_vehicle_route]
            new_state.rm_vehicle_arrive_time = new_state.calculate_rm_empty_vehicle_arrive_time(new_state.vehicle_routes)

            new_state.destroyed_node_cost = new_state.win_total_objective()
            print(f"破坏后剩余VTP节点: {sum(len(route) - 2 for route in new_state.vehicle_routes)}")

            print("=== VTP破坏阶段完成 ===\n")
            print(f"破坏后总成本: {new_state.destroyed_node_cost}")
        else:

            temp_vehicle_route = [route[:] for route in new_state.vehicle_routes]
            temp_vehicle_arrive_time = new_state.calculate_rm_empty_vehicle_arrive_time(temp_vehicle_route)
            window_total_cost, uav_tw_violation_cost, total_cost_dict = calculate_window_cost(new_state.customer_plan, new_state.uav_cost, temp_vehicle_arrive_time, self.vehicle, self.customer_time_windows_h,
            self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node)
            customer_costs = []
            for customer in current_customers:

                cost = total_cost_dict.get(customer, 0) if total_cost_dict else 0
                customer_costs.append((customer, cost))

            customer_costs.sort(key=lambda x: x[1], reverse=True)

            n = len(customer_costs)
            num_to_remove = self.rng.integers(
                max(1, int(n * 0.2)),
                max(2, int(n * 0.3)) + 1
            )
            customers_to_remove = [customer for customer, _ in customer_costs[:num_to_remove]]

            print(f"最差客户破坏：移除 {len(customers_to_remove)} 个客户点: {customers_to_remove}")
            destroyed_customers_info = {}

            for customer in customers_to_remove:
                if customer in new_state.customer_plan:
                    assignment = new_state.customer_plan.pop(customer)
                    uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle = assignment

                    customer_info = [uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle, new_state.uav_cost.get(customer, 0) if new_state.uav_cost else 0]
                    destroyed_customers_info[customer] = customer_info

                    if uav_id in new_state.uav_assignments:
                        new_state.uav_assignments[uav_id] = [
                            task for task in new_state.uav_assignments[uav_id]
                            if task[2] != customer_node
                        ]

                    if new_state.uav_cost and customer_node in new_state.uav_cost:
                        new_state.uav_cost.pop(customer_node, None)

                    vehicle_task_data = remove_vehicle_task(vehicle_task_data, assignment, new_state.vehicle_routes)

                    need_to_remove_tasks = find_chain_tasks(assignment, new_state.customer_plan, new_state.vehicle_routes, new_state.vehicle_task_data)
                    orig_vehicle_id = assignment[4]

                    for chain_customer, chain_assignment in need_to_remove_tasks:
                        if chain_customer in new_state.customer_plan:
                            chain_uav_id, chain_launch_node, chain_customer_node, chain_recovery_node, chain_launch_vehicle, chain_recovery_vehicle = chain_assignment

                            chain_customer_info = [chain_uav_id, chain_launch_node, chain_customer_node, chain_recovery_node, chain_launch_vehicle, chain_recovery_vehicle, new_state.uav_cost.get(chain_customer, 0) if new_state.uav_cost else 0]
                            destroyed_customers_info[chain_customer] = chain_customer_info

                            new_state.customer_plan.pop(chain_customer, None)

                            if chain_uav_id in new_state.uav_assignments:
                                new_state.uav_assignments[chain_uav_id] = [
                                    task for task in new_state.uav_assignments[chain_uav_id]
                                    if task[2] != chain_customer_node
                                ]

                            if new_state.uav_cost and chain_customer_node in new_state.uav_cost:
                                new_state.uav_cost.pop(chain_customer_node, None)
                            from task_data import deep_remove_vehicle_task

                            vehicle_task_data = deep_remove_vehicle_task(vehicle_task_data, chain_assignment, new_state.vehicle_routes, orig_vehicle_id)

            new_state.destroyed_node_cost = new_state.win_total_objective()

            new_state.destroyed_customers_info = destroyed_customers_info
            new_state.vehicle_task_data = vehicle_task_data
            print(f"破坏后剩余客户点: {len(new_state.customer_plan)}")
            print("=== 破坏阶段完成 ===\n")
            print(f"破坏后总成本: {new_state.destroyed_node_cost}")
        return new_state

    def destroy_shaw_removal(self, state, force_vtp_mode = None):

        new_state = state.fast_copy()

        current_customers = list(new_state.customer_plan.keys())
        if not current_customers:
            print("没有客户点需要移除")
            return new_state

        seed_customer = self.rng.choice(current_customers)
        seed_pos = np.array([
            self.node[seed_customer].latDeg,
            self.node[seed_customer].lonDeg,
            self.node[seed_customer].altMeters
        ])

        print(f"Shaw破坏：选择种子客户点 {seed_customer}")

        customer_distances = []
        for customer in current_customers:
            if customer == seed_customer:
                continue
            pos = np.array([
                self.node[customer].latDeg,
                self.node[customer].lonDeg,
                self.node[customer].altMeters
            ])

            dist = np.linalg.norm(pos - seed_pos)
            customer_distances.append((customer, dist))

        customer_distances.sort(key=lambda x: x[1])
        n = len(current_customers)
        num_to_remove = self.rng.integers(
            max(1, int(n * 0.2)),
            max(2, int(n * 0.3)) + 1
        )

        customers_to_remove = [seed_customer] + [customer for customer, _ in customer_distances[:num_to_remove-1]]

        print(f"Shaw破坏：移除 {len(customers_to_remove)} 个相似客户点: {customers_to_remove}")
        destroyed_customers_info = {}

        for customer in customers_to_remove:
            if customer in new_state.customer_plan:
                assignment = new_state.customer_plan.pop(customer)
                uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle = assignment

                customer_info = [uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle, new_state.uav_cost.get(customer, 0) if new_state.uav_cost else 0]
                destroyed_customers_info[customer] = customer_info

                if uav_id in new_state.uav_assignments:
                    new_state.uav_assignments[uav_id] = [
                        task for task in new_state.uav_assignments[uav_id]
                        if task[2] != customer_node
                    ]

                if new_state.uav_cost and customer_node in new_state.uav_cost:
                    new_state.uav_cost.pop(customer_node, None)

                need_to_remove_tasks = find_chain_tasks(assignment, new_state.customer_plan, new_state.vehicle_routes, new_state.vehicle_task_data)

                for chain_customer, chain_assignment in need_to_remove_tasks:
                    if chain_customer in new_state.customer_plan:
                        chain_uav_id, chain_launch_node, chain_customer_node, chain_recovery_node, chain_launch_vehicle, chain_recovery_vehicle = chain_assignment

                        chain_customer_info = [chain_uav_id, chain_launch_node, chain_customer_node, chain_recovery_node, chain_launch_vehicle, chain_recovery_vehicle, new_state.uav_cost.get(chain_customer, 0) if new_state.uav_cost else 0]
                        destroyed_customers_info[chain_customer] = chain_customer_info

                        new_state.customer_plan.pop(chain_customer, None)

                        if chain_uav_id in new_state.uav_assignments:
                            new_state.uav_assignments[chain_uav_id] = [
                                task for task in new_state.uav_assignments[chain_uav_id]
                                if task[2] != chain_customer_node
                            ]

                        if new_state.uav_cost and chain_customer_node in new_state.uav_cost:
                            new_state.uav_cost.pop(chain_customer_node, None)

                vehicle_task_data = new_state.vehicle_task_data
                vehicle_task_data = remove_vehicle_task(vehicle_task_data, assignment, new_state.vehicle_routes)
                new_state.vehicle_task_data = vehicle_task_data

        new_state.destroyed_node_cost = new_state.update_calculate_plan_cost(new_state.uav_cost, new_state.vehicle_routes)

        new_state.destroyed_customers_info = destroyed_customers_info

        print(f"破坏后剩余客户点: {len(new_state.customer_plan)}")
        print("=== Shaw破坏阶段完成 ===\n")
        return new_state

    def destroy_vtp_removal(self, state):

        new_state = state.fast_copy()

        all_vtp_in_routes = []
        for vehicle_id, route in new_state.vehicle_routes.items():
            for node in route[1:-1]:
                if node in self.A_vtp:
                    all_vtp_in_routes.append((vehicle_id, node))

        if not all_vtp_in_routes:
            print("VTP破坏：没有找到可移除的VTP节点")
            return new_state

        num_to_remove = self.rng.integers(1, min(3, len(all_vtp_in_routes)) + 1)
        vtp_to_remove = self.rng.choice(all_vtp_in_routes, num_to_remove, replace=False)

        print(f"VTP破坏：选择移除 {len(vtp_to_remove)} 个VTP节点: {vtp_to_remove}")

        destroyed_customers_info = {}
        removed_vtp_info = {}

        for vehicle_id, vtp_node in vtp_to_remove:
            print(f"VTP破坏：移除车辆 {vehicle_id} 的VTP节点 {vtp_node}")

            route = new_state.vehicle_routes[vehicle_id]
            if vtp_node in route:
                route.remove(vtp_node)
                new_state.vehicle_routes[vehicle_id] = route
                removed_vtp_info[(vehicle_id, vtp_node)] = True

            affected_customers = self._find_customers_using_vtp(vtp_node, new_state.customer_plan)

            for customer in affected_customers:
                if customer in new_state.customer_plan:
                    assignment = new_state.customer_plan.pop(customer)
                    uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle = assignment

                    customer_info = [uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle,
                                   new_state.uav_cost.get(customer, 0) if new_state.uav_cost else 0]
                    destroyed_customers_info[customer] = customer_info

                    if uav_id in new_state.uav_assignments:
                        new_state.uav_assignments[uav_id] = [
                            task for task in new_state.uav_assignments[uav_id]
                            if task[2] != customer_node
                        ]

                    if new_state.uav_cost and customer_node in new_state.uav_cost:
                        new_state.uav_cost.pop(customer_node, None)

                    need_to_remove_tasks = find_chain_tasks(assignment, new_state.customer_plan, new_state.vehicle_routes, new_state.vehicle_task_data)

                    for chain_customer, chain_assignment in need_to_remove_tasks:
                        if chain_customer in new_state.customer_plan:
                            chain_uav_id, chain_launch_node, chain_customer_node, chain_recovery_node, chain_launch_vehicle, chain_recovery_vehicle = chain_assignment

                            chain_customer_info = [chain_uav_id, chain_launch_node, chain_customer_node, chain_recovery_node,
                                                 chain_launch_vehicle, chain_recovery_vehicle,
                                                 new_state.uav_cost.get(chain_customer, 0) if new_state.uav_cost else 0]
                            destroyed_customers_info[chain_customer] = chain_customer_info

                            new_state.customer_plan.pop(chain_customer, None)

                            if chain_uav_id in new_state.uav_assignments:
                                new_state.uav_assignments[chain_uav_id] = [
                                    task for task in new_state.uav_assignments[chain_uav_id]
                                    if task[2] != chain_customer_node
                                ]

                            if new_state.uav_cost and chain_customer_node in new_state.uav_cost:
                                new_state.uav_cost.pop(chain_customer_node, None)

                    vehicle_task_data = new_state.vehicle_task_data
                    vehicle_task_data = remove_vehicle_task(vehicle_task_data, assignment, new_state.vehicle_routes)
                    new_state.vehicle_task_data = vehicle_task_data

                    print(f"VTP破坏：移除客户点 {customer}（使用VTP节点 {vtp_node}）")

        new_state.destroyed_node_cost = new_state.update_calculate_plan_cost(new_state.uav_cost, new_state.vehicle_routes)
        new_state.destroyed_customers_info = destroyed_customers_info
        new_state.removed_vtp_info = removed_vtp_info

        print(f"VTP破坏：移除 {len(vtp_to_remove)} 个VTP节点，影响 {len(destroyed_customers_info)} 个客户点")
        print(f"VTP破坏后剩余客户点: {len(new_state.customer_plan)}")
        print("=== VTP破坏阶段完成 ===\n")
        return new_state

    def _find_customers_using_vtp(self, vtp_node, customer_plan):

        affected_customers = []
        for customer, assignment in customer_plan.items():
            uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle = assignment
            if launch_node == vtp_node or recovery_node == vtp_node:
                affected_customers.append(customer)
        return affected_customers

    def destroy_important_removal(self, state):

        new_state = state.fast_copy()

        rm_empty_vehicle_route = state.rm_empty_vehicle_route
        new_state.vehicle_routes = [route.copy() for route in rm_empty_vehicle_route]
        all_customers = list(self.A_c)
        new_state.destroyed_customers_info = state.destroyed_customers_info
        if not all_customers:
            print("没有客户点")
            return new_state

        task_count = {c: 0 for c in all_customers}

        for uav_id, tasks in new_state.uav_assignments.items():
            for task in tasks:

                _, launch_node, customer, recovery_node, launch_vehicle, recovery_vehicle = task
                if customer in task_count:
                    task_count[customer] += 1

        noise = {c: self.rng.normal(0, 1) for c in all_customers}

        customer_scores = [(c, task_count[c] + noise[c]) for c in all_customers]
        customer_scores.sort(key=lambda x: x[1], reverse=True)

        n = len(customer_scores)
        num_to_remove = self.rng.integers(
            max(1, int(n * 0.2)),
            max(2, int(n * 0.3)) + 1
        )
        customers_to_remove = [c for c, _ in customer_scores[:num_to_remove]]

        for customer in customers_to_remove:
            if customer in new_state.customer_plan:
                assignment = new_state.customer_plan.pop(customer)
                uav_id, launch_node, customer_node, recovery_node, launch_vehicle, recovery_vehicle = assignment

                if uav_id in new_state.uav_assignments:
                    new_state.uav_assignments[uav_id] = [
                        task for task in new_state.uav_assignments[uav_id]
                        if task[1] != customer_node
                    ]

                if hasattr(new_state, 'uav_cost') and new_state.uav_cost is not None:
                    new_state.uav_cost.pop(customer_node, None)

                vehicle_task_data = new_state.vehicle_task_data
                vehicle_task_data = remove_vehicle_task(vehicle_task_data, assignment, new_state.vehicle_routes)
                new_state.vehicle_task_data = vehicle_task_data

        new_state.update_rm_empty_task()
        new_state.total_cost = new_state.objective()
        return new_state

    def cluster_vtp_for_customers(self, k):

        vtp_ids = list(self.A_vtp)
        customer_ids = list(self.A_c)
        if not customer_ids or not vtp_ids:
            return {}

        vtp_coords = np.array([
            [self.node[vtp_id].latDeg, self.node[vtp_id].lonDeg, self.node[vtp_id].altMeters]
            for vtp_id in vtp_ids
        ])

        customer_coords = np.array([
            [self.node[cid].latDeg, self.node[cid].lonDeg, self.node[cid].altMeters]
            for cid in customer_ids
        ])

        from scipy.spatial.distance import cdist
        dist_matrix = cdist(customer_coords, vtp_coords)

        customer_vtp_dict = {}
        for i, cid in enumerate(customer_ids):

            sorted_indices = np.argsort(dist_matrix[i])[:k]

            sorted_vtps = [vtp_ids[j] for j in sorted_indices]
            customer_vtp_dict[cid] = sorted_vtps

        map_customer_vtp_dict = {}
        for cid, sorted_vtps in customer_vtp_dict.items():
            map_customer_vtp_dict[cid] = [self.node[vtp_id].map_key for vtp_id in sorted_vtps]

        return customer_vtp_dict, map_customer_vtp_dict

    def _create_snapshot(self, state):

        return FastMfstspState(
            copy.deepcopy(state.vehicle_routes),
            copy.deepcopy(state.uav_assignments),
            copy.deepcopy(state.customer_plan),
            copy.deepcopy(state.vehicle_task_data),
            copy.deepcopy(state.global_reservation_table),
            copy.deepcopy(state._total_cost)
        )

    def _incremental_destroy(self, state, modification_stack):

        all_customers = list(state.customer_plan.keys())
        if not all_customers:
            return

        num_to_remove = self.rng.integers(
            max(1, len(all_customers) // 5),
            max(2, len(all_customers) // 3)
        )

        customers_to_remove = self.rng.choice(all_customers, num_to_remove, replace=False)

        for customer in customers_to_remove:
            if customer in state.customer_plan:
                assignment = state.customer_plan.pop(customer)
                modification_stack.append(("add_customer", customer, assignment))

                uav_id, _, _, _, _ = assignment
                if uav_id in state.uav_assignments:
                    old_assignments = state.uav_assignments[uav_id].copy()
                    state.uav_assignments[uav_id] = [
                        task for task in state.uav_assignments[uav_id]
                        if task[1] != customer
                    ]
                    modification_stack.append(("restore_uav_assignments", uav_id, old_assignments))

        state._total_cost = None

    def _incremental_repair(self, state, modification_stack):

        pass

    def _rollback_modifications(self, state, modification_stack):

        for modification in reversed(modification_stack):
            if modification[0] == "add_customer":
                _, customer, assignment = modification
                state.customer_plan[customer] = assignment
            elif modification[0] == "restore_uav_assignments":
                _, uav_id, assignments = modification
                state.uav_assignments[uav_id] = assignments

        state._total_cost = None

    def validate_customer_plan(self, vehicle_routes, customer_plan, base_drone_assignment):

        is_valid = True

        drone_state = {}
        all_drones = set(d for drones in base_drone_assignment.values() for d in drones)

        try:
            drones_in_plan = set(assignment[0] for assignment in customer_plan.values())
            all_drones.update(drones_in_plan)
        except (TypeError, IndexError):
            print("  > 警告: customer_plan 格式可能不完全正确，无法提取所有无人机ID。")

        for drone_id in all_drones:
            drone_state[drone_id] = {'location': 'depot', 'last_event_node': None, 'current_task': None}

        for vehicle_id, drones_on_vehicle in base_drone_assignment.items():
            for drone_id in drones_on_vehicle:
                if drone_id in drone_state:
                    drone_state[drone_id]['location'] = vehicle_id
                else:
                    print(f"  > 警告: 基础分配中的无人机 {drone_id} 未在状态字典中初始化。")

        launch_tasks_at_node = {}
        recovery_tasks_at_node = {}

        for customer, assignment in customer_plan.items():
            try:

                if len(assignment) != 6:
                    raise ValueError("Assignment tuple length mismatch")
                drone_id, ln, _, rn, lv, rv = assignment

                if drone_id not in drone_state:
                    print(f"  > 错误: 客户 {customer} 的任务引用了未知的无人机 ID: {drone_id}。")
                    is_valid = False
                    continue

                launch_key = (lv, ln)
                if launch_key not in launch_tasks_at_node: launch_tasks_at_node[launch_key] = []
                launch_tasks_at_node[launch_key].append((drone_id, customer, assignment))

                recovery_key = (rv, rn)
                if recovery_key not in recovery_tasks_at_node: recovery_tasks_at_node[recovery_key] = []
                recovery_tasks_at_node[recovery_key].append((drone_id, customer, assignment))
            except (TypeError, ValueError, IndexError) as e:
                print(f"  > 错误: customer_plan 中客户 {customer} 的任务数据格式无效: {assignment}。错误: {e}")
                is_valid = False

        if not is_valid: return False

        processed_routes = []

        for vehicle_id, route in processed_routes:
            if len(route) < 2: continue
            print(f"\n--- 正在验证车辆 {vehicle_id} 的路线: {route} ---")

            for node_idx in range(1, len(route) - 1):
                vtp_node = route[node_idx]
                node_key = (vehicle_id, vtp_node)

                print(f"  节点 {vtp_node} (索引 {node_idx}):")

                if node_key in recovery_tasks_at_node:
                    for drone_id, customer, assignment in recovery_tasks_at_node[node_key]:
                        print(f"    - 检查回收: 无人机 {drone_id} (来自客户 {customer})")
                        state = drone_state[drone_id]

                        if state['location'] != 'flying':
                            error_msg = (f"    -> !! 回收冲突 !! 无人机 {drone_id} 试图在节点 {vtp_node} (车辆 {vehicle_id}) 回收，"
                                        f"但其当前状态是 '{state['location']}' (应为 'flying')。")
                            if state['last_event_node']:
                                error_msg += f" 上次事件发生在 {state['last_event_node']}."
                            print(error_msg)
                            is_valid = False

                        elif state['current_task'] != customer:
                            print(f"    -> !! 任务不匹配 !! 无人机 {drone_id} 试图回收服务客户 {customer} 的任务，"
                                f"但记录显示它正在执行的任务是 {state['current_task']}。")
                            is_valid = False

                        else:

                            state['location'] = vehicle_id
                            state['last_event_node'] = node_key
                            state['current_task'] = None
                            print(f"      状态更新: 无人机 {drone_id} 已回收至车辆 {vehicle_id}。")

                if node_key in launch_tasks_at_node:
                    for drone_id, customer, assignment in launch_tasks_at_node[node_key]:
                        print(f"    - 检查发射: 无人机 {drone_id} (飞往客户 {customer})")
                        state = drone_state[drone_id]

                        if state['location'] != vehicle_id:
                            error_msg = (f"    -> !! 发射冲突 !! 无人机 {drone_id} 试图从节点 {vtp_node} (车辆 {vehicle_id}) 发射，"
                                        f"但其当前状态是 '{state['location']}' (应在车辆 {vehicle_id} 上)。")
                            if state['last_event_node']:
                                error_msg += f" 上次事件发生在 {state['last_event_node']}."
                            print(error_msg)
                            is_valid = False

                        elif state['current_task'] is not None:
                            print(f"    -> !! 状态冲突 !! 无人机 {drone_id} 试图发射新任务 (客户 {customer})，"
                                f"但它仍在执行上一个任务 (客户 {state['current_task']})。")
                            is_valid = False

                        else:

                            state['location'] = 'flying'
                            state['last_event_node'] = node_key
                            state['current_task'] = customer
                            print(f"      状态更新: 无人机 {drone_id} 已发射，状态为 'flying'，目标客户 {customer}。")

        unfinished_drones = []
        for drone_id, state in drone_state.items():
            if state['location'] == 'flying' or state['current_task'] is not None:
                unfinished_drones.append((drone_id, state['current_task']))

        if unfinished_drones:
            print(f"\n  > 警告: 验证结束时，以下无人机仍处于飞行状态或有未完成的任务:")
            for d_id, c_id in unfinished_drones:
                print(f"    - 无人机 {d_id} (目标客户: {c_id})")

        if is_valid:
            print("\n=== customer_plan 约束验证通过 ===")
        else:
            print("\n=== customer_plan 存在约束冲突 ===")

        return is_valid

def create_fast_initial_state(init_total_cost, init_uav_plan, init_customer_plan, init_uav_cost,
                             init_time_uav_task_dict, init_vehicle_route,
                             init_vehicle_plan_time, init_vehicle_task_data,
                             init_global_reservation_table,node, DEPOT_nodeID,
                             V, T, vehicle, uav_travel, veh_distance, veh_travel, N, N_zero, N_plus,
                             A_total, A_cvtp, A_vtp, A_aerial_relay_node, G_air, G_ground,
                             air_matrix, ground_matrix, air_node_types, ground_node_types, A_c, xeee,
                             customer_time_windows_h, early_arrival_cost, late_arrival_cost):

    return FastMfstspState(
        vehicle_routes=init_vehicle_route,
        uav_assignments=init_time_uav_task_dict,
        customer_plan=init_customer_plan,
        vehicle_task_data=init_vehicle_task_data,
        global_reservation_table=init_global_reservation_table,
        total_cost=init_total_cost,
        uav_cost = init_uav_cost,
        init_uav_plan=init_uav_plan,
        init_vehicle_plan_time = init_vehicle_plan_time,
        vehicle = vehicle,
        T = T,
        V = V,
        veh_distance = veh_distance,
        veh_travel = veh_travel,
        node = node,
        DEPOT_nodeID = DEPOT_nodeID,
        uav_travel = uav_travel,
        N = N,
        N_zero = N_zero,
        N_plus = N_plus,
        A_total = A_total,
        A_cvtp = A_cvtp,
        A_vtp = A_vtp,
        A_aerial_relay_node = A_aerial_relay_node,
        G_air = G_air,
        G_ground = G_ground,
        air_matrix = air_matrix,
        ground_matrix = ground_matrix,
        air_node_types = air_node_types,
        ground_node_types = ground_node_types,
        A_c = A_c,
        xeee = xeee,
        customer_time_windows_h = customer_time_windows_h,
        early_arrival_cost = early_arrival_cost,
        late_arrival_cost = late_arrival_cost
    )

def solve_with_fast_alns(initial_solution, node, DEPOT_nodeID, V, T, vehicle, uav_travel, veh_distance, veh_travel, N, N_zero, N_plus, A_total, A_cvtp, A_vtp,
		A_aerial_relay_node, G_air, G_ground,air_matrix, ground_matrix, air_node_types, ground_node_types, A_c, xeee, customer_time_windows_h, early_arrival_cost, late_arrival_cost, problemName,
        iter, max_iterations, max_runtime=60, summary_dir=None, use_incremental=True, seed=None, algo_seed=None):

    if use_incremental:

        alns_solver = IncrementalALNS(node, DEPOT_nodeID, V, T, vehicle, uav_travel,
        veh_distance, veh_travel, N, N_zero, N_plus, A_total, A_cvtp, A_vtp,
		A_aerial_relay_node, G_air, G_ground,air_matrix, ground_matrix, air_node_types,
        ground_node_types, A_c, xeee, customer_time_windows_h, early_arrival_cost, late_arrival_cost, problemName,
        iter=iter, max_iterations=max_iterations, summary_dir=summary_dir, max_runtime=max_runtime, seed=seed, algo_seed=algo_seed)

    best_state, best_final_state, best_objective, best_final_objective, best_final_uav_cost, best_final_win_cost, best_total_win_cost, best_final_global_max_time, best_global_max_time, best_window_total_cost, best_total_uav_tw_violation_cost, best_total_vehicle_cost, elapsed_time, win_cost, uav_route_cost, vehicle_route_cost, final_uav_cost, final_total_list, final_win_cost, final_total_objective, y_cost, y_best, work_time, final_work_time = alns_solver.solve(initial_solution)

    return best_state, best_final_state, best_objective, best_final_objective, best_final_uav_cost, best_final_win_cost, best_total_win_cost, best_final_global_max_time, best_global_max_time, best_window_total_cost, best_total_uav_tw_violation_cost, best_total_vehicle_cost, elapsed_time, win_cost, uav_route_cost, vehicle_route_cost, final_uav_cost, final_total_list, final_win_cost, final_total_objective, y_cost, y_best, work_time, final_work_time

