import math
import numpy as np
import numpy.random as rnd
from collections import defaultdict
from initialize import deep_copy_vehicle_task_data
from destroy_repair_operator import (
    find_chain_tasks, remove_vehicle_task, update_vehicle_task,
    is_time_feasible
)
from cost_y import calculate_plan_cost
from task_data import deep_remove_vehicle_task

def destroy_proximity_removal(self, state, force_vtp_mode=None):

    new_state = state.fast_copy()
    new_state.destroyed_customers_info = {}
    vehicle_task_data = new_state.vehicle_task_data
    current_customers = list(new_state.customer_plan.keys())
    if not current_customers:
        return new_state

    mode = 'vtp' if force_vtp_mode else 'customer'

    if mode == 'vtp':

        active_vtps = []
        for vehicle_id, route in enumerate(new_state.vehicle_routes):
            v_id = vehicle_id + 1
            for vtp_node in route[1:-1]:
                active_vtps.append((v_id, vtp_node))

        if not active_vtps:
            return new_state

        low, high = self.vtp_destroy_quantity.get('random', (1, 2))
        num_to_remove = self.rng.integers(low, min(len(active_vtps), high) + 1)

        seed_idx = self.rng.integers(0, len(active_vtps))
        seed_v_id, seed_vtp = active_vtps[seed_idx]
        seed_air = self.node[seed_vtp].map_key
        default_uav_id = self.V[0] if self.V else 1

        vtp_distances = []
        for v_id, vtp_node in active_vtps:
            if (v_id, vtp_node) == (seed_v_id, seed_vtp):
                continue
            try:
                air_node = self.node[vtp_node].map_key
                dist = self.uav_travel[default_uav_id][seed_air][air_node].totalDistance
                vtp_distances.append(((v_id, vtp_node), dist))
            except (KeyError, AttributeError):
                continue

        vtp_distances.sort(key=lambda x: x[1])
        vtps_to_destroy = [(seed_v_id, seed_vtp)]
        for (v_id, vtp_node), _ in vtp_distances:
            if len(vtps_to_destroy) >= num_to_remove:
                break
            vtps_to_destroy.append((v_id, vtp_node))

        destroyed_vts_info = {}
        destroyed_customers_info = {}
        destroyed_vtp_count = 0

        for vehicle_id, vtp_node in vtps_to_destroy:
            if vtp_node not in new_state.rm_empty_vehicle_route[vehicle_id - 1]:
                continue

            customers_to_remove = []
            for customer, assignment in list(new_state.customer_plan.items()):
                uav_id, ln, cn, rn, lv, rv = assignment
                if (ln == vtp_node and lv == vehicle_id) or (rn == vtp_node and rv == vehicle_id):
                    customers_to_remove.append(customer)

            all_tasks_to_remove = []
            temp_customer_plan = {k: v for k, v in new_state.customer_plan.items()}
            temp_rm_route = [route[:] for route in new_state.rm_empty_vehicle_route]

            for customer in customers_to_remove:
                if customer in temp_customer_plan:
                    assignment = temp_customer_plan[customer]
                    all_tasks_to_remove.append((customer, assignment))
                    chain_tasks = find_chain_tasks(assignment, temp_customer_plan,
                                                   new_state.vehicle_routes, vehicle_task_data)
                    all_tasks_to_remove.extend(chain_tasks)

            for customer, assignment in all_tasks_to_remove:
                temp_customer_plan.pop(customer, None)

            if vtp_node in temp_rm_route[vehicle_id - 1]:
                temp_rm_route[vehicle_id - 1].remove(vtp_node)

            temp_arrive = new_state.calculate_rm_empty_vehicle_arrive_time(temp_rm_route)
            if not is_time_feasible(temp_customer_plan, temp_arrive):
                continue

            new_state.rm_empty_vehicle_route[vehicle_id - 1].remove(vtp_node)
            destroyed_vts_info[(vehicle_id, vtp_node)] = True
            destroyed_vtp_count += 1

            for customer, assignment in all_tasks_to_remove:
                if customer in new_state.customer_plan:
                    uav_id, ln, cn, rn, lv, rv = assignment
                    info = [uav_id, ln, cn, rn, lv, rv,
                            new_state.uav_cost.get(customer, 0) if new_state.uav_cost else 0]
                    destroyed_customers_info[customer] = info
                    new_state.customer_plan.pop(customer, None)
                    if uav_id in new_state.uav_assignments:
                        new_state.uav_assignments[uav_id] = [
                            t for t in new_state.uav_assignments[uav_id] if t[2] != cn]
                    if new_state.uav_cost and cn in new_state.uav_cost:
                        new_state.uav_cost.pop(cn, None)
                    vehicle_task_data = remove_vehicle_task(vehicle_task_data, assignment,
                                                           new_state.vehicle_routes)

        if not destroyed_customers_info:
            return state

        new_state.destroyed_vts_info = destroyed_vts_info
        new_state.destroyed_customers_info = destroyed_customers_info
        new_state.vehicle_task_data = vehicle_task_data
        new_state.vehicle_routes = [r[:] for r in new_state.rm_empty_vehicle_route]
        new_state.rm_vehicle_arrive_time = new_state.calculate_rm_empty_vehicle_arrive_time(
            new_state.vehicle_routes)
        new_state.destroyed_node_cost = new_state.update_calculate_plan_cost(
            new_state.uav_cost, new_state.vehicle_routes)
        return new_state

    else:

        n = len(current_customers)
        num_to_remove = self.rng.integers(max(1, int(n * 0.2)), max(2, int(n * 0.3)) + 1)

        destroyed_customers_info = {}
        removed_count = 0
        removed_set = set()

        seed_customer = self.rng.choice(current_customers)

        while removed_count < num_to_remove and current_customers:
            if seed_customer not in new_state.customer_plan:
                if current_customers:
                    seed_customer = self.rng.choice(current_customers)
                else:
                    break
                continue

            assignment = new_state.customer_plan.pop(seed_customer)
            uav_id, ln, cn, rn, lv, rv = assignment
            removed_set.add(cn)

            info = [uav_id, ln, cn, rn, lv, rv,
                    new_state.uav_cost.get(seed_customer, 0) if new_state.uav_cost else 0]
            destroyed_customers_info[seed_customer] = info
            if uav_id in new_state.uav_assignments:
                new_state.uav_assignments[uav_id] = [
                    t for t in new_state.uav_assignments[uav_id] if t[2] != cn]
            if new_state.uav_cost and cn in new_state.uav_cost:
                new_state.uav_cost.pop(cn, None)
            orig_vid = assignment[4]
            vehicle_task_data = remove_vehicle_task(vehicle_task_data, assignment,
                                                    new_state.vehicle_routes)
            removed_count += 1
            if seed_customer in current_customers:
                current_customers.remove(seed_customer)

            chain_tasks = find_chain_tasks(assignment, new_state.customer_plan,
                                           new_state.vehicle_routes, new_state.vehicle_task_data)
            for cc, ca in chain_tasks:
                if cc in new_state.customer_plan:
                    c_uid, c_ln, c_cn, c_rn, c_lv, c_rv = ca
                    c_info = [c_uid, c_ln, c_cn, c_rn, c_lv, c_rv,
                              new_state.uav_cost.get(cc, 0) if new_state.uav_cost else 0]
                    destroyed_customers_info[cc] = c_info
                    new_state.customer_plan.pop(cc, None)
                    if c_uid in new_state.uav_assignments:
                        new_state.uav_assignments[c_uid] = [
                            t for t in new_state.uav_assignments[c_uid] if t[2] != c_cn]
                    if new_state.uav_cost and c_cn in new_state.uav_cost:
                        new_state.uav_cost.pop(c_cn, None)
                    vehicle_task_data = deep_remove_vehicle_task(
                        vehicle_task_data, ca, new_state.vehicle_routes, orig_vid)
                    removed_count += 1
                    removed_set.add(c_cn)
                    if cc in current_customers:
                        current_customers.remove(cc)

            if removed_count < num_to_remove and current_customers:

                if float(self.rng.random()) < 0.1:
                    seed_customer = self.rng.choice(current_customers)
                else:
                    min_dist = float('inf')
                    best_candidate = None
                    for cand in current_customers:
                        cand_uav = new_state.customer_plan[cand][0]
                        cand_node = new_state.customer_plan[cand][2]

                        d_min = float('inf')
                        for removed_node in removed_set:
                            try:
                                d = self.uav_travel[cand_uav][removed_node][cand_node].totalDistance
                                d_min = min(d_min, d)
                            except (KeyError, AttributeError):
                                continue
                        if d_min < min_dist:
                            min_dist = d_min
                            best_candidate = cand
                    seed_customer = best_candidate if best_candidate else self.rng.choice(current_customers)

        new_state.destroyed_node_cost = new_state.update_calculate_plan_cost(
            new_state.uav_cost, new_state.vehicle_routes)
        new_state.destroyed_customers_info = destroyed_customers_info
        new_state.vehicle_task_data = vehicle_task_data
        return new_state

def destroy_window_removal(self, state, force_vtp_mode=None):

    new_state = state.fast_copy()
    new_state.destroyed_customers_info = {}
    vehicle_task_data = new_state.vehicle_task_data
    current_customers = list(new_state.customer_plan.keys())
    if not current_customers:
        return new_state

    n = len(current_customers)
    num_to_remove = self.rng.integers(max(1, int(n * 0.2)), max(2, int(n * 0.3)) + 1)

    destroyed_customers_info = {}
    removed_count = 0

    alpha_tw = 0.7
    beta_dist = 0.3

    seed_customer = self.rng.choice(current_customers)

    while removed_count < num_to_remove and current_customers:
        if seed_customer not in new_state.customer_plan:
            if current_customers:
                seed_customer = self.rng.choice(current_customers)
            else:
                break
            continue

        assignment = new_state.customer_plan.pop(seed_customer)
        uav_id, ln, cn, rn, lv, rv = assignment

        seed_ready_h = 0
        if seed_customer in self.customer_time_windows_h:
            seed_ready_h = self.customer_time_windows_h[seed_customer].get('ready_h', 0)
        elif cn in self.customer_time_windows_h:
            seed_ready_h = self.customer_time_windows_h[cn].get('ready_h', 0)
        seed_node = cn

        info = [uav_id, ln, cn, rn, lv, rv,
                new_state.uav_cost.get(seed_customer, 0) if new_state.uav_cost else 0]
        destroyed_customers_info[seed_customer] = info
        if uav_id in new_state.uav_assignments:
            new_state.uav_assignments[uav_id] = [
                t for t in new_state.uav_assignments[uav_id] if t[2] != cn]
        if new_state.uav_cost and cn in new_state.uav_cost:
            new_state.uav_cost.pop(cn, None)
        orig_vid = assignment[4]
        vehicle_task_data = remove_vehicle_task(vehicle_task_data, assignment,
                                                new_state.vehicle_routes)
        removed_count += 1
        if seed_customer in current_customers:
            current_customers.remove(seed_customer)

        chain_tasks = find_chain_tasks(assignment, new_state.customer_plan,
                                       new_state.vehicle_routes, new_state.vehicle_task_data)
        for cc, ca in chain_tasks:
            if cc in new_state.customer_plan:
                c_uid, c_ln, c_cn, c_rn, c_lv, c_rv = ca
                c_info = [c_uid, c_ln, c_cn, c_rn, c_lv, c_rv,
                          new_state.uav_cost.get(cc, 0) if new_state.uav_cost else 0]
                destroyed_customers_info[cc] = c_info
                new_state.customer_plan.pop(cc, None)
                if c_uid in new_state.uav_assignments:
                    new_state.uav_assignments[c_uid] = [
                        t for t in new_state.uav_assignments[c_uid] if t[2] != c_cn]
                if new_state.uav_cost and c_cn in new_state.uav_cost:
                    new_state.uav_cost.pop(c_cn, None)
                vehicle_task_data = deep_remove_vehicle_task(
                    vehicle_task_data, ca, new_state.vehicle_routes, orig_vid)
                removed_count += 1
                if cc in current_customers:
                    current_customers.remove(cc)

        if removed_count < num_to_remove and current_customers:
            best_candidate = None
            min_similarity = float('inf')

            all_ready_h = []
            for cand in current_customers:
                cand_node = new_state.customer_plan[cand][2]
                rh = 0
                if cand in self.customer_time_windows_h:
                    rh = self.customer_time_windows_h[cand].get('ready_h', 0)
                elif cand_node in self.customer_time_windows_h:
                    rh = self.customer_time_windows_h[cand_node].get('ready_h', 0)
                all_ready_h.append(rh)
            max_tw_diff = max(all_ready_h) - min(all_ready_h) if all_ready_h else 1.0
            max_tw_diff = max(max_tw_diff, 1e-6)

            for cand in current_customers:
                cand_uav = new_state.customer_plan[cand][0]
                cand_node = new_state.customer_plan[cand][2]

                cand_ready_h = 0
                if cand in self.customer_time_windows_h:
                    cand_ready_h = self.customer_time_windows_h[cand].get('ready_h', 0)
                elif cand_node in self.customer_time_windows_h:
                    cand_ready_h = self.customer_time_windows_h[cand_node].get('ready_h', 0)
                tw_diff = abs(cand_ready_h - seed_ready_h) / max_tw_diff

                try:
                    dist = self.uav_travel[cand_uav][seed_node][cand_node].totalDistance
                except (KeyError, AttributeError):
                    dist = float('inf')

                similarity = alpha_tw * tw_diff + beta_dist * dist
                if similarity < min_similarity:
                    min_similarity = similarity
                    best_candidate = cand

            seed_customer = best_candidate if best_candidate else self.rng.choice(current_customers)

    new_state.destroyed_node_cost = new_state.update_calculate_plan_cost(
        new_state.uav_cost, new_state.vehicle_routes)
    new_state.destroyed_customers_info = destroyed_customers_info
    new_state.vehicle_task_data = vehicle_task_data
    return new_state

def destroy_drone_workload_removal(self, state, force_vtp_mode=None):

    new_state = state.fast_copy()
    new_state.destroyed_customers_info = {}
    vehicle_task_data = new_state.vehicle_task_data
    current_customers = list(new_state.customer_plan.keys())
    if not current_customers:
        return new_state

    drone_workload = defaultdict(lambda: {'tasks': [], 'total_cost': 0.0, 'count': 0})
    for customer, assignment in new_state.customer_plan.items():
        uav_id = assignment[0]
        cost = new_state.uav_cost.get(customer, 0) if new_state.uav_cost else 0
        drone_workload[uav_id]['tasks'].append((customer, assignment))
        drone_workload[uav_id]['total_cost'] += cost
        drone_workload[uav_id]['count'] += 1

    if not drone_workload:
        return new_state

    strategy = 'busiest' if float(self.rng.random()) < 0.6 else 'most_expensive'

    if strategy == 'busiest':

        target_drone = max(drone_workload.keys(),
                           key=lambda d: drone_workload[d]['count'])
    else:

        target_drone = max(drone_workload.keys(),
                           key=lambda d: drone_workload[d]['total_cost'])

    target_tasks = drone_workload[target_drone]['tasks']
    if not target_tasks:
        return new_state

    n_tasks = len(target_tasks)
    num_to_remove = max(1, self.rng.integers(
        max(1, int(n_tasks * 0.3)),
        max(2, int(n_tasks * 0.7)) + 1
    ))

    target_tasks_sorted = sorted(target_tasks,
                                  key=lambda x: new_state.uav_cost.get(x[0], 0)
                                  if new_state.uav_cost else 0,
                                  reverse=True)
    tasks_to_remove = target_tasks_sorted[:num_to_remove]

    destroyed_customers_info = {}
    for customer, assignment in tasks_to_remove:
        if customer not in new_state.customer_plan:
            continue
        uav_id, ln, cn, rn, lv, rv = assignment
        info = [uav_id, ln, cn, rn, lv, rv,
                new_state.uav_cost.get(customer, 0) if new_state.uav_cost else 0]
        destroyed_customers_info[customer] = info

        new_state.customer_plan.pop(customer, None)
        if uav_id in new_state.uav_assignments:
            new_state.uav_assignments[uav_id] = [
                t for t in new_state.uav_assignments[uav_id] if t[2] != cn]
        if new_state.uav_cost and cn in new_state.uav_cost:
            new_state.uav_cost.pop(cn, None)
        orig_vid = assignment[4]
        vehicle_task_data = remove_vehicle_task(vehicle_task_data, assignment,
                                                new_state.vehicle_routes)

        chain_tasks = find_chain_tasks(assignment, new_state.customer_plan,
                                       new_state.vehicle_routes, new_state.vehicle_task_data)
        for cc, ca in chain_tasks:
            if cc in new_state.customer_plan:
                c_uid, c_ln, c_cn, c_rn, c_lv, c_rv = ca
                c_info = [c_uid, c_ln, c_cn, c_rn, c_lv, c_rv,
                          new_state.uav_cost.get(cc, 0) if new_state.uav_cost else 0]
                destroyed_customers_info[cc] = c_info
                new_state.customer_plan.pop(cc, None)
                if c_uid in new_state.uav_assignments:
                    new_state.uav_assignments[c_uid] = [
                        t for t in new_state.uav_assignments[c_uid] if t[2] != c_cn]
                if new_state.uav_cost and c_cn in new_state.uav_cost:
                    new_state.uav_cost.pop(c_cn, None)
                vehicle_task_data = deep_remove_vehicle_task(
                    vehicle_task_data, ca, new_state.vehicle_routes, orig_vid)

    new_state.destroyed_node_cost = new_state.update_calculate_plan_cost(
        new_state.uav_cost, new_state.vehicle_routes)
    new_state.destroyed_customers_info = destroyed_customers_info
    new_state.vehicle_task_data = vehicle_task_data
    return new_state

def repair_closest_insertion(self, state, strategic_bonus=0, num_destroyed=1, force_vtp_mode=True):

    repaired_state = state.fast_copy()
    repaired_state.repair_objective = 0
    destroy_node = list(state.destroyed_customers_info.keys())
    insert_plan = []
    cN = []
    force_vtp_mode = True

    if not force_vtp_mode or not destroy_node:
        repaired_state.destroyed_customers_info = {}
        return repaired_state, insert_plan

    while len(destroy_node) > 0:
        target_idx = self.rng.integers(0, len(destroy_node))
        customer = destroy_node.pop(target_idx)

        customer_info = state.destroyed_customers_info.get(customer)
        c_node = customer_info[2] if customer_info else customer

        current_customers = list(repaired_state.customer_plan.keys())
        target_vehicle_id = None
        if current_customers:
            min_dist = float('inf')
            for c_prime in current_customers:
                c_prime_uav = repaired_state.customer_plan[c_prime][0]
                c_prime_node = repaired_state.customer_plan[c_prime][2]
                try:
                    dist = self.uav_travel[c_prime_uav][c_node][c_prime_node].totalDistance
                except (KeyError, AttributeError):
                    continue
                if dist < min_dist:
                    min_dist = dist
                    target_vehicle_id = repaired_state.customer_plan[c_prime][4]

        try:
            cur_at = repaired_state.calculate_rm_empty_vehicle_arrive_time(repaired_state.vehicle_routes)
            base_cost, uav_tw_vc, total_cd = calculate_window_cost(
                repaired_state.customer_plan, repaired_state.uav_cost, cur_at,
                self.vehicle, self.customer_time_windows_h,
                self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node)
        except Exception:
            base_cost = float('inf')
            uav_tw_vc, total_cd = {}, {}

        vr = repaired_state.vehicle_routes
        vtd = repaired_state.vehicle_task_data
        vat = repaired_state.calculate_rm_empty_vehicle_arrive_time(vr)

        candidates = []
        trad_result, is_swap = self._evaluate_traditional_insertion(
            customer, vr, vtd, vat, base_cost, uav_tw_vc, total_cd, repaired_state)
        if trad_result[0] or trad_result[1] is not None:
            tc, ts = trad_result
            if is_swap:
                candidates.append({'customer': customer, 'scheme': ts, 'cost': tc,
                                   'win_cost': ts['win_cost'], 'total_cost': ts['total_cost'],
                                   'type': 'heuristic_swap', 'vtp_node': None})
            else:
                candidates.append({'customer': customer, 'scheme': ts, 'cost': tc,
                                   'win_cost': tc, 'total_cost': tc + base_cost,
                                   'type': 'traditional', 'vtp_node': None})

        vtp_result, vtp_info = self._evaluate_vtp_expansion_insertion(
            customer, vr, vtd, vat, repaired_state)
        if vtp_info is not None:
            vn, vi_vid, vi_idx = vtp_info
            if vtp_result is not None:
                vc, vs = vtp_result
                candidates.append({'customer': customer, 'scheme': vs, 'cost': vc,
                                   'total_cost': vc, 'type': 'vtp_expansion',
                                   'vtp_node': vn, 'vtp_insert_vehicle_id': vi_vid,
                                   'vtp_insert_index': vi_idx})

        candidates = [c for c in candidates if c['scheme'] is not None]

        if target_vehicle_id is not None:
            filtered = []
            for cand in candidates:
                if cand['type'] == 'heuristic_swap':
                    if (cand['scheme']['new_scheme'][4] == target_vehicle_id or
                            cand['scheme']['new_scheme'][5] == target_vehicle_id):
                        filtered.append(cand)
                else:
                    if cand['scheme'][4] == target_vehicle_id or cand['scheme'][5] == target_vehicle_id:
                        filtered.append(cand)
            candidates = filtered

        candidates.sort(key=lambda x: x['total_cost'])

        success = _try_execute_candidates(self, repaired_state, candidates, customer,
                                          insert_plan, vr, vtd)
        if not success:
            cN.append(customer)

    for customer in cN:
        destroy_node_fallback = [customer]

        repaired_state.destroyed_customers_info = {
            customer: state.destroyed_customers_info.get(customer, [0]*7)}

        repaired_state, fb_plan = self.repair_random_greedy_insertion(
            repaired_state, strategic_bonus=0, num_destroyed=1, force_vtp_mode=True)
        insert_plan.extend(fb_plan)

    repaired_state._total_cost = repaired_state.win_total_objective()
    repaired_state.destroyed_customers_info = {}
    return repaired_state, insert_plan

def repair_nearby_random_insertion(self, state, strategic_bonus=0, num_destroyed=1,
                                    force_vtp_mode=True):

    repaired_state = state.fast_copy()
    repaired_state.repair_objective = 0
    destroy_node = list(state.destroyed_customers_info.keys())
    insert_plan = []
    distance_threshold = 25.0
    force_vtp_mode = True

    if not destroy_node:
        repaired_state.destroyed_customers_info = {}
        return repaired_state, insert_plan

    num_repaired = 0
    while len(destroy_node) > 0:
        target_idx = self.rng.integers(0, len(destroy_node))
        customer = destroy_node.pop(target_idx)

        customer_info = state.destroyed_customers_info.get(customer)
        c_node = customer_info[2] if customer_info else customer

        nearby_vehicles = set()
        current_customers = list(repaired_state.customer_plan.keys())
        if current_customers:
            for c_prime in current_customers:
                c_prime_node = repaired_state.customer_plan[c_prime][2]
                c_prime_uav = repaired_state.customer_plan[c_prime][0]
                try:
                    dist = self.uav_travel[c_prime_uav][c_node][c_prime_node].totalDistance
                except (KeyError, AttributeError):
                    continue
                if dist < distance_threshold:
                    nearby_vehicles.add(repaired_state.customer_plan[c_prime][4])
                    nearby_vehicles.add(repaired_state.customer_plan[c_prime][5])

        try:
            cur_at = repaired_state.calculate_rm_empty_vehicle_arrive_time(repaired_state.vehicle_routes)
            base_cost, uav_tw_vc, total_cd = calculate_window_cost(
                repaired_state.customer_plan, repaired_state.uav_cost, cur_at,
                self.vehicle, self.customer_time_windows_h,
                self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node)
        except Exception:
            base_cost = float('inf')
            uav_tw_vc, total_cd = {}, {}

        vr = repaired_state.vehicle_routes
        vtd = repaired_state.vehicle_task_data
        vat = repaired_state.calculate_rm_empty_vehicle_arrive_time(vr)

        all_candidates = []
        trad_result, is_swap = self._evaluate_traditional_insertion(
            customer, vr, vtd, vat, base_cost, uav_tw_vc, total_cd, repaired_state)
        if trad_result[0] or trad_result[1] is not None:
            tc, ts = trad_result
            if is_swap:
                all_candidates.append({'customer': customer, 'scheme': ts, 'cost': tc,
                                       'win_cost': ts['win_cost'], 'total_cost': ts['total_cost'],
                                       'type': 'heuristic_swap', 'vtp_node': None})
            else:
                all_candidates.append({'customer': customer, 'scheme': ts, 'cost': tc,
                                       'win_cost': tc, 'total_cost': tc + base_cost,
                                       'type': 'traditional', 'vtp_node': None})

        vtp_result, vtp_info = self._evaluate_vtp_expansion_insertion(
            customer, vr, vtd, vat, repaired_state)
        if vtp_info is not None:
            vn, vi_vid, vi_idx = vtp_info
            if vtp_result is not None:
                vc, vs = vtp_result
                all_candidates.append({'customer': customer, 'scheme': vs, 'cost': vc,
                                       'total_cost': vc, 'type': 'vtp_expansion',
                                       'vtp_node': vn, 'vtp_insert_vehicle_id': vi_vid,
                                       'vtp_insert_index': vi_idx})

        all_candidates = [c for c in all_candidates if c['scheme'] is not None]

        nearby_cands = []
        fallback_cands = []
        for cand in all_candidates:
            if cand['type'] == 'heuristic_swap':
                v1 = cand['scheme']['new_scheme'][4]
                v2 = cand['scheme']['new_scheme'][5]
            else:
                v1 = cand['scheme'][4]
                v2 = cand['scheme'][5]
            if v1 in nearby_vehicles or v2 in nearby_vehicles:
                nearby_cands.append(cand)
            else:
                fallback_cands.append(cand)

        if nearby_cands:
            self.rng.shuffle(nearby_cands)
        fallback_cands.sort(key=lambda x: x['total_cost'])
        final_candidates = nearby_cands + fallback_cands

        success = _try_execute_candidates(self, repaired_state, final_candidates, customer,
                                          insert_plan, vr, vtd)
        if not success:
            repaired_state.repair_objective = float('inf')
            repaired_state.destroyed_customers_info = {}
            return repaired_state, insert_plan

        if customer in destroy_node:
            destroy_node.remove(customer)
        num_repaired += 1

    repaired_state._total_cost = repaired_state.win_total_objective()
    repaired_state.destroyed_customers_info = {}
    return repaired_state, insert_plan

def repair_adaptive_noise_insertion(self, state, strategic_bonus=0, num_destroyed=1,
                                     force_vtp_mode=True):

    repaired_state = state.fast_copy()
    repaired_state.repair_objective = 0
    destroy_node = list(state.destroyed_customers_info.keys())
    insert_plan = []
    force_vtp_mode = True

    if not destroy_node:
        repaired_state.destroyed_customers_info = {}
        return repaired_state, insert_plan

    eta = self.temperature / max(self.initial_temperature, 1e-6)
    eta = max(0.01, min(1.0, eta))

    num_repaired = 0
    while len(destroy_node) > 0:
        target_idx = self.rng.integers(0, len(destroy_node))
        customer = destroy_node.pop(target_idx)

        try:
            cur_at = repaired_state.calculate_rm_empty_vehicle_arrive_time(repaired_state.vehicle_routes)
            base_cost, uav_tw_vc, total_cd = calculate_window_cost(
                repaired_state.customer_plan, repaired_state.uav_cost, cur_at,
                self.vehicle, self.customer_time_windows_h,
                self.early_arrival_cost, self.late_arrival_cost, self.uav_travel, self.node)
        except Exception:
            base_cost = float('inf')
            uav_tw_vc, total_cd = {}, {}

        vr = repaired_state.vehicle_routes
        vtd = repaired_state.vehicle_task_data
        vat = repaired_state.calculate_rm_empty_vehicle_arrive_time(vr)

        candidates = []
        trad_result, is_swap = self._evaluate_traditional_insertion(
            customer, vr, vtd, vat, base_cost, uav_tw_vc, total_cd, repaired_state)
        if trad_result[0] or trad_result[1] is not None:
            tc, ts = trad_result
            if is_swap:
                candidates.append({'customer': customer, 'scheme': ts, 'cost': tc,
                                   'win_cost': ts['win_cost'], 'total_cost': ts['total_cost'],
                                   'type': 'heuristic_swap', 'vtp_node': None})
            else:
                candidates.append({'customer': customer, 'scheme': ts, 'cost': tc,
                                   'win_cost': tc, 'total_cost': tc + base_cost,
                                   'type': 'traditional', 'vtp_node': None})

        vtp_result, vtp_info = self._evaluate_vtp_expansion_insertion(
            customer, vr, vtd, vat, repaired_state)
        if vtp_info is not None:
            vn, vi_vid, vi_idx = vtp_info
            if vtp_result is not None:
                vc, vs = vtp_result
                candidates.append({'customer': customer, 'scheme': vs, 'cost': vc,
                                   'total_cost': vc, 'type': 'vtp_expansion',
                                   'vtp_node': vn, 'vtp_insert_vehicle_id': vi_vid,
                                   'vtp_insert_index': vi_idx})

        candidates = [c for c in candidates if c['scheme'] is not None]

        if candidates:

            costs = [c['total_cost'] for c in candidates if c['total_cost'] < float('inf')]
            if len(costs) >= 2:
                sigma_cost = np.std(costs)
            elif costs:
                sigma_cost = abs(costs[0]) * 0.1
            else:
                sigma_cost = 1.0

            for cand in candidates:
                noise = eta * float(self.rng.normal(0, 1)) * sigma_cost
                cand['noisy_cost'] = cand['total_cost'] + noise

            candidates.sort(key=lambda x: x.get('noisy_cost', x['total_cost']))

        success = _try_execute_candidates(self, repaired_state, candidates, customer,
                                          insert_plan, vr, vtd)
        if not success:
            repaired_state.repair_objective = float('inf')
            repaired_state.destroyed_customers_info = {}
            return repaired_state, insert_plan

        if customer in destroy_node:
            destroy_node.remove(customer)
        num_repaired += 1

    repaired_state._total_cost = repaired_state.win_total_objective()
    repaired_state.destroyed_customers_info = {}
    return repaired_state, insert_plan

def _try_execute_candidates(solver, repaired_state, candidates, customer,
                            insert_plan, vehicle_route, vehicle_task_data):

    for candidate in candidates:
        if candidate['type'] == 'traditional':
            best_scheme = candidate['scheme']
            best_cost = solver.drone_insert_cost(
                best_scheme[0], best_scheme[2], best_scheme[1], best_scheme[3])
            did, ln, cn, rn, lv_id, rv_id = best_scheme

            temp_cp = {k: v for k, v in repaired_state.customer_plan.items()}
            temp_cp[cn] = best_scheme
            temp_at = repaired_state.calculate_rm_empty_vehicle_arrive_time(vehicle_route)
            if not is_time_feasible(temp_cp, temp_at):
                continue

            repaired_state.customer_plan[cn] = best_scheme
            if did not in repaired_state.uav_assignments:
                repaired_state.uav_assignments[did] = []
            repaired_state.uav_assignments[did].append(best_scheme)
            if repaired_state.uav_cost is None:
                repaired_state.uav_cost = {}
            repaired_state.uav_cost[cn] = best_cost
            update_vehicle_task(vehicle_task_data, best_scheme, vehicle_route)
            insert_plan.append((customer, best_scheme, best_cost, 'traditional'))
            return True

        elif candidate['type'] == 'heuristic_swap':
            orig_s = candidate['scheme']['orig_scheme']
            new_s = candidate['scheme']['new_scheme']
            orig_c = candidate['scheme']['orig_cost']
            new_c = candidate['scheme']['new_cost']
            o_did, o_ln, o_cust, o_rn, o_lv, o_rv = orig_s
            n_did, n_ln, n_cust, n_rn, n_lv, n_rv = new_s
            del_cust = o_cust

            temp_cp = {k: v for k, v in repaired_state.customer_plan.items()}
            del_task = temp_cp[o_cust]
            del temp_cp[o_cust]
            temp_cp[o_cust] = orig_s
            temp_cp[n_cust] = new_s
            temp_at = repaired_state.calculate_rm_empty_vehicle_arrive_time(vehicle_route)
            if not is_time_feasible(temp_cp, temp_at):
                continue

            del repaired_state.customer_plan[del_cust]
            repaired_state.customer_plan[o_cust] = orig_s
            repaired_state.customer_plan[n_cust] = new_s
            for did_x, scheme_x in [(o_did, orig_s), (n_did, new_s)]:
                if did_x not in repaired_state.uav_assignments:
                    repaired_state.uav_assignments[did_x] = []
                repaired_state.uav_assignments[did_x].append(scheme_x)
            del repaired_state.uav_cost[del_cust]
            repaired_state.uav_cost[o_cust] = orig_c
            repaired_state.uav_cost[n_cust] = new_c
            remove_vehicle_task(vehicle_task_data, del_task, vehicle_route)
            o_lt = temp_at[o_lv][o_ln]
            n_lt = temp_at[n_lv][n_ln]
            if o_lt <= n_lt:
                update_vehicle_task(vehicle_task_data, orig_s, vehicle_route)
                update_vehicle_task(vehicle_task_data, new_s, vehicle_route)
            else:
                update_vehicle_task(vehicle_task_data, new_s, vehicle_route)
                update_vehicle_task(vehicle_task_data, orig_s, vehicle_route)
            insert_plan.append((del_cust, orig_s, orig_c, 'heuristic_swap'))
            insert_plan.append((customer, new_s, new_c, 'heuristic_swap'))
            return True

        elif candidate['type'] == 'vtp_expansion':
            vtp_node = candidate['vtp_node']
            vi_idx = candidate['vtp_insert_index']
            vi_vid = candidate['vtp_insert_vehicle_id']
            best_scheme = candidate['scheme']
            did, ln, cn, rn, lv_id, rv_id = best_scheme
            orig_cost = solver.drone_insert_cost(did, cn, ln, rn)

            temp_cp = {k: v for k, v in repaired_state.customer_plan.items()}
            temp_cp[cn] = best_scheme
            temp_vr = [r[:] for r in vehicle_route]
            temp_vr[vi_vid - 1].insert(vi_idx, vtp_node)
            temp_at = repaired_state.calculate_rm_empty_vehicle_arrive_time(temp_vr)
            if not is_time_feasible(temp_cp, temp_at):
                continue

            route = vehicle_route[vi_vid - 1]
            route.insert(vi_idx, vtp_node)
            last_node = route[vi_idx - 1]
            if vi_idx == 1 or last_node == solver.DEPOT_nodeID:
                dl = solver.base_drone_assignment[vi_vid][:]
            else:
                dl = vehicle_task_data[vi_vid][last_node].drone_list[:]
            vehicle_task_data[vi_vid][vtp_node].drone_list = dl
            vehicle_task_data[vi_vid][vtp_node].launch_drone_list = []
            vehicle_task_data[vi_vid][vtp_node].recovery_drone_list = []
            update_vehicle_task(vehicle_task_data, best_scheme, vehicle_route)

            repaired_state.customer_plan[cn] = best_scheme
            if did not in repaired_state.uav_assignments:
                repaired_state.uav_assignments[did] = []
            repaired_state.uav_assignments[did].append(best_scheme)
            if repaired_state.uav_cost is None:
                repaired_state.uav_cost = {}
            repaired_state.uav_cost[cn] = orig_cost
            repaired_state.rm_empty_vehicle_route = [r[:] for r in repaired_state.vehicle_routes]
            repaired_state.rm_empty_vehicle_arrive_time = \
                repaired_state.calculate_rm_empty_vehicle_arrive_time(
                    repaired_state.rm_empty_vehicle_route)
            insert_plan.append((customer, best_scheme, orig_cost, 'vtp_expansion'))
            return True

    return False

def enhanced_accept(self, current_cost, new_cost, temperature):

    if new_cost < current_cost:
        return True

    if hasattr(self, '_best_objective_for_accept'):
        temp_ratio = temperature / max(self.initial_temperature, 1e-6)
        threshold = 0.05 * temp_ratio
        if new_cost < self._best_objective_for_accept * (1.0 + threshold):
            return True

    delta = new_cost - current_cost
    if temperature > 1e-10:
        prob = np.exp(-delta / temperature)
        return float(self.rng.random()) < prob
    return False

def enhanced_stagnation_handler(self, no_improve_count, no_improve_threshold,
                                 current_state, best_state, best_objective, iteration):

    level1 = max(30, no_improve_threshold // 3)
    level2 = max(50, no_improve_threshold * 2 // 3)
    level3 = no_improve_threshold

    if no_improve_count >= level3:

        print(f"  > [Level-3 Escape] 连续 {no_improve_count} 轮无改善, 强力重启")
        new_state = best_state.fast_copy()
        new_temp = self.initial_temperature * 0.5

        for strat in self.operator_weights:
            for op_type in ['destroy', 'repair']:
                for op_name in self.operator_weights[strat][op_type]:
                    self.operator_weights[strat][op_type][op_name] = 1.0
        return new_state, best_objective, new_temp, 0

    elif no_improve_count >= level2:

        print(f"  > [Level-2 Escape] 连续 {no_improve_count} 轮无改善, 回退最优解并重热")
        new_state = best_state.fast_copy()
        new_temp = min(self.temperature * 3.0, self.initial_temperature * 0.3)
        return new_state, best_objective, new_temp, 0

    elif no_improve_count >= level1:

        print(f"  > [Level-1 Escape] 连续 {no_improve_count} 轮无改善, 温和重热")
        new_temp = min(self.temperature * 2.0, self.initial_temperature * 0.2)
        return current_state, None, new_temp, no_improve_count

    return current_state, None, self.temperature, no_improve_count

def calculate_window_cost(customer_plan, uav_cost, vehicle_arrive_time,
                          vehicle, customer_time_windows_h, early_arrival_cost,
                          late_arrival_cost, uav_travel, node):

    from cost_y import calculate_window_cost as _cwc
    return _cwc(customer_plan, uav_cost, vehicle_arrive_time, vehicle,
                customer_time_windows_h, early_arrival_cost, late_arrival_cost,
                uav_travel, node)

def patch_incremental_alns(alns_instance):

    import types

    alns_instance.destroy_proximity_removal = types.MethodType(
        destroy_proximity_removal, alns_instance)
    alns_instance.destroy_window_removal = types.MethodType(
        destroy_window_removal, alns_instance)
    alns_instance.destroy_drone_workload_removal = types.MethodType(
        destroy_drone_workload_removal, alns_instance)

    alns_instance.repair_closest_insertion = types.MethodType(
        repair_closest_insertion, alns_instance)
    alns_instance.repair_nearby_random_insertion = types.MethodType(
        repair_nearby_random_insertion, alns_instance)
    alns_instance.repair_adaptive_noise_insertion = types.MethodType(
        repair_adaptive_noise_insertion, alns_instance)

    new_destroy_ops = [
        alns_instance.destroy_proximity_removal,
        alns_instance.destroy_window_removal,
        alns_instance.destroy_drone_workload_removal,
    ]
    new_repair_ops = [
        alns_instance.repair_closest_insertion,
        alns_instance.repair_nearby_random_insertion,
        alns_instance.repair_adaptive_noise_insertion,
    ]
    alns_instance.destroy_operators.extend(new_destroy_ops)
    alns_instance.repair_operators.extend(new_repair_ops)

    for strat in alns_instance.operator_weights:
        for op in new_destroy_ops:
            alns_instance.operator_weights[strat]['destroy'][op.__name__] = 1.0
        for op in new_repair_ops:
            alns_instance.operator_weights[strat]['repair'][op.__name__] = 1.0

    alns_instance._original_sa_accept = alns_instance._simulated_annealing_accept
    alns_instance._simulated_annealing_accept = types.MethodType(
        enhanced_accept, alns_instance)

    alns_instance.enhanced_stagnation_handler = types.MethodType(
        enhanced_stagnation_handler, alns_instance)

    alns_instance._best_objective_for_accept = float('inf')

    print("=" * 70)
    print("  [PATCH] IncrementalALNS 增强补丁已安装!")
    print(f"  新增破坏算子: {[op.__name__ for op in new_destroy_ops]}")
    print(f"  新增修复算子: {[op.__name__ for op in new_repair_ops]}")
    print(f"  接受准则: SA + Record-to-Record Travel 混合")
    print(f"  停滞策略: 三级自适应逃逸")
    print(f"  总算子数: {len(alns_instance.destroy_operators)}D × "
          f"{len(alns_instance.repair_operators)}R = "
          f"{len(alns_instance.destroy_operators) * len(alns_instance.repair_operators)} 组合")
    print("=" * 70)

"""
在 IncrementalALNS.solve() 方法中需要做的两处小改动:

改动 1 — 更新 best_objective 时同步 RtRT 追踪变量:
    在 "if new_objective < best_objective:" 分支内添加:
        self._best_objective_for_accept = best_objective

改动 2 — 替换停滞检测逻辑:
    将原来的:
        if no_improve_count >= no_improve_threshold:
            current_state = best_state.fast_copy()
            current_objective = best_objective
            self.temperature = min(self.temperature * 3.0, self.initial_temperature * 0.5)
            no_improve_count = 0

    替换为:
        if no_improve_count >= no_improve_threshold // 3:  # 更早触发
            result = self.enhanced_stagnation_handler(
                no_improve_count, no_improve_threshold,
                current_state, best_state, best_objective, iteration)
            new_state, new_obj, new_temp, no_improve_count = result
            current_state = new_state
            if new_obj is not None:
                current_objective = new_obj
            self.temperature = new_temp
"""
