import contextlib
import copy
import json
import math
import os
import random
import time
from collections import Counter, defaultdict

import numpy as np
import fast_alns_solver as legacy
from fast_alns_solver import IncrementalALNS
from task_data import save_alns_results, extract_arrive_time_from_plan
from cost_y import calculate_window_cost, get_max_completion_time

def _identity_layout(routes, plan, base, arrive):

    launches, recoveries, invalid = defaultdict(list), defaultdict(list), set()
    positions = {v: {n: i for i, n in enumerate(r)} for v, r in enumerate(routes, 1)}
    for c, mission in sorted(plan.items()):
        d, l, _, r, lv, rv = mission
        if l not in positions.get(lv, {}) or r not in positions.get(rv, {}):
            invalid.add(c)
            continue
        if arrive[lv][l] >= arrive[rv][r]:
            invalid.add(c)
        launches[lv, l].append((d, c))
        recoveries[rv, r].append((d, c))
    layout = {}
    for v, route in enumerate(routes, 1):
        onboard = set(base[v])
        for n in route:
            recovered, launched = recoveries[v, n], launches[v, n]
            for d, c in recovered:
                onboard.add(d)
            for d, c in launched:
                if d not in onboard:
                    invalid.add(c)
                onboard.discard(d)
            layout[v, n] = (sorted(onboard), sorted(d for d, _ in launched), sorted(d for d, _ in recovered))

    by_drone = defaultdict(list)
    owner = {d: v for v, ds in base.items() for d in ds}
    for c, m in plan.items():
        if m[1] in positions.get(m[4], {}) and m[3] in positions.get(m[5], {}):
            by_drone[m[0]].append((arrive[m[4]][m[1]], c, m))
    for d, missions in by_drone.items():
        last_v, last_t, last_node = owner[d], -math.inf, None
        for launch_t, c, m in sorted(missions):
            if m[4] != last_v or launch_t < last_t - 1e-10:
                invalid.add(c)
            if launch_t == last_t and last_node is not None and m[1] != last_node:
                invalid.add(c)
            last_v, last_t, last_node = m[5], arrive[m[5]][m[3]], m[3]
    return layout, invalid

_FAST_VERIFY_IDENTITY = os.environ.get('FAST_VERIFY_IDENTITY') == '1'

def _identity_add_invalid(routes, d_missions, customer, mission, base, owner, arrive):

    d, l, _, r, lv, rv = mission
    route_l = routes[lv - 1] if 1 <= lv <= len(routes) else ()
    route_r = routes[rv - 1] if 1 <= rv <= len(routes) else ()
    if l not in route_l or r not in route_r:
        return True
    if arrive[lv][l] >= arrive[rv][r]:
        return True
    missions = d_missions + [(customer, mission)]
    launch_count, recovery_at = {}, set()
    for c, m in missions:
        key = (m[4], m[1])
        launch_count[key] = launch_count.get(key, 0) + 1
        recovery_at.add((m[5], m[3]))
    for v in {key[0] for key in launch_count}:
        onboard = d in base[v]
        for n in routes[v - 1]:
            if (v, n) in recovery_at:
                onboard = True
            k = launch_count.get((v, n), 0)
            if k:
                if not onboard or k > 1:
                    return True
                onboard = False
    last_v, last_t, last_node = owner[d], -math.inf, None
    for launch_t, c, m in sorted((arrive[m[4]][m[1]], c, m) for c, m in missions):
        if m[4] != last_v or launch_t < last_t - 1e-10:
            return True
        if launch_t == last_t and last_node is not None and m[1] != last_node:
            return True
        last_v, last_t, last_node = m[5], arrive[m[5]][m[3]], m[3]
    return False

def rebuild_drone_identity(state, base_assignment, validate_only=False):

    arrive = state.calculate_rm_empty_vehicle_arrive_time(state.vehicle_routes)
    layout, invalid = _identity_layout(state.vehicle_routes, state.customer_plan, base_assignment, arrive)
    fixes = 0
    for (v, n), fields in layout.items():
        entry = state.vehicle_task_data[v][n]
        for attr, expected in zip(('drone_list', 'launch_drone_list', 'recovery_drone_list'), fields):
            if list(getattr(entry, attr) or []) != expected:
                fixes += 1
                if not validate_only:
                    setattr(entry, attr, expected)
        for d in state.V:
            dentry = state.vehicle_task_data[d][n]
            expected = v if d in fields[0] or d in fields[1] or d in fields[2] else None
            if dentry.dict_vehicle[v]['drone_belong'] != expected:
                fixes += 1
                if not validate_only:
                    dentry.dict_vehicle[v]['drone_belong'] = expected
    return {'corrections': fixes, 'invalid_customers': sorted(invalid)}

def validate_drone_identity(state, base_assignment):
    return rebuild_drone_identity(state, base_assignment, validate_only=True)

class _QuietOutput:

    def __init__(self, stream):
        self.stream = stream
        self.pending = ''

    def write(self, text):
        self.pending += text
        while '\n' in self.pending:
            line, self.pending = self.pending.split('\n', 1)
            if '[r12-hfix]' in line or '[bcplus' in line:
                self.stream.write(line + '\n')
        return len(text)

    def flush(self):
        self.stream.flush()

class IncrementalALNSPlus(IncrementalALNS):

    DEFAULT_FLAGS = dict(cbs_objective=True, search_control=True, repair_fallback=True,
                         identity_rebuild=True, externality_learning=True, cbs_consistency=True,
                         externality_insertion=True, route_local_search=True,
                         elite_restart=True, forbid_idle_trucks=False, cop_repair=False)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.bcplus_flags = self.DEFAULT_FLAGS.copy()
        self.bcplus_flags.update(json.loads(os.environ.get('BCPLUS_FLAGS', '{}')))
        unknown = set(self.bcplus_flags) - set(self.DEFAULT_FLAGS)
        if unknown or any(type(v) is not bool for v in self.bcplus_flags.values()):
            raise ValueError('BCPLUS_FLAGS must contain known boolean flags')
        self.destroy_operators = [self.destroy_plus_random, self.destroy_plus_worst, self.destroy_plus_related]
        self.repair_operators = [self.repair_plus_greedy, self.repair_plus_regret, self.repair_plus_noise]
        if self.bcplus_flags.get('cop_repair', False):
            self.repair_operators.append(self.repair_class_regret)
        self.stats = Counter()
        self.fallback_counts = Counter(a=0, b=0, c=0)
        self.iteration_log = []
        self._flight_cache = {}
        self._destroy_scale = 1.0
        self._effort = 0
        self._externality_ema = {'launch': {}, 'recovery': {}, 'cross': {}}
        self._externality_lambda = 0.0
        self._externality_relative_error_ema = 1.0
        self._externality_seen_cross = False
        self._elite_pool = []
        self._externality_error = self._externality_scale = 0.0
        self.timing_replay = None
        self.learning_clock = os.environ.get('BCPLUS_LEARNING_CLOCK', 'work')
        if self.learning_clock not in ('work', 'wall'):
            raise ValueError('BCPLUS_LEARNING_CLOCK must be work or wall')
        replay_path = os.environ.get('BCPLUS_TIMING_REPLAY')
        if replay_path:
            with open(replay_path, encoding='utf-8') as handle:
                self.timing_replay = json.load(handle)['iteration_operator_log']

    def _rebuild(self, state):
        if self.bcplus_flags['identity_rebuild']:
            report = rebuild_drone_identity(state, self.base_drone_assignment)
            self.stats['identity_fix'] += report['corrections']
            self.stats['identity_rebuilds'] += 1
            return report['invalid_customers']
        return []

    def _copy(self, state):

        result = state.fast_copy()
        self._rebuild(result)
        return result

    def _sync(self, state):
        state.uav_assignments = {d: [] for d in self.V}
        for c, m in sorted(state.customer_plan.items()):
            state.uav_assignments[m[0]].append(tuple(m))
        state.uav_cost = {c: self._flight(m) for c, m in state.customer_plan.items()}
        state.rm_empty_vehicle_route = [r[:] for r in state.vehicle_routes]
        state.vehicle_arrive_time = state.calculate_rm_empty_vehicle_arrive_time(state.vehicle_routes)
        state.rm_empty_vehicle_arrive_time = state.vehicle_arrive_time
        state.uav_plan = legacy.update_re_uav_plan(state, {})
        state.repair_objective = 0.0
        self._rebuild(state)

    def _flight(self, mission):
        d, l, c, r = mission[:4]
        key = (d, l, c, r)
        if key not in self._flight_cache:
            self._flight_cache[key] = self.drone_insert_cost(d, c, l, r)
        return self._flight_cache[key]

    def _energy_feasible(self, mission):
        d, l, c, r = mission[:4]
        value = self.xeee.get(d, {}).get(self.node[l].map_key, {}).get(c, {}).get(self.node[r].map_key)
        return value is not None and value > 0

    def _window(self, state, costs, arrive, plan=None):
        return calculate_window_cost(state.customer_plan if plan is None else plan, costs, arrive,
                                     self.vehicle, self.customer_time_windows_h, self.early_arrival_cost,
                                     self.late_arrival_cost, self.uav_travel, self.node)

    def _idle_truck_ids(self, routes, plan):

        used = {(m[4], m[1]) for m in plan.values()} | {(m[5], m[3]) for m in plan.values()}
        return [v for v, route in enumerate(routes, 1)
                if not route[1:-1] or not any((v, n) in used for n in route[1:-1])]

    def _idle_trucks(self, state):
        return self._idle_truck_ids(state.vehicle_routes, state.customer_plan)

    def _fleet_feasible(self, state):
        return not self.bcplus_flags.get('forbid_idle_trucks', False) or not self._idle_trucks(state)

    def _nonidle_removal(self, state, requested):

        removed, remaining = set(), dict(state.customer_plan)
        initially_idle = set(self._idle_trucks(state))
        times = state.calculate_rm_empty_vehicle_arrive_time(state.vehicle_routes)
        for customer in sorted(requested):
            trial_removed = removed | {customer}
            while True:
                trial_plan = {c: m for c, m in state.customer_plan.items() if c not in trial_removed}
                _, invalid = _identity_layout(state.vehicle_routes, trial_plan, self.base_drone_assignment, times)
                if not invalid - trial_removed:
                    break
                trial_removed.update(invalid)
            if set(self._idle_truck_ids(state.vehicle_routes, trial_plan)) <= initially_idle:
                removed, remaining = trial_removed, trial_plan
        return removed, remaining

    def _remove(self, state, customers, structural):
        result = self._copy(state)
        removed = set(customers)
        requested = set(removed)

        while True:
            remaining = {c: m for c, m in state.customer_plan.items() if c not in removed}
            times = state.calculate_rm_empty_vehicle_arrive_time(state.vehicle_routes)
            _, invalid = _identity_layout(state.vehicle_routes, remaining, self.base_drone_assignment, times)
            additional = invalid - removed
            if not additional:
                break
            removed.update(additional)
        if self.bcplus_flags.get('forbid_idle_trucks', False) and self._idle_truck_ids(state.vehicle_routes, remaining):
            removed, remaining = self._nonidle_removal(state, requested)
        result.customer_plan = remaining
        if not self.bcplus_flags['identity_rebuild']:
            for c in sorted(removed):
                legacy.remove_vehicle_task(result.vehicle_task_data, state.customer_plan[c], result.vehicle_routes)
        if structural:
            used = {(m[4], m[1]) for m in remaining.values()} | {(m[5], m[3]) for m in remaining.values()}
            for v, route in enumerate(result.vehicle_routes, 1):
                kept = [n for n in route[1:-1] if (v, n) in used]

                for n in route[1:-1]:
                    if len(kept) >= 2:
                        break
                    if n not in kept:
                        kept.append(n)
                result.vehicle_routes[v - 1] = [route[0]] + [n for n in route[1:-1] if n in kept] + [route[-1]]
        arrive = result.calculate_rm_empty_vehicle_arrive_time(result.vehicle_routes)
        _, invalid = _identity_layout(result.vehicle_routes, remaining, self.base_drone_assignment, arrive)
        if invalid:
            result.vehicle_routes = [r[:] for r in state.vehicle_routes]
        result.destroyed_customers_info = {c: tuple(state.customer_plan[c]) for c in sorted(removed)}
        self._sync(result)
        return result

    def _destroy_count(self, state):
        ratio = float(self.rng.uniform(0.12, 0.28))
        return max(1, int(math.ceil(len(state.customer_plan) * ratio * self._destroy_scale)))

    def destroy_plus_random(self, state, force_vtp_mode=False):
        customers = sorted(state.customer_plan)
        selected = self.rng.choice(customers, size=min(len(customers), self._destroy_count(state)), replace=False)
        return self._remove(state, selected, force_vtp_mode)

    def destroy_plus_worst(self, state, force_vtp_mode=False):
        costs = getattr(state, 'bcplus_metrics', {}).get('post_costs', state.uav_cost)
        ranked = sorted(state.customer_plan, key=lambda c: (-costs[c] * float(self.rng.uniform(.9, 1.1)), c))
        return self._remove(state, ranked[:self._destroy_count(state)], force_vtp_mode)

    def destroy_plus_related(self, state, force_vtp_mode=False):
        seed = int(self.rng.choice(sorted(state.customer_plan)))
        position = np.asarray(self.node[seed].position)
        ranked = sorted(state.customer_plan, key=lambda c: (float(np.linalg.norm(np.asarray(self.node[c].position) - position)), c))
        return self._remove(state, ranked[:self._destroy_count(state)], force_vtp_mode)

    def destroy_cbs_externality_removal(self, state, force_vtp_mode=False):
        externality = getattr(state, 'bcplus_externality', {})
        magnitude = max([abs(x) for x in externality.values()] + [1.0])
        ranked = sorted(state.customer_plan, key=lambda c: (-externality.get(c, 0.0) - float(self.rng.uniform(0, .05)) * magnitude, c))
        seed = ranked[0]
        selected = {seed}
        missions = state.customer_plan

        d = missions[seed][0]
        if any(m[4] != m[5] for m in missions.values() if m[0] == d):
            selected.update(c for c, m in missions.items() if m[0] == d)
        target = getattr(self, "_destroy_count_target", None) or self._destroy_count(state)
        for c in ranked:
            if len(selected) >= target:
                break
            selected.add(c)
        return self._remove(state, selected, force_vtp_mode)

    def _variants(self, state, customer, expand, exhaustive=False):
        yield state.vehicle_routes, None
        if not expand:
            return
        near = sorted((n for n in self.N if n != self.DEPOT_nodeID and n in state.vehicle_task_data[self.T[0]]), key=lambda n: (float(np.linalg.norm(np.asarray(self.node[n].position) - np.asarray(self.node[customer].position))), n))
        if not exhaustive:
            near = near[:6]
        for v, route in enumerate(state.vehicle_routes, 1):
            for n in near:
                if n in route or not isinstance(self.veh_travel[v].get(route[0], {}).get(n), (int, float, np.number)):
                    continue
                positions = sorted(range(1, len(route)), key=lambda i: (self._calculate_vehicle_cost_increase(v, route, i, n), i))
                for i in positions if exhaustive else positions[:2]:
                    routes = [r[:] for r in state.vehicle_routes]
                    routes[v - 1].insert(i, n)
                    yield routes, (v, n, i)

    def _candidates(self, state, customer, expand=False, exhaustive=False):
        self._rebuild(state)
        externality_weight = self._externality_weight(state, track=True)
        candidates = []
        for routes, extra in self._variants(state, customer, expand, exhaustive):
            arrive = state.calculate_rm_empty_vehicle_arrive_time(routes)
            layout, invalid = _identity_layout(routes, state.customer_plan, self.base_drone_assignment, arrive)
            if invalid:
                continue
            endpoints = [(v, n) for v, route in enumerate(routes, 1) for n in route[1:-1]]
            fast_ok = customer not in state.customer_plan
            if fast_ok:
                d_index = defaultdict(list)
                for c0, m0 in state.customer_plan.items():
                    d_index[m0[0]].append((c0, m0))
                owner = {dd: vv for vv, ds in self.base_drone_assignment.items() for dd in ds}
            base_costs = state.uav_cost
            for lv, l in endpoints:
                for d in layout[lv, l][0]:
                    for rv, r in endpoints:
                        if arrive[lv][l] >= arrive[rv][r] or (extra and (extra[:2] not in ((lv, l), (rv, r)))):
                            continue
                        mission = (d, l, customer, r, lv, rv)
                        self._effort += 1
                        if not self._energy_feasible(mission):
                            continue
                        plan = dict(state.customer_plan)
                        plan[customer] = mission
                        if fast_ok:
                            bad = _identity_add_invalid(routes, d_index.get(d, []), customer, mission, self.base_drone_assignment, owner, arrive)
                            if _FAST_VERIFY_IDENTITY:
                                _, _inv = _identity_layout(routes, plan, self.base_drone_assignment, arrive)
                                assert bool(_inv) == bad, ('identity mismatch', mission)
                        else:
                            _, invalid = _identity_layout(routes, plan, self.base_drone_assignment, arrive)
                            bad = bool(invalid)
                        if bad:
                            continue
                        costs = dict(base_costs)
                        costs[customer] = self._flight(mission)
                        _, _, total = self._window(state, costs, arrive, plan)
                        objective = state.update_calculate_plan_cost(total, routes)
                        if externality_weight > 0:
                            objective += externality_weight * self._estimate_externality(plan)
                        candidates.append((float(objective), mission, extra))
        candidates.sort(key=lambda item: (item[0], item[1], item[2] or (0, 0, 0)))
        return candidates

    def _insert(self, state, candidate):
        _, mission, extra = candidate
        if extra:
            v, n, i = extra
            state.vehicle_routes[v - 1].insert(i, n)
        if not self.bcplus_flags['identity_rebuild']:
            legacy.update_vehicle_task(state.vehicle_task_data, mission, state.vehicle_routes)
        state.customer_plan[mission[2]] = mission
        self._sync(state)

    def _repair(self, state, mode, force_vtp_mode):
        result = self._copy(state)
        pending = sorted(set(state.destroyed_customers_info) - set(result.customer_plan))
        while pending:
            if mode == 'regret':
                options = [(c, self._candidates(result, c, expand=force_vtp_mode)) for c in pending]
                options = [(c, opts) for c, opts in options if opts]
                if not options:
                    break
                c, candidates = max(options, key=lambda item: ((item[1][1][0] - item[1][0][0]) if len(item[1]) > 1 else 1e6, -item[0]))
            else:
                c = pending[int(self.rng.integers(len(pending)))]
                candidates = self._candidates(result, c, expand=force_vtp_mode)
                if not candidates:
                    break
            candidate = candidates[0]
            if mode == 'noise':
                short = candidates[:5]
                candidate = min(short, key=lambda x: x[0] * float(self.rng.uniform(.98, 1.02)))
            self._insert(result, candidate)
            pending.remove(c)
        result.repair_objective = math.inf if pending else result.win_total_objective()
        return result, []

    def repair_plus_greedy(self, state, strategic_bonus=0, num_destroyed=1, force_vtp_mode=False):
        return self._repair(state, 'greedy', force_vtp_mode)

    def repair_plus_regret(self, state, strategic_bonus=0, num_destroyed=1, force_vtp_mode=False):
        return self._repair(state, 'regret', force_vtp_mode)

    def repair_plus_noise(self, state, strategic_bonus=0, num_destroyed=1, force_vtp_mode=False):
        return self._repair(state, 'noise', force_vtp_mode)

    def repair_class_regret(self, state, strategic_bonus=0, num_destroyed=1, force_vtp_mode=False):
        result = self._copy(state)
        pending = sorted(set(state.destroyed_customers_info) - set(result.customer_plan))
        self.stats['cop_calls'] += 1
        while pending:
            choices = []
            for cust in pending:
                pool = self._candidates(result, cust, expand=force_vtp_mode)
                cands = sorted(
                    (c for c in pool if math.isfinite(float(c[0]))),
                    key=lambda c: (float(c[0]), tuple(c[1]), tuple(c[2]) if c[2] is not None else (0, 0, 0)),
                )
                if not cands:
                    continue
                loc = [float(c[0]) for c in cands if c[2] is None]
                stc = [float(c[0]) for c in cands if c[2] is not None]
                both = bool(loc and stc)
                gap = abs(min(loc) - min(stc)) if both else 0.0
                reg = float(cands[1][0]) - float(cands[0][0]) if len(cands) > 1 else 0.0
                prio = (len(cands) == 1, not both, gap + reg, -int(cust))
                choices.append((prio, int(cust), cands[0]))
            if not choices:
                self.stats['cop_all_empty'] += 1
                break
            _, chosen_cust, chosen_cand = max(choices, key=lambda x: x[0])
            self._insert(result, chosen_cand)
            self.stats['cop_insertions'] += 1
            pending.remove(chosen_cust)
        result.repair_objective = math.inf if pending else result.win_total_objective()
        return result, []

    def _complete(self, state):
        if set(state.customer_plan) != self._customers or set(state.uav_cost) != self._customers:
            return False
        arrive = state.calculate_rm_empty_vehicle_arrive_time(state.vehicle_routes)
        _, invalid = _identity_layout(state.vehicle_routes, state.customer_plan, self.base_drone_assignment, arrive)
        return not invalid and all(self._energy_feasible(m) for m in state.customer_plan.values())

    def _restore(self, state, previous, customer):
        mission = tuple(previous.customer_plan[customer])
        routes = [r[:] for r in state.vehicle_routes]

        for v, n in ((mission[4], mission[1]), (mission[5], mission[3])):
            if n not in routes[v - 1]:
                old_route = previous.vehicle_routes[v - 1]
                successors = old_route[old_route.index(n) + 1:]
                successor = next((x for x in successors if x in routes[v - 1]), routes[v - 1][-1])

                pos = routes[v - 1].index(successor, 1)
                if successor == routes[v - 1][0]:
                    print(f'[bcplus-hpfix] restore: successor {successor} is the start depot of truck {v}; insert {n} at {pos} (v1.1: 0)')
                routes[v - 1].insert(pos, n)
        plan = dict(state.customer_plan)
        plan[customer] = mission
        arrive = state.calculate_rm_empty_vehicle_arrive_time(routes)
        _, invalid = _identity_layout(routes, plan, self.base_drone_assignment, arrive)
        if invalid or not self._energy_feasible(mission):
            return False
        state.vehicle_routes, state.customer_plan = routes, plan
        self._sync(state)
        return True

    def _fallback(self, state, previous, levels):
        missing = sorted(self._customers - set(state.customer_plan))
        if not missing:
            return state
        levels['a'] += 1

        for c in missing:
            candidates = self._candidates(state, c, expand=True, exhaustive=True)
            if candidates:
                self._insert(state, candidates[0])
                self.stats['fallback_a_customers'] += 1
        missing = sorted(self._customers - set(state.customer_plan))
        if missing:
            levels['b'] += 1

            missing.sort(key=lambda c: (previous.vehicle_arrive_time[previous.customer_plan[c][4]][previous.customer_plan[c][1]], c))
            for c in missing:
                if self._restore(state, previous, c):
                    self.stats['fallback_b_customers'] += 1
        return state

    def _cbs_consistency_errors(self, state):

        errors, by_drone = [], defaultdict(list)
        owner = {d: v for v, drones in self.base_drone_assignment.items() for d in drones}
        for m in state.final_uav_plan.values():
            d, c = m['drone_id'], m['customer']
            lt, rt = float(m['launch_time']), float(m['recovery_time'])
            if not (math.isfinite(lt) and math.isfinite(rt) and lt < rt):
                errors.append(f'launch_not_before_recovery:{c}')
            by_drone[d].append((lt, rt, m['launch_vehicle'], m['recovery_vehicle'], c))
        for d, missions in sorted(by_drone.items()):
            last_v, last_t = owner.get(d), -math.inf
            for lt, rt, lv, rv, c in sorted(missions):
                if lv != last_v:
                    errors.append(f'identity_chain_vehicle:{c}')
                if lt < last_t:
                    errors.append(f'identity_chain_time:{c}')
                last_v, last_t = rv, rt
        return errors

    def _cbs_feasible(self, state):
        return not self.bcplus_flags['cbs_consistency'] or state.bcplus_metrics.get('cbs_consistent', True)

    def _estimate_externality(self, plan):
        tables = getattr(self, '_externality_ema', {'launch': {}, 'recovery': {}, 'cross': {}})
        estimate = 0.0
        for m in plan.values():
            cross = m[4] != m[5]
            prior = tables['cross'].get(cross, 0.0)
            launch = tables['launch'].get((m[4], m[1], cross), prior)
            recovery = tables['recovery'].get((m[5], m[3], cross), prior)

            estimate += .5 * max(0.0, launch) + .5 * max(0.0, recovery)
            if cross:
                estimate += .25 * max(0.0, prior - tables['cross'].get(False, 0.0))
        return estimate

    def _externality_weight(self, state, track=False):
        trucks = getattr(self, 'T', getattr(state, 'vehicle_routes', ()))
        cross = getattr(self, '_externality_seen_cross', False) or any(m[4] != m[5] for m in state.customer_plan.values())
        error = getattr(self, '_externality_relative_error_ema', 1.0)
        ready = (self.bcplus_flags['externality_insertion'] and len(trucks) > 1 and cross and
                 self.stats['externality_observations'] >= 8 and math.isfinite(error) and error < .5)
        self._externality_lambda = 1.0 / (1.0 + error) if ready else 0.0
        if track and ready:
            self._ext_ins_used = True
        return self._externality_lambda

    def _learn_insertion_externality(self, state):
        if not self.bcplus_flags['externality_insertion'] or not self._cbs_feasible(state):
            self._externality_weight(state)
            return
        if not hasattr(self, '_externality_ema'):
            self._externality_ema = {'launch': {}, 'recovery': {}, 'cross': {}}
            self._externality_error = self._externality_scale = 0.0
            self._externality_lambda = 0.0
        predicted = self._estimate_externality(state.customer_plan)
        actual = sum(max(0.0, amount) for amount in state.bcplus_externality.values())
        alpha = .2
        relative_error = abs(predicted - actual) / max(1.0, actual)
        self._externality_relative_error_ema = ((1 - alpha) * getattr(self, '_externality_relative_error_ema', 1.0) +
                                                alpha * relative_error)
        self._externality_seen_cross = getattr(self, '_externality_seen_cross', False) or any(m[4] != m[5] for m in state.customer_plan.values())
        self._externality_error = (1 - alpha) * self._externality_error + alpha * abs(predicted - actual)
        self._externality_scale = (1 - alpha) * self._externality_scale + alpha * actual
        grouped = {name: defaultdict(list) for name in self._externality_ema}
        for c, m in sorted(state.customer_plan.items()):
            cross, value = m[4] != m[5], max(0.0, state.bcplus_externality[c])
            grouped['launch'][m[4], m[1], cross].append(value)
            grouped['recovery'][m[5], m[3], cross].append(value)
            grouped['cross'][cross].append(value)

        for name, samples in grouped.items():
            table = self._externality_ema[name]
            for key, values in samples.items():
                value = sum(values) / len(values)
                table[key] = (1 - alpha) * table.get(key, 0.0) + alpha * value
        self.stats['externality_observations'] += 1
        self._externality_weight(state)

    def _surrogate(self, state, routes, plan):
        arrive = state.calculate_rm_empty_vehicle_arrive_time(routes)
        costs = {c: self._flight(m) for c, m in plan.items()}
        _, _, total = self._window(state, costs, arrive, plan)
        objective = float(state.update_calculate_plan_cost(total, routes))
        externality_weight = self._externality_weight(state, track=True)
        if externality_weight > 0:
            objective += externality_weight * self._estimate_externality(plan)
        return objective

    def _evaluate(self, state):

        routes, _ = state.update_rm_empty_task()
        arrive = state.calculate_rm_empty_vehicle_arrive_time(routes)
        _, invalid = _identity_layout(routes, state.customer_plan, self.base_drone_assignment, arrive)
        if not invalid:
            state.vehicle_routes = routes
        self._sync(state)
        pre_window, pre_penalty, pre_costs = self._window(state, state.uav_cost, state.vehicle_arrive_time)
        pre = float(state.update_calculate_plan_cost(pre_costs, state.vehicle_routes))
        state._total_cost = state.destroyed_node_cost = pre

        (state.final_uav_plan, state.final_uav_cost, state.final_vehicle_plan_time,
         state.final_vehicle_task_data, state.final_global_reservation_table) = state.re_update_time(
             state.vehicle_routes, state.vehicle_arrive_time, state.vehicle_task_data, state)
        if set(state.final_uav_cost) != set(state.customer_plan):
            raise ValueError('CBS returned incomplete customer costs')
        if {m[2] for m in state.final_uav_plan} != set(state.customer_plan):
            raise ValueError('CBS returned incomplete missions')
        final_arrive = extract_arrive_time_from_plan(state.final_vehicle_plan_time)
        post_window, post_penalty, post_costs = self._window(state, state.final_uav_cost, final_arrive)
        post = float(state.update_calculate_plan_cost(post_costs, state.vehicle_routes))
        pre_max, pre_global = get_max_completion_time(state.vehicle_arrive_time)
        post_max, post_global = get_max_completion_time(final_arrive)
        metrics = dict(pre=pre, post=post, pre_window=pre_window, pre_penalty=pre_penalty,
                       pre_costs=pre_costs, post_window=post_window, post_penalty=post_penalty,
                       post_costs=post_costs, pre_max=pre_max, pre_global=pre_global,
                       post_max=post_max, post_global=post_global)
        state.bcplus_metrics = metrics
        errors = self._cbs_consistency_errors(state) if self.bcplus_flags['cbs_consistency'] else []
        metrics.update(cbs_consistent=not errors, cbs_errors=errors)
        self.stats['cbs_evaluations'] += 1
        if errors:
            self.stats['cbs_inconsistent'] += 1
            print(f'[bcplus-r2] CBS inconsistent: {errors}')
        state.final_best_objective = post

        state.bcplus_externality = {c: float(post_costs[c] - pre_costs[c]) for c in sorted(state.customer_plan)}
        state.bcplus_vtp_externality = defaultdict(float)
        state.bcplus_vehicle_externality = defaultdict(float)
        for c, amount in state.bcplus_externality.items():
            m = state.customer_plan[c]
            for v, n in ((m[4], m[1]), (m[5], m[3])):
                state.bcplus_vtp_externality[v, n] += amount / 2
                state.bcplus_vehicle_externality[v] += amount / 2
        if not math.isfinite(post):
            raise ValueError('Nonfinite CBS objective')
        self._learn_insertion_externality(state)
        return metrics

    def _route_neighborhoods(self, state):

        original = state.vehicle_routes

        def internal(reverse):
            for v, route in enumerate(original):
                for i in range(1, len(route) - 1):
                    for j in range(i + 1 if reverse else 1, len(route) - 1):
                        if i == j:
                            continue
                        routes = [r[:] for r in original]
                        if reverse:
                            routes[v][i:j + 1] = reversed(routes[v][i:j + 1])
                        else:
                            routes[v].insert(j, routes[v].pop(i))
                        yield routes, {}, 'two_opt' if reverse else 'or_opt'

        def transfers(short=False):

            sources = sorted(range(len(original)), key=lambda v: (len(original[v]), v))
            for v in sources:
                route = original[v]
                if short and not 1 <= len(route) - 2 <= 2:
                    continue
                blocks = [route[1:-1]] if short else [[n] for n in route[1:-1]]
                for block in blocks:
                    for w, target in enumerate(original):
                        if v == w:
                            continue
                        for j in range(1, len(target)):
                            routes = [r[:] for r in original]
                            routes[v] = [n for n in route if n not in block]
                            routes[w][j:j] = [n for n in block if n not in target]
                            remap = {(v + 1, n): w + 1 for n in block}
                            yield routes, remap, 'short_route' if short else 'relocate'

        def exchanges():
            for v, route in enumerate(original):
                for w in range(v + 1, len(original)):
                    target = original[w]
                    for i, n in enumerate(route[1:-1], 1):
                        for j, m in enumerate(target[1:-1], 1):
                            if n in target or m in route:
                                continue
                            routes = [r[:] for r in original]
                            routes[v][i], routes[w][j] = m, n
                            yield routes, {(v + 1, n): w + 1, (w + 1, m): v + 1}, 'exchange'

        families = [iter(internal(True)), iter(internal(False)), iter(transfers()),
                    iter(exchanges()), iter(transfers(True))]
        while families:
            active = []
            for family in families:
                candidate = next(family, None)
                if candidate is not None:
                    active.append(family)
                    routes, remap, _ = candidate
                    if self.bcplus_flags.get('forbid_idle_trucks', False):
                        plan = self._route_remap_plan(state.customer_plan, remap)
                        if self._idle_truck_ids(routes, plan):
                            continue
                    yield candidate
            families = active

    def _route_remap_plan(self, plan, remap):

        return {c: tuple(m[:4]) + (remap.get((m[4], m[1]), m[4]), remap.get((m[5], m[3]), m[5]))
                for c, m in plan.items()}

    def _route_repair(self, incumbent, routes, plan):

        if self.bcplus_flags.get('forbid_idle_trucks', False) and self._idle_truck_ids(routes, plan):
            return None
        arrive = incumbent.calculate_rm_empty_vehicle_arrive_time(routes)
        remaining, removed = dict(plan), {}
        while True:
            _, invalid = _identity_layout(routes, remaining, self.base_drone_assignment, arrive)
            if not invalid:
                break
            for c in invalid:
                removed[c] = remaining.pop(c)
            if len(removed) > 8:
                self.stats['route_ls_repair_budget'] += 1
                return None
        trial = self._copy(incumbent)
        trial.vehicle_routes, trial.customer_plan = [r[:] for r in routes], remaining
        trial.destroyed_customers_info = removed
        if not self.bcplus_flags['identity_rebuild']:
            rebuild_drone_identity(trial, self.base_drone_assignment)
        self._sync(trial)
        order = sorted(removed, key=lambda c: (arrive[removed[c][4]][removed[c][1]], c))
        for c in order:
            options = self._candidates(trial, c, expand=False)
            if not options:
                return None
            self._insert(trial, options[0])
        if not self._complete(trial) or not self._fleet_feasible(trial):
            return None
        trial.destroyed_customers_info = {}
        self.stats['route_ls_reinserted'] += len(removed)
        return trial

    def _route_local_search(self, incumbent, objective_key='post'):
        tick = time.perf_counter()
        self.stats['route_ls_calls'] += 1
        feasible, broken, seen = [], [], set()

        for index, (routes, remap, kind) in enumerate(self._route_neighborhoods(incumbent)):
            if index >= 512:
                self.stats['route_ls_enumeration_budget'] += 1
                break
            plan = self._route_remap_plan(incumbent.customer_plan, remap)
            if self.bcplus_flags.get('forbid_idle_trucks', False) and self._idle_truck_ids(routes, plan):
                continue
            key = (tuple(tuple(r) for r in routes), tuple(sorted(plan.items())))
            if key in seen or (routes == incumbent.vehicle_routes and plan == incumbent.customer_plan):
                continue
            seen.add(key)
            self.stats['route_ls_candidates'] += 1
            self.stats['route_ls_' + kind] += 1
            self._effort += 1
            arrive = incumbent.calculate_rm_empty_vehicle_arrive_time(routes)
            _, invalid = _identity_layout(routes, plan, self.base_drone_assignment, arrive)
            score = self._surrogate(incumbent, routes, plan)
            if math.isfinite(score):
                (broken if invalid else feasible).append((score, key, routes, plan, kind, None))
        for _, _, routes, plan, kind, _ in sorted(broken, key=lambda x: (x[0], x[1]))[:4]:
            self.stats['route_ls_repair_attempts'] += 1
            trial = self._route_repair(incumbent, routes, plan)
            if trial is None:
                continue
            key = (tuple(tuple(r) for r in trial.vehicle_routes), tuple(sorted(trial.customer_plan.items())))
            score = self._surrogate(trial, trial.vehicle_routes, trial.customer_plan)
            if math.isfinite(score):
                feasible.append((score, key, trial.vehicle_routes, trial.customer_plan, kind, trial))
        best, evaluated = incumbent, set()
        for _, key, routes, plan, kind, trial in sorted(feasible, key=lambda x: (x[0], x[1])):
            if key in evaluated:
                continue
            if len(evaluated) == 3:
                break
            if trial is None:
                trial = self._copy(incumbent)
                trial.vehicle_routes, trial.customer_plan = [r[:] for r in routes], dict(plan)
                trial.destroyed_customers_info = {}
                if not self.bcplus_flags['identity_rebuild']:
                    rebuild_drone_identity(trial, self.base_drone_assignment)
                self._sync(trial)
            if not self._complete(trial) or not self._fleet_feasible(trial):
                continue
            evaluated.add(key)
            self.stats['route_ls_cbs_calls'] += 1
            metrics = self._evaluate(trial)
            if not self._cbs_feasible(trial) or not self._fleet_feasible(trial):
                continue
            if not self._cbs_feasible(best) or not self._fleet_feasible(best) or metrics[objective_key] < best.bcplus_metrics[objective_key] - 1e-9:
                best = trial
                self.stats['route_ls_improvements'] += 1
                self.stats['route_ls_accepted_' + kind] += 1
        self.stats['route_ls_elapsed_s'] += time.perf_counter() - tick
        print(f'[bcplus-r2] route_local_search proposals={len(seen)} cbs={len(evaluated)} improved={int(best is not incumbent)}')
        return best

    def _choose(self, weights):
        names = list(weights)
        values = np.asarray([weights[n] for n in names], dtype=float)
        probabilities = values / values.sum()
        floor = min(.05, 1 / len(names))
        probabilities = floor + (1 - floor * len(names)) * probabilities
        return str(self.rng.choice(names, p=probabilities))

    def _learn(self, segment):

        positive = [r['rate'] for r in segment if r['rate'] > 0]
        normalizer = float(np.mean(positive)) if positive else 1.0
        for r in segment:
            r['score'] = min(10.0, r['rate'] / normalizer)
        groups = [(self.strategy_weights, 'strategy', None)]
        for strategy in self.strategy_weights:
            for kind in ('destroy', 'repair'):
                groups.append((self.operator_weights[strategy][kind], kind + '_operator', strategy))
        for weights, key, strategy in groups:
            for name in weights:
                rows = [r for r in segment if r[key] == name and (strategy is None or r['strategy'] == strategy)]
                if rows:
                    failure_rate = sum(r['repair_failed'] for r in rows) / len(rows)
                    target = max(.05, (.1 + sum(r['score'] for r in rows) / len(rows)) * (1 - failure_rate))
                    weights[name] = .5 * weights[name] + .5 * target

    def _elite_signature(self, state):
        return (tuple(tuple(route) for route in state.vehicle_routes),
                tuple((c, tuple(m)) for c, m in sorted(state.customer_plan.items())))

    def _remember_elite(self, state):
        if not self.bcplus_flags.get('elite_restart', True) or not self._fleet_feasible(state):
            return
        if not self._complete(state) or not self._cbs_feasible(state) or not math.isfinite(state.bcplus_metrics['post']):
            return
        pool = list(getattr(self, '_elite_pool', []))
        signature = self._elite_signature(state)
        duplicate = next((s for s in pool if self._elite_signature(s) == signature), None)
        if duplicate is not None:
            if duplicate.bcplus_metrics['post'] <= state.bcplus_metrics['post']:
                return
            pool = [s for s in pool if s is not duplicate]
        if len(pool) >= 5 and state.bcplus_metrics['post'] >= pool[-1].bcplus_metrics['post']:
            return
        pool.append(self._copy(state))
        self._elite_pool = sorted(pool, key=lambda s: (s.bcplus_metrics['post'], self._elite_signature(s)))[:5]

    def _elite_restart(self, current, iteration):
        pool = getattr(self, '_elite_pool', [])
        if not self.bcplus_flags.get('elite_restart', True) or not pool:
            return current
        self.stats['elite_restart_attempts'] += 1
        weights = 1.0 / np.arange(1, len(pool) + 1, dtype=float)
        rank = int(self.rng.choice(len(pool), p=weights / weights.sum()))
        parent = self._copy(pool[rank])
        customers = sorted(parent.customer_plan)
        lower, upper = max(1, math.ceil(.3 * len(customers))), max(1, math.floor(.5 * len(customers)))
        destroyed = None

        for _ in range(4):
            count = int(self.rng.integers(lower, max(lower, upper) + 1))
            selected = self.rng.choice(customers, size=min(count, len(customers)), replace=False)
            trial = self._remove(parent, selected, True)
            if lower <= len(trial.destroyed_customers_info) <= max(lower, upper):
                destroyed = trial
                break
        if destroyed is None:
            self.stats['elite_restart_failed'] += 1
            return current
        repaired, _ = self._repair(destroyed, 'noise', True)
        levels = Counter(a=0, b=0, c=0)
        if not self._complete(repaired) and self.bcplus_flags['repair_fallback']:
            repaired = self._fallback(repaired, parent, levels)
        self.fallback_counts.update(levels)
        if not self._complete(repaired) or not self._fleet_feasible(repaired):
            self.stats['elite_restart_failed'] += 1
            return current
        self._evaluate(repaired)
        if not self._cbs_feasible(repaired) or not self._fleet_feasible(repaired):
            self.stats['elite_restart_failed'] += 1
            return current
        if self._elite_signature(repaired) in (self._elite_signature(parent), self._elite_signature(current)):
            self.stats['elite_restart_unchanged'] += 1
            self.stats['elite_restart_failed'] += 1
            return current
        repaired.destroyed_customers_info = {}
        self._remember_elite(repaired)
        self.temperature = max(self.temperature, self.initial_temperature * .5)
        self.stats['elite_restarts'] += 1
        self._last_elite_restart = dict(iteration=iteration, rank=rank + 1, seeds=count,
                                       removed=len(destroyed.destroyed_customers_info),
                                       actual_fraction=len(destroyed.destroyed_customers_info) / len(customers),
                                       post=repaired.bcplus_metrics['post'])
        print(f'[bcplus-r3] elite_restart={self._last_elite_restart}')
        return repaired

    def _escape(self, stagnation, fired, current, best, iteration):
        thresholds = (30, 60, 100)
        for level, threshold in enumerate(thresholds, 1):
            if stagnation < threshold or level in fired:
                continue
            fired.add(level)
            self.stats[f'escape_l{level}'] += 1
            if level == 1:
                self.temperature = max(self.temperature, self.initial_temperature * .25)
            elif level == 2:
                current = self._copy(best)
                self.temperature = max(self.temperature, self.initial_temperature * .5)
                self._destroy_scale = 1.5
            else:
                self.temperature = self.initial_temperature
                self._destroy_scale = 2.0
                for group in [self.strategy_weights] + [g for pool in self.operator_weights.values() for g in pool.values()]:
                    mean = sum(group.values()) / len(group)
                    for name in group:
                        group[name] = .5 * (group[name] + mean)
            print(f'[bcplus-escape] iter={iteration} level={level} stagnation={stagnation}')
        return current

    def solve(self, initial_state):
        import sys
        with contextlib.redirect_stdout(_QuietOutput(sys.stdout)):
            return self._solve_plus(initial_state)

    def _solve_plus(self, initial_state):
        if self.bcplus_flags['externality_learning']:
            self.destroy_operators.append(self.destroy_cbs_externality_removal)
        fixed_d = os.environ.get('BCPLUS_FIXED_DESTROY', '').strip()
        fixed_r = os.environ.get('BCPLUS_FIXED_REPAIR', '').strip()
        if fixed_d:
            d_map = {
                'RAND': self.destroy_plus_random,
                'WORST': self.destroy_plus_worst,
                'REL': self.destroy_plus_related,
                'EXT': self.destroy_cbs_externality_removal,
            }
            if fixed_d not in d_map:
                raise ValueError(f'Unknown BCPLUS_FIXED_DESTROY={fixed_d!r}; expected one of {sorted(d_map)}')
            self.destroy_operators = [d_map[fixed_d]]
        if fixed_r:
            r_map = {
                'GRD': self.repair_plus_greedy,
                'REG': self.repair_plus_regret,
                'NOISE': self.repair_plus_noise,
                'CoP': self.repair_class_regret,
            }
            if fixed_r not in r_map:
                raise ValueError(f'Unknown BCPLUS_FIXED_REPAIR={fixed_r!r}; expected one of {sorted(r_map)}')
            self.repair_operators = [r_map[fixed_r]]
        self.strategy_weights = dict(structural=1.0, internal=1.0)
        self.operator_weights = {s: {kind: {op.__name__: 1.0 for op in ops}
                                    for kind, ops in [('destroy', self.destroy_operators), ('repair', self.repair_operators)]}
                                 for s in self.strategy_weights}
        self._customers = set(initial_state.customer_plan)
        current = self._copy(initial_state)
        current.vehicle_routes, _ = current.update_rm_empty_task()
        self._sync(current)
        initial_identity = validate_drone_identity(current, self.base_drone_assignment)
        if initial_identity['invalid_customers'] and self.bcplus_flags['identity_rebuild']:

            bad = initial_identity['invalid_customers']
            stripped = self._remove(current, bad, structural=False)
            removed_init = sorted(stripped.destroyed_customers_info)
            repaired, _ = self._repair(stripped, 'regret', False)
            repaired = self._fallback(repaired, current, Counter(a=0, b=0, c=0))
            self.stats['init_invalid_customers'] = len(bad)
            self.stats['init_reinserted_customers'] = len(removed_init)
            print(f'[bcplus-init-repair] invalid={bad} removed={removed_init} complete={self._complete(repaired)}')
            current = repaired
            initial_identity = validate_drone_identity(current, self.base_drone_assignment)
            if not self._complete(current):
                air_to_ground = {self.node[g].map_key: g for g in self.N}
                for c_miss in sorted(self._customers - set(current.customer_plan)):
                    placed = False

                    truck_order = sorted(self.T, key=lambda tv: (len(current.vehicle_routes[tv - 1]), tv))
                    for tv in truck_order:
                        drones_tv = list(self.base_drone_assignment.get(tv, ()))
                        if not drones_tv:
                            continue
                        d_pick = drones_tv[0]
                        best_pair = None
                        for i_air, cust_map in self.xeee.get(d_pick, {}).items():
                            k_map = cust_map.get(c_miss)
                            if not k_map:
                                continue
                            l_g = air_to_ground.get(i_air)
                            if l_g is None:
                                continue
                            for k_air, rem_e in k_map.items():
                                if rem_e is None or rem_e <= 0 or k_air == i_air:
                                    continue
                                r_g = air_to_ground.get(k_air)
                                if r_g is None or r_g == l_g:
                                    continue
                                fc = self.drone_insert_cost(d_pick, c_miss, l_g, r_g)
                                if best_pair is None or fc < best_pair[0]:
                                    best_pair = (fc, l_g, r_g)
                        if best_pair is not None:
                            _, l_g, r_g = best_pair
                            rt = current.vehicle_routes[tv - 1][:]

                            arr_tmp = current.calculate_rm_empty_vehicle_arrive_time(current.vehicle_routes)
                            lay_tmp, _ = _identity_layout(current.vehicle_routes, current.customer_plan, self.base_drone_assignment, arr_tmp)

                            carrier_tv = None
                            for cand_tv in truck_order:
                                cr = current.vehicle_routes[cand_tv - 1]
                                last_stop = cr[-2] if len(cr) >= 2 else cr[0]
                                if d_pick in lay_tmp.get((cand_tv, last_stop), ([], [], []))[0]:
                                    carrier_tv = cand_tv
                                    break
                            if carrier_tv is None:
                                continue
                            rt = current.vehicle_routes[carrier_tv - 1][:]
                            if l_g not in rt[1:-1]:
                                rt.insert(len(rt) - 1, l_g)
                            pos_l = rt.index(l_g, 1)
                            if r_g not in rt[pos_l + 1 : -1]:
                                if r_g in rt[1:pos_l]:

                                    continue
                                rt.insert(len(rt) - 1, r_g)
                            cand_routes = [r[:] for r in current.vehicle_routes]
                            cand_routes[carrier_tv - 1] = rt
                            cand_plan = dict(current.customer_plan)
                            cand_plan[c_miss] = (d_pick, l_g, c_miss, r_g, carrier_tv, carrier_tv)
                            arr_chk = current.calculate_rm_empty_vehicle_arrive_time(cand_routes)
                            _, inv_chk = _identity_layout(cand_routes, cand_plan, self.base_drone_assignment, arr_chk)
                            if not inv_chk:
                                current.vehicle_routes = cand_routes
                                current.customer_plan = cand_plan
                                self._sync(current)
                                placed = True
                                break
                    if not placed:

                        arr_tmp = current.calculate_rm_empty_vehicle_arrive_time(current.vehicle_routes)
                        lay_tmp, _ = _identity_layout(current.vehicle_routes, current.customer_plan, self.base_drone_assignment, arr_tmp)
                        for d_pick in self.V:
                            carrier_tv = None
                            for cand_tv in self.T:
                                cr = current.vehicle_routes[cand_tv - 1]
                                last_stop = cr[-2] if len(cr) >= 2 else cr[0]
                                if d_pick in lay_tmp.get((cand_tv, last_stop), ([], [], []))[0]:
                                    carrier_tv = cand_tv
                                    break
                            if carrier_tv is None:
                                continue
                            rt_base = current.vehicle_routes[carrier_tv - 1]
                            used_stops = set(rt_base[1:-1])
                            for i_air, cust_map in self.xeee.get(d_pick, {}).items():
                                k_map = cust_map.get(c_miss)
                                if not k_map:
                                    continue
                                l_g = air_to_ground.get(i_air)
                                if l_g is None or l_g in used_stops:
                                    continue
                                for k_air, rem_e in k_map.items():
                                    if rem_e is None or rem_e <= 0 or k_air == i_air:
                                        continue
                                    r_g = air_to_ground.get(k_air)
                                    if r_g is None or r_g == l_g or r_g in used_stops:
                                        continue
                                    rt = rt_base[:-1] + [l_g, r_g, rt_base[-1]]
                                    cand_routes = [r[:] for r in current.vehicle_routes]
                                    cand_routes[carrier_tv - 1] = rt
                                    cand_plan = dict(current.customer_plan)
                                    cand_plan[c_miss] = (d_pick, l_g, c_miss, r_g, carrier_tv, carrier_tv)
                                    arr_chk = current.calculate_rm_empty_vehicle_arrive_time(cand_routes)
                                    _, inv_chk = _identity_layout(cand_routes, cand_plan, self.base_drone_assignment, arr_chk)
                                    if not inv_chk:
                                        current.vehicle_routes = cand_routes
                                        current.customer_plan = cand_plan
                                        self._sync(current)
                                        placed = True
                                        break
                                if placed:
                                    break
                            if placed:
                                break
                initial_identity = validate_drone_identity(current, self.base_drone_assignment)
                print(f'[clean-v2][init-repair2] complete={self._complete(current)} invalid={initial_identity["invalid_customers"]}')
        if initial_identity['invalid_customers'] or not self._complete(current):
            raise ValueError(f'Initial identity chain invalid: {initial_identity}')
        self._evaluate(current)
        if not self._cbs_feasible(current):
            self.stats['cbs_inconsistent_initial'] += 1
            print('[bcplus-r2] Initial CBS output inconsistent; recorded only, awaiting a feasible incumbent.')
        self.stats.update(ext_ins_active_iters=0, elite_restarts=0)
        initial_idle = self._idle_trucks(current)
        self.stats['idle_trucks_initial'] = len(initial_idle)
        if initial_idle and self.bcplus_flags.get('forbid_idle_trucks', False):
            print(f'[bcplus-r3] Initial idle trucks recorded only: {initial_idle}')
        self._elite_pool = []
        self._remember_elite(current)
        best = self._copy(current)
        objective_key = 'post' if self.bcplus_flags['cbs_objective'] else 'pre'
        self.temperature = max(1e-6, -current.bcplus_metrics[objective_key] * .05 / math.log(.2))
        self.initial_temperature = self.temperature
        self._best_objective_for_accept = best.bcplus_metrics[objective_key]
        schedule_iters = int(os.environ.get('R12_SCHEDULE_ITERS') or self.max_iterations)
        cooling = max(.95, min(.9999, (.01 / self.temperature) ** (1 / max(1, schedule_iters))))
        traces = {name: [] for name in ('y_best', 'win_cost', 'uav_route_cost', 'vehicle_route_cost',
                                      'final_uav_cost', 'final_total_list', 'final_win_cost',
                                      'final_total_objective', 'final_vehicle_route_cost', 'work_time', 'final_work_time')}
        traces['y_cost'] = legacy._R12TimedList()
        self._record(traces, current, current, best, objective_key)
        start = time.perf_counter()
        stagnation, fired, segment, history = 0, set(), [], []
        route_l1_fired = False
        restart_due = 60
        for iteration in range(1, self.max_iterations + 1):
            tick = time.perf_counter()
            self._ext_ins_used = False
            restart_before = self.stats['elite_restarts']
            cbs_before = self.stats['cbs_evaluations']
            strategy = self._choose(self.strategy_weights)
            dn = self._choose(self.operator_weights[strategy]['destroy'])
            rn = self._choose(self.operator_weights[strategy]['repair'])
            structural = strategy == 'structural'
            previous = current
            before = current.bcplus_metrics[objective_key]
            self._destroy_count_target = self._destroy_count(previous)
            effort_before = self._effort
            identity_before = self.stats['identity_fix']
            route_cbs_before = self.stats['route_ls_cbs_calls']
            destroyed = self._r12_safe_destroy(getattr(self, dn), previous, force_vtp_mode=structural)
            self._rebuild(destroyed)
            repaired, _ = self._r12_safe_repair(getattr(self, rn), destroyed, 0, len(destroyed.destroyed_customers_info), force_vtp_mode=structural)
            if not self.bcplus_flags['search_control'] and not structural:
                repaired, _ = self._r12_safe_repair(getattr(self, rn), destroyed, 0, len(destroyed.destroyed_customers_info), force_vtp_mode=True)
            repair_failed = not self._complete(repaired)
            levels = Counter(a=0, b=0, c=0)
            if repair_failed and self.bcplus_flags['repair_fallback']:
                repaired = self._fallback(repaired, previous, levels)
                if not self._complete(repaired):
                    levels['c'] += 1
                    removed = sorted(destroyed.destroyed_customers_info)

                    cap = max(1, len(removed) // 2)
                    retry = None
                    for c in removed:
                        trial = self._remove(previous, [c], structural)
                        if len(trial.destroyed_customers_info) <= cap:
                            selected = set(trial.destroyed_customers_info)
                            for other in removed:
                                candidate = self._remove(previous, selected | {other}, structural)
                                if len(candidate.destroyed_customers_info) <= cap:
                                    trial, selected = candidate, set(candidate.destroyed_customers_info)
                            retry = trial
                            break
                    if retry is not None:
                        repaired, _ = self._r12_safe_repair(getattr(self, rn), retry, 0, len(retry.destroyed_customers_info), force_vtp_mode=structural)
                        repaired = self._fallback(repaired, previous, levels)
            complete = self._complete(repaired) and self._fleet_feasible(repaired)
            accepted, new_best, improved = False, False, False
            proposed, gain = math.inf, 0.0
            cbs_valid = True
            if complete:
                metrics = self._evaluate(repaired)
                proposed = metrics[objective_key]
                cbs_valid = self._cbs_feasible(repaired) and self._fleet_feasible(repaired)
                self._remember_elite(repaired)
                accepted = cbs_valid and (not self._cbs_feasible(current) or not self._fleet_feasible(current) or proposed < before or
                                          self._simulated_annealing_accept(before, proposed, self.temperature))
                improved = accepted and proposed < before - 1e-9
                gain = max(0.0, before - proposed) if accepted else 0.0
                if accepted:
                    repaired.destroyed_customers_info = {}
                    current = repaired
                    if not self._cbs_feasible(best) or not self._fleet_feasible(best) or proposed < best.bcplus_metrics[objective_key] - 1e-9:
                        best = self._copy(repaired)
                        self._best_objective_for_accept = proposed
                        new_best = True
            else:
                self.stats['skipped'] += 1
                print('修复后方案为空，跳过此次迭代。')
                current = self._copy(best)
            stagnation = 0 if new_best else stagnation + 1
            route_trigger = new_best or (stagnation >= 30 and not route_l1_fired)
            if self.bcplus_flags['route_local_search'] and route_trigger:
                route_l1_fired = True
                local = self._route_local_search(best, objective_key)
                if local is not best:
                    best, current = local, self._copy(local)
                    self._remember_elite(best)
                    self._best_objective_for_accept = best.bcplus_metrics[objective_key]
                    new_best, stagnation = True, 0
            if new_best:
                route_l1_fired = False
                restart_due = 60
                fired.clear()
                self._destroy_scale = 1.0
            self.temperature *= cooling
            if self.bcplus_flags['search_control']:
                current = self._escape(stagnation, fired, current, best, iteration)
            if self.bcplus_flags.get('elite_restart', True) and stagnation >= restart_due:
                current = self._elite_restart(current, iteration)
                restart_due = stagnation + 80
                if self._cbs_feasible(current) and self._fleet_feasible(current) and (
                        not self._cbs_feasible(best) or not self._fleet_feasible(best) or
                        current.bcplus_metrics[objective_key] < best.bcplus_metrics[objective_key] - 1e-9):
                    best = self._copy(current)
                    self._best_objective_for_accept = best.bcplus_metrics[objective_key]
                    new_best, stagnation, restart_due, route_l1_fired = True, 0, 60, False
                    fired.clear()
                    self._destroy_scale = 1.0
            self.stats['ext_ins_active_iters'] += int(self._ext_ins_used)

            elapsed = max(time.perf_counter() - tick, 1e-9)

            route_cbs_calls = self.stats['route_ls_cbs_calls'] - route_cbs_before
            work_seconds = max(1e-5 * (self._effort - effort_before) +
                               .001 * len(self._customers) * max(self.stats['cbs_evaluations'] - cbs_before, int(complete) + route_cbs_calls), 1e-6)
            learning_time = elapsed if getattr(self, 'learning_clock', 'work') == 'wall' else work_seconds
            if self.timing_replay is not None:
                entry = self.timing_replay[iteration - 1]
                if entry['destroy_operator'] != dn or entry['repair_operator'] != rn or entry['strategy'] != strategy:
                    raise ValueError('Timing replay diverged from operator trajectory')
                learning_time = entry['learning_time_s']
            row = dict(iteration=iteration, strategy=strategy, destroy_operator=dn, repair_operator=rn,
                       score=(gain / max(abs(before), 1.0)) / learning_time, accepted=accepted, found_new_best=new_best, found_improvement=improved,
                       current_objective=current.bcplus_metrics[objective_key], new_objective=proposed,
                       best_objective=best.bcplus_metrics[objective_key], temperature=self.temperature,
                       improvement_delta=gain, rate=(gain / max(abs(before), 1.0)) / learning_time,
                       elapsed_s=elapsed, learning_time_s=learning_time, work_seconds=work_seconds, effort=self._effort - effort_before,
                       repair_failed=repair_failed, skipped=not complete, fallback=dict(levels),
                       identity_fix=self.stats['identity_fix'] - identity_before, stagnation=stagnation)
            row.update(cbs_consistent=cbs_valid if complete else None, route_ls_cbs_calls=route_cbs_calls,
                       externality_lambda=getattr(self, '_externality_lambda', 0.0),
                       ext_ins_active=self._ext_ins_used, externality_relative_error_ema=getattr(self, '_externality_relative_error_ema', 1.0),
                       elite_restart=self.stats['elite_restarts'] > restart_before, elite_pool_size=len(self._elite_pool))
            if row['elite_restart']:
                row['elite_restart_details'] = dict(self._last_elite_restart)
            self.iteration_log.append(row)
            segment.append(row)
            self.fallback_counts.update(levels)
            if self.bcplus_flags['externality_learning'] and iteration % 50 == 0:
                self._learn(segment)
                segment = []
                history.append(dict(iteration=iteration, weights=copy.deepcopy(self.operator_weights)))
            elif not self.bcplus_flags['externality_learning']:
                score = 10 if new_best else 5 if improved else 2 if accepted else 0
                row['score'] = score
                for kind, name in [('destroy', dn), ('repair', rn)]:
                    weights = self.operator_weights[strategy][kind]
                    weights[name] = max(.1, .5 * weights[name] + .5 * score)
            self._record(traces, current, repaired if complete and cbs_valid else current, best, objective_key)
            print(f'[bcplus-fallback] iter={iteration} a={levels["a"]} b={levels["b"]} c={levels["c"]} skipped={int(not complete)}')
            if iteration % 10 == 0:
                print(f'[bcplus-identity] iter={iteration} corrected_fields={self.stats["identity_fix"]} hfix={getattr(self, "r12_hfix_events", 0)}')
        elapsed = time.perf_counter() - start
        best.bcplus_validation = validate_drone_identity(best, self.base_drone_assignment)
        best.bcplus_validation['cbs_consistent'] = self._cbs_feasible(best)
        best.bcplus_validation['cbs_errors'] = best.bcplus_metrics.get('cbs_errors', [])
        best.bcplus_validation['idle_trucks'] = self._idle_trucks(best)
        best.bcplus_validation['fleet_feasible'] = self._fleet_feasible(best)
        summary = dict(self.stats, skipped=self.stats['skipped'], fallback=dict(self.fallback_counts),
                       hfix=getattr(self, 'r12_hfix_events', 0), flags=self.bcplus_flags, best_validation=best.bcplus_validation, learning_clock=getattr(self, 'learning_clock', 'work'))
        summary.update(cbs_inconsistent=self.stats['cbs_inconsistent'],
                       externality_lambda=getattr(self, '_externality_lambda', 0.0),
                       externality_relative_error_ema=getattr(self, '_externality_relative_error_ema', 1.0),
                       elite_pool_size=len(self._elite_pool), initial_idle_trucks=initial_idle)
        print(f'[bcplus-summary] skipped={summary["skipped"]} fallback={dict(self.fallback_counts)} identity_fix={self.stats["identity_fix"]} hfix={summary["hfix"]}')
        if self.summary_dir:
            os.makedirs(self.summary_dir, exist_ok=True)
            path = os.path.join(self.summary_dir, self.problemName + f'_HP_{self.iter}_diagnostics.json')
            with open(path, 'w', encoding='utf-8') as handle:
                json.dump(dict(summary=summary, iteration_operator_log=self.iteration_log, operator_history=history), handle, indent=2, default=lambda value: value.item() if isinstance(value, np.generic) else str(value))
        return self._save(best, traces, elapsed, history)

    def _record(self, traces, current, evaluated, best, objective_key):
        m = evaluated.bcplus_metrics
        values = dict(y_best=best.bcplus_metrics[objective_key], win_cost=sum(m['pre_penalty'].values()),
                      uav_route_cost=m['pre_window'] - sum(m['pre_penalty'].values()),
                      vehicle_route_cost=m['pre'] - m['pre_window'], final_uav_cost=sum(evaluated.final_uav_cost.values()),
                      final_total_list=m['post_window'], final_win_cost=sum(m['post_penalty'].values()),
                      final_total_objective=m['post'], final_vehicle_route_cost=m['post'] - m['post_window'],
                      work_time=m['pre_global'], final_work_time=m['post_global'])
        for name, value in values.items():
            traces[name].append(value)
        traces['y_cost'].append(current.bcplus_metrics[objective_key])

    def _save(self, best, traces, elapsed, history):
        m = best.bcplus_metrics
        statistics = {}
        for kind in ('destroy', 'repair'):
            statistics[kind] = {}
            for op in (self.destroy_operators if kind == 'destroy' else self.repair_operators):
                rows = [r for r in self.iteration_log if r[kind + '_operator'] == op.__name__]
                uses, improvements = len(rows), sum(r['found_improvement'] for r in rows)
                score, gain = sum(r['score'] for r in rows), sum(r['improvement_delta'] for r in rows)
                statistics[kind][op.__name__] = dict(total_uses=uses, total_score=score,
                    improvements=improvements, new_best_count=sum(r['found_new_best'] for r in rows),
                    better_current_count=improvements, total_improvement=gain,
                    avg_score=score / max(uses, 1), avg_improvement=gain / max(improvements, 1))
        save_alns_results(instance_name=self.problemName + 'HP' + str(self.iter), **traces,
            strategy_weights=self.strategy_weights, operator_weights=self.operator_weights, elapsed_time=elapsed,
            best_objective=m['pre'], best_vehicle_max_times=m['pre_max'], best_global_max_time=m['pre_global'],
            best_arrive_time=best.vehicle_arrive_time, best_window_total_cost=m['pre_window'],
            best_uav_tw_violation_cost=m['pre_penalty'], best_total_cost_dict=m['pre_costs'], best_state=best,
            best_final_uav_cost=sum(best.final_uav_cost.values()), best_final_objective=m['post'],
            best_final_win_cost=sum(m['post_penalty'].values()), best_total_win_cost=m['post_window'],
            best_final_vehicle_route_cost=m['post'] - m['post_window'], best_final_vehicle_max_times=m['post_max'],
            best_final_global_max_time=m['post_global'], best_final_state=best,
            operator_statistics=statistics, iteration_operator_log=self.iteration_log,
            operator_history=history, base_dir=self.summary_dir)
        return (best, best, m['pre'], m['post'], sum(best.final_uav_cost.values()), sum(m['post_penalty'].values()),
                m['post_window'], m['post_global'], m['pre_global'], m['pre_window'], sum(m['pre_penalty'].values()),
                m['pre'] - m['pre_window'], elapsed, traces['win_cost'], traces['uav_route_cost'],
                traces['vehicle_route_cost'], traces['final_uav_cost'], traces['final_total_list'], traces['final_win_cost'],
                traces['final_total_objective'], traces['y_cost'], traces['y_best'], traces['work_time'], traces['final_work_time'])

def solve_with_bc_alns_plus(initial_solution, node, DEPOT_nodeID, V, T, vehicle, uav_travel, veh_distance, veh_travel, N, N_zero, N_plus, A_total, A_cvtp, A_vtp,
        A_aerial_relay_node, G_air, G_ground, air_matrix, ground_matrix, air_node_types, ground_node_types, A_c, xeee, customer_time_windows_h, early_arrival_cost, late_arrival_cost, problemName,
        iter, max_iterations, max_runtime=60, summary_dir=None, use_incremental=True, seed=None, algo_seed=None):
    solver = IncrementalALNSPlus(node, DEPOT_nodeID, V, T, vehicle, uav_travel, veh_distance, veh_travel, N,
        N_zero, N_plus, A_total, A_cvtp, A_vtp, A_aerial_relay_node, G_air, G_ground, air_matrix, ground_matrix,
        air_node_types, ground_node_types, A_c, xeee, customer_time_windows_h, early_arrival_cost, late_arrival_cost,
        problemName, iter=iter, max_iterations=max_iterations, max_runtime=max_runtime, summary_dir=summary_dir,
        seed=seed, algo_seed=algo_seed)
    return solver.solve(initial_solution)
