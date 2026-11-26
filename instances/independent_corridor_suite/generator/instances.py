from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import random
from pathlib import Path

import networkx as nx

TICK_HOURS = 3.0 / 3600.0
SCALES = {15: dict(base_nodes=50, relays=10, vtps=24, trucks=2, drones=4),
          30: dict(base_nodes=100, relays=15, vtps=54, trucks=3, drones=6)}
DISTRIBUTIONS = ("uniform", "clustered", "mixed")
TOPOLOGIES = ("mesh", "bottleneck")
CONFIRMATION_REPLICATES = (0, 1, 2)
SEED_BASE = {"pilot": 25090000, "confirmation": 25110000}

def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()

def ticks_up(hours):
    return int(math.ceil(hours / TICK_HOURS - 1e-10))

def _point(rng):
    return [round(rng.uniform(.5, 19.5), 6), round(rng.uniform(.5, 19.5), 6)]

def _customers(n, distribution, rng):
    centers = [[5.+rng.uniform(-1, 1), 5.+rng.uniform(-1, 1)],
               [15.+rng.uniform(-1, 1), 5.+rng.uniform(-1, 1)],
               [10.+rng.uniform(-1, 1), 15.+rng.uniform(-1, 1)]]
    points = []
    for i in range(n):
        clustered = distribution == "clustered" or (distribution == "mixed" and i >= n // 2)
        if clustered:
            center = centers[rng.randrange(3)]
            while True:
                p = [round(rng.gauss(c, 1.2), 6) for c in center]
                if all(.5 <= x <= 19.5 for x in p):
                    break
        else:
            p = _point(rng)
        points.append(p)
    return points, centers

def _nearest_neighbor_cycle(ids, positions):

    todo = set(ids)
    start = min(todo)
    route = [start]
    todo.remove(start)
    while todo:
        nxt = min(todo, key=lambda j: (math.dist(positions[route[-1]], positions[j]), j))
        route.append(nxt)
        todo.remove(nxt)

    changed = True
    while changed:
        changed = False
        for i in range(len(route)-2):
            for j in range(i+2, len(route)):
                a, b = route[i], route[(i+1) % len(route)]
                c, d = route[j], route[(j+1) % len(route)]
                if a == d:
                    continue
                if (math.dist(positions[a], positions[c]) + math.dist(positions[b], positions[d])
                        < math.dist(positions[a], positions[b]) + math.dist(positions[c], positions[d]) - 1e-9):
                    route[i+1:j+1] = reversed(route[i+1:j+1])
                    changed = True

    return [(route[i], route[(i+1) % len(route)]) for i in range(len(route))]

def make_air_graph(positions, topology):
    n = len(positions)

    ordered = sorted(range(n), key=lambda i: (positions[i][0], positions[i][1], i))
    left = set(ordered[:n//2])
    right = set(ordered[n//2:])
    graph = nx.Graph()
    graph.add_nodes_from(range(n))
    for ids in (left, right):
        for u, v in _nearest_neighbor_cycle(ids, positions):
            graph.add_edge(u, v, weight=math.dist(positions[u], positions[v]))
    cross = sorted((math.dist(positions[u], positions[v]), u, v) for u in left for v in right)
    cross_count = 4 if topology == "mesh" else 1

    used_left, used_right = set(), set()
    selected = []
    for distance, u, v in cross:
        if u not in used_left and v not in used_right:
            selected.append((distance, u, v))
            used_left.add(u)
            used_right.add(v)
            if len(selected) == cross_count:
                break
    for distance, u, v in selected:
        graph.add_edge(u, v, weight=distance)
    local = sorted((math.dist(positions[u], positions[v]), u, v)
                   for ids in (left, right) for u in sorted(ids) for v in sorted(ids) if u < v)
    target_edges = 2 * n
    for distance, u, v in local:
        if graph.number_of_edges() == target_edges:
            break
        graph.add_edge(u, v, weight=distance)
    assert graph.number_of_edges() == target_edges
    assert nx.is_connected(graph)
    bridges = list(nx.bridges(graph))
    assert len(bridges) == (1 if topology == "bottleneck" else 0)
    return graph, dict(left_nodes=sorted(left), right_nodes=sorted(right),
                       cross_district_edges=[[u, v] for _, u, v in selected],
                       cut_x_km=.5*(positions[ordered[n//2-1]][0]+positions[ordered[n//2]][0]))

def _statistics(graph, positions, customer_air_ids, split):
    distances = dict(nx.all_pairs_dijkstra_path_length(graph, weight="weight"))
    stretches = [distances[u][v] / math.dist(positions[u], positions[v])
                 for u in graph for v in graph if u < v]
    centrality = nx.edge_betweenness_centrality(graph, weight="weight", normalized=True)
    customer_points = [positions[i][:2] for i in customer_air_ids]
    nearest = [min(math.dist(p, q) for j, q in enumerate(customer_points) if j != i)
               for i, p in enumerate(customer_points)]
    return dict(air_nodes=graph.number_of_nodes(), air_edges=graph.number_of_edges(),
                mean_degree=2.*graph.number_of_edges()/graph.number_of_nodes(),
                total_corridor_length_km=sum(x["weight"] for _, _, x in graph.edges(data=True)),
                mean_edge_length_km=sum(x["weight"] for _, _, x in graph.edges(data=True))/graph.number_of_edges(),
                bridge_count=len(list(nx.bridges(graph))),
                global_edge_connectivity=nx.edge_connectivity(graph),
                cross_district_link_count=len(split["cross_district_edges"]),
                mean_path_stretch=sum(stretches)/len(stretches),
                max_path_stretch=max(stretches),
                maximum_edge_betweenness=max(centrality.values()),
                mean_customer_nearest_neighbor_km=sum(nearest)/len(nearest),
                customer_count=len(customer_air_ids), connected=True)

def construction_options(instance):

    graph = nx.Graph()
    for a in instance["air_arcs"]:
        graph.add_edge(a["source"], a["target"], distance=a["distance_km"], duration=a["duration_ticks"])
    access = {p["ground_node"]: p for p in instance["portals"]}
    result = []
    for c in instance["customers"]:
        distances = nx.single_source_dijkstra_path_length(graph, c["air_node"], weight="distance")
        durations = nx.single_source_dijkstra_path_length(graph, c["air_node"], weight="duration")
        options = []
        for v in instance["vtps"]:
            vp, cp = access[v], access[c["ground_node"]]
            vertical_ticks = sum(p[d]["duration_ticks"] for p in (vp, cp) for d in ("up", "down"))
            options.append(dict(vtp=v, distance_km=2.*distances[vp["air_node"]]+.2,
                                nominal_ticks=2*durations[vp["air_node"]]+vertical_ticks+c["service_ticks"]))
        result.append(dict(customer_id=c["customer_id"],
                           ranked_vtps=sorted(options, key=lambda r: (r["distance_km"], r["vtp"]))))
    return result

def make_instance(customer_count, distribution, topology, replicate, stage="confirmation"):
    cfg = SCALES[customer_count]
    distribution_index = DISTRIBUTIONS.index(distribution)
    seed = SEED_BASE[stage] + customer_count*1000 + distribution_index*100 + replicate
    rng_geometry = random.Random(seed)
    rng_service = random.Random(seed + 1000000)
    customer_xy, centers = _customers(customer_count, distribution, rng_geometry)
    relay_xy = [_point(rng_geometry) for _ in range(cfg["relays"])]
    vtp_xy = [_point(rng_geometry) for _ in range(cfg["vtps"])]
    all_xy = relay_xy + vtp_xy + customer_xy
    assert len({tuple(x) for x in all_xy}) == len(all_xy)
    air_positions = [p + [.05] for p in all_xy]
    n_air = len(air_positions)
    ground_positions = [[10., 0., 0.]] + [p + [0.] for p in vtp_xy + customer_xy]
    air_types = (["Aerial Relay Node"]*cfg["relays"]
                 + ["VTP Takeoff/Landing Node"]*cfg["vtps"]
                 + ["CUSTOMER Takeoff/Landing Node"]*customer_count)
    ground_types = ["DEPOT"] + ["VTP"]*cfg["vtps"] + ["CUSTOMER"]*customer_count
    graph, district_split = make_air_graph(air_positions, topology)
    air_arcs = []
    for u, v, attrs in sorted(graph.edges(data=True)):
        d = float(attrs["weight"])
        for source, target in ((u, v), (v, u)):
            air_arcs.append(dict(source=source, target=target, distance_km=d,
                                 duration_ticks=ticks_up(d/50.)))
    portals = []
    for ground_local in range(1, len(ground_positions)):
        ground = n_air + ground_local
        air = cfg["relays"] + ground_local - 1
        portals.append(dict(ground_node=ground, air_node=air,
                            up=dict(source=ground, target=air, distance_km=.05,
                                    duration_ticks=ticks_up(.05/36.)),
                            down=dict(source=air, target=ground, distance_km=.05,
                                      duration_ticks=ticks_up(.05/18.))))
    ground_arcs = []
    for u in range(cfg["vtps"]+1):
        for v in range(cfg["vtps"]+1):
            if u == v:
                continue
            d = math.dist(ground_positions[u], ground_positions[v])
            ground_arcs.append(dict(source=n_air+u, target=n_air+v, distance_km=d,
                                    duration_ticks=ticks_up(d/30.)))
    customers, mapping = [], []
    for i in range(customer_count):
        air = cfg["relays"] + cfg["vtps"] + i
        ground_local = 1 + cfg["vtps"] + i
        start = rng_service.uniform(0., 4.)
        end = start + rng_service.uniform(1., 3.)
        window_start, window_end = ticks_up(start), int(math.floor(end/TICK_HOURS + 1e-10))
        customer = dict(customer_id=i+1, air_node=air, ground_node=n_air+ground_local,
                        payload_lbs=rng_service.randint(1, 5), window_start=window_start,
                        window_end=window_end, service_ticks=100,
                        source_window_hours=dict(cust=i+1, ready_h=window_start*TICK_HOURS,
                                                 due_h=window_end*TICK_HOURS, service_h=5./60.))
        customers.append(customer)
        mapping.append(dict(source_customer_id=i+1, air_node=air,
                            ground_local_node=ground_local, ground_node=n_air+ground_local))
    drones = [dict(drone_id=11+d, capacity_lbs=5, endurance_ticks=3600,
                   launch_handling_ticks=20, recovery_handling_ticks=10) for d in range(cfg["drones"])]
    trucks = [dict(truck_id=t+1, capacity_drones=10, initial_drone_ids=[11+2*t, 12+2*t])
              for t in range(cfg["trucks"])]
    meta = [dict(node=i, type=air_types[i], capacity=1, position_km=p)
            for i, p in enumerate(air_positions)]
    for i, p in enumerate(ground_positions):
        row = dict(node=n_air+i, type=ground_types[i],
                   capacity="unlimited" if ground_types[i] in {"VTP", "DEPOT"} else 1,
                   position_km=p)
        if i:
            row["air_portal"] = cfg["relays"]+i-1
        meta.append(row)
    layout_id = f"R25_{stage}_C{customer_count}_{distribution}_g{replicate}"
    instance = dict(schema_version="r29.instance.v1", instance_id=f"{layout_id}_{topology}",
                    layout_id=layout_id, stage=stage, distribution=distribution, topology=topology,
                    scale=cfg["base_nodes"], customer_count=customer_count,
                    topology_seed=seed, geometry_seed=seed, replication_index=replicate,
                    tick_hours=TICK_HOURS, epsilon_ticks=1, horizon_tick=28800, depot=n_air,
                    vtps=list(range(n_air+1, n_air+cfg["vtps"]+1)),
                    air_arcs=air_arcs, portals=portals, ground_arcs=ground_arcs,
                    wait_nodes=list(range(cfg["relays"])), customers=customers, trucks=trucks, drones=drones,
                    costs=dict(truck_per_km=2., drone_per_km=1.5, early_per_hour=5., late_per_hour=20.),
                    physical_settings=dict(endurance_hours=3., horizon_hours=24.,
                                           truck_speed_kmh=30., drone_speed_kmh=50.,
                                           vertical_ascent_kmh=36., vertical_descent_kmh=18.,
                                           payload_capacity_lbs=5, launch_seconds=60,
                                           recovery_seconds=30, service_seconds=300,
                                           safety_seconds=3, clock_seconds=3),
                    node_metadata=meta, source_customer_mapping=mapping, district_split=district_split,
                    source_statistics=_statistics(graph, air_positions, [c["air_node"] for c in customers], district_split),
                    legacy=dict(num_points=cfg["base_nodes"], split=[cfg["relays"], cfg["vtps"], customer_count],
                                air_positions=air_positions, air_types=air_types,
                                ground_positions=ground_positions, ground_types=ground_types),
                    provenance=dict(data_family="independent_synthetic_R25", source="procedural_generation",
                                    external_benchmark_files_used=[], source_coordinates_reused=False,
                                    geometry_rng="python.random.Random", geometry_seed=seed,
                                    service_seed=seed+1000000, domain_km=[0., 20., 0., 20.],
                                    clustered_centers_km=centers, cluster_standard_deviation_km=1.2,
                                    topology_design="two bridge-free districts, equal 2N edges; 4 distinct-endpoint links versus 1 bridge",
                                    generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()))
    instance["construction_options"] = construction_options(instance)
    validate_instance(instance)
    return instance

def load_instance(path):
    return json.loads(Path(path).read_text())

def load_legacy_tuple(path):

    import numpy as np
    import pandas as pd
    instance = load_instance(path) if not isinstance(path, dict) else path
    legacy = instance["legacy"]
    air_positions = {i: tuple(p) for i, p in enumerate(legacy["air_positions"])}
    ground_positions = {i: tuple(p) for i, p in enumerate(legacy["ground_positions"])}
    air_types = dict(enumerate(legacy["air_types"]))
    ground_types = dict(enumerate(legacy["ground_types"]))
    air, ground = nx.Graph(), nx.Graph()
    for i, pos in air_positions.items():
        air.add_node(i, pos=pos, type=air_types[i])
    for arc in instance["air_arcs"]:
        air.add_edge(arc["source"], arc["target"], weight=arc["distance_km"])
    for i, pos in ground_positions.items():
        ground.add_node(i, pos=pos, type=ground_types[i])
    for u in ground_positions:
        for v in ground_positions:
            if u < v:
                ground.add_edge(u, v, weight=math.dist(ground_positions[u], ground_positions[v]))
    customers_ground = {c["ground_node"]-len(air_positions): c for c in instance["customers"]}
    rows = []
    for i, pos in ground_positions.items():
        customer = customers_ground.get(i)
        window = customer["source_window_hours"] if customer else dict(ready_h=0., due_h=24., service_h=0.)
        rows.append(dict(CUST=customer["customer_id"] if customer else 100000+i,
                         **{"XCOORD.":pos[0], "YCOORD.":pos[1]}, Z_COORD=pos[2],
                         DEMAND=customer["payload_lbs"] if customer else 0,
                         READY=60.*window["ready_h"], DUE=60.*window["due_h"],
                         SERVICE=60.*window["service_h"], NODE_TYPE=ground_types[i]))
    windows = {c["air_node"]: dict(c["source_window_hours"]) for c in instance["customers"]}
    return (air, ground, nx.to_numpy_array(air, nodelist=list(air_positions), weight="weight"), air_positions,
            nx.to_numpy_array(ground, nodelist=list(ground_positions), weight="weight"), ground_positions,
            pd.DataFrame(rows), air_types, ground_types, windows)

def validate_instance(instance):
    assert instance["provenance"]["external_benchmark_files_used"] == []
    customers = instance["customers"]
    assert len(customers) == instance["customer_count"]
    assert [c["customer_id"] for c in customers] == list(range(1, len(customers)+1))
    assert len({c["ground_node"] for c in customers}) == len(customers)
    assert all(1 <= c["payload_lbs"] <= 5 and c["window_start"] < c["window_end"] for c in customers)
    graph = nx.Graph()
    for arc in instance["air_arcs"]:
        graph.add_edge(arc["source"], arc["target"], weight=arc["distance_km"], duration=arc["duration_ticks"])
    assert sorted(graph) == list(range(len(instance["legacy"]["air_positions"])))
    assert nx.is_connected(graph)
    assert len(list(nx.bridges(graph))) == (1 if instance["topology"] == "bottleneck" else 0)
    portal_by_ground = {p["ground_node"]: p for p in instance["portals"]}
    feasible_options = {}
    for c in customers:
        lengths = nx.single_source_dijkstra_path_length(graph, c["air_node"], weight="duration")
        options = []
        for v in instance["vtps"]:
            vp, cp = portal_by_ground[v], portal_by_ground[c["ground_node"]]
            duration = 2*lengths[vp["air_node"]] + c["service_ticks"] + sum(
                p[d]["duration_ticks"] for p in (vp, cp) for d in ("up", "down"))
            if duration <= min(d["endurance_ticks"] for d in instance["drones"]):
                options.append(v)
        assert options, f"Customer {c['customer_id']} lacks a nominal endurance-feasible VTP"
        feasible_options[c["customer_id"]] = len(options)
    return dict(customer_count=len(customers), source_coverage_complete=True,
                payload_feasible=True, connected=True, nominal_endurance_feasible=True,
                min_feasible_vtps=min(feasible_options.values()),
                max_feasible_vtps=max(feasible_options.values()),
                full_joint_schedule_feasibility="requires solver and independent final-state validation")

def generate_suite(out, stage):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    records = []
    for customer_count in SCALES:
        for distribution in DISTRIBUTIONS:
            for replicate in (CONFIRMATION_REPLICATES if stage == "confirmation" else (0,)):
                pair = []
                for topology in TOPOLOGIES:
                    instance = make_instance(customer_count, distribution, topology, replicate, stage)
                    path = out / f"{instance['instance_id']}.json"
                    payload = json.dumps(instance, indent=2, sort_keys=True, allow_nan=False) + "\n"
                    if path.exists() and path.read_text() != payload:
                        raise FileExistsError(f"Refusing to overwrite frozen instance: {path}")
                    path.write_text(payload)
                    records.append(dict(instance_id=instance["instance_id"], layout_id=instance["layout_id"],
                                        distribution=distribution, topology=topology,
                                        customer_count=customer_count, base_nodes=instance["scale"],
                                        geometry_seed=instance["geometry_seed"], replicate=replicate,
                                        path=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                                        content_fingerprint=fingerprint(instance),
                                        validation=validate_instance(instance),
                                        statistics=instance["source_statistics"]))
                    pair.append(instance)
                for key in ("customers", "trucks", "drones", "portals", "ground_arcs", "legacy"):
                    assert pair[0][key] == pair[1][key], key
                assert len(pair[0]["air_arcs"]) == len(pair[1]["air_arcs"])
    manifest = dict(schema_version="r25.suite_manifest.v1", stage=stage,
                    independent_layout_count=len(records)//2, instance_count=len(records),
                    planned_algorithm_repetitions=5, planned_iterations=500,
                    algorithms=["HP_ALNS", "H_ALNS", "T_ALNS", "T_I_ALNS", "TA_I_ALNS", "A_I_ALNS", "DA_I_ALNS"],
                    source="independent procedural generation; no Solomon or Homberger coordinates/windows",
                    paired_factor="corridor topology within layout",
                    independent_unit="layout_id; sizes and customer distributions use distinct geometry seeds",
                    generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), instances=records)
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    with (out / "instance_summary.csv").open("w", newline="") as stream:
        rows = [{**{k:v for k, v in r.items() if k not in {"validation", "statistics"}}, **r["statistics"]} for r in records]
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return manifest

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--stage", choices=tuple(SEED_BASE), required=True)
    args = parser.parse_args()
    result = generate_suite(args.out, args.stage)
    print(json.dumps({k:v for k, v in result.items() if k != "instances"}, indent=2))
