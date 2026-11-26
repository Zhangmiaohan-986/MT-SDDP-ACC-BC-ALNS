from __future__ import annotations

import argparse
import hashlib
import json
import sys
from collections import defaultdict
from pathlib import Path

import networkx as nx

from instances import fingerprint, load_instance, load_legacy_tuple, make_instance, validate_instance

def validate_suite(folder, r29_code=None, schedule=False):
    folder = Path(folder)
    manifest = load_instance(folder / "manifest.json")
    if r29_code is not None:
        sys.path.insert(0, str(Path(r29_code).resolve()))
        from r29_generate import construct_candidate
        from r29_model import build_master
        from r28_validate_master import inspect_payload_certificate
        from r29_evaluate import evaluate_candidate, Config, Budget
    reports, layouts = [], defaultdict(list)
    source_seeds, coordinate_hashes = {}, {}
    for record in manifest["instances"]:
        path = folder / record["path"]
        instance = load_instance(path)
        assert hashlib.sha256(path.read_bytes()).hexdigest() == record["sha256"]
        assert fingerprint(instance) == record["content_fingerprint"]
        regenerated = make_instance(record["customer_count"], record["distribution"], record["topology"],
                                    record["replicate"], manifest["stage"])
        assert fingerprint(regenerated) == fingerprint(instance), "Regeneration differs from frozen input"
        report = dict(instance_id=instance["instance_id"], file_sha256=record["sha256"],
                      generation=validate_instance(instance), byte_hash_verified=True,
                      regeneration_exact=True)
        air, ground, aa, apos, ga, gpos, rows, atypes, gtypes, windows = load_legacy_tuple(instance)
        assert len(apos) == len(air) == len(aa) == len(atypes)
        assert len(gpos) == len(ground) == len(ga) == len(gtypes)
        assert nx.is_connected(air) and nx.is_connected(ground)
        assert sorted(apos) == list(range(len(apos)))
        assert sorted(gpos) == list(range(len(gpos)))
        for c in instance["customers"]:
            local_ground = c["ground_node"]-len(apos)
            assert apos[c["air_node"]][:2] == gpos[local_ground][:2]
            assert atypes[c["air_node"]] == "CUSTOMER Takeoff/Landing Node"
            assert gtypes[local_ground] == "CUSTOMER"
            assert windows[c["air_node"]]["cust"] == c["customer_id"]
        report["legacy_mapping_exact"] = True
        lid = instance["layout_id"]
        seed = instance["geometry_seed"]
        chash = fingerprint(instance["legacy"]["air_positions"])
        assert source_seeds.get(seed, lid) == lid, "Independent layouts share a seed"
        assert coordinate_hashes.get(chash, lid) == lid, "Independent layouts reuse geometry"
        source_seeds[seed] = lid
        coordinate_hashes[chash] = lid
        layouts[lid].append(instance)
        if r29_code is not None:
            candidate = construct_candidate(instance, variant=0, seed=0)
            master = build_master(instance, candidate)
            coverage = inspect_payload_certificate(master)
            assert coverage["state"] == "valid", coverage
            assert set(coverage["expected"]) == set(c["customer_id"] for c in instance["customers"])
            report["static_master"] = dict(coverage="valid", missions=len(master["missions"]),
                                            complete_source_customer_ids=True)
            if schedule:
                rec, _, _ = evaluate_candidate(instance, candidate, Config(),
                                               Budget(timeout_s=30., per_batch_timeout_s=3.))
                report["coupled_schedule_smoke"] = dict(status=rec["status"], reason=rec["reason"],
                                                        validator=rec["validator"],
                                                        wall_time_s=rec["wall_time_s"])

        reports.append(report)
    for lid, pair in layouts.items():
        assert len(pair) == 2 and {x["topology"] for x in pair} == {"mesh", "bottleneck"}
        for key in ("customers", "trucks", "drones", "portals", "ground_arcs", "legacy"):
            assert pair[0][key] == pair[1][key], (lid, key)
        assert len(pair[0]["air_arcs"]) == len(pair[1]["air_arcs"])
    result = dict(schema_version="r25.generator_validation.v1", stage=manifest["stage"],
                  instance_count=len(reports), independent_layout_count=len(layouts),
                  integrity_passed=True, paired_topology_invariants_passed=True,
                  independent_seed_and_geometry_checks_passed=True, records=reports)
    (folder / "generation_validation.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    return result

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("folder", type=Path)
    parser.add_argument("--r29-code", type=Path)
    parser.add_argument("--schedule-smoke", action="store_true")
    args = parser.parse_args()
    result = validate_suite(args.folder, args.r29_code, args.schedule_smoke)
    print(json.dumps({k:v for k,v in result.items() if k != "records"}, indent=2))
