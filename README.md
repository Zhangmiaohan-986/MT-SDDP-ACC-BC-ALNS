# MT-SDDP-ACC: BC-ALNS and Benchmark Suite

Source code and benchmark instances for:

**Safe Truck–Drone Delivery in Urban Air Corridors: Joint Task Assignment and Conflict-Aware Scheduling**

## Contents

- `instances/`: Benchmark input datasets and instance generators.
  - `solomon_benchmark/`: Multi-scale Solomon-derived ground–air instance generator (`generate_test_problems.py`), raw Solomon coordinate files (`map_test/`), and fleet configuration tables (`Problems/`).
  - `independent_corridor_suite/`: Deterministic generator (`generator/instances.py`, `generator/validate_suite.py`, `generator/SPEC.md`) and 36 input JSON instances spanning uniform, clustered, and mixed customer distributions under paired mesh and bottleneck corridor topologies.
  - `small_scale_bounds/`: Nine small-scale input JSON instances (`S1–S9`, 4–8 customers).
- `src/`: Core simulation and solver implementation for BC-ALNS.
  - `bc_alns.py`: Upper-level BC-ALNS solver with two-scale destroy/repair neighborhoods, drone-identity preservation, and coupled post-CBS acceptance.
  - `improved_operators.py`, `destroy_repair_operator.py`, `fast_alns_solver.py`, `local_search.py`: Two-scale destroy/repair operators (`RAND`, `WORST`, `REL`, `EXT` × `GRD`, `REG`, `NOISE`, `CoP`) and adaptive search utilities.
  - `cbs_plan.py`, `cbs_candidate.py`, `call_function.py`: Lower-level rolling-horizon Space–Time CBS and Monetary ST-A\* aerial scheduler.
  - `main.py`, `solve_mfstsp_heuristic.py`, `initialize.py`, `create_vehicle_route.py`, `generate_test_problems.py`: Simulation entry points, initialization, and truck–VTP route construction.
  - `constraint_validator.py`, `constraints_satisfied.py`, `drone_plan_constraint_checker.py`, `cost_y.py`, `endurance_calculator.py`, `distance_functions.py`, `task_data.py`: Feasibility verification, cost evaluation, and data structures.

## Requirements

Python 3.9 or later. Install dependencies with:

```bash
pip install -r requirements.txt
```

## Quick Start

Verify core module imports:

```bash
PYTHONPATH=src python3 -c "import bc_alns, cbs_plan, cbs_candidate, improved_operators; print('OK')"
```

Validate the independent corridor benchmark suite:

```bash
python3 instances/independent_corridor_suite/generator/validate_suite.py instances/independent_corridor_suite/data
```

## License

Released for academic research and reproducibility.
