# Independent R2-5 instance definition

The confirmation design contains 36 frozen instances: three customer distributions × two corridor networks × two customer counts × three independent layout realizations. The two networks share each layout, giving 18 independent layouts. Five algorithm repetitions measure search variability within an instance. Instance layouts, not the 180 run observations per algorithm, are the independent experimental units.

## Geometry and source independence

- Every coordinate, customer window and parcel weight is generated procedurally. No Solomon, Homberger or prior study coordinate/window file is read.
- All layouts occupy a 20 × 20 km square. Coordinates are sampled inside [0.5,19.5] km; the depot is (10,0) km. Aerial altitude is 0.05 km.
- Uniform customers are sampled uniformly over the domain. Clustered customers use a three-component Gaussian mixture with equally likely centers around (5,5), (15,5), (10,15) km; each center is independently perturbed by up to 1 km per axis, and each component has 1.2 km standard deviation. Samples outside the domain are redrawn. Mixed layouts use floor(C/2) uniform customers and the remaining clustered customers.
- VTPs and aerial relays are independently uniform. The customer, VTP and relay coordinates are shared exactly within the mesh/bottleneck pair.
- C15 has 10 relays, 24 VTPs, 15 customers and one depot (50 base locations), two trucks and four drones. C30 has 15 relays, 54 VTPs, 30 customers and one depot (100 base locations), three trucks and six drones. Each truck initially carries two drones. The number of available positions on a truck is 10.
- Airside/ground-side copies are explicit graph nodes. C15 therefore has 49 air nodes and 40 ground nodes; C30 has 99 air nodes and 85 ground nodes. The base-location count is not described as the total graph-node count.

## Controlled network factor

Sort all air nodes by x coordinate and split at the median into two geographic districts. Each district receives a deterministic nearest-neighbor cycle improved by geometric 2-opt. This creates a connected local structure with no bridge. Short intra-district edges complete the common edge budget. Mesh networks have four cross-district connections with distinct endpoints; bottleneck networks have one cross-district connection. Both have exactly 2N undirected air edges (98 for C15; 198 for C30), mean degree four, and identical nodes.

The mesh has no bridge, whereas the bottleneck has exactly one bridge. The design holds edge count and customer demand constant. Total corridor length and shortest-path stretch are measured, because changing connectivity also changes path geometry; they are reported covariates rather than assumed equal. All corridor edges are represented by two directed arcs. Opposite directions share the physical corridor resource in the common evaluator.

## Time, costs and identity

Customer window starts are independent U(0,4) hours and widths are U(1,3) hours. Bounds are rounded inward to the 3-second clock. Service takes five minutes. Parcels are integer 1–5 lb with drone capacity 5 lb. The planning horizon is 24 hours, mission endurance is three hours, truck speed is 30 km/h, drone cruise is 50 km/h, ascent is 36 km/h and descent is 18 km/h. Launch/recovery handling takes 60/30 seconds. The safety separation and physical clock are three seconds.

Costs are CNY 2 per truck kilometre, CNY 1.5 per drone kilometre, CNY 5 per early hour and CNY 20 per late hour. These settings are common to all methods and to initial and final evaluation. The full source customer ID set is 1..C. JSON records explicit source ID, air node ID and merged ground ID mappings; final coverage must equal this source set exactly, with one mission per customer.

The fleets are fixed active fleets: every truck must participate in at least one mission launch or recovery. The common candidate precheck enforces this restriction for every algorithm; BC-ALNS+ also enables `forbid_idle_trucks=True` within its search. Every truck and drone remains in the candidate inventory. There is no fixed vehicle-use cost. This requirement is recorded in `data/confirmation/protocol.json`; the instance geometry, demand and byte hashes are unchanged.

## Seeds and freezing

The geometry seed equals stage base + 1000C + 100 × distribution index + layout replicate. Distribution indices are uniform=0, clustered=1, mixed=2. Stage bases are 25,090,000 for pilot and 25,110,000 for confirmation. Service/window/payload randomness uses geometry seed + 1,000,000. Confirmation replicates are 0,1,2; pilot uses replicate 0. Each distribution and scale has distinct seeds. Network topology is a deterministic transformation of the layout.

`manifest.json` stores SHA-256 for every JSON file and the generator source. The generator refuses to overwrite different content at a frozen instance path. `validate_suite.py` checks byte hashes, exact regeneration, paired invariants, unique independent geometry, source coverage, payload, nominal endurance and the legacy/R29 mapping. Optional coupled schedule smoke checks are recorded without removing failed instances. Formal run files must cite the input hash, algorithm revision, seed and actual status.

## Interface

`load_instance(path)` returns an `r29.instance.v1` object with physical settings and `construction_options`. `load_legacy_tuple(path)` returns the ten values expected by `main.generate_complex_network`. The execution adapter must override the legacy random parcel weights, cruise speed and cost defaults using the same JSON. Initial-solution caches must be isolated by instance content hash; the old C/T/U/N filename alone is not an adequate cache key.

Generation and validation commands, from the project root:

```sh
python R25_study_20261115/generator/instances.py --stage confirmation --out R25_study_20261115/data/confirmation
python R25_study_20261115/generator/validate_suite.py R25_study_20261115/data/confirmation --r29-code VDAC_r29_study
```

The old `data/pilot/` folder predates the addition of construction options. `data/pilot_v2/` is the current pilot interface. Both are development data and are excluded from confirmatory algorithm comparisons.
