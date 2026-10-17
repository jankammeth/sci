# Fleet partition experiment

Recommendation: use connected components as the candidate for the next solver
default. Rebuilding the graph is cheap enough here that incremental union-find
or cached component state is unnecessary. Keep `full` and `active` selectable
for comparison; this change leaves `full` as the default.

## Implementations

- `full`: the existing fleet-wide Hermite system, with stage timers added.
- `active`: one Hermite system containing only robots in the current contact set.
- `components`: one Hermite system per connected component of the contact graph.

The two reductions share a driver and reuse the existing matching equations,
banded Newton, sparse fallback, and reaction recovery. They do not add dependencies.
Each component has its own unconstrained endpoint barycenter. Robots outside the
contact graph follow their unconstrained endpoint cubics. Every iteration checks
continuous separation across the full fleet, including free robots and pairs
belonging to different components.

Union-find uses path compression and union by size. It is rebuilt after every
insertion/removal, so deleting a bridge naturally splits a component. Graph work
is O(m alpha(A)); deterministic ordering adds O(A log A), where m is the number
of contacts and A is the number of participating robots. Union-find storage is
O(A), plus the O(m) contact-index output. All current components are reassembled
and checked after each change; there is no cache of unchanged components.

Component polynomials are translated onto the union of their knot sequences,
without refitting cubics across short intervals. Independent contacts may occur
simultaneously or change their global order. If a later contact merges groups
with coincident or insufficiently spaced existing knots, the current Hermite
representation cannot handle that merge: it returns `merged_component_knot_gap`
and preserves the last iterate. No solved contact times are perturbed.

## Protocol and results

Measured on 2026-10-17, macOS arm64, Python 3.14.7, NumPy 2.5.3, SciPy 1.18.1,
with the native collision checker and numerical-library thread limits set to one.
The inputs are the five existing `tests/data/fleet20_seed*.npz` regression cases.
Each uses its saved endpoints/velocities/horizon, minimum distance 0.5,
separation allowance 0.001, matching tolerance 1e-7, and a 30-second deadline.
Reaction recovery is enabled in every mode.

One warmup per case/mode is followed by seven measured repetitions. Case and
mode order are shuffled deterministically within each repetition. Runs execute
sequentially in one process. The table reports median solve time, excluding
final output sampling. Raw data also records setup and complete wall time.
The workstation showed timing variation; the CSV records interquartile ranges.

| Seed | Full (ms) | Active (ms) | Components (ms) | Full / components | Final equations: full / active / components |
| --- | ---: | ---: | ---: | ---: | ---: |
| 2 | 79.72 | 19.71 | 19.90 | 4.01 | 234 / 42 / 42 |
| 9 | 1584.31 | 768.25 | 568.19 | 2.79 | 1014 / 598 / 598 |
| 15 | 96.91 | 50.52 | 24.43 | 3.97 | 624 / 304 / 144 |
| 35 | 1249.74 | 303.48 | 183.50 | 6.81 | 546 / 294 / 90 |
| 47 | 134.51 | 73.63 | 17.67 | 7.61 | 468 / 180 / 92 |

All 105 measured runs succeeded. Contact counts agree across modes. The largest
relative cost difference against the paired full-fleet run was 1.19e-11;
the largest coordinate difference over 201 comparison times was 6.31e-7.
Sampling is only a comparison diagnostic: acceptance still uses continuous
collision search. Tests separately check polynomial separation, endpoint states,
continuity, jerk balance, and contact geometry on the reconstructed full fleet.
The full suite passes 169 tests. A separate comparison of the instrumented full
mode against the pinned original solver matches all 29 cases, with zero maximum
array difference; [baseline parity](../validation/partition-baseline-parity.json)
records source hashes and environments.

Summing the five per-case median times gives 3145.19 ms for full, 1215.59 ms for
active, and 813.70 ms for components: 3.87 times faster for components than full,
and 1.49 times faster than active. These are five selected regression cases,
several deliberately stressing recovery, rather than a representative fleet-size
sweep or evidence of a universal speedup.

## Where time goes

Median accumulated stage times for the component mode, in milliseconds:

| Seed | Partition, including UFDS | Assembly | Newton | Recovery | Reconstruction | Validation | Collision search |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 2 | 0.029 | 1.103 | 16.014 | 0 | 1.124 | 0.616 | 0.526 |
| 9 | 0.144 | 8.406 | 535.980 | 7.634 | 5.556 | 4.097 | 2.748 |
| 15 | 0.057 | 4.065 | 12.522 | 0 | 2.991 | 2.360 | 1.282 |
| 35 | 0.068 | 4.058 | 169.453 | 1.334 | 3.112 | 2.038 | 1.250 |
| 47 | 0.052 | 3.326 | 8.364 | 0 | 2.408 | 1.732 | 1.010 |

Stages are disjoint within a run. Medians across runs need not add to the median
total. Recovery includes its own assembly/reconstruction/checks; assembly
includes local warm-start trajectories. `other` accounts for untimed bookkeeping.
The partition stage includes union-find, grouping contact indices, and sorting,
so its measured cost is an upper bound on union-find alone. It runs 5–15 times
per solve and costs 0.029–0.144 ms in total: under 0.3% of each median solve time.

The dominant improvement is in Newton solves. For seeds 15 and 47, components
also avoid recoveries used by the full and active modes. This comparison measures
complete solver behavior, including changes in nonlinear convergence, rather
than just matrix-factorization cost. Seed 2 has one interacting group, making
active and components effectively tied. Seed 9 ends with one component but still
benefits from smaller independent systems earlier in insertion.

The practical next choice is the component mode with simple graph rebuilding.
Further optimization, if warranted on a broader suite, should target repeated
local assembly/reconstruction or difficult Newton solves before union-find.

## Reproduction and evidence

From the SCI repository, with the native kernel built:

```sh
PYTHONPATH=src python tools/benchmark_partitions.py \
  --repeats 7 --warmups 1 --output validation/fleet-partitions.json
python -m pytest
ruff check .
```

The benchmark script sets thread limits before importing numerical libraries.
[Raw JSON](../validation/fleet-partitions.json) contains every run, all root
records, component histories, stage counts/timings, source/input hashes, and the
environment. [Summary CSV](../validation/fleet-partitions.csv) contains medians,
quartiles, costs, and stage breakdowns for all three modes. The recorded Git
commit is the pre-change base; source hashes identify the implementation tested
in the working tree.

Select the experimental mode using
`SolverConfig(sci_fleet_mode="components")`. See [the API guide](api.md) for
timing semantics and low-level component diagnostics.
