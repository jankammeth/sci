# Component reuse benchmark

The three general improvements reduce component-driver overhead without any
fleet-size threshold. The selected small interacting cases improve by 11–24%
against the prototype, but remain 9–24% slower than full-fleet solves. At N=20,
three cases improve by 22–30%; the other two change little. The optimized
components are 3–10 times faster than full-fleet solves on these five N=20 cases.
Union-find is a small fraction of runtime. Newton evaluation and sparse fallback
are the useful next profiling targets.

## Changes

1. Components hold a matching system, vector, endpoint data, and trajectory.
   They no longer construct nested `Config` and `ContactSolver` objects. Contact
   initialization, recovery, and validation use shared numerical functions.
2. Initialization reads only the selected relative coordinates. Reconstruction
   assembles relative cubic coefficients directly. Fleet-indexed public contact
   vectors are exported once at the end. When a single component already has the
   full fleet layout, its trajectory is returned directly; this is a representation
   identity that applies at every N.
3. Unchanged robot/contact groups retain their solution, trajectory, and checks.
   Stable contact identities distinguish retained contacts from newly inserted
   ones, including after recovery reorders contacts. Merges, splits, insertions,
   and removals rebuild affected groups. No component state persists across solves.

Union-find is still rebuilt after every contact-set update. No dependencies,
matching equations, numerical tolerances, or solver selection thresholds were
added. The default remains `full`; no full experiment campaign was rerun.

These measurements evaluate the three changes together, not their isolated
contributions. Stage timings and component reuse counts explain where work was
removed, but are not a three-way ablation.

## Protocol

Measured on 2026-10-17, macOS arm64, Python 3.14.7, NumPy 2.5.3, SciPy 1.18.1,
native collision backend, and numerical-library thread limits of one. The input
settings match the prior benchmarks: minimum distance 0.5, separation allowance
0.001, residual tolerance 1e-7, recovery enabled, and a 30-second deadline.

The selection was fixed before timing: N=2 uses the first free seed and first two
interacting seeds; N=4 uses the first seed at each observed contact count 0–3;
N=20 uses all five previously benchmarked regression cases. Endpoints and
configuration are embedded in the raw report.

`Before` is SCI commit `5e7a8e3c667f70c7230db2024d9525254eb409f2`, archived
before editing and imported as a separate package. `Full` and `After` use the
current sources, whose hashes are recorded. All run in the same process with one
warmup per case/variant, then 21 repetitions for N=2/4 and seven for N=20. Case
and variant order are shuffled deterministically. This gives 546 timed solves.

Detailed Newton profiling is disabled in this comparison. A separate pass uses
three profiled repetitions per N=20 case. Reported times are medians; stage
medians need not sum exactly to the median total. Quartiles are in the CSV.

## Solve times

Milliseconds. `Reduction` compares optimized components against the prototype.
Solve time excludes final output sampling and contact-vector export.

| N | Seed | Final contacts | Full | Components before | Components after | Reduction |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 2 | 0 | 0 | 0.117 | 0.129 | 0.100 | 22.9% |
| 2 | 2 | 1 | 0.825 | 1.212 | 0.925 | 23.7% |
| 2 | 10 | 1 | 0.837 | 1.208 | 0.915 | 24.3% |
| 4 | 0 | 1 | 0.869 | 1.220 | 1.078 | 11.7% |
| 4 | 1 | 0 | 0.128 | 0.136 | 0.106 | 22.4% |
| 4 | 8 | 3 | 3.638 | 4.917 | 4.001 | 18.6% |
| 4 | 40 | 2 | 1.951 | 2.616 | 2.336 | 10.7% |
| 20 | 2 | 3 | 82.280 | 19.562 | 19.436 | 0.6% |
| 20 | 9 | 13 | 1610.075 | 545.170 | 537.159 | 1.5% |
| 20 | 15 | 8 | 101.151 | 24.667 | 17.956 | 27.2% |
| 20 | 35 | 7 | 1101.601 | 169.983 | 118.573 | 30.2% |
| 20 | 47 | 6 | 135.752 | 17.466 | 13.587 | 22.2% |

Seed 2 has overlapping interquartile ranges (before 19.353–20.522 ms,
after 19.160–20.046 ms); its apparent 0.6% gain should be treated as inconclusive.
Seed 9 improves little because most time is inside Newton, not component setup.
The small-case selection is representative of contact counts, not an estimate
over the entire previous 100-case set.

The CSV also records the complete `generate_trajectories` call, including final
contact export and sampled output. Its before/after reduction is 8–17% on the
small interacting cases, and 0.7%, 1.4%, 24.8%, 29.9%, and 19.2% on the five N=20
cases respectively. Thus the improvement remains when moved export work is
included. For N=20 the export itself takes 0.066–0.106 ms. Configuration and cost
evaluation are outside both timers.

For N=2 seed 2, component assembly falls from 0.171 to 0.076 ms and reconstruction
from 0.221 to 0.053 ms. Newton stays at about 0.473 ms in all three variants.
The remaining component/full gap is 0.100 ms. For N=4 seed 40, the gap is 0.386 ms
over two contact updates, with reconstruction contributing 0.290 ms in components
versus 0.069 ms in full mode. The overhead scales with components, knots, and
updates; it is not a constant tax determined only by the fleet size.

## N=20 outer stages

Accumulated milliseconds per optimized component solve, from the unprofiled
seven-repeat comparison. Stages are disjoint.

| Stage | Seed 2 | Seed 9 | Seed 15 | Seed 35 | Seed 47 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Partition: union-find, grouping, sorting | 0.028 | 0.135 | 0.059 | 0.064 | 0.049 |
| Component assembly | 0.604 | 3.122 | 0.878 | 0.980 | 0.903 |
| Newton | 16.189 | 516.226 | 11.606 | 111.093 | 7.882 |
| Reaction recovery | 0.000 | 7.170 | 0.000 | 1.185 | 0.000 |
| Trajectory reconstruction | 0.840 | 3.204 | 1.854 | 1.886 | 1.654 |
| Validation | 0.637 | 2.609 | 1.250 | 1.107 | 1.130 |
| Collision search | 0.562 | 2.705 | 1.224 | 1.187 | 1.020 |
| Other bookkeeping | 0.600 | 2.095 | 1.128 | 1.156 | 0.989 |
| Total solve | 19.436 | 537.159 | 17.956 | 118.573 | 13.587 |

There are respectively 5, 15, 8, 9, and 8 partition calls: approximately
5.6–9.0 microseconds per call. Partitioning is 0.025–0.362% of solve time and
includes more than union-find alone. Graph rebuilding costs
O(m alpha(A)) plus O(A log A) deterministic ordering, for m contacts and A active
robots. It is not constant, but is negligible on these cases.

Actual component solves and reuse events explain the savings:

| Seed | Newton calls before | Newton calls after | Unchanged groups reused | Function evaluations before → after |
| ---: | ---: | ---: | ---: | ---: |
| 2 | 5 | 5 | 0 | 143 → 143 |
| 9 | 32 | 15 | 17 | 658 → 641 |
| 15 | 22 | 8 | 14 | 96 → 82 |
| 35 | 20 | 9 | 11 | 598 → 470 |
| 47 | 17 | 9 | 8 | 78 → 70 |

Recovery evaluations are included in the last column, separately timed above.
The changes also alter floating-point warm starts by avoiding global coordinate
round trips; all numerical comparisons and physical checks pass below. The
seed-35 gain should not be attributed entirely to its 11 skipped initial residual
evaluations: its total evaluation count also falls substantially.

## Inside Newton

Milliseconds accumulated across all Newton calls in each solve, measured in the
separate three-repeat profiling pass. These are subdivisions of Newton, not
additional time to add to the outer table. Recovery has its own timer.

| Newton stage | Seed 2 | Seed 9 | Seed 15 | Seed 35 | Seed 47 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Residual and Jacobian evaluation | 5.723 | 44.742 | 5.557 | 14.398 | 3.791 |
| Band matrix assembly | 0.989 | 17.833 | 0.955 | 3.013 | 0.576 |
| Banded linear solve | 1.088 | 28.689 | 1.155 | 3.352 | 0.789 |
| Sparse assembly and scaling | 0.000 | 12.852 | 0.000 | 3.455 | 0.000 |
| Sparse LSMR solve | 0.000 | 332.378 | 0.000 | 55.412 | 0.000 |
| Line-search residual evaluations | 6.827 | 71.132 | 2.647 | 28.192 | 1.732 |
| Other Newton work | 1.593 | 7.558 | 1.237 | 4.734 | 1.057 |
| Total Newton | 16.273 | 514.929 | 11.608 | 112.604 | 7.986 |
| Total profiled solve | 19.377 | 535.144 | 17.900 | 120.019 | 13.723 |

| Seed | Newton iterations | Residual/Jacobian calls | Line-search residual calls | LSMR calls | LSMR iterations |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 2 | 37 | 42 | 101 | 0 | 0 |
| 9 | 106 | 121 | 482 | 14 | 15,232 |
| 15 | 34 | 42 | 40 | 0 | 0 |
| 35 | 82 | 91 | 369 | 10 | 2,880 |
| 47 | 29 | 38 | 32 | 0 | 0 |

Seed 9 spends 62% of its complete profiled solve in LSMR and another 22% in
residual/Jacobian and line-search evaluation. For seed 35 those shares are 46%
and 35%. The easier cases use no LSMR; evaluation accounts for 40–65% of their
total solve time. Banded linear solves take only 3–6% across these cases.

The next general optimization target is therefore numerical evaluation and the
fallback path. For the hard cases, inspect why fallback needs so many iterations
before changing the linear solver or safeguards. For the easier cases, reducing
repeated residual/Jacobian allocations and polynomial work is more promising
than a more elaborate graph algorithm. These are directions justified by the
profile, not measured speedup claims. The current results do not justify an
N-dependent policy.

## Validation and reproduction

All 546 timed runs and 15 profiled runs converge with valid matching checks and
continuous separation. The largest relative cost difference from paired full
mode is 1.19e-11; the largest position-coordinate difference over 201 times is
6.31e-7. Separate independent polynomial checks pass for all 12 cases in
`full`, `active`, and `components` (36 checks): endpoints, continuity, jerk
balance, contact geometry, reaction signs, and continuous separation.

Profiling on/off produces exactly equal knot times and trajectory coefficients
on all five N=20 cases. The refactored full mode matches the preserved version
exactly across the existing 29-case comparison: identical records and zero
difference in all 174 arrays, including sampled derivatives and failed solves.
The complete test suite passes 172 tests; lint and whitespace checks pass.

From the SCI checkout, prepare an archive of `5e7a8e3` at `SCI_REFERENCE` with
the same native collision extension built for the current Python interpreter.
Then run:

```sh
PYTHONPATH=src python tools/benchmark_component_reuse.py \
  --reference-source "$SCI_REFERENCE"

PYTHONPATH="$SCI_REFERENCE/src" python tools/compare_upstream.py \
  --worker sci_solver --output /tmp/sci-reuse-reference-full.json
PYTHONPATH=src python tools/compare_upstream.py \
  --worker sci_solver --output /tmp/sci-reuse-current-full.json
PYTHONPATH=src python tools/validate_component_reuse.py \
  --full-before /tmp/sci-reuse-reference-full.json \
  --full-after /tmp/sci-reuse-current-full.json
PYTHONPATH=src python -m pytest -q
ruff check .
```

- [Raw runs and source hashes](../validation/component-reuse.json.gz)
- [Per-case timing CSV and quartiles](../validation/component-reuse.csv)
- [Detailed Newton profiling CSV](../validation/component-reuse-profile.csv)
- [Validation records](../validation/component-reuse-validation.json)
