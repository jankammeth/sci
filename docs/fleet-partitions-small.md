# Fleet reductions at N=2 and N=4

The original component prototype was slower than the full-fleet solver for
these small cases. The subsequent [component reuse benchmark](component-reuse.md)
measures a general optimization of that implementation. The results below are
historical measurements; they do not prescribe fleet-size dispatch. No solver
code, defaults, experiment campaigns, or manuscript data were changed for this
original comparison.

## Inputs and protocol

All 50 saved fleet-study seeds for N=2 and all 50 for N=4 are included, without
filtering by outcome. The exact endpoints, velocities, horizons, minimum distance,
and separation allowance are copied into the self-contained
[input snapshot](../validation/fleet-small-inputs.json). It records the original
table, input, and configuration hashes. The selected settings are minimum
distance 0.5 and separation allowance 0.001; matching uses the current defaults
including reaction recovery and a 30-second deadline.

Each case/mode receives one warmup and 21 timed repetitions. Case and mode order
are deterministically shuffled within every repetition; execution is sequential
in one process. The native collision backend is used, with numerical-library
thread limits set to one. Environment: macOS arm64, Python 3.14.7, NumPy 2.5.3,
SciPy 1.18.1. The solver implementation is unchanged from the N=20 comparison.

Reported times are the median across cases of each case's median over 21 runs.
They include planning, reconstruction within planning, and collision search,
and exclude final output sampling. Setup and complete wall time are also logged.
Input loading is outside the solve timer. Contact-free and interacting cases are
reported separately because 74 of the 100 cases need no matching equations.

## Results

Times in milliseconds:

| N | Cases | Count | Full | Active | Components |
| --- | --- | ---: | ---: | ---: | ---: |
| 2 | Contact-free | 41 | 0.115 | 0.121 | 0.122 |
| 2 | Interacting | 9 | 0.818 | 1.196 | 1.189 |
| 4 | Contact-free | 33 | 0.121 | 0.129 | 0.129 |
| 4 | Interacting | 17 | 0.873 | 1.225 | 1.214 |

On the interacting cases, the median paired slowdown for components is 45.5%
at N=2 and 39.9% at N=4. Components are slower on every interacting case.
Contact-free cases show a smaller median paired slowdown of 5.4% and 6.7%.
Summing all 50 per-case median times gives:

| N | Full sum (ms) | Active sum (ms) | Components sum (ms) |
| --- | ---: | ---: | ---: |
| 2 | 12.095 | 15.632 | 15.596 |
| 4 | 23.054 | 30.556 | 30.765 |

Thus the component implementation takes 28.9% and 33.4% more total time across
the two respective 50-case sets. This is a ratio of summed per-case medians,
distinct from the median paired slowdown above. All 6,300 timed solves succeeded.
The largest relative cost difference from the paired full solve is 4.27e-15;
the largest coordinate difference over 201 comparison times is 1.21e-7.

## Why it is slower

Median accumulated stages over the interacting cases, in microseconds:

| N | Mode | Partition including UFDS | Assembly | Newton | Reconstruction | Validation | Collision search |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 2 | Full | 0 | 69.5 | 468.6 | 41.5 | 111.3 | 77.0 |
| 2 | Active | 1.9 | 170.4 | 472.0 | 208.1 | 145.4 | 77.7 |
| 2 | Components | 4.0 | 170.0 | 470.9 | 208.1 | 145.0 | 75.8 |
| 4 | Full | 0 | 77.2 | 489.5 | 41.6 | 117.1 | 87.4 |
| 4 | Active | 1.9 | 174.0 | 470.7 | 215.0 | 152.0 | 84.8 |
| 4 | Components | 4.3 | 173.4 | 468.0 | 213.0 | 151.5 | 86.4 |

Recovery was unused. Other bookkeeping is separately logged, and medians need
not add to the median total. Partition timing includes union-find, grouping,
and sorting, so it bounds union-find alone from above. It is zero on all
contact-free runs and about four microseconds per interacting solve.

With N=2 there are no variables to eliminate once the two robots interact.
At N=4 the smaller Newton systems save little time. In both sizes, the extra
local system assembly, trajectory reconstruction, and bookkeeping outweigh that
saving. Graph construction contributes only about 1% of the added time in the
typical interacting case.

These measurements identify avoidable setup and reconstruction overhead in the
prototype. They do not establish a crossover for intermediate fleet sizes.
The subsequent implementation removes duplicate work and reuses unchanged
components without introducing any fleet-size thresholds.

## Validation and reproduction

All 100 cases and three modes also passed separate physical checks outside the
timing runs: independent polynomial separation, endpoints, continuity, jerk
balance, contact geometry, and multiplier signs. This gives 300 independent
case/mode checks. All 6,300 timed records were checked for convergence, residuals,
continuous separation, paired cost/position differences, and timing accounting.
[Validation details](../validation/fleet-partitions-small-validation.json) record
the checker and solver source hashes.

From the SCI repository:

```sh
PYTHONPATH=src python tools/benchmark_partitions.py \
  --cases validation/fleet-small-inputs.json --repeats 21 --warmups 1 \
  --output validation/fleet-partitions-small.json
gzip -n validation/fleet-partitions-small.json
```

The original five-case N=20 benchmark remains the default when `--cases` is
omitted. The new input snapshot is portable and needs no local paper run files.

- [Raw runs, compressed JSON](../validation/fleet-partitions-small.json.gz): every
  run, stage timing, root record, environment, and source/input hash.
- [Per-case CSV](../validation/fleet-partitions-small.csv): medians and quartiles
  for all 300 case/mode combinations.
- [Aggregate summary](../validation/fleet-partitions-small-summary.json): the
  grouped figures and paired slowdown calculations used above.

The benchmark's recorded Git commit is its pre-change base; source hashes
identify the benchmark script used in the working tree. Only benchmark tooling,
inputs, results, and documentation changed in this comparison.
