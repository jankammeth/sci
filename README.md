# SCI — Sequential Contact Insertion

Continuous-time motion planning for robot fleets, using analytic cubic trajectories
and a progressively constructed set of contacts.

SCI plans for at least two robots in two or more spatial dimensions with prescribed
endpoint positions and velocities, a fixed horizon, and a shared minimum pairwise
distance. It minimizes squared acceleration through local contact matching and
checks separation over the entire trajectory.

## Installation

Python 3.10 or newer is required. After the first PyPI release:

```sh
pip install sci-solver
```

To install directly from GitHub now:

```sh
python -m pip install git+https://github.com/jankammeth/sci.git
```

Or install a local checkout:

```sh
python -m pip install .
```

NumPy and SciPy are the only runtime dependencies. Supported binary wheels include
the C collision checker. A source installation builds it when a C compiler is
available; otherwise it uses the slower Python checker and emits a warning on use.
The chosen backend is reported in `solution.metrics["collision_backend"]`.

## Quick start

```python
import numpy as np
from sci_solver import SolverConfig, solve

solution = solve(
    initial_positions=[[-2, -0.2], [2, 0.2]],
    final_positions=[[2, -0.2], [-2, 0.2]],
    time_horizon=5.0,
    min_distance=1.0,
    options=SolverConfig(sci_detection_tol=1e-6),
)
if not solution.success:
    raise RuntimeError(solution.metrics["convergence_reason"])

times = np.linspace(0, 5, 201)
positions = solution.sample(times)              # (2 robots, 201 times, 2 dimensions)
velocities = solution.sample(times, derivative=1)
print(solution.acceleration_cost)
print(solution.metrics["min_distance"])
```

Omitted endpoint velocities are zero. Sampling is postprocessing and does not
determine whether a trajectory passes separation checks. `solution.contacts`
contains the solved contact states, and `solution.trajectory` exposes the cubic
coefficients and knot times.

## Examples

```sh
python -m pip install '.[plot]'
python examples/two_robots.py --output outputs/two-robots.png
python examples/fleet.py --output outputs/fleet.png
sci-solve --smoke --output outputs/smoke.json
```

The plotting dependency is optional. Both examples also run without plotting when
`--output` is omitted. The CLI accepts saved NPZ endpoint states and a JSON config;
see [the API guide](docs/api.md).

## Numerical scope

The solver uses the current contact insertion, contact removal, banded Newton,
sparse fallback, and condensed reaction recovery implementation. It is a local
numerical method: convergence and global optimality are not guaranteed. The
continuous collision checks use floating-point polynomial calculations, rather
than interval-arithmetic certificates. Obstacles and finite position, velocity,
acceleration, or jerk bounds are outside this release's scope.

Always inspect `solution.success`. An unsuccessful solve retains its last iterate
and diagnostics; it is not a usable collision-free plan. Tolerances are absolute
and should be chosen for the physical scale of the problem. The reference default
separation allowance is zero; the examples explicitly use `1e-6`.

For fleet-reduction experiments, select `SolverConfig(sci_fleet_mode="active")`
to include only participating robots, or `sci_fleet_mode="components"` to solve
each connected contact group separately. The default remains `"full"` for
comparison. All modes check continuous separation across the entire fleet.
[The benchmark report](docs/fleet-partitions.md) records stage timings on five
saved 20-robot cases and explains the current limitations.

## Development and provenance

```sh
python -m pip install -e '.[dev,plot]'
python -m pytest
ruff check .
```

The numerical implementation is preserved from
[`contact-solver` at `641a234`](https://github.com/jankammeth/contact-solver/tree/641a2341a6a59938e285b8ff872bd484713e0605).
[UPSTREAM.json](UPSTREAM.json) records source paths, SHA-256 hashes, and every
extraction edit. Five core files remain byte-identical. The collision wrapper changes
only the package name in its fallback warning; the C kernel adds equivalent
stack workspace allocation for Microsoft's C compiler. Numerical formulas and
control flow in the default full-fleet solver are retained; its driver now records
stage timings and dispatches optional fleet reductions. The original extraction
hashes are retained alongside the current driver hash and its change description.
[The migration guide](docs/migration.md) describes the extraction and comparison
procedure. The paper and historical experiment infrastructure remain separate.

This software accompanies *Contact-based motion planning for robot fleets*.
Software citation metadata is in [CITATION.cff](CITATION.cff). Publication metadata
for the manuscript will be added when available.

Licensed under Apache-2.0; see [LICENSE](LICENSE) and [NOTICE](NOTICE).
