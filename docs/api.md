# API and numerical options

`solve(initial_positions, final_positions, *, time_horizon, min_distance,
initial_velocities=None, final_velocities=None, options=None)` returns a `Solution`.
Endpoint arrays have shape `(N, d)` with `N >= 2` and `d >= 2`.

## Results

| Member | Meaning |
| --- | --- |
| `success` | Matching checks and continuous separation both passed. |
| `sample(times, derivative=0)` | Array with shape `(N, len(times), d)`; derivative orders 0–3. |
| `trajectory` | Knot times, relative cubic coefficients, and barycenter coefficients. |
| `contacts` | Pair indices, times, normals, relative velocities, and reaction masses. |
| `acceleration_cost` | Integrated sum of squared robot accelerations. |
| `metrics` | Status, reason, residuals, separation, backend, timing, and recovery details. |

Times may be unordered but must be finite and inside the horizon. At a contact,
the third derivative uses the segment to its right; the final endpoint uses the
last segment. Orders 0–2 are continuous up to the matching tolerance.

## Options

Pass `SolverConfig(...)` through `options`. Defaults match the reference implementation.

| Option | Default | Meaning |
| --- | --- | --- |
| `sci_detection_tol` | `0.0` | Allowed absolute separation deficit. |
| `sci_residual_tol` | `1e-7` | Matching and endpoint residual tolerance. |
| `sci_mass_tol` | `1e-9` | Allowed negative reaction mass within tolerance. |
| `sci_min_contact_gap` | `0.0` | Minimum knot gap; zero still requires strict ordering. |
| `sci_root_max_iterations` | `100` | Newton iterations per matching solve. |
| `sci_reaction_recovery` | `True` | Enable condensed recovery when matching fails. |
| `timeout` | `300.0` | Cooperative solve deadline in seconds. |
| `use_analytical_jacobian` | `True` | Use the analytical matching Jacobian. |
| `sci_fleet_mode` | `"full"` | `"full"`, `"active"`, or `"components"`; see below. |

Statuses are `success`, `unresolved`, `root_failure`, and `invalid_solution`.
`metrics["convergence_reason"]` supplies details. Unsuccessful iterates remain
available for diagnosis. The deadline is checked between numerical operations;
it does not interrupt an in-progress linear solve.

For direct control, use `Config`, `ProblemConfig`, and `ContactSolver`, followed by
`set_initial_states`, `set_final_states`, and `generate_trajectories`. This retains
the reference interface. Its `ProblemConfig.timestep` is a legacy configuration
field used by automatic horizon selection when the horizon is omitted; it does
not discretize contact matching or collision checking. The convenience `solve`
function requires an explicit horizon.

## Experimental fleet reductions

`full` uses all robots in every contact block. `active` eliminates robots with no
current contacts and solves one system for the remaining robots. `components`
also partitions the contact graph and solves one system per connected group.
The graph is rebuilt with union-find after every insertion or removal; there is
no persistent graph cache. Every group uses the same Newton and recovery routines
and its own endpoint barycenter. Uninvolved robots follow their unconstrained
endpoint cubics. Collision search still covers all pairs and can activate robots
or join components on later iterations.

Component trajectories are joined by translating cubic coefficients onto the
union of their knots. Independent components can have simultaneous contacts.
Minimum knot gaps and ordering constraints apply within each component. If a
new edge joins components whose existing contact times coincide or violate the
configured gap, the solver returns `unresolved` with reason
`merged_component_knot_gap`; it does not move already solved contact times to
make the merge fit the current Hermite representation.

`Solution`, sampling, contact indices, and CLI exports retain their fleet-wide
shape. For low-level diagnostics in the reduced modes, `ContactSolver.system`
is a tuple of local matching systems, `solution` concatenates their local vectors,
and `matching_components` associates each local system/vector/trajectory with
its original robot IDs. `metrics["system_size"]` is the sum of local system sizes;
`partition_history` records robot groups, contact counts, and sizes after each
contact-set change. Each root record also reports the size actually solved.

`metrics["timing"]` includes `stage_seconds` and `stage_calls` for partitioning,
assembly, Newton, recovery, reconstruction, validation, and collision search.
Stages do not overlap. Partitioning includes union-find, index grouping, and
sorting; assembly includes initial local paths; recovery includes its own
assembly/reconstruction/checks. `other` accounts for remaining solve overhead.
Final output sampling is separately reported as `postprocess_time`.
Use these stages for comparisons: the legacy `matching_seconds` in full mode
also contains active-set bookkeeping, while reduced modes report Newton plus
recovery there. Timings are wall-clock seconds, not CPU-time measurements.

See [the fleet benchmark](fleet-partitions.md) for measured costs and reproduction.

## Saved problems

```sh
sci-solve --states endpoints.npz --config examples/config.json \
  --output outputs/metrics.json --trajectories outputs/trajectories.npz
```

Input NPZ keys are `initial_positions`, `final_positions`, and optionally
`initial_velocities`, `final_velocities`. JSON configuration uses `problem` and
`solver` objects; unknown fields are rejected. `Config.to_json` and
`Config.from_json` support roundtrips without a YAML dependency.

The CLI exits zero only for a successful solve. Sampled output includes times,
positions, velocities, accelerations, contact states, and contact pairs.
