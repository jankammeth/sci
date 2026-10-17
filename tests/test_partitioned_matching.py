"""Check reductions against physical trajectories, including graph changes."""

from itertools import combinations
from pathlib import Path

import numpy as np
from numpy.polynomial import Polynomial
import pytest
from scipy.interpolate import CubicHermiteSpline

from sci_solver import Config, ContactSolver, ProblemConfig, SolverConfig
from sci_solver.cli.vector_sci import smoke_endpoints
from sci_solver.evaluation import continuous_acceleration_cost
from sci_solver.solvers.partitioned_matching import contact_groups


def run(start, goal, mode, *, velocities=None, horizon=5, radius=1, **options):
    solver = ContactSolver(
        Config(
            ProblemConfig(len(start), radius, horizon, dimension=start.shape[1]),
            SolverConfig(sci_fleet_mode=mode, **{"sci_detection_tol": 1e-6, **options}),
        )
    )
    v0, vf = (None, None) if velocities is None else velocities
    solver.set_initial_states(start, v0)
    solver.set_final_states(goal, vf)
    return solver, solver.generate_trajectories()["metrics"]


def check_physics(solver):
    """Validate the assembled fleet, independently of component residual checks."""
    path, contacts = solver.trajectory, solver.contacts
    n, d = solver.N, solver.d
    for order, targets in enumerate(
        (
            (solver.initial_positions, solver.final_positions),
            (solver.initial_velocities, solver.final_velocities),
        )
    ):
        actual = path.robots([0, solver.T], order)
        for k, target in enumerate(targets):
            np.testing.assert_allclose(actual[:, k], target.reshape(n, d), atol=1e-9)
    c = path.coefficients
    for k, t in enumerate(path.times[1:-1]):
        h = t - path.times[k]
        for order in (0, 1, 2):
            left = np.array([[Polynomial(axis).deriv(order)(h) for axis in rel.T] for rel in c[k]])
            np.testing.assert_allclose(left, c[k + 1, :, order] * (1, 1, 2)[order], atol=3e-7)
        jump = 6 * (c[k + 1, :, 3] - c[k, :, 3])
        first = jump.sum(axis=0) / n
        actual = np.vstack((first, first - jump))
        expected = np.zeros((n, d))
        for contact in contacts:
            if contact.t_star == t:
                p, q = contact.pair
                force = solver.R * contact.mass * contact.normal
                expected[p] += force
                expected[q] -= force
        np.testing.assert_allclose(actual, expected, atol=3e-7)
    for contact in contacts:
        p, q = contact.pair
        positions = path.robots([contact.t_star])[:, 0]
        velocities = path.robots([contact.t_star], 1)[:, 0]
        np.testing.assert_allclose(
            positions[p] - positions[q], solver.R * contact.normal, atol=1e-8
        )
        np.testing.assert_allclose(velocities[p] - velocities[q], contact.velocity, atol=1e-8)
        assert abs(contact.normal @ contact.velocity) <= solver.options.sci_residual_tol
        assert abs(contact.normal @ contact.normal - 1) <= solver.options.sci_residual_tol
        assert contact.mass >= -solver.options.sci_mass_tol
        assert contact.to_vector().size == 2 * d * (n - 1) + 2
    minimum = np.inf
    for k, h in enumerate(np.diff(path.times)):
        for p, q in combinations(range(n), 2):
            relative = c[k, q - 1] - (c[k, p - 1] if p else 0)
            polynomials = [Polynomial(axis) for axis in relative.T]
            squared = sum((p * p for p in polynomials), Polynomial([0]))
            times = [0, h] + [
                r.real for r in squared.deriv().roots() if abs(r.imag) < 1e-6 and 0 <= r.real <= h
            ]
            minimum = min(minimum, *(np.linalg.norm([p(t) for p in polynomials]) for t in times))
    assert minimum >= solver.R - solver.options.sci_detection_tol - 1e-9
    assert path.closest()[2] == pytest.approx(minimum, abs=1e-8)


def test_partition_rebuild_handles_merges_splits_and_isolated_vertices():
    pairs = [(8, 12), (2, 7), (7, 9), (2, 9)]
    assert contact_groups(pairs, "components") == [((2, 7, 9), (1, 2, 3)), ((8, 12), (0,))]
    merged = pairs + [(7, 8)]
    assert contact_groups(merged, "components") == [((2, 7, 8, 9, 12), (0, 1, 2, 3, 4))]
    assert contact_groups(merged[:-1], "components") == contact_groups(pairs, "components")
    assert contact_groups([(2, 7)], "components") == [((2, 7), (0,))]
    assert contact_groups([], "components") == []
    assert contact_groups(pairs, "active") == [((2, 7, 8, 9, 12), (0, 1, 2, 3))]


@pytest.mark.parametrize("mode", ["active", "components"])
@pytest.mark.parametrize("d", [2, 3, 5])
def test_nonparticipating_reference_robot_and_moving_free_robots(mode, d):
    start = np.zeros((5, d))
    start[:, :2] = [[100, 100], [-2, -0.2], [150, 150], [2, 0.2], [200, 200]]
    goal = start.copy()
    goal[[1, 3], 0] *= -1
    goal[[0, 2, 4]] += 1
    v0 = np.zeros_like(start)
    v0[[0, 2, 4]] = 0.12
    vf = -0.5 * v0
    solver, metrics = run(start, goal, mode, velocities=(v0, vf))
    assert metrics["converged"] and metrics["system_size"] == 2 * d + 2
    assert {p for c in solver.contacts for p in c.pair} == {1, 3}
    times = np.linspace(0, 5, 41)
    free = CubicHermiteSpline([0, 5], [start, goal], [v0, vf])
    for order in range(4):
        np.testing.assert_allclose(
            solver.trajectory.robots(times, order)[[0, 2, 4]],
            free(times, order).transpose(1, 0, 2)[[0, 2, 4]],
            atol=1e-10,
        )
    check_physics(solver)


def test_simultaneous_independent_contacts_need_no_offsets():
    start = np.array([[-2, -0.2], [2, 0.2], [-2, 9.8], [2, 10.2]])
    goal = start.copy()
    goal[:, 0] *= -1
    solver, metrics = run(start, goal, "components")
    assert metrics["converged"]
    assert metrics["system_size"] == 12
    assert len(solver.matching_components) == 2
    assert not metrics["insertion_offsets"]
    assert metrics["component_builds"] == 2
    assert metrics["component_reuses"] == 1
    assert len(metrics["roots"]) == 2  # the first component is not solved again
    np.testing.assert_allclose([c.t_star for c in solver.contacts], 2.5, atol=1e-12)
    check_physics(solver)


def test_merge_and_split_invalidate_only_changed_components():
    from sci_solver.solvers.contact_solver import RelativeTrajectory
    from sci_solver.solvers.cubic_utils import cubic_coefficients
    from sci_solver.solvers.partitioned_matching import assemble_components, initialize_event

    start = np.array([[-2, -0.2], [2, 0.2], [-2, 9.8], [2, 10.2], [-2, 19.8], [2, 20.2]])
    goal = start.copy()
    goal[:, 0] *= -1
    owner = ContactSolver(Config(ProblemConfig(6, 1, 5), SolverConfig(sci_fleet_mode="components")))
    owner.set_initial_states(start)
    owner.set_final_states(goal)
    owner.min_gap = 0
    endpoints = np.array([start, np.zeros_like(start), goal, np.zeros_like(start)])
    free = np.array(cubic_coefficients(*endpoints, 5)).transpose(1, 0, 2)
    path = RelativeTrajectory(np.array([0.0, 5.0]), (free[0] - free[1:])[None], free.mean(0))
    events = [
        initialize_event(owner, pair, t, path, uid)
        for uid, (pair, t) in enumerate([((0, 1), 1.5), ((2, 3), 2.5), ((4, 5), 3.5)])
    ]
    groups = contact_groups([e.pair for e in events], "components")
    original, _, _ = assemble_components(owner, events, groups, path, [], endpoints)
    bridge = initialize_event(owner, (1, 2), 2.0, path, 3)
    merged, rebuilt, reused = assemble_components(
        owner,
        events + [bridge],
        contact_groups([e.pair for e in events] + [bridge.pair], "components"),
        path,
        original,
        endpoints,
    )
    assert rebuilt == [[0, 1, 2, 3]] and reused == [[4, 5]]
    assert merged[1] is original[2]
    split, rebuilt, reused = assemble_components(owner, events, groups, path, merged, endpoints)
    assert rebuilt == [[0, 1], [2, 3]] and reused == [[4, 5]]
    assert split[2] is original[2]
    assert split[0] is not original[0] and split[1] is not original[1]


def test_no_nested_solver_construction(monkeypatch):
    solver = ContactSolver(
        Config(
            ProblemConfig(2, 1, 5),
            SolverConfig(sci_detection_tol=1e-6, sci_fleet_mode="components"),
        )
    )
    solver.set_initial_states([[-2, -0.2], [2, 0.2]])
    solver.set_final_states([[2, -0.2], [-2, 0.2]])

    def forbidden(*args, **kwargs):
        raise AssertionError("A component must not construct another solver")

    monkeypatch.setattr(ContactSolver, "__init__", forbidden)
    assert solver.generate_trajectories()["metrics"]["converged"]
    check_physics(solver)


def test_newton_profiling_preserves_the_trajectory():
    start, goal = smoke_endpoints(3, "fleet5")
    plain, _ = run(start, goal, "components", horizon=6)
    profiled, metrics = run(start, goal, "components", horizon=6, sci_profile_newton=True)
    assert metrics["converged"]
    np.testing.assert_array_equal(plain.trajectory.times, profiled.trajectory.times)
    np.testing.assert_array_equal(plain.trajectory.coefficients, profiled.trajectory.coefficients)
    nested = [r["newton_timing"] for r in metrics["roots"] if "newton_timing" in r]
    assert nested
    assert (
        sum(sum(r["stage_seconds"].values()) for r in nested)
        <= metrics["timing"]["stage_seconds"]["newton"]
    )
    assert sum(r["stage_calls"]["band_solve"] for r in nested) > 0
    check_physics(profiled)


@pytest.mark.parametrize("mode", ["active", "components"])
@pytest.mark.parametrize("case", ["free", "offset", "head_on", "fleet", "fleet5"])
def test_smoke_reductions_preserve_solution(mode, case):
    start, goal = smoke_endpoints(3, case)
    solver, metrics = run(start, goal, mode, horizon=6)
    full, baseline = run(start, goal, "full", horizon=6)
    assert metrics["converged"] and baseline["converged"]
    assert continuous_acceleration_cost(solver) == pytest.approx(
        continuous_acceleration_cost(full), rel=1e-6
    )
    np.testing.assert_allclose(
        solver.trajectory.robots(np.linspace(0, 6, 101)),
        full.trajectory.robots(np.linspace(0, 6, 101)),
        atol=1e-6,
    )
    check_physics(solver)


@pytest.mark.parametrize("mode", ["active", "components"])
@pytest.mark.parametrize("seed", [2, 9, 15, 35, 47])
def test_saved_fleet20_cases(mode, seed):
    with np.load(Path(__file__).parent / f"data/fleet20_seed{seed}.npz") as data:
        solver, metrics = run(
            data["p0"],
            data["pf"],
            mode,
            horizon=float(data["T"]),
            radius=0.5,
            velocities=(data["v0"], data["vf"]),
            sci_detection_tol=1e-3,
        )
    assert metrics["converged"]
    if seed == 2:
        assert len(metrics["removed_contacts"]) == 1
        assert metrics["removed_contacts"][0]["pair"] == (16, 17)
    if seed == 9:
        # Separate groups grow and join during this actual experiment.
        assert any(len(h["robots"]) > 1 for h in metrics["partition_history"]) == (
            mode == "components"
        )
        assert len(solver.matching_components) == 1
    check_physics(solver)


@pytest.mark.parametrize("mode", ["active", "components"])
@pytest.mark.parametrize(
    "options,status,reason",
    [
        ({"sci_root_max_iterations": 0}, "root_failure", "iteration_limit"),
        ({"sci_min_contact_gap": 3}, "unresolved", "minimum_knot_gap"),
        ({"timeout": 1e-12}, "unresolved", "timeout"),
    ],
)
def test_failure_retains_last_iterate(mode, options, status, reason):
    start = np.array([[-2, -0.2], [2, 0.2], [100, 100]])
    goal = start.copy()
    goal[:2, 0] *= -1
    solver, metrics = run(start, goal, mode, **options)
    assert not metrics["converged"]
    assert (metrics["status"], metrics["convergence_reason"]) == (status, reason)
    assert solver.trajectory.robots([0, 5]).shape == (3, 2, 2)


def test_mode_validation_and_stage_accounting():
    with pytest.raises(ValueError, match="sci_fleet_mode"):
        SolverConfig(sci_fleet_mode="typo")
    for mode in ("full", "active", "components"):
        start, goal = smoke_endpoints(2, "offset")
        _, metrics = run(start, goal, mode)
        timing = metrics["timing"]
        assert sum(timing["stage_seconds"].values()) == pytest.approx(timing["solve_time"])
        assert all(t >= 0 for t in timing["stage_seconds"].values())
        assert (timing["stage_calls"]["partition"] > 0) == (mode != "full")


@pytest.mark.parametrize("mode", ["active", "components"])
def test_cli_exports_fleet_indexed_contact_states(mode, tmp_path):
    from sci_solver.cli.vector_sci import main

    start = np.array([[-2, -0.2], [2, 0.2], [100, 100]])
    goal = start.copy()
    goal[:2, 0] *= -1
    cfg = Config(ProblemConfig(3, 1, 5), SolverConfig(sci_detection_tol=1e-6, sci_fleet_mode=mode))
    cfg.to_json(tmp_path / "config.json")
    np.savez(tmp_path / "states.npz", initial_positions=start, final_positions=goal)
    assert (
        main(
            [
                "--states",
                str(tmp_path / "states.npz"),
                "--config",
                str(tmp_path / "config.json"),
                "--output",
                str(tmp_path / "metrics.json"),
                "--trajectories",
                str(tmp_path / "paths.npz"),
            ]
        )
        == 0
    )
    with np.load(tmp_path / "paths.npz") as data:
        assert data["contact_states"].shape == (1, 10)
        np.testing.assert_array_equal(data["contact_pairs"], [[0, 1]])


def test_legacy_configuration_uses_current_default():
    from types import SimpleNamespace

    cfg = Config(ProblemConfig(2, 1, 5))
    cfg.solver = SimpleNamespace(
        **{k: v for k, v in vars(cfg.solver).items() if k != "sci_fleet_mode"}
    )
    solver = ContactSolver(cfg)
    solver.set_initial_states([[0, 0], [2, 0]])
    solver.set_final_states([[1, 1], [3, 1]])
    assert solver.generate_trajectories()["metrics"]["fleet_mode"] == "components"
