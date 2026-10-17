"""Public API and installed-package checks."""
import subprocess
import sys

import numpy as np
import pytest

from sci_solver import Config, ContactSolver, ProblemConfig, SolverConfig, solve


def test_default_api_partitions_contacts_and_reuses_unchanged_groups():
    start = np.array([[-2, -.2], [2, .2], [-2, 9.8], [2, 10.2], [100, 100.]])
    goal = start.copy()
    goal[:4, 0] *= -1
    goal[4] += 1
    solution = solve(start, goal, time_horizon=5, min_distance=1,
                     options=SolverConfig(sci_detection_tol=1e-6))
    assert solution.success
    assert solution.metrics["fleet_mode"] == "components"
    assert solution.metrics["system_size"] == 12  # two independent two-robot systems
    assert solution.metrics["component_builds"] == 2
    assert solution.metrics["component_reuses"] == 1
    assert {c.pair for c in solution.contacts} == {(0, 1), (2, 3)}
    assert solution.sample([0, 2.5, 5]).shape == (5, 3, 2)
    np.testing.assert_allclose(solution.sample([0, 5])[:, 0], start, atol=1e-12)
    np.testing.assert_allclose(solution.sample([0, 5])[:, 1], goal, atol=1e-12)
    np.testing.assert_allclose(solution.sample([2.5])[4, 0], [100.5, 100.5], atol=1e-12)


def test_omitted_options_use_components_without_contact_systems():
    solution = solve([[0, 0], [3, 0]], [[1, 1], [4, 1]],
                     time_horizon=5, min_distance=1)
    assert solution.success
    assert solution.metrics["fleet_mode"] == "components"
    assert solution.metrics["system_size"] == 0
    assert solution.metrics["timing"]["stage_calls"]["newton"] == 0


def test_full_mode_remains_available_through_public_api():
    solution = solve([[-2, -.2], [2, .2]], [[2, -.2], [-2, .2]],
                     time_horizon=5, min_distance=1,
                     options=SolverConfig(sci_fleet_mode="full", sci_detection_tol=1e-6))
    assert solution.success
    assert solution.metrics["fleet_mode"] == "full"
    assert len(solution.contacts) == 1


def test_convenience_api_preserves_reference_result():
    start = np.array([[-2., -.2, .1], [2., .2, -.1]])
    goal = start.copy()
    goal[:, 0] *= -1
    options = SolverConfig(sci_detection_tol=1e-6)
    solution = solve(start, goal, time_horizon=6, min_distance=1, options=options)
    reference = ContactSolver(Config(ProblemConfig(2, 1, 6, dimension=3), options))
    reference.set_initial_states(start)
    reference.set_final_states(goal)
    result = reference.generate_trajectories()
    assert solution.success and result["metrics"]["converged"]
    times = np.array([6., 0., 3., 1.])
    for order in (0, 1, 2, 3):
        np.testing.assert_array_equal(solution.sample(times, order), reference.trajectory.robots(times, order))
    np.testing.assert_allclose(solution.sample([0, 6])[:, 0], start, atol=1e-12)
    np.testing.assert_allclose(solution.sample([0, 6])[:, 1], goal, atol=1e-12)
    assert solution.acceleration_cost > 0


def test_unsuccessful_solve_keeps_diagnostics_and_iterate():
    solution = solve([[-2, -.2], [2, .2]], [[2, -.2], [-2, .2]],
                     time_horizon=5, min_distance=1,
                     options=SolverConfig(sci_root_max_iterations=0))
    assert not solution.success
    assert solution.metrics["status"] == "root_failure"
    assert solution.sample([0, 5]).shape == (2, 2, 2)


@pytest.mark.parametrize("times", [[-1], [6], [float("nan")], [[0, 1]]])
def test_sample_rejects_invalid_times(times):
    solution = solve([[0, 0], [3, 0]], [[1, 1], [4, 1]], time_horizon=5, min_distance=1)
    with pytest.raises(ValueError, match="Sample times"):
        solution.sample(times)


def test_core_import_does_not_load_experiment_dependencies():
    subprocess.run([sys.executable, "-c", "import sci_solver, sys; "
                    "assert not ({'osqp', 'pandas', 'matplotlib', 'omegaconf'} & sys.modules.keys())"],
                   check=True)


def test_configuration_json_roundtrip_and_unknown_options(tmp_path):
    cfg = Config(ProblemConfig(2, 1, 5))
    path = tmp_path / "config.json"
    cfg.to_json(path)
    assert Config.from_json(path).to_dict() == cfg.to_dict()
    assert Config.from_dict({"problem": {"n_robots": 2, "min_distance": 1}}).solver.sci_fleet_mode == "components"
    with pytest.raises(TypeError):
        Config.from_dict({"problem": {"n_robots": 2, "min_distance": 1}, "solver": {"typo": 1}})


@pytest.mark.parametrize("start,goal", [([0, 1], [1, 0]), ([[0, 0]], [[1, 1]]),
                                        ([[0, 0], [3, 0]], [[1, 1]])])
def test_invalid_endpoint_shapes(start, goal):
    with pytest.raises(ValueError):
        solve(start, goal, time_horizon=5, min_distance=1)
