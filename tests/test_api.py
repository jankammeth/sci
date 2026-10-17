"""Public API and installed-package checks."""
import subprocess
import sys

import numpy as np
import pytest

from sci_solver import Config, ContactSolver, ProblemConfig, SolverConfig, solve


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
    with pytest.raises(TypeError):
        Config.from_dict({"problem": {"n_robots": 2, "min_distance": 1}, "solver": {"typo": 1}})


@pytest.mark.parametrize("start,goal", [([0, 1], [1, 0]), ([[0, 0]], [[1, 1]]),
                                        ([[0, 0], [3, 0]], [[1, 1]])])
def test_invalid_endpoint_shapes(start, goal):
    with pytest.raises(ValueError):
        solve(start, goal, time_horizon=5, min_distance=1)
