"""Convenience interface around the reference joint solver."""
from dataclasses import dataclass

import numpy as np

from .config import Config, ProblemConfig, SolverConfig
from .evaluation import continuous_acceleration_cost
from .solvers.contact_solver import ContactSolver, RelativeTrajectory
from .solvers.vector_matching import VectorContact


@dataclass
class Solution:
    """A returned iterate and its diagnostics; inspect success before using it.

    The trajectory remains available when solving fails, for diagnosis.
    """

    trajectory: RelativeTrajectory
    contacts: tuple[VectorContact, ...]
    metrics: dict
    acceleration_cost: float

    @property
    def success(self) -> bool:
        return bool(self.metrics["converged"])

    def sample(self, times, derivative: int = 0) -> np.ndarray:
        """Evaluate position or derivative 1, 2, or 3; shape (robots, times, dimension).

        Times must be a finite one-dimensional array within the planning horizon.
        At an interior contact the third derivative uses the segment on its right.
        """
        return self.trajectory.robots(times, derivative)


def solve(
    initial_positions,
    final_positions,
    *,
    time_horizon: float,
    min_distance: float,
    initial_velocities=None,
    final_velocities=None,
    options: SolverConfig | None = None,
) -> Solution:
    """Plan for N >= 2 robots in d >= 2 dimensions with prescribed endpoints.

    Positions and optional velocities have shape (N, d). Omitted velocities
    are zero. Use SolverConfig(sci_detection_tol=...) to set the absolute
    separation allowance explicitly. By default, union-find groups contacting
    robots into independently solved components. This local solver can return
    an unsuccessful iterate.
    """
    initial_positions = np.asarray(initial_positions, dtype=float)
    final_positions = np.asarray(final_positions, dtype=float)
    if initial_positions.ndim != 2 or final_positions.shape != initial_positions.shape:
        raise ValueError("Endpoint positions must have matching shape (N, d)")
    n, d = initial_positions.shape
    if time_horizon is None:
        raise ValueError("time_horizon must be positive and finite")
    config = Config(
        problem=ProblemConfig(
            n_robots=n, dimension=d, time_horizon=time_horizon, min_distance=min_distance,
        ),
        solver=SolverConfig() if options is None else options,
    )
    solver = ContactSolver(config)
    solver.set_initial_states(initial_positions, initial_velocities)
    solver.set_final_states(final_positions, final_velocities)
    result = solver.generate_trajectories()
    return Solution(
        trajectory=solver.trajectory,
        contacts=tuple(solver.contacts),
        metrics=result["metrics"],
        acceleration_cost=continuous_acceleration_cost(solver),
    )
