"""SCI: Sequential Contact Insertion for continuous-time robot fleet planning."""
from .api import Solution, solve
from .config import Config, ProblemConfig, SolverConfig
from .solvers import ContactSolver, RelativeTrajectory

__all__ = [
    "Config", "ProblemConfig", "SolverConfig", "ContactSolver",
    "RelativeTrajectory", "Solution", "solve",
]
