"""Measurements of the continuous trajectory."""
import numpy as np

def continuous_acceleration_cost(solver, *, knots=None):
    """Exact cubic-path integral; explicit knots support saved baseline adapters."""
    if knots is None:
        if hasattr(solver, "trajectory"):
            knots = solver.trajectory.times
        else:
            knots = sorted(
                {
                    t
                    for path in solver._trajectories.values()
                    for _, a, b in path.segments
                    for t in (a, b)
                }
            )
    cost = 0.0
    for a, b in zip(knots[:-1], knots[1:], strict=True):
        times = (a + b) / 2 + (b - a) / (2 * np.sqrt(3)) * np.array([-1.0, 1.0])
        cost += (b - a) / 2 * np.sum(np.asarray(solver.sample(times=times)[3]) ** 2)
    return float(cost)
