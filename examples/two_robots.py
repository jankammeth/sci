"""Solve a two-robot crossing; optionally save a plot with --output."""
import argparse
from pathlib import Path

import numpy as np
from sci_solver import SolverConfig, solve


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    solution = solve([[-2, -.2], [2, .2]], [[2, -.2], [-2, .2]],
                     time_horizon=5, min_distance=1,
                     options=SolverConfig(sci_detection_tol=1e-6))
    print(f"{solution.metrics['status']}: minimum distance {solution.metrics['min_distance']:.8f}, "
          f"cost {solution.acceleration_cost:.8f}, backend {solution.metrics['collision_backend']}")
    if not solution.success:
        raise SystemExit(solution.metrics["convergence_reason"])
    if args.output:
        import matplotlib.pyplot as plt
        positions = solution.sample(np.linspace(0, 5, 201))
        fig, ax = plt.subplots(figsize=(6, 3))
        for i, path in enumerate(positions):
            line, = ax.plot(*path.T, label=f"Robot {i + 1}")
            ax.scatter(*path[0], color=line.get_color(), marker="o")
            ax.scatter(*path[-1], color=line.get_color(), marker="x")
        ax.set(xlabel="x", ylabel="y", aspect="equal", title="Sequential Contact Insertion")
        ax.legend()
        fig.tight_layout()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.output, dpi=180)
        plt.close(fig)


if __name__ == "__main__":
    main()
