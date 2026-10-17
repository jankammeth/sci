"""Solve a saved three-robot example; optionally save a plot with --output."""
import argparse
from pathlib import Path

import numpy as np
from sci_solver import SolverConfig, solve
from sci_solver.cli.vector_sci import smoke_endpoints


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    start, goal = smoke_endpoints(2, "fleet")
    solution = solve(start, goal, time_horizon=6, min_distance=1,
                     options=SolverConfig(sci_detection_tol=1e-6))
    print(f"{solution.metrics['status']}: {len(solution.contacts)} contacts, "
          f"minimum distance {solution.metrics['min_distance']:.8f}")
    if not solution.success:
        raise SystemExit(solution.metrics["convergence_reason"])
    if args.output:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(6, 5))
        for i, path in enumerate(solution.sample(np.linspace(0, 6, 301))):
            line, = ax.plot(*path.T, label=f"Robot {i + 1}")
            ax.scatter(*path[0], color=line.get_color(), marker="o")
            ax.scatter(*path[-1], color=line.get_color(), marker="x")
        ax.set(xlabel="x", ylabel="y", aspect="equal", title="Three-robot fleet")
        ax.legend()
        fig.tight_layout()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.output, dpi=180)
        plt.close(fig)


if __name__ == "__main__":
    main()
