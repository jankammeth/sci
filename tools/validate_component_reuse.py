"""Validate the saved benchmark and selected physical solutions outside timing."""

import argparse
import gzip
import importlib.util
import json
from pathlib import Path

import benchmark_component_reuse as benchmark
import numpy as np
from sci_solver import Config, ContactSolver, ProblemConfig, SolverConfig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--full-before", type=Path, required=True)
    parser.add_argument("--full-after", type=Path, required=True)
    parser.add_argument(
        "--benchmark", type=Path, default=benchmark.ROOT / "validation/component-reuse.json.gz"
    )
    parser.add_argument(
        "--output", type=Path, default=benchmark.ROOT / "validation/component-reuse-validation.json"
    )
    args = parser.parse_args()
    data = json.loads(gzip.decompress(args.benchmark.read_bytes()))
    assert data["current_sources"] == benchmark.hashes(benchmark.ROOT)
    cases = {c["case_id"]: c for c in data["cases"]}
    for r in data["records"] + data["profiling"]:
        m = r["metrics"]
        assert m["converged"] and m["checks"]["valid"]
        case = cases[r["case_id"]]
        assert m["min_distance"] >= (
            case["problem"]["min_distance"] - case["options"]["sci_detection_tol"]
        )
        np.testing.assert_allclose(
            sum(m["timing"]["stage_seconds"].values()), m["timing"]["solve_time"], atol=1e-12
        )
        if not r["profile"]:
            assert r["cost_relative_difference"] < 1e-9
            assert r["max_position_difference"] < 1e-6

    checker = benchmark.ROOT / "tests/test_partitioned_matching.py"
    spec = importlib.util.spec_from_file_location("physics_checks", checker)
    checks = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(checks)
    physical, profiled = [], []
    for case in cases.values():
        for mode in ("full", "active", "components"):
            options = dict(case["options"], sci_fleet_mode=mode, timeout=30.0)
            solver = ContactSolver(
                Config(ProblemConfig(**case["problem"]), SolverConfig(**options))
            )
            states = case["states"]
            solver.set_initial_states(states["p0"], states["v0"])
            solver.set_final_states(states["pf"], states["vf"])
            assert solver.generate_trajectories()["metrics"]["converged"]
            checks.check_physics(solver)
            physical.append(dict(case_id=case["case_id"], mode=mode, passed=True))
            if mode == "components" and solver.N == 20:
                original = solver.trajectory
                solver.options.sci_profile_newton = True
                assert solver.generate_trajectories()["metrics"]["converged"]
                np.testing.assert_array_equal(original.times, solver.trajectory.times)
                np.testing.assert_array_equal(original.coefficients, solver.trajectory.coefficients)
                profiled.append(case["case_id"])

    before, after = (json.loads(p.read_text()) for p in (args.full_before, args.full_after))
    assert before["records"] == after["records"]
    maximum = 0.0
    with (
        np.load(args.full_before.with_suffix(".npz")) as old,
        np.load(args.full_after.with_suffix(".npz")) as new,
    ):
        assert old.files == new.files
        for key in old.files:
            np.testing.assert_array_equal(old[key], new[key])
            maximum = max(maximum, float(np.max(np.abs(old[key] - new[key]), initial=0)))
        arrays = len(old.files)
    result = dict(
        passed=True,
        benchmark_sha256=benchmark.common.sha(args.benchmark),
        measured_runs=len(data["records"]),
        profiling_runs=len(data["profiling"]),
        physical_checks=physical,
        profile_trajectory_exact_matches=profiled,
        full_mode_parity=dict(
            cases=len(before["records"]),
            arrays=arrays,
            records_equal=True,
            max_absolute_array_difference=maximum,
            before=before,
            after=after,
        ),
        max_relative_cost_difference=max(r["cost_relative_difference"] for r in data["records"]),
        max_position_difference=max(r["max_position_difference"] for r in data["records"]),
        checker_sha256=benchmark.common.sha(checker),
        validator_sha256=benchmark.common.sha(Path(__file__)),
        current_sources=benchmark.hashes(benchmark.ROOT),
    )
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(
        f"Validated {len(data['records'])} timed runs, {len(data['profiling'])} profiling runs, "
        f"{len(physical)} physical checks, and {len(before['records'])} full-mode parity cases."
    )


if __name__ == "__main__":
    main()
