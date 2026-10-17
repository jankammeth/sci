"""Compare installed SCI against the pinned upstream in separate Python processes."""
import argparse
import hashlib
import importlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import tempfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def worker(package, output):
    api = importlib.import_module(package)
    config = importlib.import_module(package + ".config")
    cli = importlib.import_module(package + ".cli.vector_sci")
    evaluation = importlib.import_module(package + ".evaluation")
    collision = importlib.import_module(package + ".solvers.collision_search")
    inputs = []
    for d in (2, 3, 5, 10):
        for case in ("free", "head_on", "offset", "fleet", "fleet5"):
            start, goal = cli.smoke_endpoints(d, case)
            inputs.append((f"{case}-{d}d", start, goal, None, None, 6., 1., 1e-6, {}))
    start, goal = cli.smoke_endpoints(3, "offset")
    velocity = np.array([[.2, -.1, .05], [-.15, .05, -.03]])
    inputs.append(("endpoint-velocities", start, goal, velocity, -.5 * velocity, 6., 1., 1e-6, {}))
    for seed in (2, 9, 15, 35, 47):
        with np.load(ROOT / f"tests/data/fleet20_seed{seed}.npz") as data:
            inputs.append((f"fleet20-seed{seed}", data["p0"], data["pf"], data["v0"], data["vf"],
                           float(data["T"]), .5, 1e-3, {}))
    start, goal = cli.smoke_endpoints(2, "offset")
    for name, options in (("iteration-limit", {"sci_root_max_iterations": 0}),
                          ("minimum-gap", {"sci_min_contact_gap": 4.})):
        inputs.append((name, start, goal, None, None, 6., 1., 1e-6, options))
    inputs.append(("invalid-endpoints", np.zeros((2, 2)), goal, None, None, 6., 1., 1e-6, {}))
    records, arrays = [], {}
    for name, start, goal, v0, vf, horizon, distance, tolerance, options in inputs:
        # Compare the retained reference formulation, including its local-vector
        # layout. The default component algorithm has separate physical/parity tests.
        if package == "sci_solver":
            options = dict(options, sci_fleet_mode="full")
        cfg = api.Config(
            name=name,
            problem=config.ProblemConfig(n_robots=len(start), dimension=start.shape[1],
                                         time_horizon=horizon, timestep=.1, min_distance=distance),
            solver=config.SolverConfig(sci_detection_tol=tolerance, **options),
        )
        solver = api.ContactSolver(cfg)
        solver.set_initial_states(start, v0)
        solver.set_final_states(goal, vf)
        metrics = solver.generate_trajectories()["metrics"]
        record = {key: metrics[key] for key in (
            "status", "convergence_reason", "converged", "num_contacts", "min_distance",
            "checks", "active_set_updates", "removed_contacts", "collision_backend",
        )}
        record.update(name=name, cost=evaluation.continuous_acceleration_cost(solver),
                      recovery_count=sum(r["message"] == "reaction_recovery" for r in metrics["roots"]))
        records.append(record)
        times = np.linspace(0, horizon, 101)
        for order in range(4):
            arrays[f"{name}-derivative-{order}"] = solver.trajectory.robots(times, order)
        arrays[f"{name}-solution"] = solver.solution
        arrays[f"{name}-knots"] = solver.trajectory.times
    manifest = json.loads((ROOT / "UPSTREAM.json").read_text())
    sources = {path: hashlib.sha256((Path(collision.__file__).parent / Path(path).name).read_bytes()).hexdigest()
               for path in manifest["core_files"]}
    report = dict(records=records, sources=sources,
                  environment=dict(python=sys.version, platform=platform.platform(),
                                   packages={name: version(name) for name in ("numpy", "scipy")}))
    output.write_text(json.dumps(report, indent=2, default=lambda x: x.item()) + "\n")
    np.savez_compressed(output.with_suffix(".npz"), **arrays)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--upstream", type=Path)
    parser.add_argument("--upstream-python", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--worker", choices=["contact_solver", "sci_solver"])
    args = parser.parse_args()
    if args.worker:
        worker(args.worker, args.output)
        return
    if not args.upstream or not args.upstream_python:
        parser.error("--upstream and --upstream-python are required")
    manifest = json.loads((ROOT / "UPSTREAM.json").read_text())
    commit = subprocess.check_output(["git", "-C", str(args.upstream), "rev-parse", "HEAD"], text=True).strip()
    if commit != manifest["upstream_commit"]:
        raise ValueError("Upstream checkout is not the recorded reference commit")
    for path, item in manifest["core_files"].items():
        transformed = (args.upstream / item["upstream_path"]).read_bytes()
        for replacement in item.get("text_replacements", []):
            transformed = transformed.replace(replacement["old"].encode(), replacement["new"].encode())
        if hashlib.sha256(transformed).hexdigest() != item["extracted_sha256"]:
            raise ValueError(f"Recorded extraction edits do not reproduce the extracted hash: {path}")
        for source, expected in ((ROOT / path, item.get("current_sha256", item["extracted_sha256"])),
                                 (args.upstream / item["upstream_path"], item["sha256"])):
            if hashlib.sha256(source.read_bytes()).hexdigest() != expected:
                raise ValueError(f"Reference source changed: {source}")
    env = dict(os.environ, OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1", VECLIB_MAXIMUM_THREADS="1")
    env.pop("PYTHONPATH", None)
    with tempfile.TemporaryDirectory() as tmp:
        reports = []
        for package, python in (("contact_solver", args.upstream_python), ("sci_solver", sys.executable)):
            path = Path(tmp) / (package + ".json")
            subprocess.run([str(python), str(Path(__file__).resolve()), "--worker", package,
                            "--output", str(path)], env=env, cwd=tmp, check=True, timeout=600)
            reports.append(json.loads(path.read_text()))
        for index, report in enumerate(reports):
            for path, item in manifest["core_files"].items():
                expected = item["sha256"] if index == 0 else item.get("current_sha256", item["extracted_sha256"])
                if report["sources"][path] != expected:
                    raise ValueError(f"Installed source differs from recorded upstream: {path}")
        for old, new in zip(reports[0]["records"], reports[1]["records"], strict=True):
            for key in ("name", "status", "convergence_reason", "converged", "num_contacts",
                        "active_set_updates", "recovery_count", "collision_backend"):
                assert old[key] == new[key], (old["name"], key, old[key], new[key])
            for key in ("cost", "min_distance"):
                np.testing.assert_allclose(old[key], new[key], rtol=1e-9, atol=1e-10)
            for key in old["checks"]:
                np.testing.assert_allclose(old["checks"][key], new["checks"][key], rtol=1e-7, atol=1e-10)
            assert len(old["removed_contacts"]) == len(new["removed_contacts"])
            for a, b in zip(old["removed_contacts"], new["removed_contacts"], strict=True):
                assert a["pair"] == b["pair"]
                np.testing.assert_allclose([a["time"], a["mass"]], [b["time"], b["mass"]], rtol=1e-9, atol=1e-10)
        max_error = 0.
        with np.load(Path(tmp) / "contact_solver.npz") as old, np.load(Path(tmp) / "sci_solver.npz") as new:
            assert set(old.files) == set(new.files)
            for key in old.files:
                np.testing.assert_allclose(old[key], new[key], rtol=1e-8, atol=1e-9, err_msg=key)
                max_error = max(max_error, float(np.max(np.abs(old[key] - new[key]), initial=0)))
        output = dict(upstream_commit=commit, sci_fleet_mode="full", passed=True,
                      cases=len(reports[0]["records"]),
                      max_absolute_array_difference=max_error, reference=reports[0], extracted=reports[1])
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(output, indent=2) + "\n")
        print(f"{output['cases']} cases matched; maximum array difference {max_error:g}")


if __name__ == "__main__":
    main()
