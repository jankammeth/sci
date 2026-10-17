"""Interleaved timings on saved regression experiments or an input snapshot.

Run from the SCI repository with its package installed (or PYTHONPATH=src).
Thread limits are set before importing NumPy. Cost/sample comparisons are outside
the timed solve. No cases are dropped on failure.
"""

# Thread limits must precede numerical imports.
# ruff: noqa: E402

import argparse
from collections import defaultdict
import csv
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import time

THREAD_LIMITS = dict.fromkeys(
    (
        "OPENBLAS_NUM_THREADS",
        "OMP_NUM_THREADS",
        "MKL_NUM_THREADS",
        "VECLIB_MAXIMUM_THREADS",
        "NUMEXPR_NUM_THREADS",
    ),
    "1",
)
os.environ.update(THREAD_LIMITS)

import numpy as np
import scipy

from sci_solver import Config, ContactSolver, ProblemConfig, SolverConfig
from sci_solver.evaluation import continuous_acceleration_cost
from sci_solver.solvers.collision_search import COLLISION_BACKEND

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def default_cases(seeds):
    cases = []
    for seed in seeds:
        path = ROOT / f"tests/data/fleet20_seed{seed}.npz"
        with np.load(path) as data:
            cases.append(
                dict(
                    case_id=f"n20_seed{seed}",
                    seed=seed,
                    problem=dict(
                        n_robots=20, dimension=2, min_distance=0.5, time_horizon=float(data["T"])
                    ),
                    options=dict(sci_detection_tol=1e-3),
                    states={key: data[key].tolist() for key in ("p0", "v0", "pf", "vf")},
                    input_source=str(path.relative_to(ROOT)),
                    input_sha256=sha(path),
                )
            )
    return cases


def load_cases(path, seeds):
    cases = default_cases(seeds) if path is None else json.loads(path.read_text())["cases"]
    identities = [c["case_id"] for c in cases]
    keys = [(c["problem"]["n_robots"], c["seed"]) for c in cases]
    if not cases or len(set(identities)) != len(cases) or len(set(keys)) != len(cases):
        raise ValueError("Cases must be nonempty with unique IDs and (N, seed) pairs")
    return cases


def run(case, mode, timeout, recovery):
    started = time.perf_counter()
    config = Config(
        ProblemConfig(**case["problem"]),
        SolverConfig(
            **{
                **case.get("options", {}),
                "sci_fleet_mode": mode,
                "timeout": timeout,
                "sci_reaction_recovery": recovery,
            }
        ),
    )
    solver = ContactSolver(config)
    data = case["states"]
    solver.set_initial_states(data["p0"], data["v0"])
    solver.set_final_states(data["pf"], data["vf"])
    setup_seconds = time.perf_counter() - started
    metrics = solver.generate_trajectories()["metrics"]
    wall_seconds = time.perf_counter() - started
    record = dict(
        case_id=case["case_id"],
        n_robots=config.problem.n_robots,
        seed=case["seed"],
        mode=mode,
        config=config.to_dict(),
        metrics=metrics,
        setup_seconds=setup_seconds,
        wall_seconds=wall_seconds,
        cost=continuous_acceleration_cost(solver),
        contacts=[
            dict(pair=c.pair, time=float(c.t_star), mass=float(c.mass)) for c in solver.contacts
        ],
    )
    # These comparisons supplement, never replace, continuous separation checks.
    samples = solver.trajectory.robots(np.linspace(0, solver.T, 201))
    return record, samples


def summarize(records):
    groups = defaultdict(list)
    for r in records:
        groups[r["n_robots"], r["seed"], r["mode"]].append(r)
    rows = []
    for (n_robots, seed, mode), values in sorted(groups.items()):
        metrics = [v["metrics"] for v in values]
        times = [m["timing"]["solve_time"] * 1000 for m in metrics]
        row = dict(
            n_robots=n_robots,
            seed=seed,
            mode=mode,
            runs=len(values),
            successes=sum(m["converged"] for m in metrics),
            solve_ms=float(np.median(times)),
            p25_ms=float(np.percentile(times, 25)),
            p75_ms=float(np.percentile(times, 75)),
            wall_ms=float(np.median([v["wall_seconds"] * 1000 for v in values])),
            contacts=float(np.median([m["num_contacts"] for m in metrics])),
            baseline_contacts=values[0]["baseline_contacts"],
            system_size=float(np.median([m["system_size"] for m in metrics])),
            max_cost_relative_difference=max(v["cost_relative_difference"] for v in values),
            max_position_difference=max(v["max_position_difference"] for v in values),
            recovery_count=float(
                np.median(
                    [sum(r["message"] == "reaction_recovery" for r in m["roots"]) for m in metrics]
                )
            ),
        )
        for stage in metrics[0]["timing"]["stage_seconds"]:
            row[stage + "_ms"] = float(
                np.median([m["timing"]["stage_seconds"][stage] * 1000 for m in metrics])
            )
        row["partition_calls"] = float(
            np.median([m["timing"]["stage_calls"]["partition"] for m in metrics])
        )
        rows.append(row)
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, nargs="+", default=[2, 9, 15, 35, 47])
    parser.add_argument(
        "--cases", type=Path, help="JSON snapshot with a cases list; overrides --seeds"
    )
    parser.add_argument("--repeats", type=int, default=7)
    parser.add_argument("--warmups", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=30)
    parser.add_argument("--no-recovery", action="store_true")
    parser.add_argument("--output", type=Path, default=ROOT / "validation/fleet-partitions.json")
    args = parser.parse_args()
    if args.repeats < 1 or args.warmups < 0:
        parser.error("repeats must be positive and warmups nonnegative")
    if COLLISION_BACKEND != "native":
        parser.error("Build the native collision kernel before comparing runtimes")
    modes = ("full", "active", "components")
    cases = load_cases(args.cases, args.seeds)
    records, warmups = [], []
    for case in cases:
        for _ in range(args.warmups):
            for mode in modes:
                record, _ = run(case, mode, args.timeout, not args.no_recovery)
                warmups.append(
                    dict(case_id=case["case_id"], mode=mode, status=record["metrics"]["status"])
                )
    rng = np.random.default_rng(20261017)
    for repeat in range(args.repeats):
        for index in rng.permutation(len(cases)):
            case = cases[index]
            batch = {}
            for mode in rng.permutation(modes):
                mode = str(mode)
                record, samples = run(case, mode, args.timeout, not args.no_recovery)
                record["repeat"] = repeat
                batch[mode] = (record, samples)
                records.append(record)
                print(
                    f"repeat={repeat} case={case['case_id']} mode={mode}: {record['metrics']['status']} "
                    f"{record['metrics']['timing']['solve_time'] * 1000:.3f} ms",
                    flush=True,
                )
            reference, positions = batch["full"]
            for record, samples in batch.values():
                record["cost_relative_difference"] = abs(record["cost"] - reference["cost"]) / max(
                    abs(reference["cost"]), 1e-15
                )
                record["max_position_difference"] = float(np.max(np.abs(samples - positions)))
                record["reference_success"] = reference["metrics"]["converged"]
                record["baseline_contacts"] = reference["metrics"]["num_contacts"]
    sources = sorted(
        p for p in (ROOT / "src").rglob("*") if p.suffix in (".py", ".c", ".so", ".pyd")
    )
    sources += [Path(__file__)]
    report = dict(
        timestamp_utc=datetime.now(timezone.utc).isoformat(),
        environment=dict(
            python=sys.version,
            platform=platform.platform(),
            machine=platform.machine(),
            numpy=np.__version__,
            scipy=scipy.__version__,
            collision_backend=COLLISION_BACKEND,
            thread_limits=THREAD_LIMITS,
        ),
        git_commit=subprocess.check_output(
            ["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True
        ).strip(),
        source_hashes={str(p.relative_to(ROOT)): sha(p) for p in sources},
        input_hashes={c["case_id"]: c["input_sha256"] for c in cases},
        input_snapshot_sha256=sha(args.cases) if args.cases else None,
        protocol=dict(
            repeats=args.repeats,
            warmups=args.warmups,
            timeout=args.timeout,
            shuffle_seed=20261017,
            recovery=not args.no_recovery,
            default_options=asdict(SolverConfig()),
            description="One process, sequential interleaved cases/modes; disjoint stages exclude sampling. "
            "Recovery stage includes its assembly, reconstruction and acceptance checks. "
            "Partition stage includes grouping, union-find and deterministic sorting. "
            "Solve times include failed attempts; postprocessing and setup are reported separately.",
        ),
        warmups=warmups,
        records=records,
        summary=summarize(records),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # One compact line per run keeps large repeated benchmarks reviewable.
    raw_records = report.pop("records")
    metadata = json.dumps(report, indent=2, allow_nan=False)
    args.output.write_text(
        metadata[:-2]
        + ',\n  "records": [\n'
        + ",\n".join(
            "    " + json.dumps(r, separators=(",", ":"), allow_nan=False) for r in raw_records
        )
        + "\n  ]\n}\n"
    )
    with args.output.with_suffix(".csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(report["summary"][0]))
        writer.writeheader()
        writer.writerows(report["summary"])
    print(f"Saved {args.output} and {args.output.with_suffix('.csv')}")


if __name__ == "__main__":
    main()
