"""Compare a preserved component prototype with the current implementation."""

import argparse
from collections import defaultdict
import csv
import gzip
import importlib
import importlib.util
import json
from pathlib import Path
import sys
import time

# Sets numerical-library thread limits before numerical imports.
import benchmark_partitions as common
import numpy as np
import scipy
import sci_solver

ROOT = common.ROOT


def load_reference(path):
    source = path / "src/sci_solver"
    spec = importlib.util.spec_from_file_location(
        "sci_reference", source / "__init__.py", submodule_search_locations=[str(source)]
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def cases():
    small = common.load_cases(ROOT / "validation/fleet-small-inputs.json", [])
    selection = {2: {0, 2, 10}, 4: {1, 0, 40, 8}}
    return [
        c for c in small if c["seed"] in selection[c["problem"]["n_robots"]]
    ] + common.default_cases([2, 9, 15, 35, 47])


def run(module, case, variant, profile=False):
    options = {
        **case["options"],
        "sci_fleet_mode": "full" if variant == "full" else "components",
        "timeout": 30.0,
    }
    if profile:
        options["sci_profile_newton"] = True
    config = module.Config(module.ProblemConfig(**case["problem"]), module.SolverConfig(**options))
    solver = module.ContactSolver(config)
    s = case["states"]
    solver.set_initial_states(s["p0"], s["v0"])
    solver.set_final_states(s["pf"], s["vf"])
    started = time.perf_counter()
    metrics = solver.generate_trajectories()["metrics"]
    call_seconds = time.perf_counter() - started
    evaluate = importlib.import_module(module.__name__ + ".evaluation")
    record = dict(
        case_id=case["case_id"],
        n_robots=solver.N,
        seed=case["seed"],
        variant=variant,
        profile=profile,
        metrics=metrics,
        call_seconds=call_seconds,
        cost=evaluate.continuous_acceleration_cost(solver),
        contacts=[
            dict(pair=c.pair, time=float(c.t_star), mass=float(c.mass)) for c in solver.contacts
        ],
    )
    return record, solver.trajectory.robots(np.linspace(0, solver.T, 201))


def summarize(records):
    groups = defaultdict(list)
    for r in records:
        groups[r["n_robots"], r["seed"], r["variant"]].append(r)
    rows = []
    for (n, seed, variant), values in sorted(groups.items()):
        metrics = [v["metrics"] for v in values]

        def median(xs):
            return float(np.median(xs))

        times = [m["timing"]["solve_time"] * 1000 for m in metrics]
        row = dict(
            n_robots=n,
            seed=seed,
            variant=variant,
            runs=len(values),
            successes=sum(m["converged"] for m in metrics),
            solve_ms=median(times),
            p25_ms=float(np.percentile(times, 25)),
            p75_ms=float(np.percentile(times, 75)),
            call_ms=median([v["call_seconds"] * 1000 for v in values]),
            contacts=median([m["num_contacts"] for m in metrics]),
            builds=median(
                [
                    m.get(
                        "component_builds",
                        sum(len(h["robots"]) for h in m.get("partition_history", [])),
                    )
                    for m in metrics
                ]
            ),
            reuses=median([m.get("component_reuses", 0) for m in metrics]),
            newton_calls=median([m["timing"]["stage_calls"]["newton"] for m in metrics]),
            nfev=median([m["nfev"] for m in metrics]),
            recovery_count=median(
                [sum(r["message"] == "reaction_recovery" for r in m["roots"]) for m in metrics]
            ),
            max_relative_cost_difference=max(v.get("cost_relative_difference", 0) for v in values),
            max_position_difference=max(v.get("max_position_difference", 0) for v in values),
        )
        for stage in metrics[0]["timing"]["stage_seconds"]:
            row[stage + "_ms"] = median(
                [m["timing"]["stage_seconds"][stage] * 1000 for m in metrics]
            )
        rows.append(row)
    return rows


def profiles(records):
    rows = []
    for seed in sorted({r["seed"] for r in records}):
        runs = [r for r in records if r["seed"] == seed]
        details = []
        for run in runs:
            detail = defaultdict(float)
            metrics = run["metrics"]
            for root in metrics["roots"]:
                if "newton_timing" in root:
                    for stage, seconds in root["newton_timing"]["stage_seconds"].items():
                        detail[stage + "_ms"] += seconds * 1000
                    for stage, count in root["newton_timing"]["stage_calls"].items():
                        detail[stage + "_calls"] += count
                    detail["lsmr_iterations"] += root["lsmr_iterations"]
                    detail["newton_iterations"] += root["iterations"]
                detail["sparse_fallbacks"] += root["sparse_fallbacks"]
            detail["solve_ms"] = metrics["timing"]["solve_time"] * 1000
            detail["newton_ms"] = metrics["timing"]["stage_seconds"]["newton"] * 1000
            details.append(detail)
        row = dict(seed=seed, runs=len(runs))
        row.update(
            {
                key: float(np.median([d[key] for d in details]))
                for key in sorted({k for d in details for k in d})
            }
        )
        rows.append(row)
    return rows


def write_csv(path, rows):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=sorted({k for r in rows for k in r}))
        writer.writeheader()
        writer.writerows(rows)


def hashes(root):
    return {
        str(p.relative_to(root)): common.sha(p)
        for p in sorted((root / "src").rglob("*"))
        if p.is_file() and p.suffix in (".py", ".c", ".so", ".pyd")
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference-source", type=Path, required=True)
    parser.add_argument("--small-repeats", type=int, default=21)
    parser.add_argument("--large-repeats", type=int, default=7)
    parser.add_argument("--profile-repeats", type=int, default=3)
    parser.add_argument("--output", type=Path, default=ROOT / "validation/component-reuse.json.gz")
    args = parser.parse_args()
    reference = load_reference(args.reference_source)
    current_backend = importlib.import_module(
        "sci_solver.solvers.collision_search"
    ).COLLISION_BACKEND
    reference_backend = importlib.import_module(
        "sci_reference.solvers.collision_search"
    ).COLLISION_BACKEND
    if current_backend != "native" or reference_backend != "native":
        parser.error("Both revisions must use the native collision kernel")
    if min(args.small_repeats, args.large_repeats, args.profile_repeats) < 1:
        parser.error("All repetition counts must be positive")
    selected = cases()
    variants = {"full": sci_solver, "before": reference, "after": sci_solver}
    warmups = []
    for case in selected:
        for variant, module in variants.items():
            r, _ = run(module, case, variant)
            warmups.append(
                dict(case_id=case["case_id"], variant=variant, status=r["metrics"]["status"])
            )
    rng = np.random.default_rng(20261017)
    records = []
    for repeat in range(max(args.small_repeats, args.large_repeats)):
        for index in rng.permutation(len(selected)):
            case = selected[index]
            repetitions = (
                args.large_repeats if case["problem"]["n_robots"] == 20 else args.small_repeats
            )
            if repeat >= repetitions:
                continue
            batch = {}
            for variant in rng.permutation(list(variants)):
                variant = str(variant)
                r, positions = run(variants[variant], case, variant)
                r["repeat"] = repeat
                records.append(r)
                batch[variant] = r, positions
                print(
                    f"repeat={repeat} {case['case_id']} {variant}: {r['metrics']['status']} "
                    f"{r['metrics']['timing']['solve_time'] * 1000:.3f} ms",
                    flush=True,
                )
            full, positions = batch["full"]
            for r, samples in batch.values():
                r["cost_relative_difference"] = abs(r["cost"] - full["cost"]) / max(
                    abs(full["cost"]), 1e-15
                )
                r["max_position_difference"] = float(np.max(np.abs(samples - positions)))
    profiling = []
    for repeat in range(args.profile_repeats):
        for case in selected:
            if case["problem"]["n_robots"] == 20:
                r, _ = run(sci_solver, case, "after", profile=True)
                r["repeat"] = repeat
                profiling.append(r)
    report = dict(
        environment=dict(
            python=sys.version,
            platform=common.platform.platform(),
            numpy=np.__version__,
            scipy=scipy.__version__,
            collision_backend=current_backend,
            thread_limits=common.THREAD_LIMITS,
        ),
        protocol=dict(
            small_repeats=args.small_repeats,
            large_repeats=args.large_repeats,
            warmups=1,
            profile_repeats=args.profile_repeats,
            shuffle_seed=20261017,
            reference_commit="5e7a8e3c667f70c7230db2024d9525254eb409f2",
            selection="N=2: first free seed and first two interacting seeds. "
            "N=4: first seed at each observed contact count 0,1,2,3. "
            "N=20: all five previously benchmarked regression cases.",
            timing="Main comparison has detailed profiling disabled; separate N=20 profiling runs. "
            "Solve excludes output sampling and final contact-vector export. "
            "call_seconds includes both, but excludes construction and cost evaluation.",
        ),
        cases=selected,
        warmups=warmups,
        records=records,
        profiling=profiling,
        summary=summarize(records),
        profile_summary=profiles(profiling),
        current_sources=hashes(ROOT),
        reference_sources=hashes(args.reference_source),
        benchmark_sha256=common.sha(Path(__file__)),
        benchmark_helpers_sha256=common.sha(ROOT / "tools/benchmark_partitions.py"),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with gzip.GzipFile(str(args.output), "wb", mtime=0) as stream:
        stream.write(json.dumps(report, indent=2, allow_nan=False).encode())
    base = args.output.with_suffix("").with_suffix("")
    write_csv(base.with_suffix(".csv"), report["summary"])
    write_csv(base.with_name(base.name + "-profile.csv"), report["profile_summary"])
    print(f"Saved {args.output}")


if __name__ == "__main__":
    main()
