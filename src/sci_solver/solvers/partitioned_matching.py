"""Experimental fleet reductions using the existing local matching solver.

Rebuild the contact graph after each insertion/removal. Components have their own
endpoint barycenters and knot sequences; only collision checking uses their union.
No sampled trajectories or time perturbations are used to join solved components.
"""

from collections import Counter
from dataclasses import dataclass, replace
import time

import numpy as np

from .collision_search import COLLISION_BACKEND
from .cubic_utils import cubic_coefficients
from .matching_numerics import insertion_time
from .stage_timing import StageTiming


def contact_groups(pairs, mode):
    """Return (sorted robot IDs, contact indices), in deterministic order.

    Union by size and path compression; O(m alpha(A)) graph work and O(A)
    storage. Sorting robot IDs also makes local pair orientation deterministic.
    Isolated robots do not enter the data structure.
    """
    if not pairs:
        return []
    if mode == "active":
        return [(tuple(sorted({p for pair in pairs for p in pair})), tuple(range(len(pairs))))]
    parent, size = {}, {}

    def find(p):
        while p != parent[p]:
            parent[p] = parent[parent[p]]
            p = parent[p]
        return p

    for p, q in pairs:
        for r in (p, q):
            if r not in parent:
                parent[r], size[r] = r, 1
        a, b = find(p), find(q)
        if a != b:
            if size[a] < size[b]:
                a, b = b, a
            parent[b] = a
            size[a] += size[b]
    robots, indices = {}, {}
    for p in sorted(parent):
        robots.setdefault(find(p), []).append(p)
    for k, (p, _) in enumerate(pairs):
        indices.setdefault(find(p), []).append(k)
    return [(tuple(ids), tuple(indices[root])) for root, ids in robots.items()]


def shift_cubics(coefficients, offsets):
    """Translate local polynomial origins without refitting endpoint data."""
    a, b, c, d = np.moveaxis(coefficients, -2, 0)
    h = np.asarray(offsets)[..., None, None]
    return np.stack(
        (
            ((d * h + c) * h + b) * h + a,
            (3 * d * h + 2 * c) * h + b,
            c + 3 * d * h,
            d + np.zeros_like(a),
        ),
        axis=-2,
    )


@dataclass
class Component:
    robots: tuple
    solver: object
    system: object
    x: np.ndarray
    trajectory: object

    def global_contacts(self):
        return [
            replace(c, pair=tuple(self.robots[p] for p in c.pair))
            for c in self.system.contacts(self.x)
        ]


def assemble_components(owner, contacts, groups, trajectory):
    # Import locally to keep ContactSolver's dispatch free of import cycles.
    from .contact_solver import ContactSolver, RelativeTrajectory
    from ..config import Config, ProblemConfig

    endpoints = [
        a.reshape(owner.N, owner.d)
        for a in (
            owner.initial_positions,
            owner.initial_velocities,
            owner.final_positions,
            owner.final_velocities,
        )
    ]
    components = []
    for robots, indices in groups:
        local = ContactSolver(
            Config(
                problem=ProblemConfig(
                    len(robots), owner.R, owner.T, dimension=owner.d, timestep=owner.h
                ),
                solver=owner.options,
            )
        )
        local.min_gap = owner.min_gap
        states = [a[list(robots)] for a in endpoints]
        local.set_initial_states(*states[:2])
        local.set_final_states(*states[2:])
        lookup = {p: j for j, p in enumerate(robots)}
        selected = sorted((contacts[k] for k in indices), key=lambda c: c.t_star)
        times = [c.t_star for c in selected]
        positions = trajectory.robots(times)[list(robots)].transpose(1, 0, 2)
        velocities = trajectory.robots(times, 1)[list(robots)].transpose(1, 0, 2)
        r = positions[:, :1] - positions[:, 1:]
        v = velocities[:, :1] - velocities[:, 1:]
        local_contacts = []
        for k, c in enumerate(selected):
            pair = tuple(lookup[p] for p in c.pair)
            others = [j for j in range(len(robots) - 1) if j != pair[1] - 1]
            local_contacts.append(
                replace(c, pair=pair, r_others=r[k, others], v_others=v[k, others])
            )
        system = local.matching_system(local_contacts)
        x = np.concatenate([c.to_vector() for c in local_contacts])
        center = np.array(cubic_coefficients(*[a.mean(axis=0) for a in states], owner.T))
        path = RelativeTrajectory.from_system(system, x, center)
        components.append(Component(robots, local, system, x, path))
    return components


def reconstruct_fleet(owner, components, free):
    from .contact_solver import RelativeTrajectory

    times = np.unique(np.concatenate(([0.0, owner.T], *[c.trajectory.times for c in components])))
    left = times[:-1]
    robot_coefficients = shift_cubics(np.broadcast_to(free, (len(left), *free.shape)), left)
    for component in components:
        path = component.trajectory
        segments = np.searchsorted(path.times, left, side="right") - 1
        relative = shift_cubics(path.coefficients[segments], left - path.times[segments])
        center = shift_cubics(np.broadcast_to(path.barycenter, (len(left), 1, 4, owner.d)), left)[
            :, 0
        ]
        first = center + relative.sum(axis=1) / len(component.robots)
        robot_coefficients[:, component.robots] = np.concatenate(
            (first[:, None], first[:, None] - relative), axis=1
        )
    relative = robot_coefficients[:, :1] - robot_coefficients[:, 1:]
    path = RelativeTrajectory(times, relative, free.mean(axis=0))
    contacts = sorted(
        (c for block in components for c in block.global_contacts()), key=lambda c: c.t_star
    )
    if contacts:
        r = path.relative([c.t_star for c in contacts])
        v = path.relative([c.t_star for c in contacts], 1)
        # Public contact vectors keep the original fleet indexing/shape.
        contacts = [
            replace(
                c,
                r_others=r[k, [j for j in range(owner.N - 1) if j != c.pair[1] - 1]],
                v_others=v[k, [j for j in range(owner.N - 1) if j != c.pair[1] - 1]],
            )
            for k, c in enumerate(contacts)
        ]
    return path, contacts


def fleet_checks(owner, components, trajectory):
    checks = [c.solver._checks(c.system, c.x, c.trajectory) for c in components]
    result = {
        key: max((c[key] for c in checks), default=0.0)
        for key in (
            "residual",
            "contact_position_error",
            "acceleration_jump",
            "jerk_balance",
            "normalization",
            "tangency",
            "endpoint_error",
        )
    }
    result["minimum_mass"] = min((c["minimum_mass"] for c in checks), default=0.0)
    # Knot restrictions are local. Independent components can meet simultaneously.
    result["minimum_knot_gap"] = min((c["minimum_knot_gap"] for c in checks), default=owner.T)
    endpoints = trajectory.endpoint_states()
    error = max(
        float(np.max(np.abs(endpoints[order, :, end] - target.reshape(owner.N, owner.d))))
        for order, targets in enumerate(
            (
                (owner.initial_positions, owner.final_positions),
                (owner.initial_velocities, owner.final_velocities),
            )
        )
        for end, target in enumerate(targets)
    )
    result["endpoint_error"] = max(result["endpoint_error"], error)
    result["valid"] = bool(
        all(c["valid"] for c in checks) and error <= owner.options.sci_residual_tol
    )
    return result


def solve_components(components, deadline, stages, roots):
    from .contact_solver import RelativeTrajectory

    info = dict(success=True, message="empty_active_set")
    for c in components:
        initial = c.x.copy()
        with stages.measure("newton"):
            c.x, info = c.solver._solve_system(c.system, c.x, deadline)
        roots.append({**info, "robots": c.robots, "system_size": len(c.x)})
        recovered = None
        if (
            c.solver.options.sci_reaction_recovery
            and not info["success"]
            and info["message"] != "timeout"
            and c.solver.options.sci_root_max_iterations > 0
        ):
            with stages.measure("recovery"):
                recovered = c.solver._recover_reactions(
                    c.system, initial, c.trajectory.barycenter, deadline
                )
            if recovered is not None:
                c.system, c.x, c.trajectory, info = recovered
                roots.append({**info, "robots": c.robots})
        if recovered is None:
            with stages.measure("reconstruction"):
                c.trajectory = RelativeTrajectory.from_system(
                    c.system, c.x, c.trajectory.barycenter
                )
        if not info["success"]:
            break
    return info


def generate_partitioned(owner, verbose=None):
    """Shared insertion/removal loop for active-robot and component reductions."""
    started = time.perf_counter()
    stages = StageTiming()
    if owner.initial_positions is None or owner.final_positions is None:
        raise ValueError("Set initial and final states before solving")
    if owner.T is None or not np.isfinite(owner.T) or owner.T <= 0:
        raise ValueError("Time horizon must be positive and finite")
    owner.min_gap = owner.options.sci_min_contact_gap
    if owner.T < owner.min_gap:
        raise ValueError("Invalid empty contact state or minimum gap")
    verbose = owner.verbose if verbose is None else verbose
    mode = owner.fleet_mode
    deadline = started + owner.options.timeout
    with stages.measure("reconstruction"):
        endpoints = [
            a.reshape(owner.N, owner.d)
            for a in (
                owner.initial_positions,
                owner.initial_velocities,
                owner.final_positions,
                owner.final_velocities,
            )
        ]
        free = np.array(cubic_coefficients(*endpoints, owner.T)).transpose(1, 0, 2)
        components = []
        trajectory, contacts = reconstruct_fleet(owner, components, free)
    roots, removed, offsets, history = [], [], [], []
    updates = 0
    status, reason = "success", "converged"
    while True:
        with stages.measure("collision_search"):
            closest = trajectory.closest()
        with stages.measure("validation"):
            checks = fleet_checks(owner, components, trajectory)
        if not checks["valid"]:
            status, reason = "invalid_solution", "invalid_matching_solution"
            break
        pair, contact_time, distance = closest
        if distance >= owner.R - owner.options.sci_detection_tol:
            break
        p, q = np.triu_indices(owner.N, 1)
        endpoint_positions = trajectory.endpoint_states()[0]
        if np.min(np.linalg.norm(endpoint_positions[p] - endpoint_positions[q], axis=-1)) < (
            owner.R - owner.options.sci_detection_tol
        ):
            status, reason = "invalid_solution", "infeasible_endpoint_separation"
            break
        if time.perf_counter() >= deadline:
            status, reason = "unresolved", "timeout"
            break
        if owner.contact_selection == "earliest":
            with stages.measure("collision_search"):
                pair, contact_time, _ = trajectory.first_violation(
                    owner.R - owner.options.sci_detection_tol
                )
        with stages.measure("partition"):
            groups = contact_groups([c.pair for c in contacts] + [pair], mode)
        # A new edge can merge components. Do not perturb already solved times
        # if the merged local Hermite problem cannot represent them.
        indices = next(indices for _, indices in groups if len(contacts) in indices)
        existing = sorted(contacts[k].t_star for k in indices if k < len(contacts))
        gaps = np.diff([0.0, *existing, owner.T])
        if np.any(gaps <= 0) or np.any(gaps < owner.min_gap):
            status, reason = "unresolved", "merged_component_knot_gap"
            break
        try:
            initial_time = insertion_time(contact_time, existing, owner.T)
        except ValueError:
            status, reason = "unresolved", "coincident_initialization_failed"
            break
        if (
            not 0 < initial_time < owner.T
            or np.min(np.abs(np.array([0.0, *existing, owner.T]) - initial_time)) < owner.min_gap
        ):
            status, reason = "unresolved", "minimum_knot_gap"
            break
        if initial_time != contact_time:
            offsets.append(dict(pair=pair, collision_time=contact_time, initial_time=initial_time))
        contacts.append(owner.initialize_contact(pair, initial_time, trajectory))
        updates += 1
        while True:
            with stages.measure("assembly"):
                components = assemble_components(owner, contacts, groups, trajectory)
            history.append(
                dict(
                    robots=[list(c.robots) for c in components],
                    contacts=[c.system.m for c in components],
                    system_sizes=[len(c.x) for c in components],
                )
            )
            info = solve_components(components, deadline, stages, roots)
            with stages.measure("reconstruction"):
                trajectory, contacts = reconstruct_fleet(owner, components, free)
            if not info["success"] or not contacts:
                break
            index = min(range(len(contacts)), key=lambda k: contacts[k].mass)
            if contacts[index].mass >= -owner.options.sci_mass_tol:
                break
            if time.perf_counter() >= deadline:
                info = dict(success=False, message="timeout")
                break
            c = contacts.pop(index)
            removed.append(dict(pair=c.pair, time=c.t_star, mass=float(c.mass)))
            updates += 1
            with stages.measure("partition"):
                groups = contact_groups([c.pair for c in contacts], mode)
        if verbose:
            print(f"ContactSolver ({mode}): m={len(contacts)}, root={info['message']}")
        if not info["success"]:
            status, reason = "root_failure", info["message"]
            with stages.measure("validation"):
                checks = fleet_checks(owner, components, trajectory)
            with stages.measure("collision_search"):
                closest = trajectory.closest()
            break
    owner.matching_components = components
    owner.system = tuple(c.system for c in components)
    owner.solution = np.concatenate([c.x for c in components]) if components else np.empty(0)
    owner.trajectory, owner.contacts = trajectory, contacts
    solve_time = time.perf_counter() - started
    times, positions, velocities, accelerations = owner.sample()
    owner.trajectories = dict(
        positions=positions, velocities=velocities, accelerations=accelerations
    )
    size = sum(len(c.x) for c in components)
    m = len(contacts)
    return dict(
        trajectories=owner.trajectories,
        times=times,
        contacts=contacts,
        contact_times=[c.t_star for c in contacts],
        metrics=dict(
            converged=status == "success",
            status=status,
            convergence_reason=reason,
            collision_backend=COLLISION_BACKEND,
            dimension=owner.d,
            num_contacts=m,
            min_distance=closest[2],
            closest_pair=closest[0],
            closest_time=closest[1],
            checks=checks,
            roots=roots,
            reaction_recovery_enabled=owner.options.sci_reaction_recovery,
            contact_selection=owner.contact_selection,
            analytical_jacobian=owner.options.use_analytical_jacobian,
            removed_contacts=removed,
            insertion_offsets=offsets,
            active_set_updates=updates,
            intrinsic_degrees_of_freedom=size - 2 * m,
            system_size=size,
            max_contacts_single_pair=max(Counter(c.pair for c in contacts).values(), default=0),
            fleet_mode=mode,
            partition_history=history,
            matching_seconds=stages.seconds["newton"] + stages.seconds["recovery"],
            matching_solves=len(roots),
            nfev=sum(r["nfev"] for r in roots),
            linear_solver="banded Newton / sparse LSMR fallback",
            tolerances=dict(
                residual=owner.options.sci_residual_tol,
                mass=owner.options.sci_mass_tol,
                separation=owner.options.sci_detection_tol,
                minimum_knot_gap=owner.min_gap,
            ),
            timing=dict(
                total_time=solve_time,
                solve_time=solve_time,
                postprocess_time=time.perf_counter() - started - solve_time,
                wall_time=time.perf_counter() - started,
                **stages.result(solve_time),
            ),
        ),
    )
