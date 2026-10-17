"""Experimental fleet reductions using the existing local matching solver.

Rebuild the contact graph after each insertion/removal. Components have their own
endpoint barycenters and knot sequences; only collision checking uses their union.
No sampled trajectories or time perturbations are used to join solved components.
"""

from collections import Counter
from dataclasses import dataclass
import time

import numpy as np

from .collision_search import COLLISION_BACKEND
from .cubic_utils import cubic_coefficients
from .matching_numerics import insertion_time, damped_newton
from .matching_operations import check_matching, recover_reactions, initial_contact_geometry
from .vector_matching import MatchingSystem, VectorContact
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


@dataclass(frozen=True)
class ContactEvent:
    uid: int
    pair: tuple
    t_star: float
    normal: np.ndarray
    velocity: np.ndarray
    mass: float


@dataclass
class Component:
    robots: tuple
    events: tuple
    endpoints: np.ndarray
    system: MatchingSystem
    x: np.ndarray
    barycenter: np.ndarray
    trajectory: object = None
    checks: dict | None = None
    dirty: bool = True

    def update_events(self, order=None):
        previous = self.events if order is None else tuple(self.events[k] for k in order)
        d = self.system.d
        self.events = tuple(
            ContactEvent(
                event.uid,
                event.pair,
                z[0],
                z[1 : 1 + d].copy(),
                z[1 + d : 1 + 2 * d].copy(),
                z[1 + 2 * d],
            )
            for event, z in zip(previous, self.system.unpack(self.x), strict=True)
        )


def relative_states(trajectory, robots, times, derivative=0):
    """Read only the selected relative coordinates, without absolute positions."""
    if robots[0] == 0:
        return trajectory.relative(times, derivative, np.asarray(robots[1:]) - 1)
    values = trajectory.relative(times, derivative, np.asarray(robots) - 1)
    return values[:, 1:] - values[:, :1]


def initialize_event(owner, pair, time, trajectory, uid):
    position = relative_states(trajectory, pair, [time])[0, 0]
    velocity = relative_states(trajectory, pair, [time], 1)[0, 0]
    normal, velocity = initial_contact_geometry(position, velocity, owner.R)
    return ContactEvent(uid, pair, time, normal, velocity, 0.0)


def assemble_components(owner, contacts, groups, trajectory, previous, endpoints):
    """Rebuild only groups whose contact identities changed, including splits."""
    previous = {c.robots: c for c in previous}
    components, rebuilt, reused = [], [], []
    for robots, indices in groups:
        selected = tuple(sorted((contacts[k] for k in indices), key=lambda c: c.t_star))
        old = previous.get(robots)
        if old is not None and {c.uid for c in selected} == {c.uid for c in old.events}:
            components.append(old)
            reused.append(list(robots))
            continue
        states = endpoints[:, robots, :]
        relative = states[:, :1] - states[:, 1:]
        lookup = {p: j for j, p in enumerate(robots)}
        pairs = [tuple(lookup[p] for p in c.pair) for c in selected]
        system = MatchingSystem(pairs, *relative, owner.R, owner.T, owner.min_gap)
        times = [c.t_star for c in selected]
        r = relative_states(trajectory, robots, times)
        v = relative_states(trajectory, robots, times, 1)
        z = np.empty((len(selected), system.b))
        d = owner.d
        for k, (c, (_, q)) in enumerate(zip(selected, pairs, strict=True)):
            z[k, 0] = c.t_star
            z[k, 1 : 1 + d] = c.normal
            z[k, 1 + d : 1 + 2 * d] = c.velocity
            z[k, 1 + 2 * d] = c.mass
            others = [j for j in range(len(robots) - 1) if j != q - 1]
            z[k, 2 * d + 2 :] = np.stack((r[k, others], v[k, others]), axis=1).ravel()
        center = np.array(cubic_coefficients(*states.mean(axis=1), owner.T))
        components.append(Component(robots, selected, states, system, z.ravel(), center))
        rebuilt.append(list(robots))
    return components, rebuilt, reused


def reconstruct_fleet(owner, components, free, free_path):
    """Assemble relative coefficients once; retain an already identical layout."""
    from .contact_solver import RelativeTrajectory

    if not components:
        return free_path
    if len(components) == 1 and len(components[0].robots) == owner.N:
        return components[0].trajectory
    times = np.unique(np.concatenate([c.trajectory.times for c in components]))
    left = times[:-1]
    relative = shift_cubics(
        np.broadcast_to(free[0] - free[1:], (len(left), owner.N - 1, 4, owner.d)), left
    )
    first_free = shift_cubics(np.broadcast_to(free[0], (len(left), 1, 4, owner.d)), left)[:, 0]
    local = []
    first = first_free
    for component in components:
        path = component.trajectory
        segments = np.searchsorted(path.times, left, side="right") - 1
        c = shift_cubics(path.coefficients[segments], left - path.times[segments])
        center = shift_cubics(np.broadcast_to(path.barycenter, (len(left), 1, 4, owner.d)), left)[
            :, 0
        ]
        local_first = center + c.sum(axis=1) / len(component.robots)
        local.append((component.robots, c, local_first))
        if component.robots[0] == 0:
            first = local_first
    relative += (first - first_free)[:, None]
    for robots, c, local_first in local:
        if robots[0] == 0:
            relative[:, np.asarray(robots[1:]) - 1] = c
        else:
            offset = first - local_first
            relative[:, robots[0] - 1] = offset
            relative[:, np.asarray(robots[1:]) - 1] = offset[:, None] + c
    return RelativeTrajectory(times, relative, free_path.barycenter)


def export_contacts(trajectory, contacts, n_robots):
    """Full-fleet contact vectors are an output format, not solver state."""
    if not contacts:
        return []
    times = [c.t_star for c in contacts]
    r, v = trajectory.relative(times), trajectory.relative(times, 1)
    result = []
    for k, c in enumerate(contacts):
        others = [j for j in range(n_robots - 1) if j != c.pair[1] - 1]
        result.append(
            VectorContact(
                c.pair, c.t_star, c.normal, c.velocity, c.mass, r[k, others], v[k, others]
            )
        )
    return result


def fleet_checks(owner, components, trajectory):
    for c in components:
        if c.checks is None:
            c.checks = check_matching(c.system, c.x, c.trajectory, c.endpoints, owner.options)
    if len(components) == 1 and trajectory is components[0].trajectory:
        return components[0].checks.copy()
    checks = [c.checks for c in components]
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


def solve_components(owner, components, deadline, stages, roots):
    from .contact_solver import RelativeTrajectory

    info = dict(success=True, message="empty_active_set")
    for c in components:
        if not c.dirty:
            continue
        initial = c.x.copy()
        with stages.measure("newton"):
            c.x, info = damped_newton(c.system, c.x, deadline, owner.options)
        roots.append({**info, "robots": c.robots, "system_size": len(c.x)})
        recovered = None
        order = None
        if (
            owner.options.sci_reaction_recovery
            and not info["success"]
            and info["message"] != "timeout"
            and owner.options.sci_root_max_iterations > 0
        ):
            with stages.measure("recovery"):
                recovered = recover_reactions(
                    c.system, initial, c.barycenter, c.endpoints, owner.options, deadline
                )
            if recovered is not None:
                c.system, c.x, c.trajectory, info = recovered
                order = info["contact_order"]
                roots.append({**info, "robots": c.robots})
        if recovered is None:
            with stages.measure("reconstruction"):
                c.trajectory = RelativeTrajectory.from_system(c.system, c.x, c.barycenter)
        c.update_events(order)
        c.dirty = False
        if not info["success"]:
            # Retain validly represented initial iterates for unvisited groups.
            for pending in components:
                if pending.trajectory is None:
                    with stages.measure("reconstruction"):
                        pending.trajectory = RelativeTrajectory.from_system(
                            pending.system, pending.x, pending.barycenter
                        )
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
        from .contact_solver import RelativeTrajectory

        free_path = RelativeTrajectory(
            np.array([0.0, owner.T]), (free[0] - free[1:])[None], free.mean(axis=0)
        )
        trajectory, contacts = free_path, []
        endpoints = np.array(endpoints)
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
        contacts.append(initialize_event(owner, pair, initial_time, trajectory, updates))
        updates += 1
        while True:
            with stages.measure("assembly"):
                components, rebuilt, reused = assemble_components(
                    owner, contacts, groups, trajectory, components, endpoints
                )
            history.append(
                dict(
                    robots=[list(c.robots) for c in components],
                    rebuilt=rebuilt,
                    reused=reused,
                    contacts=[c.system.m for c in components],
                    system_sizes=[len(c.x) for c in components],
                )
            )
            info = solve_components(owner, components, deadline, stages, roots)
            with stages.measure("reconstruction"):
                trajectory = reconstruct_fleet(owner, components, free, free_path)
                contacts = sorted((e for c in components for e in c.events), key=lambda c: c.t_star)
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
    owner.trajectory = trajectory
    solve_time = time.perf_counter() - started
    export_started = time.perf_counter()
    owner.contacts = export_contacts(trajectory, contacts, owner.N)
    export_seconds = time.perf_counter() - export_started
    times, positions, velocities, accelerations = owner.sample()
    owner.trajectories = dict(
        positions=positions, velocities=velocities, accelerations=accelerations
    )
    size = sum(len(c.x) for c in components)
    m = len(contacts)
    return dict(
        trajectories=owner.trajectories,
        times=times,
        contacts=owner.contacts,
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
            component_builds=sum(len(h["rebuilt"]) for h in history),
            component_reuses=sum(len(h["reused"]) for h in history),
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
                contact_export_seconds=export_seconds,
                wall_time=time.perf_counter() - started,
                **stages.result(solve_time),
            ),
        ),
    )
