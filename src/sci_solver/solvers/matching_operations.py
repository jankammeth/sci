"""Numerical contact operations shared by fleet and component drivers."""

import numpy as np

from .reaction_matching import ReactionMatching
from .vector_matching import MatchingSystem, VectorContact


def initial_contact_geometry(relative_position, relative_velocity, radius):
    distance = np.linalg.norm(relative_position)
    if distance > 1e-12 * max(1.0, radius):
        normal = relative_position / distance
    else:
        speed = np.linalg.norm(relative_velocity)
        normal = np.zeros(len(relative_position))
        if speed > 1e-12:
            direction = relative_velocity / speed
            normal[np.argmin(np.abs(direction))] = 1
            normal -= (normal @ direction) * direction
            normal /= np.linalg.norm(normal)
        else:
            normal[0] = 1
    velocity = relative_velocity - (normal @ relative_velocity) * normal
    return normal, velocity


def recover_reactions(system, initial, barycenter, endpoints, options, deadline):
    from .contact_solver import RelativeTrajectory

    reduced = ReactionMatching(system)
    try:
        result = reduced.solve(initial, deadline, max(100, 20 * len(initial)))
        times, coefficients, order = reduced.coefficients(result.x)
        if (
            not result.success
            or not np.all(np.isfinite(result.x))
            or np.min(np.diff(times)) <= 0
            or np.min(np.diff(times)) < system.min_gap
        ):
            return None
        trajectory = RelativeTrajectory(times, coefficients, barycenter, True)
        states = result.x.reshape(system.m, system.d + 2)
        relative = trajectory.relative(times[1:-1])
        velocities = trajectory.relative(times[1:-1], 1)
        contacts = []
        for k, index in enumerate(order):
            p, q = system.pairs[index]
            others = [j for j in range(system.N - 1) if j != q - 1]
            velocity = velocities[k, q - 1] - (velocities[k, p - 1] if p else 0)
            contacts.append(
                VectorContact(
                    (p, q),
                    states[index, 0],
                    states[index, 1 : 1 + system.d],
                    velocity,
                    states[index, -1],
                    relative[k, others],
                    velocities[k, others],
                )
            )
        recovered = MatchingSystem(
            [c.pair for c in contacts],
            system.r0,
            system.v0,
            system.rf,
            system.vf,
            system.R,
            system.T,
            system.min_gap,
        )
        x = np.concatenate([c.to_vector() for c in contacts])
        checks = check_matching(recovered, x, trajectory, endpoints, options)
        # Negative masses are handled by the active-set loop, not accepted.
        if (
            checks["residual"] > options.sci_residual_tol
            or checks["endpoint_error"] > options.sci_residual_tol
        ):
            return None
        info = dict(
            success=True,
            message="reaction_recovery",
            termination_status=int(result.status),
            termination_message=str(result.message),
            nfev=result.nfev,
            iterations=None,
            sparse_fallbacks=0,
            reordered=bool(np.any(order != np.arange(system.m))),
            contact_order=order.tolist(),
            system_size=system.m * (system.d + 2),
        )
        return recovered, x, trajectory, info
    except (TimeoutError, ValueError, FloatingPointError):
        return None


def check_matching(system, x, trajectory, endpoints, options, endpoint_states=None):
    z = system.unpack(x) if system.m else x.reshape(0, system.b)
    md = (system.N - 1) * system.d
    if trajectory.reaction_constructed:
        residual = np.zeros((system.m, system.b))
        h = np.diff(trajectory.times)[:-1, None, None]
        c = trajectory.coefficients
        residual[:, :md] = (2 * c[1:, :, 2] - 2 * c[:-1, :, 2] - 6 * c[:-1, :, 3] * h).reshape(
            system.m, md
        )
        jump = 6 * (c[1:, :, 3] - c[:-1, :, 3])
        mass_normal = z[:, 1 + 2 * system.d, None] * z[:, 1 : 1 + system.d]
        residual[:, md : 2 * md] = (
            jump - system.R * system.jerk_factors[:, :, None] * mass_normal[:, None]
        ).reshape(system.m, md)
        residual[:, -2] = np.sum(z[:, 1 : 1 + system.d] ** 2, axis=1) - 1
        residual[:, -1] = np.sum(
            z[:, 1 : 1 + system.d] * z[:, 1 + system.d : 1 + 2 * system.d], axis=1
        )
        r = trajectory.relative(trajectory.times[1:-1])
        contact_error = max(
            np.max(
                np.abs(r[k, q - 1] - (r[k, p - 1] if p else 0) - system.R * z[k, 1 : 1 + system.d])
            )
            for k, (p, q) in enumerate(system.pairs)
        )
    else:
        residual = (
            system.evaluate(x).reshape(system.m, system.b) if system.m else np.empty((0, system.b))
        )
        contact_error = 0.0

    def norm(a):
        return float(np.max(np.abs(a))) if a.size else 0.0

    if endpoint_states is None:
        endpoint_states = trajectory.endpoint_states()
    endpoint_error = max(
        norm(states[:, endpoint] - target.reshape(system.N, system.d))
        for states, targets in zip(
            endpoint_states,
            (
                (endpoints[0], endpoints[2]),
                (endpoints[1], endpoints[3]),
            ),
            strict=True,
        )
        for endpoint, target in enumerate(targets)
    )
    checks = dict(
        residual=max(norm(residual), contact_error),
        contact_position_error=contact_error,
        acceleration_jump=norm(residual[:, :md]),
        jerk_balance=norm(residual[:, md : 2 * md]),
        normalization=norm(residual[:, -2]),
        tangency=norm(residual[:, -1]),
        endpoint_error=endpoint_error,
        minimum_mass=float(np.min(z[:, 1 + 2 * system.d])) if len(z) else 0.0,
        minimum_knot_gap=(float(np.min(np.diff(trajectory.times))) if system.m else system.T),
    )
    checks["valid"] = bool(
        checks["residual"] <= options.sci_residual_tol
        and endpoint_error <= options.sci_residual_tol
        and checks["minimum_mass"] >= -options.sci_mass_tol
        and checks["minimum_knot_gap"] >= system.min_gap
    )
    return checks
