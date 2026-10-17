"""Solver configuration, independent of experiment and plotting dependencies."""
from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
import numpy as np

@dataclass
class EnvironmentConfig:
    pos_x_min: float | None = None
    pos_x_max: float | None = None
    pos_y_min: float | None = None
    pos_y_max: float | None = None
    # Full-dimensional alternatives to the legacy x/y fields; do not mix them.
    position_lower: list[float] | None = None
    position_upper: list[float] | None = None

    def bounds(self, dimension: int) -> tuple[np.ndarray, np.ndarray]:
        if dimension < 2:
            raise ValueError("Position bounds require dimension >= 2")
        legacy = (self.pos_x_min, self.pos_y_min, self.pos_x_max, self.pos_y_max)
        if (self.position_lower is not None or self.position_upper is not None) and any(
            value is not None for value in legacy
        ):
            raise ValueError("Do not mix position_lower/upper with legacy x/y bounds")
        lower, upper = np.full(dimension, -np.inf), np.full(dimension, np.inf)
        for axis, value in enumerate(legacy[:2]):
            if value is not None:
                lower[axis] = value
        for axis, value in enumerate(legacy[2:]):
            if value is not None:
                upper[axis] = value
        for values, target in ((self.position_lower, lower), (self.position_upper, upper)):
            if values is not None:
                array = np.asarray(values, dtype=float)
                if array.shape != (dimension,):
                    raise ValueError("Position bounds must have one entry per spatial dimension")
                target[:] = array
        if np.any(np.isnan(lower)) or np.any(np.isnan(upper)) or np.any(lower > upper):
            raise ValueError("Invalid position bounds")
        return lower, upper

    @property
    def pos_min(self) -> np.ndarray:
        dimension = len(
            self.position_lower
            if self.position_lower is not None
            else self.position_upper if self.position_upper is not None else [0, 0]
        )
        return self.bounds(dimension)[0]

    @property
    def pos_max(self) -> np.ndarray:
        dimension = len(
            self.position_lower
            if self.position_lower is not None
            else self.position_upper if self.position_upper is not None else [0, 0]
        )
        return self.bounds(dimension)[1]

@dataclass
class DynamicsConfig:
    vel_min: float | None = None
    vel_max: float | None = None
    acc_min: float | None = None
    acc_max: float | None = None
    jerk_min: float | None = None
    jerk_max: float | None = None

@dataclass
class ProblemConfig:
    n_robots: int
    min_distance: float
    time_horizon: float | None = None
    dimension: int = 2
    timestep: float = 0.1
    environment: EnvironmentConfig = field(default_factory=EnvironmentConfig)
    dynamics: DynamicsConfig = field(default_factory=DynamicsConfig)

    def __post_init__(self):
        if isinstance(self.n_robots, bool) or not isinstance(self.n_robots, int) or self.n_robots < 2:
            raise ValueError("n_robots must be an integer >= 2")
        if isinstance(self.dimension, bool) or not isinstance(self.dimension, int) or self.dimension < 2:
            raise ValueError("dimension must be an integer >= 2")
        if not np.isfinite(self.timestep) or self.timestep <= 0:
            raise ValueError("timestep must be positive and finite")
        if self.time_horizon is not None and (not np.isfinite(self.time_horizon) or self.time_horizon <= 0):
            raise ValueError("time_horizon must be positive and finite")

    @property
    def n_timesteps(self) -> int | None:
        return None if self.time_horizon is None else int(self.time_horizon / self.timestep)


@dataclass
class SolverConfig:
    sci_detection_tol: float = 0.0
    sci_min_contact_gap: float = 0.0
    sci_residual_tol: float = 1e-7
    sci_mass_tol: float = 1e-9
    sci_root_max_iterations: int = 100
    sci_reaction_recovery: bool = True
    timeout: float = 300.0
    use_analytical_jacobian: bool = True

    def __post_init__(self):
        if (isinstance(self.sci_root_max_iterations, bool)
                or not isinstance(self.sci_root_max_iterations, int)
                or self.sci_root_max_iterations < 0):
            raise ValueError("sci_root_max_iterations must be a nonnegative integer")


@dataclass
class Config:
    problem: ProblemConfig
    solver: SolverConfig = field(default_factory=SolverConfig)
    name: str = "sci"

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, values: dict) -> "Config":
        values = dict(values)
        problem = dict(values.pop("problem"))
        for key, schema in (("environment", EnvironmentConfig), ("dynamics", DynamicsConfig)):
            if key in problem:
                problem[key] = schema(**problem[key])
        return cls(problem=ProblemConfig(**problem),
                   solver=SolverConfig(**values.pop("solver", {})), **values)

    @classmethod
    def from_json(cls, path: str | Path) -> "Config":
        return cls.from_dict(json.loads(Path(path).read_text()))

    def to_json(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.to_dict(), indent=2, allow_nan=False) + "\n")
