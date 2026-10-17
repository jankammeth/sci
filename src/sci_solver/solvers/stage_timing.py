"""Disjoint wall-clock stages for comparing matching strategies."""

from contextlib import contextmanager
from time import perf_counter


class StageTiming:
    def __init__(self):
        self.seconds = dict.fromkeys(
            (
                "partition",
                "assembly",
                "newton",
                "recovery",
                "reconstruction",
                "validation",
                "collision_search",
            ),
            0.0,
        )
        self.calls = dict.fromkeys(self.seconds, 0)

    @contextmanager
    def measure(self, stage):
        started = perf_counter()
        try:
            yield
        finally:
            self.seconds[stage] += perf_counter() - started
            self.calls[stage] += 1

    def result(self, solve_seconds):
        return dict(
            stage_seconds={
                **self.seconds,
                "other": max(0.0, solve_seconds - sum(self.seconds.values())),
            },
            stage_calls=self.calls.copy(),
        )
