"""Build the optional continuous collision kernel without NumPy build headers."""
import os
import sys

from setuptools import Extension, setup

extensions = []
if os.environ.get("SCI_NO_NATIVE") != "1":
    extensions.append(
        Extension(
            "sci_solver.solvers._cubic_minima",
            ["src/sci_solver/solvers/_cubic_minima.c"],
            extra_compile_args=(
                ["/O2", "/fp:precise"] if sys.platform == "win32"
                else ["-O3", "-fno-fast-math"]
            ),
            optional=os.environ.get("SCI_REQUIRE_NATIVE") != "1",
        )
    )
setup(ext_modules=extensions)
