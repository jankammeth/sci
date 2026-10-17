# Releasing SCI Solver

Releases are available on [GitHub](https://github.com/jankammeth/sci/releases) and
[PyPI](https://pypi.org/project/sci-solver/). Publishing uses the configured GitHub
trusted publisher and the `pypi` environment's required review.

1. Run the tests and `tools/compare_upstream.py`; review the source and validation
   report. Verify `NOTICE`, `LICENSE`, and `CITATION.cff`.
2. Push the release changes to `jankammeth/sci` on `main` after approval.
   Wait for the Tests workflow to pass on Linux, macOS, and Windows, including
   native, Python-fallback, and minimum-dependency jobs.
3. Record the release version in `pyproject.toml` and `CITATION.cff`. Review the
   release notes, create the matching tag (for example, `v0.2.0`), and push it after
   approval. The tag must point to the tested commit.
4. Run **Build release** manually with that tag and `publish=false`. It builds
   and tests CPython 3.10–3.14 wheels for Linux x86_64, Windows AMD64, and macOS
   arm64/x86_64, plus a source distribution. Native wheel builds must compile
   the C extension; they cannot silently fall back to Python.
5. Verify the PyPI trusted publisher for project `sci-solver`, GitHub
   owner `jankammeth`, repository `sci`, workflow `release.yml`, environment
   `pypi`. Keep that GitHub environment protected by a required reviewer.
6. After publication approval, run the same release workflow with `publish=true`
   and approve its environment deployment. It publishes only after every build
   and artifact test succeeds; it does not need a stored PyPI API token.
7. In a fresh environment, install the released version from PyPI (for example,
   `sci-solver==0.2.0`) and run the README
   example. Confirm `metrics["fleet_mode"] == "components"` without selecting a
   mode, and check `collision_backend` on each advertised wheel platform. Record
   the release URL and pin the paper's solver integration to the released commit.

Ordinary pushes and pull requests never publish packages. The workflow is manual
and publication defaults to false. No publication or hosted CI run is implied by
successful local validation.

## Local builds

```sh
SCI_REQUIRE_NATIVE=1 uv build
```

This produces a source distribution and the native wheel for the current Python
and platform. For a portable fallback build from a clean source tree:

```sh
SCI_NO_NATIVE=1 uv build --wheel
```

Use a fresh extracted source distribution for the fallback build to avoid stale
compiled files. Inspect archives and test installed artifacts outside the checkout.

References: [PyPA packaging](https://packaging.python.org/en/latest/guides/writing-pyproject-toml/),
[cibuildwheel](https://cibuildwheel.pypa.io/en/stable/), and
[PyPI trusted publishing](https://docs.pypi.org/trusted-publishers/using-a-publisher/).
