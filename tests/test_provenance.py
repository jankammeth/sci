"""Keep the extracted numerical sources traceable in installed distributions."""
import hashlib
import json
from pathlib import Path

import pytest
import sci_solver

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = json.loads((ROOT / "UPSTREAM.json").read_text())


@pytest.mark.parametrize("path,item", MANIFEST["core_files"].items())
def test_installed_numerical_source_matches_extraction_manifest(path, item):
    source = Path(sci_solver.__file__).parent / "solvers" / Path(path).name
    assert hashlib.sha256(source.read_bytes()).hexdigest() == item["extracted_sha256"]
