#!/usr/bin/env python3
"""Run tests and capture coverage."""

import subprocess
import sys

# Run pytest with coverage
result = subprocess.run(
    [
        sys.executable,
        "-m",
        "pytest",
        "tests/",
        "--cov=src",
        "--cov-report=term-missing",
        "--cov-report=xml:data/logs/coverage.xml",
        "-q",
    ],
    timeout=1200,
    cwd=".",
)

sys.exit(result.returncode)
