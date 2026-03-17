#!/usr/bin/env python3
"""Quick verification that tests can be imported."""

import sys
import time

print("Testing import...")
start = time.time()

try:
    from tests import test_coverage_expansion

    print(f"✓ test_coverage_expansion imported in {time.time() - start:.2f}s")
except Exception as e:
    print(f"✗ Failed: {e}")
    sys.exit(1)

import inspect

test_count = 0
for name, obj in inspect.getmembers(test_coverage_expansion):
    if inspect.isclass(obj) and name.startswith("Test"):
        for member_name, member in inspect.getmembers(obj):
            if member_name.startswith("test_"):
                test_count += 1

print(f"Found {test_count} test methods")
print("Ready for pytest run")
