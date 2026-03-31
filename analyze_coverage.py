#!/usr/bin/env python3
"""Parse and display coverage report."""

import xml.etree.ElementTree as ET


def _float_attr(value: str | None, default: float = 0.0) -> float:
    return default if value is None else float(value)


def _int_attr(value: str | None, default: int = 0) -> int:
    return default if value is None else int(value)

# Parse the coverage XML
tree = ET.parse("data/logs/coverage.xml")
root = tree.getroot()

# Get overall stats
overall_rate = root.get("line-rate")
lines_valid = root.get("lines-valid")
lines_covered = root.get("lines-covered")

print(f"\nOVERALL COVERAGE: {_float_attr(overall_rate) * 100:.2f}%")
print(f"Lines Covered: {lines_covered} / {lines_valid}")
print(f"Lines Missed: {_int_attr(lines_valid) - _int_attr(lines_covered)}\n")

# Get per-module coverage
print("=" * 80)
print("Coverage by Module:")
print("=" * 80)

modules = {}
for package in root.findall(".//package"):
    for cls in package.findall(".//class"):
        filename = cls.get("filename")
        cls_rate = cls.get("line-rate")
        lines = cls.findall(".//line")
        if lines:
            hits_count = sum(1 for line in lines if int(line.get("hits", 0)) > 0)
            total_lines = len(lines)
            modules[filename] = {
                "rate": float(cls_rate) if cls_rate else 0.0,
                "covered": hits_count,
                "total": total_lines,
            }

# Sort by coverage percentage
sorted_modules = sorted(modules.items(), key=lambda x: x[1]["rate"])

for filename, data in sorted_modules:
    rate_pct = data["rate"] * 100
    print(f"{rate_pct:6.2f}%  {data['covered']:4d}/{data['total']:4d}  {filename}")

print("\n" + "=" * 80)
print("\nLow Coverage Modules (need more tests):")
print("-" * 80)
for filename, data in sorted_modules:
    if data["rate"] < 1.0:  # Less than 100%
        rate_pct = data["rate"] * 100
        print(f"{rate_pct:6.2f}%  {data['covered']:4d}/{data['total']:4d}  {filename}")
