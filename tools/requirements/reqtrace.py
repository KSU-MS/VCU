#!/usr/bin/env python3
"""Which tests cover which rule requirements (docs/requirements.md).

A requirement is a `### <ID> — <title>` heading there. A test cites one by
putting `[<ID>]` in its name: doctest TEST_CASE names in test/, pytest
function docstrings or names elsewhere don't count, only the bracketed tag in
the source line that defines the test.

    uv run tools/requirements/reqtrace.py        # the matrix, as markdown
"""
from __future__ import annotations

import re
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SPEC = ROOT / "docs" / "requirements.md"

RULE_ID = r"(?:T|EV|IC|D|IN|F|V|GR|AD)\.\d+(?:\.\d+)+"
HEADING = re.compile(rf"^###\s+({RULE_ID})\s+[—-]\s+(.*)$")
TAG = re.compile(rf"\[({RULE_ID})\]")
# doctest TEST_CASE("...") and pytest `def test_...` with a tagged docstring
TEST_DEF = re.compile(r'TEST_CASE\("((?:[^"\\]|\\.)*)"|^\s*"""(.*)"""', re.M)


def requirements() -> dict[str, str]:
    reqs = {}
    for line in SPEC.read_text().splitlines():
        m = HEADING.match(line)
        if m:
            reqs[m.group(1)] = m.group(2).strip()
    return reqs


def test_sources() -> list[Path]:
    files = sorted((ROOT / "test").rglob("*.cpp"))
    for d in ("tools", "renode"):
        files += sorted((ROOT / d).rglob("test_*.py"))
    return files


def citations() -> dict[str, list[str]]:
    """rule ID -> ["file:line test name", ...]"""
    cited: dict[str, list[str]] = defaultdict(list)
    for path in test_sources():
        text = path.read_text()
        for m in TEST_DEF.finditer(text):
            name = m.group(1) or m.group(2) or ""
            line = text.count("\n", 0, m.start()) + 1
            for rule in TAG.findall(name):
                cited[rule].append(f"{path.relative_to(ROOT)}:{line} {name}")
    return cited


def main() -> int:
    reqs, cited = requirements(), citations()
    print("| Requirement | Tests |\n|---|---|")
    for rule, title in reqs.items():
        tests = cited.get(rule, [])
        print(f"| **{rule}** {title} | {len(tests)} |")
    for rule, title in reqs.items():
        print(f"\n### {rule} {title}")
        for t in cited.get(rule, []) or ["(none)"]:
            print(f"- {t}")
    unknown = sorted(set(cited) - set(reqs))
    if unknown:
        print(f"\nCited but not in {SPEC.relative_to(ROOT)}: {', '.join(unknown)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
