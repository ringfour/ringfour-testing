#!/usr/bin/env python3
"""Readable summaries of the pytest JUnit XML runs in test-results/.

The raw JUnit XML is awkward to read; this turns it into a per-test table
(status, duration, and the test's own docstring describing what it checks)
plus a cross-run trend. Stdlib only.

    python3 scripts/test_report.py            # newest run, detailed table
    python3 scripts/test_report.py --trend    # pass/fail across all runs
    python3 scripts/test_report.py --file test-results/run-XXXX.xml
    python3 scripts/test_report.py --failures # only failed/errored tests

Test docstrings are read straight from tests/*.py (the source of "what does
this test check"), matched to the XML by function name.
"""

from __future__ import annotations

import argparse
import ast
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ENGINE_DIR = Path(__file__).resolve().parents[1]
RESULTS_DIR = ENGINE_DIR / "test-results"
TESTS_DIR = ENGINE_DIR / "platform_tests"

GREEN, RED, YELLOW, DIM, BOLD, RESET = (
    "\033[32m", "\033[31m", "\033[33m", "\033[2m", "\033[1m", "\033[0m")


def _docstrings() -> dict[str, str]:
    """Map test function name -> first line of its docstring, from tests/."""
    out: dict[str, str] = {}
    for path in sorted(TESTS_DIR.glob("test_*.py")):
        try:
            tree = ast.parse(path.read_text())
        except (OSError, SyntaxError):
            continue
        for node in ast.walk(tree):
            if (isinstance(node, ast.FunctionDef)
                    and node.name.startswith("test_")):
                doc = ast.get_docstring(node)
                if doc:
                    out[node.name] = doc.strip().splitlines()[0]
    return out


def _runs() -> list[Path]:
    return sorted(RESULTS_DIR.glob("*.xml"), key=lambda p: p.stat().st_mtime)


def _cases(xml_path: Path) -> list[dict]:
    """Parse one JUnit XML into a list of testcase dicts."""
    root = ET.parse(xml_path).getroot()
    cases = []
    for case in root.iter("testcase"):
        status, detail = "passed", ""
        for tag in ("failure", "error", "skipped"):
            node = case.find(tag)
            if node is not None:
                status = {"failure": "failed", "error": "error",
                          "skipped": "skipped"}[tag]
                detail = (node.get("message") or "") + "\n" + (node.text or "")
                break
        cases.append({
            "name": case.get("name", ""),
            "classname": case.get("classname", ""),
            "time": float(case.get("time", 0.0)),
            "status": status,
            "detail": detail.strip(),
        })
    return cases


def _base_name(name: str) -> str:
    """Strip a parametrize suffix: test_x[unix] -> test_x."""
    return name.split("[", 1)[0]


def _color(status: str) -> str:
    return {"passed": GREEN, "failed": RED, "error": RED,
            "skipped": YELLOW}.get(status, "")


def _icon(status: str) -> str:
    return {"passed": "PASS", "failed": "FAIL", "error": "ERR ",
            "skipped": "SKIP"}.get(status, status)


def report_run(xml_path: Path, only_failures: bool) -> None:
    docs = _docstrings()
    cases = _cases(xml_path)
    root = ET.parse(xml_path).getroot().find("testsuite")
    total_time = float(root.get("time", 0.0)) if root is not None else 0.0
    stamp = root.get("timestamp", "?") if root is not None else "?"

    counts: dict[str, int] = {}
    for c in cases:
        counts[c["status"]] = counts.get(c["status"], 0) + 1

    print(f"{BOLD}{xml_path.name}{RESET}  ({stamp})")
    summary = "  ".join(
        f"{_color(s)}{counts[s]} {s}{RESET}"
        for s in ("passed", "failed", "error", "skipped") if s in counts)
    print(f"  {summary}   {DIM}{len(cases)} tests in {total_time:.2f}s{RESET}\n")

    shown = [c for c in cases
             if not only_failures or c["status"] in ("failed", "error")]
    if not shown:
        print(f"  {DIM}(no failures){RESET}")
        return
    # Group by module (classname), slowest first within each group.
    by_module: dict[str, list[dict]] = {}
    for c in shown:
        by_module.setdefault(c["classname"], []).append(c)
    for module in sorted(by_module):
        print(f"{DIM}{module}{RESET}")
        for c in sorted(by_module[module], key=lambda x: -x["time"]):
            doc = docs.get(_base_name(c["name"]), "")
            print(f"  {_color(c['status'])}{_icon(c['status'])}{RESET} "
                  f"{c['time']:5.2f}s  {c['name']}")
            if doc:
                print(f"            {DIM}{doc}{RESET}")
            if c["detail"] and c["status"] in ("failed", "error"):
                tail = c["detail"].splitlines()[-6:]
                for ln in tail:
                    print(f"            {RED}{ln[:100]}{RESET}")
        print()


def report_trend() -> None:
    runs = _runs()
    if not runs:
        print("no runs in test-results/")
        return
    print(f"{BOLD}Test runs (oldest first){RESET}\n")
    print(f"  {'run':<26} {'tests':>5} {'pass':>5} {'fail':>5} "
          f"{'skip':>5} {'time':>7}")
    for path in runs:
        cases = _cases(path)
        p = sum(c["status"] == "passed" for c in cases)
        f = sum(c["status"] in ("failed", "error") for c in cases)
        s = sum(c["status"] == "skipped" for c in cases)
        t = sum(c["time"] for c in cases)
        color = RED if f else GREEN
        print(f"  {color}{path.name:<26}{RESET} {len(cases):>5} {p:>5} "
              f"{f:>5} {s:>5} {t:>6.1f}s")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--trend", action="store_true",
                        help="pass/fail counts across every run")
    parser.add_argument("--failures", action="store_true",
                        help="show only failed/errored tests")
    parser.add_argument("--file", type=Path, default=None,
                        help="a specific JUnit XML (default: newest run)")
    args = parser.parse_args()

    if args.trend:
        report_trend()
        return
    xml_path = args.file or (max(_runs(), default=None))
    if xml_path is None:
        print("no runs in test-results/ yet — run pytest first")
        sys.exit(1)
    report_run(xml_path, only_failures=args.failures)


if __name__ == "__main__":
    main()
