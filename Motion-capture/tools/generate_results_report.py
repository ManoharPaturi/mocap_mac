"""Manual entry point to regenerate consolidated results.md from latest evaluation artifacts."""

from __future__ import annotations

import os
import sys


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from src.results_report import write_consolidated_results_report


def main() -> int:
    out_path = write_consolidated_results_report(repo_root=REPO_ROOT, output_root="results")
    print(f"Consolidated report updated: {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
