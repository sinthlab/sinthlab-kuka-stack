#!/usr/bin/env python3
"""Build the behavioural outcome tables from the recorded sidecars.

    python3 analysis/outcome_tables.py                          # every run in the data folder
    python3 analysis/outcome_tables.py <run folder> [...]       # these runs
    python3 analysis/outcome_tables.py --data-dir /path/to/analysis

For each run folder (YYYYMMDD_subject_task_HHMMSS/) it rewrites trials.csv (one row per trial), any task's extra
tables, and tables.md (what each column means); then it adds or replaces each run's row in sessions.csv
(one row per run, in the data folder).

The recorder writes these live as each trial ends, and the dashboard adds the sessions.csv row when a run
ends. This rebuilds them: for runs recorded before the tables existed, runs started from a terminal,
or after a task's columns change. The tables are views of the sidecars (*.meta.json), which stay the
record; see sinthlab_bringup/helpers/outcome_tables.py for the columns and for how a task adds its own.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "sinthlab_bringup"))
from sinthlab_bringup.helpers import outcome_tables as ot  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("runs", nargs="*", help="run folders (default: every recording folder in the data folder)")
    ap.add_argument("--data-dir", default=None,
                    help=f"the data folder (default: ${ot.DATA_DIR_ENV}, else {ot.DEFAULT_DATA_DIR})")
    args = ap.parse_args(argv)
    data = Path(args.data_dir).expanduser() if args.data_dir else ot.data_dir()
    runs = [Path(r) for r in args.runs] or sorted(ot.run_folders(data))
    if not runs:
        print(f"No run folders in {data}.")
        return 1
    sessions = data / ot.SESSIONS_CSV
    for run in runs:
        n = ot.rebuild_run(run)
        if not n:
            print(f"  {run.name}: no trials")
            continue
        row = ot.update_sessions(run, sessions)
        print(f"  {run.name}: {n} trials -> trials.csv"
              + (f"; {row['n_success']} successful, {row['n_rewards']} rewards" if row else ""))
    print(f"sessions.csv: {sessions}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
