#!/usr/bin/env python3
import glob
import os
import sys


def find_latest_run(log_root="logs"):
    runs = sorted(
        [d for d in glob.glob(os.path.join(log_root, "run_*")) if os.path.isdir(d)]
    )
    return runs[-1] if runs else None


def analyze(run_dir):
    lf_logs = glob.glob(os.path.join(run_dir, "line_follower_*.log"))
    goal_logs = glob.glob(os.path.join(run_dir, "goal_monitor_*.log"))
    if not lf_logs:
        print("FAIL: no line follower logs")
        return 1
    if not goal_logs:
        print("FAIL: no goal monitor logs")
        return 1

    lf_content = "\n".join(open(f).read() for f in lf_logs)
    goal_content = "\n".join(open(f).read() for f in goal_logs)

    if "line_found" not in lf_content:
        print("FAIL: line was never detected")
        return 2

    if "goal_reached" not in goal_content:
        print("FAIL: goal not reached")
        return 3

    print("SUCCESS: line followed and goal reached")
    return 0


if __name__ == "__main__":
    run_dir = find_latest_run()
    if not run_dir:
        print("FAIL: no run directories")
        sys.exit(1)
    sys.exit(analyze(run_dir))
