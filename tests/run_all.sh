#!/usr/bin/env bash
# One entry point for every safety net in this repo. ROS not required.
#   bash tests/run_all.sh            (dev: python3 with numpy+pytest)
#   PYTHON=python2.7 bash tests/run_all.sh   (board)
set -u
cd "$(dirname "$0")/.."
PYTHON="${PYTHON:-python3}"
fail=0
bash tests/characterization/run.sh || fail=1
for d in robot/albc_rl/numpy_port robot/albc_rl/scripts robot/hero_agent/scripts; do
    echo "=== pytest $d ==="
    (cd "$d" && "$PYTHON" -m pytest -q -p no:cacheprovider) || fail=1
done
[ "$fail" -eq 0 ] && echo "RUN_ALL: PASS" || { echo "RUN_ALL: FAIL"; exit 1; }
