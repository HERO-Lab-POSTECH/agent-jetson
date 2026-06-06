#!/usr/bin/env bash
# Characterization test runner — builds & runs every golden test with the
# local C++ compiler (no ROS / no catkin needed). Returns non-zero if any
# test fails. Run before AND after the redesign; outputs must stay identical.
#
#   tests/characterization/run.sh
set -u

cd "$(dirname "$0")"
CXX="${CXX:-c++}"
FLAGS="-std=c++11 -Wall -I. -I../../robot/hero_agent/include -I../../robot/albc_control/include"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

fail=0
for src in test_*.cpp; do
    bin="$TMP/${src%.cpp}"
    if ! "$CXX" $FLAGS "$src" -o "$bin" 2>"$TMP/err"; then
        echo "=== $src: COMPILE FAILED ==="
        cat "$TMP/err"
        fail=1
        continue
    fi
    echo "=== $src ==="
    if ! "$bin"; then
        fail=1
    fi
    echo
done

if [ "$fail" -ne 0 ]; then
    echo "RESULT: FAIL — behavior diverged from golden"
    exit 1
fi
echo "RESULT: PASS — all characterization tests green"
