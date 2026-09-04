#!/usr/bin/env bash
# Board gate runner on ROS1 Lunar Jetson TX2 (agent-jetson) to validate build, tests, dry launches, and firmware.
set -euo pipefail

# Fail helper reporting failing gate letter and exiting immediately
fail_gate() {
    local gate="$1"
    local reason="${2:-}"
    echo "[-] Gate $gate FAILED: $reason" >&2
    echo "BOARD_GATE: FAIL ($gate)"
    exit 1
}

# Process group management for safe teardown of only script-spawned processes
SPAWNED_PGIDS=()

kill_pgid() {
    local pgid="$1"
    if [ -n "$pgid" ] && kill -0 -- "-$pgid" 2>/dev/null; then
        kill -TERM -- "-$pgid" 2>/dev/null || true
        sleep 2
        if kill -0 -- "-$pgid" 2>/dev/null; then
            kill -KILL -- "-$pgid" 2>/dev/null || true
        fi
    fi
    # Rebuild the list without $pgid. Both expansions are guarded on a non-empty
    # count: under `set -u`, bash before 4.4 treats "${arr[@]}" on an empty array
    # as an unbound variable and aborts -- which would kill this script during the
    # LAST teardown, after Gate C passed and before Gate D ever ran.
    local remaining=()
    if [ "${#SPAWNED_PGIDS[@]}" -gt 0 ]; then
        for p in "${SPAWNED_PGIDS[@]}"; do
            if [ "$p" != "$pgid" ]; then
                remaining+=("$p")
            fi
        done
    fi
    if [ "${#remaining[@]}" -gt 0 ]; then
        SPAWNED_PGIDS=("${remaining[@]}")
    else
        SPAWNED_PGIDS=()
    fi
}

cleanup() {
    if [ "${#SPAWNED_PGIDS[@]}" -eq 0 ]; then
        return 0
    fi
    for (( i=${#SPAWNED_PGIDS[@]}-1; i>=0; i-- )); do
        local pgid="${SPAWNED_PGIDS[i]}"
        if [ -n "$pgid" ] && kill -0 -- "-$pgid" 2>/dev/null; then
            kill -TERM -- "-$pgid" 2>/dev/null || true
        fi
    done
    sleep 2
    for (( i=${#SPAWNED_PGIDS[@]}-1; i>=0; i-- )); do
        local pgid="${SPAWNED_PGIDS[i]}"
        if [ -n "$pgid" ] && kill -0 -- "-$pgid" 2>/dev/null; then
            kill -KILL -- "-$pgid" 2>/dev/null || true
        fi
    done
}
trap cleanup EXIT INT TERM

# Ensure execution only runs on the actual Jetson TX2 board environment
if [ ! -f /opt/ros/lunar/setup.bash ]; then
    echo "[-] /opt/ros/lunar/setup.bash not found. Must run on Jetson TX2 board." >&2
    echo "BOARD_GATE: FAIL (A)"
    exit 1
fi

# ==============================================================================
# Gate A: Rebuild workspace
# Clean build is required because darknet_ros was removed; stale devel headers cause false passes
# ==============================================================================
echo "=== Gate A: Rebuild Catkin Workspace ==="
# shellcheck disable=SC1091
source /opt/ros/lunar/setup.bash
cd "${HOME}/catkin_ws"
catkin_make clean || fail_gate "A" "catkin_make clean failed"
catkin_make || fail_gate "A" "catkin_make build failed"

# ==============================================================================
# Gate B: Board interpreter test
# Verify Python 2.7 + NumPy 1.11 compatibility on board (local runs on Python 3.12)
# ==============================================================================
echo "=== Gate B: Interpreter Tests (Python 2.7) ==="
# shellcheck disable=SC1091
source "${HOME}/catkin_ws/devel/setup.bash"
cd "${HOME}/catkin_ws/src"
run_all_out=$(PYTHON=python2.7 bash tests/run_all.sh 2>&1) || {
    echo "$run_all_out"
    fail_gate "B" "tests/run_all.sh failed with non-zero exit"
}
echo "$run_all_out"
if ! echo "$run_all_out" | grep -q "RUN_ALL: PASS"; then
    fail_gate "B" "RUN_ALL: PASS marker missing from test output"
fi

# ==============================================================================
# Gate C: Dry run 3 launch configurations stacked (30s each, out-of-water, relay OFF)
# ==============================================================================
echo "=== Gate C: Dry Launches ==="

# Check legacy topic naming convention (grep -P lookbehind ensures /albc/joint_currents is not flagged)
check_legacy_topics() {
    local topics
    topics=$(rostopic list 2>/dev/null || true)
    if echo "$topics" | grep -P '(?<!/albc)/joint_currents' >/dev/null; then
        echo "[-] Legacy topic detected matching '(?<!/albc)/joint_currents'" >&2
        return 1
    fi
    if echo "$topics" | grep -P 'active_joint' >/dev/null; then
        echo "[-] Legacy topic detected matching 'active_joint'" >&2
        return 1
    fi
    if echo "$topics" | grep -P '/rl/command' >/dev/null; then
        echo "[-] Legacy topic detected matching '/rl/command'" >&2
        return 1
    fi
    return 0
}

# Static safety check: verify default thruster_scale in launch file before any launch
rl_launch="${HOME}/catkin_ws/src/robot/albc_rl/launch/albc_rl.launch"
if [ ! -f "$rl_launch" ]; then
    fail_gate "C" "Launch file not found: $rl_launch"
fi
ts_default=$(grep -E '<arg[^>]*name="thruster_scale"' "$rl_launch" | sed -n 's/.*default="\([^"]*\)".*/\1/p' | tr -d ' ' || true)
if [ "$ts_default" != "0.0" ]; then
    fail_gate "C" "Static check failed: thruster_scale default in $rl_launch is '$ts_default' (expected 0.0)"
fi
echo "[+] Static check passed: thruster_scale default is 0.0"

# Layer 1. agent_launch: verify sensor publishing frequency at ~91 Hz (81 - 101 Hz) and keep running
echo "--- Launch 1/3 (Layer 1): hero_agent agent_launch.launch ---"
log_agent=$(mktemp /tmp/hero_agent_launch.XXXXXX.log)
setsid roslaunch hero_agent agent_launch.launch > "$log_agent" 2>&1 &
pid_agent=$!
SPAWNED_PGIDS+=("$pid_agent")

sleep 10
if ! kill -0 -- "-$pid_agent" 2>/dev/null; then
    cat "$log_agent"
    fail_gate "C" "hero_agent agent_launch died prematurely during startup"
fi

hz_line=$(timeout 10 rostopic hz /hero_agent/sensors 2>&1 | grep "average rate:" | tail -n 1 || true)
hz_val=$(echo "$hz_line" | awk '{print $3}')
if [ -z "$hz_val" ]; then
    cat "$log_agent"
    fail_gate "C" "Unable to read /hero_agent/sensors hz"
fi
hz_pass=$(awk -v hz="$hz_val" 'BEGIN { if (hz >= 81.0 && hz <= 101.0) print "1"; else print "0" }')
if [ "$hz_pass" != "1" ]; then
    fail_gate "C" "/hero_agent/sensors hz ($hz_val) outside allowed range [81, 101]"
fi
echo "[+] Sensor rate verified: $hz_val Hz"

sleep 10
if ! kill -0 -- "-$pid_agent" 2>/dev/null; then
    cat "$log_agent"
    fail_gate "C" "hero_agent agent_launch died after 30s observation"
fi
echo "[+] Layer 1 (hero_agent) active and healthy after 30s"

# Layer 2. albc: launch on top of agent_launch, verify process stays alive continuously for 30s
echo "--- Launch 2/3 (Layer 2): albc_control albc.launch ---"
log_albc=$(mktemp /tmp/albc_launch.XXXXXX.log)
setsid roslaunch albc_control albc.launch > "$log_albc" 2>&1 &
pid_albc=$!
SPAWNED_PGIDS+=("$pid_albc")

sleep 15
if ! kill -0 -- "-$pid_agent" 2>/dev/null; then
    cat "$log_agent"
    fail_gate "C" "hero_agent agent_launch died while starting albc"
fi
if ! kill -0 -- "-$pid_albc" 2>/dev/null; then
    cat "$log_albc"
    fail_gate "C" "albc died prematurely during first 15s"
fi

sleep 15
if ! kill -0 -- "-$pid_agent" 2>/dev/null; then
    cat "$log_agent"
    fail_gate "C" "hero_agent agent_launch died during albc 30s run"
fi
if ! kill -0 -- "-$pid_albc" 2>/dev/null; then
    cat "$log_albc"
    fail_gate "C" "albc died before completing 30s run"
fi
echo "[+] Layer 2 (albc) and Layer 1 (hero_agent) running successfully for 30s"

# Layer 3. albc_rl: launch on top of stack, verify dynamic thruster_scale, banner 72D, start gates, and joint currents topic
echo "--- Launch 3/3 (Layer 3): albc_rl albc_rl.launch ---"
log_rl=$(mktemp /tmp/albc_rl_launch.XXXXXX.log)
setsid roslaunch albc_rl albc_rl.launch > "$log_rl" 2>&1 &
pid_rl=$!
SPAWNED_PGIDS+=("$pid_rl")

# Dynamic check: verify thruster_scale parameter immediately after launch
sleep 5
if ! kill -0 -- "-$pid_agent" 2>/dev/null; then
    cat "$log_agent"
    fail_gate "C" "hero_agent agent_launch died during albc_rl startup"
fi
if ! kill -0 -- "-$pid_albc" 2>/dev/null; then
    cat "$log_albc"
    fail_gate "C" "albc died during albc_rl startup"
fi
if ! kill -0 -- "-$pid_rl" 2>/dev/null; then
    cat "$log_rl"
    fail_gate "C" "albc_rl died prematurely during startup"
fi

ts_dyn=$(rosparam get /rl_inference_node/thruster_scale 2>/dev/null) || {
    cat "$log_rl"
    fail_gate "C" "Failed to retrieve /rl_inference_node/thruster_scale from rosparam"
}
ts_dyn=$(echo "$ts_dyn" | tr -d '[:space:]')
if [ "$ts_dyn" != "0.0" ] && [ "$ts_dyn" != "0" ]; then
    cat "$log_rl"
    fail_gate "C" "Dynamic check failed: rosparam /rl_inference_node/thruster_scale is non-zero ($ts_dyn)"
fi
echo "[+] Dynamic check passed: rosparam /rl_inference_node/thruster_scale is $ts_dyn"

# 30s observation and validation of banner, start gates, and joint currents
sleep 15
if ! kill -0 -- "-$pid_agent" 2>/dev/null; then
    cat "$log_agent"
    fail_gate "C" "hero_agent agent_launch died during albc_rl run"
fi
if ! kill -0 -- "-$pid_albc" 2>/dev/null; then
    cat "$log_albc"
    fail_gate "C" "albc died during albc_rl run"
fi
if ! kill -0 -- "-$pid_rl" 2>/dev/null; then
    cat "$log_rl"
    fail_gate "C" "albc_rl died prematurely"
fi

# Verify banner in stdout
if ! grep -q "72D" "$log_rl"; then
    cat "$log_rl"
    fail_gate "C" "Banner '72D' not found in albc_rl stdout"
fi

# Verify start gate pass logs
if ! grep -q "start-pose gate OK" "$log_rl"; then
    cat "$log_rl"
    fail_gate "C" "'start-pose gate OK' log missing from albc_rl"
fi
if ! grep -q "start-state gate OK" "$log_rl"; then
    cat "$log_rl"
    fail_gate "C" "'start-state gate OK' log missing from albc_rl"
fi

# Verify /albc/joint_currents message reception (timeout 10s satisfies remaining 30s window)
if ! timeout 10 rostopic echo -n 1 /albc/joint_currents >/dev/null 2>&1; then
    cat "$log_rl"
    fail_gate "C" "Failed to receive message on /albc/joint_currents"
fi
echo "[+] albc_rl banner, start gates, and joint currents topic verified"

# Check underlying stack survival again
if ! kill -0 -- "-$pid_agent" 2>/dev/null || ! kill -0 -- "-$pid_albc" 2>/dev/null || ! kill -0 -- "-$pid_rl" 2>/dev/null; then
    fail_gate "C" "One or more layers died before completing full 30s run"
fi
echo "[+] All 3 layers ran successfully for 30s"

# Check legacy topics once with entire stack active
check_legacy_topics || fail_gate "C" "Legacy topic name found with full stack active"
echo "[+] Legacy topic verification passed across full stack"

# Teardown in reverse order: albc_rl -> albc -> hero_agent
echo "--- Teardown stacked layers in reverse order ---"
kill_pgid "$pid_rl"
kill_pgid "$pid_albc"
kill_pgid "$pid_agent"
echo "[+] All layers cleanly terminated"
# The logs are kept on the passing path too: this gate decides a merge, and the
# operator who reads the verdict later has nothing else to read it against.
echo "    launch logs: $log_agent $log_albc $log_rl"

# ==============================================================================
# Gate D: Firmware compilation gate
# Compile agent firmware and record MD5 checksum and byte size (flash prohibited)
# ==============================================================================
echo "=== Gate D: Build Firmware (Compilation only) ==="
cd "${HOME}/catkin_ws/src"
bash firmware/build_firmware.sh "${HOME}/catkin_ws/src/firmware/agent" agent || fail_gate "D" "firmware build failed"

fw_hex="${HOME}/fw_full_agent/agent.hex"
if [ ! -f "$fw_hex" ]; then
    fail_gate "D" "Target hex file $fw_hex does not exist"
fi

# Record md5sum and byte size in output
hex_md5=$(md5sum "$fw_hex" | awk '{print $1}')
hex_size=$(wc -c < "$fw_hex" | tr -d ' ')
echo "[+] Firmware built successfully: $fw_hex"
echo "    MD5:  $hex_md5"
echo "    Size: $hex_size bytes"

# All gates passed successfully
echo "BOARD_GATE: PASS"
exit 0

