#!/bin/bash
# Test script for KA2 posenetInvalid debugging

set -e

cd /data/openpilot

# Stop any existing openpilot
pkill -f "launch_openpilot" || true
pkill -f "system.manager.manager" || true
pkill -f "selfdrive.test.test_ka2_onroad" || true
sleep 2

# Clean log directory, do not delete params
rm -f /data/log/swaglog.*

# Enable verbose camera debugging
export DEBUG_CAMERA=1

# Run for seconds burn-in (KA2_BURN_IN_DURATION_S overrides)
export KA2_BURN_IN_DURATION_S=1800
export KA2_BURN_IN_TEST=1
export KA2_CAN_REPLAY=1
export KA2_SKIP_SD_TEST=1

# Optional: Set route (default from test file)
# export KA2_CAN_REPLAY_ROUTE="your-route-here"

echo "=== Starting KA2 Debug Test ==="
echo "Duration: ${KA2_BURN_IN_DURATION_S} seconds"
echo "Debug camera: ${DEBUG_CAMERA}"
echo "Can replay: ${KA2_CAN_REPLAY}"
echo "Logs will be saved to /data/log/"
echo ""

# Run the test
python3 selfdrive/test/test_ka2_onroad.py --can-replay --burn-in-test

echo ""
echo "=== Test Complete ==="
echo "Analyzing logs..."
echo ""

# Show key log entries for debugging
echo "=== MODEL STARTUP LOGS ==="
grep -h "modeld\|camerad\|CL context\|setting up\|models loaded\|stream_start\|dequeue\|camera sync\|vipc" /data/log/swaglog.* 2>/dev/null | tail -50

echo ""
echo "=== ERRORS ==="
grep -h "ERROR\|FAIL\|crash\|Traceback" /data/log/swaglog.* 2>/dev/null | grep -v "inject_assistance" | tail -30

echo ""
echo "=== CAMERA SYNC CHECKS ==="
grep -h "camera sync\|sync FAIL\|sync OK" /data/log/swaglog.* 2>/dev/null | tail -20

echo ""
echo "=== POSENET RELATED ==="
grep -h "posenet\|livePose\|cameraOdometry" /data/log/swaglog.* 2>/dev/null | tail -20
