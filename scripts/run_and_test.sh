#!/usr/bin/env bash
set -euo pipefail

RUN_ID="run_$(date +%Y%m%d_%H%M%S)"
export RUN_ID

echo "Starting stack with RUN_ID=${RUN_ID}"
docker compose up -d sim control

# Allow system to boot and robot to drive
TIMEOUT=${TIMEOUT:-120}
echo "Waiting ${TIMEOUT}s for simulation..."
sleep "${TIMEOUT}"

echo "Stopping stack"
docker compose stop sim control

echo "Analyzing logs..."
python3 scripts/analyze_logs.py
EXIT_CODE=$?

echo "Cleaning up containers"
docker compose rm -f sim control >/dev/null

exit ${EXIT_CODE}
