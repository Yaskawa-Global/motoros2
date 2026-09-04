#!/usr/bin/env bash
# run_tests.sh — build + run the host-gcc point-queue unit tests.
# Uses plain gcc (no MotoPlus toolchain, no ROS). Exits non-zero on failure.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

CC="${CC:-gcc}"
BIN="test_point_queue"
SRCS=(test_point_queue.c point_queue_ring.c admission_model.c underran_latch.c)

echo "building with: $CC -std=c99 -Wall"
"$CC" -std=c99 -Wall "${SRCS[@]}" -o "$BIN"

echo "running $BIN"
if ./"$BIN"; then
    echo "RESULT: PASS"
    exit 0
else
    rc=$?
    echo "RESULT: FAIL (exit $rc)"
    exit "$rc"
fi
