#!/usr/bin/env bash
# run_tests.sh — build + run the host-gcc point-queue unit tests.
# Uses plain gcc (no MotoPlus toolchain, no ROS). Exits non-zero on failure.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

CC="${CC:-gcc}"
BIN="test_point_queue"
# Host-tree TUs + the REAL production ring TU (../../src/PointQueue.c). The ring
# bodies are no longer duplicated; PointQueue.c is compiled with `-iquote . -I-`
# so the test-only MotoROS.h shim in this dir resolves instead of src/MotoROS.h.
SRCS=(test_point_queue.c point_queue_ring.c admission_model.c underran_latch.c ../../src/PointQueue.c)

echo "building with: $CC -std=c99 -Wall -iquote . -I-"
"$CC" -std=c99 -Wall -iquote . -I- "${SRCS[@]}" -o "$BIN"

echo "running $BIN"
if ./"$BIN"; then
    echo "RESULT: PASS"
    exit 0
else
    rc=$?
    echo "RESULT: FAIL (exit $rc)"
    exit "$rc"
fi
