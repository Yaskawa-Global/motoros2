<!--
SPDX-FileCopyrightText: 2026, C. Wolf

SPDX-License-Identifier: Apache-2.0
-->

# Off-host unit tests (point-queue ring)

Host-native unit tests for the lock-free point-queue ring buffer used by
`queue_traj_point` / `queue_traj_point_stream`. These run with **plain `gcc` on
any developer machine or CI runner** — no MotoPlus/Windows toolchain, no ROS, no
robot controller.

They complement the two other layers of verification for this feature:

1. **On-target self-test** — `src/Tests_PointQueue.c` (`Ros_Testing_PointQueue()`,
   `#ifdef MOTOROS2_TESTING_ENABLE`), which runs in the controller boot self-test
   alongside the other `Tests_*` modules, in the upstream convention.
2. **This off-host harness** — runs the *real* ring code (`src/PointQueue.c`) on
   the host, exercising the logic that is otherwise only reachable on the
   controller. This is the layer that fits into ordinary CI.
3. **Formal model-check** — the lock-free async-flush protocol is verified with
   TLA+/TLC (see `docs/verification/` in the containing workspace).

## Running

```sh
./run_tests.sh
```

Exits `0` on success (`ALL PASS`), non-zero on failure. Requires only `gcc`
(C99). Override the compiler with `CC=...`.

## How it works

`run_tests.sh` compiles the host test translation units together with the
**real production ring**, `../../src/PointQueue.c` — the ring bodies are *not*
duplicated. `PointQueue.c` is compiled with `-iquote . -I-` so that its
`#include "MotoROS.h"` resolves to the small test-only shim (`MotoROS.h` in this
directory) instead of the firmware `src/MotoROS.h`. The shim + `motoplus_stubs.h`
provide only the handful of symbols the ring references off-target (base
typedefs, `mpSetAlarm` as a no-op, `__sync_*` are real compiler builtins).

Logic that lives in `MotionControl.c` rather than the ring (the admission policy,
the consumer flush-action, the underran flag/accessor) is mirrored by small host
models (`admission_model.c`, `underran_latch.c`, the flush-action model in
`point_queue_ring.c`) whose names mirror the on-target test taxonomy.

## Coverage

Tier 1 — ring primitives (real `PointQueue.c`):
- `fifo_order`, `empty_rejects_dequeue`, `full_rejects_enqueue`, `wraparound`
- `guard_corruption_fails_safe` (sentinel fail-safe)
- `flush_clears_point_ring`, `flush_request_consumer_actions`

Tier 2 — admission policy + underran + legacy equivalence (models):
- `admit_one_deep`, `admit_fifo_fill_then_full`
- `underran_read_and_clear`
- `legacy_equivalence`, `legacy_vs_fifo_same_ring`, `legacy_inflight_busy`
  (the guardrail for "legacy `queue_traj_point` behaves exactly as before")
