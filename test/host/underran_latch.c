// underran_latch.c — host MIRROR of the REAL point-queue underran flag.
//
// The REAL flag + accessor now live in src/MotionControl.c (Task 5):
//   static volatile BOOL Ros_MotionControl_PointQueueUnderran = TRUE;
//   BOOL Ros_MotionControl_ReadAndClearPointQueueUnderran(void);
// and the single SET site is the consumer-empty (q->cnt==0) point-queue branch
// inside Ros_MotionControl_IncMoveLoopStart.
//
// The host harness cannot run the real IP_CLK IncMove loop (no MotoPlus RTOS,
// no controller). So this file MIRRORS the real flag byte-for-byte:
//   - Mirror_ReadAndClear   == Ros_MotionControl_ReadAndClearPointQueueUnderran
//   - Mirror_MarkUnderran   simulates the real consumer-empty SET line
//     (`Ros_MotionControl_PointQueueUnderran = TRUE;`) directly, since the
//     host can't drive the real loop.
// The read-and-clear CONTRACT is what these host tests exercise. It is the same
// contract the real accessor upholds.

#include "admission_model.h"

// Mirrors the real file-scope flag; starts TRUE by spec (baseline on startup).
static volatile BOOL s_point_queue_underran = TRUE;

// Direct-set mirror of the real consumer-empty SET line. Not a production hook:
// production has exactly two touch-points (the SET site + the accessor).
void Mirror_MarkPointQueueUnderran(void)
{
    s_point_queue_underran = TRUE;
}

// Mirror of Ros_MotionControl_ReadAndClearPointQueueUnderran(): atomic
// read-and-clear via __sync_lock_test_and_set (returns prior value, stores
// FALSE) — byte-for-byte the real accessor. The ONLY clear site.
BOOL Mirror_ReadAndClearPointQueueUnderran(void)
{
    return __sync_lock_test_and_set(&s_point_queue_underran, FALSE);
}
