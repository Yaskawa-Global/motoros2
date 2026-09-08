// underran_latch.c — the point-queue underran flag + accessor, for host testing.
//
// The function body below is the REAL accessor
// Ros_MotionControl_ReadAndClearPointQueueUnderran committed in
// motoros2/src/MotionControl.c. It is duplicated here (not #included) for the
// SAME reason the ring primitives are (see point_queue_ring.c banner):
// MotionControl.c pulls in MotoROS.h, which drags the full MotoPlus + micro-ROS
// header set that cannot be satisfied on a plain-gcc host. Duplicating just this
// leaf accessor + its file-scope flag keeps the test self-contained while still
// exercising the EXACT read-and-clear contract that ships:
//
//   src/MotionControl.c:
//     static volatile BOOL Ros_MotionControl_PointQueueUnderran = TRUE;   // baseline
//     BOOL Ros_MotionControl_ReadAndClearPointQueueUnderran(void) {
//         return __sync_lock_test_and_set(&Ros_MotionControl_PointQueueUnderran, FALSE);
//     }
//   and the SINGLE SET site — the consumer-empty (q->cnt==0) point-queue branch
//   of Ros_MotionControl_IncMoveLoopStart:
//     Ros_MotionControl_PointQueueUnderran = TRUE;
//
// These are the REAL function name + body (kept in lockstep with MotionControl.c
// like point_queue_ring.c is). The host harness cannot run the real IP_CLK
// IncMove loop, so SimulateConsumerEmptyUnderran() reproduces that single SET
// line (the ONLY production SET site) directly. The accessor itself — the ONLY
// clear site — is byte-for-byte the shipped code, so the tests below drive the
// real accessor contract, not a stand-in model.

#include "admission_model.h"

// The REAL file-scope flag from src/MotionControl.c; starts TRUE (baseline on
// startup). Written only by the consumer-empty SET line and the accessor.
static volatile BOOL Ros_MotionControl_PointQueueUnderran = TRUE;

// Reproduces the ONE production SET line (the consumer-empty q->cnt==0 branch in
// Ros_MotionControl_IncMoveLoopStart). Not a production hook — production has
// exactly two touch-points: this SET line and the accessor below.
void SimulateConsumerEmptyUnderran(void)
{
    Ros_MotionControl_PointQueueUnderran = TRUE;
}

// REAL accessor — verbatim from src/MotionControl.c. Atomic read-and-clear via
// __sync_lock_test_and_set (returns prior value, stores FALSE). The ONLY clear
// site.
BOOL Ros_MotionControl_ReadAndClearPointQueueUnderran(void)
{
    return __sync_lock_test_and_set(&Ros_MotionControl_PointQueueUnderran, FALSE);
}
