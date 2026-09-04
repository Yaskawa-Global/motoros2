// admission_model.c — the point-queue admission decision, for host testing.
//
// WHAT THIS IS
// ------------
// Ros_MotionControl_EnqueueTrajectoryPoint (committed in
// motoros2/src/MotionControl.c) cannot be compiled on a plain-gcc host: it
// references g_Ros_Controller, the convert/find/duplicate helpers, the
// micro-ROS request struct, and the generated QueueResultEnum symbols — all of
// which require the MotoPlus + micro-ROS header/link environment.
//
// So we extract the PART of that function that carries the interesting policy
// logic — the per-group admission decision — into a small pure predicate here,
// and drive it against the REAL ring primitives (point_queue_ring.c) and the
// REAL enum values (motoplus_stubs.h). The branch order and comparisons below
// are copied line-for-line from the admission loop in
// Ros_MotionControl_EnqueueTrajectoryPoint:
//
//     LONG cnt = Ros_MotionControl_PointQueueCount(...);
//     if (cnt == ERROR)                              -> UNABLE_TO_PROCESS_POINT
//     if (policy == ONE_DEEP && cnt >= 1)            -> BUSY
//     if (policy == FIFO && cnt >= POINT_QUEUE_DEPTH) -> QUEUE_FULL
//                                                    -> (else) SUCCESS + enqueue
//
// This models the SINGLE-GROUP admission+enqueue for one point (the shipped
// function loops this over g_Ros_Controller.numGroup groups in lockstep; with
// one group the behavior is identical). Conversion/joint-name/duplicate checks
// are out of scope for the host tier and are covered compile-only under the
// real cross-toolchain (see task-3-report.md).

#include "admission_model.h"

UINT16 PointQueue_AdmitOne(CtrlGroup* g, PointQueueAdmitPolicy policy,
                           JointMotionData* pt, UINT16* out_depth)
{
    LONG cnt = Ros_MotionControl_PointQueueCount(g);

    if (cnt == ERROR)
        return QRE_UNABLE_TO_PROCESS_POINT;
    if (policy == POINT_QUEUE_ADMIT_ONE_DEEP && cnt >= 1)
        return QRE_BUSY;
    if (policy == POINT_QUEUE_ADMIT_FIFO && cnt >= POINT_QUEUE_DEPTH)
        return QRE_QUEUE_FULL;

    pt->valid = TRUE;
    if (!Ros_MotionControl_PointQueueEnqueue(g, pt))
        return QRE_UNABLE_TO_PROCESS_POINT;

    if (out_depth)
        *out_depth = (UINT16)Ros_MotionControl_PointQueueCount(g);

    return QRE_SUCCESS;
}
