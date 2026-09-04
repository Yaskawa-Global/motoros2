// admission_model.h — declarations for the host admission model + underran latch.

#ifndef MOTOROS2_TEST_ADMISSION_MODEL_H
#define MOTOROS2_TEST_ADMISSION_MODEL_H

#include "point_queue_ring.h"

// Single-group admission+enqueue for one point. Mirrors the admission branches
// of Ros_MotionControl_EnqueueTrajectoryPoint (see admission_model.c).
// Returns a QueueResultEnum value (QRE_*). Sets *out_depth (if non-NULL) to the
// post-call ring depth on SUCCESS.
UINT16 PointQueue_AdmitOne(CtrlGroup* g, PointQueueAdmitPolicy policy,
                           JointMotionData* pt, UINT16* out_depth);

// --- Underran flag MIRROR ---------------------------------------------------
// The REAL sticky flag + accessor now exist in src/MotionControl.c (Task 5):
//   static volatile BOOL Ros_MotionControl_PointQueueUnderran = TRUE;
//   BOOL Ros_MotionControl_ReadAndClearPointQueueUnderran(void);
// with a single SET site in the consumer-empty (q->cnt==0) point-queue branch
// of Ros_MotionControl_IncMoveLoopStart.
//
// The host can't run the real IP_CLK IncMove loop, so these are a faithful
// MIRROR of the real flag + accessor. Mirror_MarkPointQueueUnderran() stands in
// for the real consumer-empty SET line. The read-and-clear CONTRACT exercised
// here is identical to the one the real accessor upholds:
//   - starts TRUE (baseline on startup)
//   - ReadAndClear returns the current value, then clears it to FALSE
//   - a consumer-empty set latches it back to TRUE
void Mirror_MarkPointQueueUnderran(void);           // simulate real consumer-empty SET
BOOL Mirror_ReadAndClearPointQueueUnderran(void);   // mirror of the real accessor

#endif // MOTOROS2_TEST_ADMISSION_MODEL_H
