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

// --- Underran latch MODEL ---------------------------------------------------
// NOTE: this is a MODEL, not real source. The real underran latch + accessor is
// Task 5 (not yet implemented in the committed tree). The semantics modeled
// here are exactly those specified for Task 5:
//   - the latch starts TRUE (no consumer has run yet)
//   - ReadAndClear returns the current value, then clears it to FALSE
//   - the consumer setting "queue empty" latches it back to TRUE
// When Task 5 lands its accessor, this model should be replaced by tests
// against the real function.
void UnderranLatch_Init(void);                 // sets latch TRUE
void UnderranLatch_SetConsumerEmpty(void);     // consumer observed empty -> latch TRUE
BOOL UnderranLatch_ReadAndClear(void);         // return current, then clear to FALSE

#endif // MOTOROS2_TEST_ADMISSION_MODEL_H
