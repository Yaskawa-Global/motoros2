// point_queue_ring.h — declarations for the host-tested ring primitives.
// Signatures are verbatim from motoros2/src/MotionControl.h.

#ifndef MOTOROS2_TEST_POINT_QUEUE_RING_H
#define MOTOROS2_TEST_POINT_QUEUE_RING_H

#include "motoplus_stubs.h"

LONG Ros_MotionControl_PointQueueCount(CtrlGroup* ctrlGroup);
BOOL Ros_MotionControl_PointQueueEnqueue(CtrlGroup* ctrlGroup, JointMotionData* pt);
BOOL Ros_MotionControl_PointQueueDequeue(CtrlGroup* ctrlGroup, JointMotionData* out);

#endif // MOTOROS2_TEST_POINT_QUEUE_RING_H
