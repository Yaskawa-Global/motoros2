// point_queue_ring.c — host-only consumer flush-action model.
//
// >>> The ring PRIMITIVE bodies are NO LONGER duplicated here. <<<
// The lock-free SPSC ring (Count / Enqueue / Dequeue / Flush / RequestFlush,
// plus the guard-word fail-safe check and the derived-depth helper) is now
// isolated in the production translation unit motoros2/src/PointQueue.c, which
// this host harness compiles DIRECTLY (see test/host/Makefile / run_tests.sh and
// the test-only MotoROS.h shim). The tests therefore exercise the REAL shipping
// bodies — no mirror to keep in lockstep.
//
// What remains here is the ONE thing that is NOT a ring primitive and does not
// live in PointQueue.c: a model of the CONSUMER's inline flush-action step that
// lives inside Ros_MotionControl_AddToIncQueueProcess in
// motoros2/src/MotionControl.c (the integration code, intentionally NOT moved
// into the ring module). That block is the ONLY place the async flush reset
// happens: a single read of producer-owned tail, a single write of
// consumer-owned head (head = tail => empty), then clear the request. It cannot
// be #included from MotionControl.c on host (that file drags the full MotoPlus +
// micro-ROS header set), so it is modeled here in lockstep with that block.

#include "point_queue_ring.h"

// CONSUMER flush-action step — mirrors the top-of-loop block in
// Ros_MotionControl_AddToIncQueueProcess (motoros2/src/MotionControl.c). This is
// the ONLY place the async flush reset happens: a single read of producer-owned
// tail, a single write of consumer-owned head (head = tail => empty), then clear
// the request. Returns TRUE if a flush was actioned this call. (The host stub
// CtrlGroup carries no trajectoryIterator, so iterator invalidation — which the
// real consumer also performs — is exercised in the integration code, not here.)
BOOL Ros_MotionControl_PointQueueActionFlushIfRequested(CtrlGroup* ctrlGroup)
{
    if (!ctrlGroup->point_q.flushRequested)
        return FALSE;
    ctrlGroup->point_q.head = ctrlGroup->point_q.tail;  // read producer tail once, write consumer head once => empty
    __sync_synchronize();                               // publish head reset
    ctrlGroup->point_q.flushRequested = FALSE;          // consumer clears the request last
    return TRUE;
}
