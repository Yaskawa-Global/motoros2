// point_queue_ring.c — the point-queue ring primitives, for host testing.
//
// The function bodies below MIRROR those committed in motoros2/src/MotionControl.c.
// They are duplicated here (not #included) because MotionControl.c pulls in
// MotoROS.h, which drags the full MotoPlus + micro-ROS header set that cannot be
// satisfied on a plain-gcc host. Duplicating just these leaf primitives keeps the
// test self-contained while still exercising the EXACT lock-free SPSC head/tail
// index math, count/full/empty logic, the __sync_synchronize() release/acquire
// barriers, AND the pre/post buffer-sentinel (guard-word) fail-safe checks that ship.
//
// The ring is LOCK-FREE single-producer / single-consumer (SPSC): no semaphore.
// tail is producer-owned (enqueue only), head is consumer-owned (dequeue only),
// and one backing slot is kept empty (array sized POINT_QUEUE_SLOTS =
// POINT_QUEUE_DEPTH+1) so full is distinguishable from empty. Depth is derived
// from (tail - head) mod POINT_QUEUE_SLOTS.
//
// Difference from production (intentional, host-only): on a guard mismatch the
// production primitives additionally call mpSetAlarm(ALARM_ASSERTION_FAIL,
// SUBCODE_POINT_QUEUE_GUARD_CORRUPTION) before failing safe. There is no host stub
// for the alarm, so that single call is omitted here; the fail-safe RETURN (FALSE /
// ERROR) behavior is mirrored exactly. The __sync_* barriers are real GCC builtins.
//
// If the primitive bodies in MotionControl.c ever change, these must be updated
// in lockstep (they are intentionally kept in sync so a diff is trivial to verify).

#include "point_queue_ring.h"

// Guard-word validator — mirrors Ros_MotionControl_PointQueueGuardsOk in
// motoros2/src/MotionControl.c (kept in lockstep; see the header banner).
// The production version, on mismatch, additionally raises an mpSetAlarm
// (ALARM_ASSERTION_FAIL / SUBCODE_POINT_QUEUE_GUARD_CORRUPTION) before failing
// safe; that call has no host stub and is intentionally omitted here.
static BOOL Ros_MotionControl_PointQueueGuardsOk(CtrlGroup* g)
{
    return (g->point_q.guard_pre == POINT_QUEUE_GUARD_MAGIC)
        && (g->point_q.guard_post == POINT_QUEUE_GUARD_MAGIC);
}

// Derived depth from a head/tail snapshot: (tail - head) mod POINT_QUEUE_SLOTS.
static LONG Ros_MotionControl_PointQueueDepth(CtrlGroup* ctrlGroup)
{
    LONG head = ctrlGroup->point_q.head;
    LONG tail = ctrlGroup->point_q.tail;
    LONG depth = tail - head;
    if (depth < 0)
        depth += POINT_QUEUE_SLOTS;
    return depth;
}

LONG Ros_MotionControl_PointQueueCount(CtrlGroup* ctrlGroup)
{
    if (!Ros_MotionControl_PointQueueGuardsOk(ctrlGroup))
        return ERROR;
    return Ros_MotionControl_PointQueueDepth(ctrlGroup);
}

BOOL Ros_MotionControl_PointQueueEnqueue(CtrlGroup* ctrlGroup, JointMotionData* pt)
{
    LONG tail, head, nextTail;

    if (!Ros_MotionControl_PointQueueGuardsOk(ctrlGroup))
        return FALSE;

    tail = ctrlGroup->point_q.tail;         // producer owns tail (exact)
    head = ctrlGroup->point_q.head;         // snapshot consumer's head
    nextTail = tail + 1;
    if (nextTail >= POINT_QUEUE_SLOTS)
        nextTail = 0;

    if (nextTail == head)                   // full (one slot kept empty)
        return FALSE;

    ctrlGroup->point_q.data[tail] = *pt;    // write payload into the free slot
    __sync_synchronize();                   // RELEASE: payload before tail publish
    ctrlGroup->point_q.tail = nextTail;     // publish the new tail
    return TRUE;
}

BOOL Ros_MotionControl_PointQueueDequeue(CtrlGroup* ctrlGroup, JointMotionData* out)
{
    LONG head, tail, nextHead;

    if (!Ros_MotionControl_PointQueueGuardsOk(ctrlGroup))
        return FALSE;

    head = ctrlGroup->point_q.head;         // consumer owns head (exact)
    tail = ctrlGroup->point_q.tail;         // observe producer's published tail
    if (head == tail)                       // empty
        return FALSE;

    __sync_synchronize();                   // ACQUIRE: observe payload before read
    *out = ctrlGroup->point_q.data[head];   // copy the oldest point out

    nextHead = head + 1;
    if (nextHead >= POINT_QUEUE_SLOTS)
        nextHead = 0;
    __sync_synchronize();                   // ensure slot read completes before head advances
    ctrlGroup->point_q.head = nextHead;     // free the slot (publish new head)
    return TRUE;
}

// Flush the ring back to empty — mirrors Ros_MotionControl_PointQueueFlush in
// motoros2/src/MotionControl.c (kept in lockstep; see the header banner). Resets
// BOTH indices to 0 so the backing buffer restarts from slot 0 with no stale
// wraparound. Deliberately does NOT touch any underran state (spec 5.5).
void Ros_MotionControl_PointQueueFlush(CtrlGroup* ctrlGroup)
{
    ctrlGroup->point_q.head = 0;
    ctrlGroup->point_q.tail = 0;
    __sync_synchronize();                   // publish the reset before returning
}
