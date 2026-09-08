//PointQueue.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

//-------------------------------------------------------------------
// Point-queue ring primitives (point-queue mode).
// Locking discipline mirrors inc_q: mpSemTake/mpSemGive with Q_LOCK_TIMEOUT.
//-------------------------------------------------------------------

// Fail-safe pre/post buffer-sentinel check. Returns TRUE iff both bracket
// guard words still hold POINT_QUEUE_GUARD_MAGIC (i.e. the data[] array has not
// been overrun on either side). Cheap: two integer compares. There is no lock
// to release now (lock-free SPSC); on mismatch callers raise a fatal alarm and
// fail safe rather than feed a corrupted point into motion.
static BOOL Ros_MotionControl_PointQueueGuardsOk(CtrlGroup* ctrlGroup)
{
    return (ctrlGroup->point_q.guard_pre == POINT_QUEUE_GUARD_MAGIC)
        && (ctrlGroup->point_q.guard_post == POINT_QUEUE_GUARD_MAGIC);
}

// Derived depth from a head/tail snapshot: (tail - head) mod POINT_QUEUE_SLOTS.
// Single producer means the producer's own view of tail is exact; head only
// grows, so the value is exact from the producer side and a safe lower/consistent
// bound from any observer. Correct for the ONE_DEEP (==0) and FIFO (>=DEPTH)
// admission decisions. Static helper: callers below have already validated guards.
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
    {
        mpSetAlarm(ALARM_ASSERTION_FAIL, "Point-queue buffer corrupted",
            SUBCODE_POINT_QUEUE_GUARD_CORRUPTION);
        return ERROR;
    }
    return Ros_MotionControl_PointQueueDepth(ctrlGroup);
}

// PRODUCER side (single enqueue-caller task; see SPSC invariant at PointQueue_q).
// Validate guards; if full return FALSE; write the slot payload; issue a RELEASE
// barrier so the payload write is globally visible BEFORE the new tail is
// published; then publish tail. The consumer, seeing the advanced tail, is
// guaranteed to observe the completed payload.
BOOL Ros_MotionControl_PointQueueEnqueue(CtrlGroup* ctrlGroup, JointMotionData* pt)
{
    LONG tail, head, nextTail;

    if (!Ros_MotionControl_PointQueueGuardsOk(ctrlGroup))
    {
        mpSetAlarm(ALARM_ASSERTION_FAIL, "Point-queue buffer corrupted",
            SUBCODE_POINT_QUEUE_GUARD_CORRUPTION);
        return FALSE;
    }

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

// CONSUMER side (single dequeue-caller task; see SPSC invariant at PointQueue_q).
// Validate guards; if empty (head == tail) return FALSE; read tail, then issue an
// ACQUIRE barrier BEFORE reading the slot so the payload the producer wrote is
// observed; copy the slot out; then advance head (consumer owns head).
BOOL Ros_MotionControl_PointQueueDequeue(CtrlGroup* ctrlGroup, JointMotionData* out)
{
    LONG head, tail, nextHead;

    if (!Ros_MotionControl_PointQueueGuardsOk(ctrlGroup))
    {
        mpSetAlarm(ALARM_ASSERTION_FAIL, "Point-queue buffer corrupted",
            SUBCODE_POINT_QUEUE_GUARD_CORRUPTION);
        return FALSE;
    }

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

// DIRECT flush: reset the point-queue ring to empty (head == tail == 0),
// discarding any queued-but-not-yet-interpolated points.
//
// >>> QUIESCED-CALLER-ONLY <<<
// This ring is LOCK-FREE single-producer (owns tail) / single-consumer (owns
// head). Writing BOTH indices from here is race-free ONLY when the consumer
// (the interpolation task, sole owner of head) is provably not running. The one
// caller that guarantees this is Ros_MotionControl_StopMotion: it raises
// bStopMotion = TRUE and then WAITS for !Ros_MotionControl_HasDataToProcess()
// before flushing, so the consumer is quiesced and not touching head, and the
// producer admission path is gated off by bStopMotion. With both owners
// quiesced, resetting both indices to 0 is race-free, and StopMotion is
// guaranteed an empty ring ON RETURN (it cannot rely on the consumer actioning
// a request, because the consumer is stopped). We deliberately zero BOTH indices
// so the backing buffer restarts from slot 0 with no stale wraparound state.
//
// ASYNC callers (IO-status monitor: alarm / WAITING_ROS-off / PFL) MUST NOT call
// this — they do not quiesce the consumer and a torn head write would corrupt
// the ring. They use Ros_MotionControl_PointQueueRequestFlush instead, which the
// consumer actions safely.
//
// CONTAINMENT: flush does NOT read or clear Ros_MotionControl_PointQueueUnderran.
// Per spec 5.5 the sticky underran flag intentionally SURVIVES a flush; it is
// cleared only by Ros_MotionControl_ReadAndClearPointQueueUnderran().
void Ros_MotionControl_PointQueueFlush(CtrlGroup* ctrlGroup)
{
    ctrlGroup->point_q.head = 0;
    ctrlGroup->point_q.tail = 0;
    __sync_synchronize();                   // publish the reset before returning
}

// ASYNC-SAFE flush: REQUEST that the consumer empty the ring. Sets the per-group
// flushRequested flag and returns immediately. Touches NEITHER head NOR tail, so
// it preserves the single-writer-each SPSC invariant and is safe to call from an
// arbitrary context that has NOT quiesced the consumer — specifically the
// IO-status monitor task (Ros_Controller_IoStatusUpdate) on alarm / WAITING_ROS
// turning off / PFL stop-escape-avoid. The consumer (sole owner of head) sees the
// flag at the top of its per-cycle loop and performs the actual reset there via
// head = tail (see Ros_MotionControl_AddToIncQueueProcess).
//
// CONTAINMENT: does NOT touch the sticky underran flag (spec 5.5: survives flush).
void Ros_MotionControl_PointQueueRequestFlush(CtrlGroup* ctrlGroup)
{
    ctrlGroup->point_q.flushRequested = TRUE;
    __sync_synchronize();                   // publish the request before returning
}
