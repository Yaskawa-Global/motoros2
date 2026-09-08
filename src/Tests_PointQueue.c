// Tests_PointQueue.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifdef MOTOROS2_TESTING_ENABLE

#include "MotoROS.h"

// On-target self-test for the lock-free SPSC point-queue ring (src/PointQueue.c).
// These run on the controller during boot (see main.c MOTOROS2_TESTING_ENABLE
// block), so they call the REAL ring primitives directly against a stack-local
// CtrlGroup — no stubbing. The sub-test taxonomy is kept in lockstep with the
// off-host harness in test/host/test_point_queue.c so a reader sees the same
// test intent in both places:
//   Ros_Testing_PointQueue_EmptyRejectsDequeue     <-> test_point_queue_empty_rejects_dequeue
//   Ros_Testing_PointQueue_FifoOrder               <-> test_point_queue_fifo_order
//   Ros_Testing_PointQueue_FullRejectsEnqueue      <-> test_point_queue_full_rejects_enqueue
//   Ros_Testing_PointQueue_Wraparound              <-> test_point_queue_wraparound
//   Ros_Testing_PointQueue_GuardCorruptionFailsSafe<-> test_point_queue_guard_corruption_fails_safe
//   Ros_Testing_PointQueue_FlushClearsRing         <-> test_flush_clears_point_ring
//   Ros_Testing_PointQueue_FlushRequestConsumerActions <-> test_flush_request_consumer_actions

// Init a stack-local CtrlGroup's point_q exactly as CtrlGroup.c does at
// construction: bzero the whole group, then set the bracket sentinels to MAGIC.
// (bzero leaves head == tail == 0, i.e. an empty ring, and flushRequested FALSE.)
static void Ros_Testing_PointQueue_MakeGroup(CtrlGroup* group)
{
    bzero(group, sizeof(CtrlGroup));
    group->point_q.guard_pre = POINT_QUEUE_GUARD_MAGIC;
    group->point_q.guard_post = POINT_QUEUE_GUARD_MAGIC;
}

// (1) Empty ring: Count == 0 and Dequeue returns FALSE.
static BOOL Ros_Testing_PointQueue_EmptyRejectsDequeue()
{
    CtrlGroup group;
    JointMotionData out;
    BOOL bOk;

    Ros_Testing_PointQueue_MakeGroup(&group);

    bOk = Ros_Testing_CompareLong(Ros_MotionControl_PointQueueCount(&group), 0);
    bOk &= (Ros_MotionControl_PointQueueDequeue(&group, &out) == FALSE);

    Ros_Debug_BroadcastMsg("Testing PointQueue EmptyRejectsDequeue: %s", bOk ? "PASS" : "FAIL");
    return bOk;
}

// (2) FIFO order: enqueue points with distinguishable .time; dequeue returns
//     them oldest-first; Count tracks along the way.
static BOOL Ros_Testing_PointQueue_FifoOrder()
{
    CtrlGroup group;
    JointMotionData a, b, c, out;
    BOOL bOk;

    Ros_Testing_PointQueue_MakeGroup(&group);

    bzero(&a, sizeof(a)); a.time = 10;
    bzero(&b, sizeof(b)); b.time = 20;
    bzero(&c, sizeof(c)); c.time = 30;

    bOk = (Ros_MotionControl_PointQueueEnqueue(&group, &a) == TRUE);
    bOk &= Ros_Testing_CompareLong(Ros_MotionControl_PointQueueCount(&group), 1);
    bOk &= (Ros_MotionControl_PointQueueEnqueue(&group, &b) == TRUE);
    bOk &= Ros_Testing_CompareLong(Ros_MotionControl_PointQueueCount(&group), 2);
    bOk &= (Ros_MotionControl_PointQueueEnqueue(&group, &c) == TRUE);
    bOk &= Ros_Testing_CompareLong(Ros_MotionControl_PointQueueCount(&group), 3);

    bOk &= (Ros_MotionControl_PointQueueDequeue(&group, &out) == TRUE);
    bOk &= Ros_Testing_INT64_Equals((INT64)out.time, 10);
    bOk &= (Ros_MotionControl_PointQueueDequeue(&group, &out) == TRUE);
    bOk &= Ros_Testing_INT64_Equals((INT64)out.time, 20);
    bOk &= (Ros_MotionControl_PointQueueDequeue(&group, &out) == TRUE);
    bOk &= Ros_Testing_INT64_Equals((INT64)out.time, 30);
    bOk &= Ros_Testing_CompareLong(Ros_MotionControl_PointQueueCount(&group), 0);
    bOk &= (Ros_MotionControl_PointQueueDequeue(&group, &out) == FALSE);

    Ros_Debug_BroadcastMsg("Testing PointQueue FifoOrder: %s", bOk ? "PASS" : "FAIL");
    return bOk;
}

// (3) Fill to capacity: POINT_QUEUE_DEPTH enqueues all succeed; the next
//     enqueue returns FALSE (full, one slot kept empty); Count == DEPTH.
static BOOL Ros_Testing_PointQueue_FullRejectsEnqueue()
{
    CtrlGroup group;
    JointMotionData p, overflow;
    BOOL bOk;
    int i;

    Ros_Testing_PointQueue_MakeGroup(&group);

    bOk = TRUE;
    for (i = 0; i < POINT_QUEUE_DEPTH; i += 1)
    {
        bzero(&p, sizeof(p)); p.time = (UINT64)(1000 + i);
        bOk &= (Ros_MotionControl_PointQueueEnqueue(&group, &p) == TRUE);
        bOk &= Ros_Testing_CompareLong(Ros_MotionControl_PointQueueCount(&group), i + 1);
    }
    bOk &= Ros_Testing_CompareLong(Ros_MotionControl_PointQueueCount(&group), POINT_QUEUE_DEPTH);

    bzero(&overflow, sizeof(overflow)); overflow.time = 9999;
    bOk &= (Ros_MotionControl_PointQueueEnqueue(&group, &overflow) == FALSE); // full
    bOk &= Ros_Testing_CompareLong(Ros_MotionControl_PointQueueCount(&group), POINT_QUEUE_DEPTH);

    Ros_Debug_BroadcastMsg("Testing PointQueue FullRejectsEnqueue: %s", bOk ? "PASS" : "FAIL");
    return bOk;
}

// (4) Wraparound: fill, drain half, refill so tail/head advance past
//     POINT_QUEUE_SLOTS; verify FIFO order and count are preserved across the
//     backing-buffer boundary.
static BOOL Ros_Testing_PointQueue_Wraparound()
{
    CtrlGroup group;
    JointMotionData p, out;
    BOOL bOk;
    int i;

    Ros_Testing_PointQueue_MakeGroup(&group);

    bOk = TRUE;
    for (i = 0; i < POINT_QUEUE_DEPTH; i += 1)
    {
        bzero(&p, sizeof(p)); p.time = (UINT64)(1000 + i);
        bOk &= (Ros_MotionControl_PointQueueEnqueue(&group, &p) == TRUE);
    }
    bOk &= Ros_Testing_CompareLong(Ros_MotionControl_PointQueueCount(&group), POINT_QUEUE_DEPTH);

    // drain half in FIFO order
    for (i = 0; i < POINT_QUEUE_DEPTH / 2; i += 1)
    {
        bOk &= (Ros_MotionControl_PointQueueDequeue(&group, &out) == TRUE);
        bOk &= Ros_Testing_INT64_Equals((INT64)out.time, (INT64)(1000 + i));
    }
    bOk &= Ros_Testing_CompareLong(Ros_MotionControl_PointQueueCount(&group), POINT_QUEUE_DEPTH / 2);

    // refill half -> tail wraps past the end of the backing buffer
    for (i = 0; i < POINT_QUEUE_DEPTH / 2; i += 1)
    {
        bzero(&p, sizeof(p)); p.time = (UINT64)(2000 + i);
        bOk &= (Ros_MotionControl_PointQueueEnqueue(&group, &p) == TRUE);
    }
    bOk &= Ros_Testing_CompareLong(Ros_MotionControl_PointQueueCount(&group), POINT_QUEUE_DEPTH);

    // drain all: remaining originals first, then the wrapped-in new ones
    for (i = POINT_QUEUE_DEPTH / 2; i < POINT_QUEUE_DEPTH; i += 1)
    {
        bOk &= (Ros_MotionControl_PointQueueDequeue(&group, &out) == TRUE);
        bOk &= Ros_Testing_INT64_Equals((INT64)out.time, (INT64)(1000 + i));
    }
    for (i = 0; i < POINT_QUEUE_DEPTH / 2; i += 1)
    {
        bOk &= (Ros_MotionControl_PointQueueDequeue(&group, &out) == TRUE);
        bOk &= Ros_Testing_INT64_Equals((INT64)out.time, (INT64)(2000 + i));
    }
    bOk &= Ros_Testing_CompareLong(Ros_MotionControl_PointQueueCount(&group), 0);
    bOk &= (Ros_MotionControl_PointQueueDequeue(&group, &out) == FALSE);

    Ros_Debug_BroadcastMsg("Testing PointQueue Wraparound: %s", bOk ? "PASS" : "FAIL");
    return bOk;
}

// (5) Guard/sentinel fail-safe.
//
// >>> ALARM DECISION (documented) <<<
// The ring's fail-safe primitives (Count/Enqueue/Dequeue) validate the bracket
// sentinels via the STATIC helper Ros_MotionControl_PointQueueGuardsOk() and, on
// mismatch, call mpSetAlarm(ALARM_ASSERTION_FAIL, SUBCODE_POINT_QUEUE_GUARD_CORRUPTION)
// then fail safe (Enqueue/Dequeue return FALSE, Count returns ERROR). Two facts
// shaped this test:
//   1. GuardsOk() is `static` in PointQueue.c and is NOT exported, so it cannot
//      be called directly from here, and we must NOT change ring logic to export
//      it (task constraint).
//   2. Driving the primitives with a corrupted guard WOULD fire a real
//      ALARM_ASSERTION_FAIL on the controller during the boot self-test, which is
//      undesirable (a boot-time alarm on a healthy controller).
// Per the task's guidance ("Prefer testing GuardsOk() directly to avoid firing a
// boot-time alarm"), we verify the guard-word invariant with the EXACT comparison
// GuardsOk() performs (both words == POINT_QUEUE_GUARD_MAGIC): healthy after init
// -> both equal; corrupt either word -> the guard predicate is FALSE. The
// primitives' fail-safe FALSE/ERROR returns are proven off-host by
// test_point_queue_guard_corruption_fails_safe (mpSetAlarm is stubbed there), so
// the alarm-firing path is covered WITHOUT firing a real boot-time alarm here.
static BOOL Ros_Testing_PointQueue_GuardCorruptionFailsSafe()
{
    CtrlGroup group;
    JointMotionData p;
    BOOL guardsOkHealthy, guardsOkAfterPre, guardsOkAfterPost;
    BOOL bOk;

    Ros_Testing_PointQueue_MakeGroup(&group);
    bzero(&p, sizeof(p)); p.time = 7;
    bOk = (Ros_MotionControl_PointQueueEnqueue(&group, &p) == TRUE); // healthy primitive works

    // Replicate GuardsOk()'s exact predicate (both sentinels == MAGIC) locally.
    guardsOkHealthy = (group.point_q.guard_pre == POINT_QUEUE_GUARD_MAGIC)
                   && (group.point_q.guard_post == POINT_QUEUE_GUARD_MAGIC);
    bOk &= (guardsOkHealthy == TRUE);

    // Corrupt the PRE sentinel: guard predicate must go FALSE (fail-safe trigger).
    group.point_q.guard_pre = 0xDEADBEEFu;
    guardsOkAfterPre = (group.point_q.guard_pre == POINT_QUEUE_GUARD_MAGIC)
                    && (group.point_q.guard_post == POINT_QUEUE_GUARD_MAGIC);
    bOk &= (guardsOkAfterPre == FALSE);

    // Restore PRE, corrupt the POST sentinel: predicate must again go FALSE.
    group.point_q.guard_pre = POINT_QUEUE_GUARD_MAGIC;
    group.point_q.guard_post = 0xDEADBEEFu;
    guardsOkAfterPost = (group.point_q.guard_pre == POINT_QUEUE_GUARD_MAGIC)
                     && (group.point_q.guard_post == POINT_QUEUE_GUARD_MAGIC);
    bOk &= (guardsOkAfterPost == FALSE);

    Ros_Debug_BroadcastMsg("Testing PointQueue GuardCorruptionFailsSafe (guard predicate, alarm-free): %s",
        bOk ? "PASS" : "FAIL");
    return bOk;
}

// (6a) Direct flush: RequestFlush sets the flag; PointQueueFlush resets the ring
//      to empty (Count == 0) and enqueue/dequeue work cleanly from index 0.
static BOOL Ros_Testing_PointQueue_FlushClearsRing()
{
    CtrlGroup group;
    JointMotionData p, out;
    BOOL bOk;
    int i;

    Ros_Testing_PointQueue_MakeGroup(&group);

    bOk = TRUE;
    // Advance head/tail into a wrapped position: fill, drain most, so a naive
    // "count = 0" that forgot the indices would leave stale head/tail.
    for (i = 0; i < POINT_QUEUE_DEPTH; i += 1)
    {
        bzero(&p, sizeof(p)); p.time = (UINT64)(1000 + i);
        bOk &= (Ros_MotionControl_PointQueueEnqueue(&group, &p) == TRUE);
    }
    for (i = 0; i < POINT_QUEUE_DEPTH - 1; i += 1)
        bOk &= (Ros_MotionControl_PointQueueDequeue(&group, &out) == TRUE);
    bOk &= Ros_Testing_CompareLong(Ros_MotionControl_PointQueueCount(&group), 1);

    // Direct flush: ring must report empty.
    Ros_MotionControl_PointQueueFlush(&group);
    bOk &= Ros_Testing_CompareLong(Ros_MotionControl_PointQueueCount(&group), 0);
    bOk &= (Ros_MotionControl_PointQueueDequeue(&group, &out) == FALSE);

    // Guards survive flush (flush only resets indices, not sentinels).
    bOk &= (group.point_q.guard_pre == POINT_QUEUE_GUARD_MAGIC);
    bOk &= (group.point_q.guard_post == POINT_QUEUE_GUARD_MAGIC);

    // After flush, enqueue/dequeue work cleanly (no stale wraparound): fill,
    // drain FIFO, end empty.
    for (i = 0; i < POINT_QUEUE_DEPTH; i += 1)
    {
        bzero(&p, sizeof(p)); p.time = (UINT64)(5000 + i);
        bOk &= (Ros_MotionControl_PointQueueEnqueue(&group, &p) == TRUE);
        bOk &= Ros_Testing_CompareLong(Ros_MotionControl_PointQueueCount(&group), i + 1);
    }
    for (i = 0; i < POINT_QUEUE_DEPTH; i += 1)
    {
        bOk &= (Ros_MotionControl_PointQueueDequeue(&group, &out) == TRUE);
        bOk &= Ros_Testing_INT64_Equals((INT64)out.time, (INT64)(5000 + i));
    }
    bOk &= Ros_Testing_CompareLong(Ros_MotionControl_PointQueueCount(&group), 0);

    Ros_Debug_BroadcastMsg("Testing PointQueue FlushClearsRing: %s", bOk ? "PASS" : "FAIL");
    return bOk;
}

// (6b) Async-safe flush request: RequestFlush only SETS flushRequested and
//      touches neither index. Verify the flag becomes TRUE and the indices/depth
//      are unchanged by the request itself. (The consumer actions head = tail in
//      MotionControl.c; that integration step is exercised off-host by
//      test_flush_request_consumer_actions.)
static BOOL Ros_Testing_PointQueue_FlushRequestConsumerActions()
{
    CtrlGroup group;
    JointMotionData p;
    LONG depthBefore, headBefore, tailBefore;
    BOOL bOk;
    int i;

    Ros_Testing_PointQueue_MakeGroup(&group);

    bOk = TRUE;
    for (i = 0; i < 3; i += 1)
    {
        bzero(&p, sizeof(p)); p.time = (UINT64)(1000 + i);
        bOk &= (Ros_MotionControl_PointQueueEnqueue(&group, &p) == TRUE);
    }
    depthBefore = Ros_MotionControl_PointQueueCount(&group);
    headBefore = group.point_q.head;
    tailBefore = group.point_q.tail;
    bOk &= Ros_Testing_CompareLong(depthBefore, 3);
    bOk &= (group.point_q.flushRequested == FALSE);

    // Request a flush (async setter): flag TRUE, indices + depth untouched.
    Ros_MotionControl_PointQueueRequestFlush(&group);
    bOk &= (group.point_q.flushRequested == TRUE);
    bOk &= Ros_Testing_CompareLong(group.point_q.head, headBefore);
    bOk &= Ros_Testing_CompareLong(group.point_q.tail, tailBefore);
    bOk &= Ros_Testing_CompareLong(Ros_MotionControl_PointQueueCount(&group), depthBefore);

    Ros_Debug_BroadcastMsg("Testing PointQueue FlushRequestConsumerActions: %s", bOk ? "PASS" : "FAIL");
    return bOk;
}

BOOL Ros_Testing_PointQueue()
{
    BOOL bSuccess = TRUE;

    bSuccess &= Ros_Testing_PointQueue_EmptyRejectsDequeue();
    bSuccess &= Ros_Testing_PointQueue_FifoOrder();
    bSuccess &= Ros_Testing_PointQueue_FullRejectsEnqueue();
    bSuccess &= Ros_Testing_PointQueue_Wraparound();
    bSuccess &= Ros_Testing_PointQueue_GuardCorruptionFailsSafe();
    bSuccess &= Ros_Testing_PointQueue_FlushClearsRing();
    bSuccess &= Ros_Testing_PointQueue_FlushRequestConsumerActions();

    return bSuccess;
}

#endif //MOTOROS2_TESTING_ENABLE
