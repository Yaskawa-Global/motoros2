// point_queue_ring.c — the point-queue ring primitives, for host testing.
//
// The function bodies below MIRROR those committed in motoros2/src/MotionControl.c.
// They are duplicated here (not #included) because MotionControl.c pulls in
// MotoROS.h, which drags the full MotoPlus + micro-ROS header set that cannot be
// satisfied on a plain-gcc host. Duplicating just these leaf primitives keeps the
// test self-contained while still exercising the EXACT index math, count/full/empty
// guards, AND the pre/post buffer-sentinel (guard-word) fail-safe checks that ship.
//
// Difference from production (intentional, host-only): on a guard mismatch the
// production primitives additionally call mpSetAlarm(ALARM_ASSERTION_FAIL,
// SUBCODE_POINT_QUEUE_GUARD_CORRUPTION) before failing safe. There is no host stub
// for the alarm, so that single call is omitted here; the fail-safe RETURN (FALSE /
// ERROR) behavior is mirrored exactly.
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

LONG Ros_MotionControl_PointQueueCount(CtrlGroup* ctrlGroup)
{
    LONG cnt = 0;
    if (mpSemTake(ctrlGroup->point_q.q_lock, Q_LOCK_TIMEOUT) == OK)
    {
        if (!Ros_MotionControl_PointQueueGuardsOk(ctrlGroup))
        {
            mpSemGive(ctrlGroup->point_q.q_lock);
            return ERROR;
        }
        cnt = ctrlGroup->point_q.cnt;
        mpSemGive(ctrlGroup->point_q.q_lock);
    }
    else
        cnt = ERROR;
    return cnt;
}

BOOL Ros_MotionControl_PointQueueEnqueue(CtrlGroup* ctrlGroup, JointMotionData* pt)
{
    BOOL ok = FALSE;
    if (mpSemTake(ctrlGroup->point_q.q_lock, Q_LOCK_TIMEOUT) == OK)
    {
        if (!Ros_MotionControl_PointQueueGuardsOk(ctrlGroup))
        {
            mpSemGive(ctrlGroup->point_q.q_lock);
            return FALSE;
        }
        if (ctrlGroup->point_q.cnt < POINT_QUEUE_DEPTH)
        {
            int writeIdx = Q_OFFSET_IDX(ctrlGroup->point_q.idx, ctrlGroup->point_q.cnt, POINT_QUEUE_DEPTH);
            ctrlGroup->point_q.data[writeIdx] = *pt;
            ctrlGroup->point_q.cnt += 1;
            ok = TRUE;
        }
        mpSemGive(ctrlGroup->point_q.q_lock);
    }
    return ok;
}

BOOL Ros_MotionControl_PointQueueDequeue(CtrlGroup* ctrlGroup, JointMotionData* out)
{
    BOOL ok = FALSE;
    if (mpSemTake(ctrlGroup->point_q.q_lock, Q_LOCK_TIMEOUT) == OK)
    {
        if (!Ros_MotionControl_PointQueueGuardsOk(ctrlGroup))
        {
            mpSemGive(ctrlGroup->point_q.q_lock);
            return FALSE;
        }
        if (ctrlGroup->point_q.cnt > 0)
        {
            *out = ctrlGroup->point_q.data[ctrlGroup->point_q.idx];
            ctrlGroup->point_q.idx = Q_OFFSET_IDX(ctrlGroup->point_q.idx, 1, POINT_QUEUE_DEPTH);
            ctrlGroup->point_q.cnt -= 1;
            ok = TRUE;
        }
        mpSemGive(ctrlGroup->point_q.q_lock);
    }
    return ok;
}
