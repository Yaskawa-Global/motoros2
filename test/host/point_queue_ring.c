// point_queue_ring.c — the point-queue ring primitives, for host testing.
//
// The three function bodies below are BYTE-IDENTICAL to those committed in
// motoros2/src/MotionControl.c (commit e5c1e62). They are duplicated here (not
// #included) because MotionControl.c pulls in MotoROS.h, which drags the full
// MotoPlus + micro-ROS header set that cannot be satisfied on a plain-gcc host.
// Duplicating just these leaf primitives keeps the test self-contained while
// still exercising the EXACT index math and count/full/empty guards that ship.
//
// If the primitive bodies in MotionControl.c ever change, these must be updated
// in lockstep (they are intentionally verbatim so a diff is trivial to verify).

#include "point_queue_ring.h"

LONG Ros_MotionControl_PointQueueCount(CtrlGroup* ctrlGroup)
{
    LONG cnt = 0;
    if (mpSemTake(ctrlGroup->point_q.q_lock, Q_LOCK_TIMEOUT) == OK)
    {
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
