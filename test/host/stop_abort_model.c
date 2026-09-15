// Host orchestration model for MotionControl stop and abort branch ordering.
#include "stop_abort_model.h"

BOOL StopModel_InterpolationMayContinue(BOOL stop_motion)
{
    return !stop_motion;
}

BOOL StopModel_FullQueueWaitMayContinue(BOOL stop_motion, BOOL queue_full)
{
    return queue_full && !stop_motion;
}

static BOOL StopModel_ClearQ_All(StopAbortModel* model)
{
    BOOL all_cleared = TRUE;
    for (int group = 0; group < model->num_groups; ++group)
    {
        model->clear_attempts++;
        model->flush_requested[group] = TRUE;
        if (!model->inc_queue_clear_ok[group])
            all_cleared = FALSE;
    }
    return all_cleared;
}

BOOL StopModel_StopMotion(StopAbortModel* model, BOOL quiesced,
    BOOL hold_apply_ok, BOOL hold_release_ok)
{
    model->stop_motion = TRUE;
    if (hold_apply_ok)
        model->hold_active = TRUE;

    BOOL queues_cleared = StopModel_ClearQ_All(model);
    if (quiesced)
    {
        for (int group = 0; group < model->num_groups; ++group)
            model->direct_flushed[group] = TRUE;
    }

    if (!hold_apply_ok || !quiesced || !queues_cleared)
        return FALSE;

    if (!hold_release_ok)
        return FALSE;
    model->hold_active = FALSE;
    model->stop_motion = FALSE;
    return TRUE;
}

BOOL StopModel_AbortPointQueue(StopAbortModel* model, BOOL quiesced,
    BOOL hold_apply_ok, BOOL hold_release_ok)
{
    if (!model->point_queue_mode)
        return FALSE;

    BOOL stopped = StopModel_StopMotion(model, quiesced,
        hold_apply_ok, hold_release_ok);
    model->point_queue_mode = FALSE;
    if (!stopped)
        return FALSE;

    model->incmove_done = TRUE;
    return TRUE;
}
