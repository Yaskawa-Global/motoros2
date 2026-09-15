#ifndef MOTOROS2_TEST_STOP_ABORT_MODEL_H
#define MOTOROS2_TEST_STOP_ABORT_MODEL_H

#include "motoplus_stubs.h"

#define STOP_MODEL_MAX_GROUPS 4

typedef struct
{
    BOOL stop_motion;
    BOOL hold_active;
    BOOL point_queue_mode;
    BOOL incmove_done;
    int num_groups;
    BOOL inc_queue_clear_ok[STOP_MODEL_MAX_GROUPS];
    BOOL flush_requested[STOP_MODEL_MAX_GROUPS];
    BOOL direct_flushed[STOP_MODEL_MAX_GROUPS];
    int clear_attempts;
} StopAbortModel;

BOOL StopModel_InterpolationMayContinue(BOOL stop_motion);
BOOL StopModel_FullQueueWaitMayContinue(BOOL stop_motion, BOOL queue_full);
BOOL StopModel_StopMotion(StopAbortModel* model, BOOL quiesced,
    BOOL hold_apply_ok, BOOL hold_release_ok);
BOOL StopModel_AbortPointQueue(StopAbortModel* model, BOOL quiesced,
    BOOL hold_apply_ok, BOOL hold_release_ok);

#endif
