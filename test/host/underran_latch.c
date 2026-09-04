// underran_latch.c — MODEL of the point-queue underran latch (Task 5 preview).
//
// NOT REAL SOURCE. The real latch/accessor is Task 5 and is not yet in the
// committed tree. This models the specified read-and-clear latch semantics so
// the host harness can lock them in now; replace with tests against the real
// accessor once Task 5 lands. See admission_model.h for the full note.

#include "admission_model.h"

static BOOL s_underran_since_last_call = TRUE; // starts TRUE by spec

void UnderranLatch_Init(void)
{
    s_underran_since_last_call = TRUE;
}

void UnderranLatch_SetConsumerEmpty(void)
{
    // The single SET site: the consumer observed the ring empty.
    s_underran_since_last_call = TRUE;
}

BOOL UnderranLatch_ReadAndClear(void)
{
    BOOL v = s_underran_since_last_call;
    s_underran_since_last_call = FALSE;
    return v;
}
