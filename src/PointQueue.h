// PointQueue.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_POINT_QUEUE_H
#define MOTOROS2_POINT_QUEUE_H

#define POINT_QUEUE_DEPTH 32   // depth of the point-queue FIFO (see streaming-point-fifo spec); tunable, single source of truth
#define POINT_QUEUE_GUARD_MAGIC 0x51504751u   // bracket-word sentinel value guarding the point-queue data[] array

// Usable slots must equal POINT_QUEUE_DEPTH. This ring is LOCK-FREE SPSC using
// the "keep one slot empty" full/empty discriminator, so the backing array is
// sized POINT_QUEUE_DEPTH+1: one slot is always left unused to tell full
// (head == (tail+1) % SLOTS) from empty (head == tail). Usable capacity is
// therefore POINT_QUEUE_DEPTH, exactly as the macro means. This scheme is
// depth-agnostic (no power-of-two requirement).
#define POINT_QUEUE_SLOTS (POINT_QUEUE_DEPTH + 1)

// Ring FIFO of trajectory points awaiting interpolation (point-queue mode).
//
// LOCK-FREE single-producer / single-consumer (SPSC). LOAD-BEARING INVARIANT:
// single enqueue-caller task + single dequeue-caller task; NOT lock-free-safe
// if a second producer or consumer, or a multi-threaded executor, is introduced.
//   - Producer OWNS `tail` (advanced only by enqueue). Consumer OWNS `head`
//     (advanced only by dequeue). Neither side writes the other's index.
//   - Depth is DERIVED from (tail - head) mod SLOTS; there is no shared count.
//   - Memory ordering via GCC __sync_synchronize() barriers (see enqueue/
//     dequeue in PointQueue.c): producer publishes payload-then-tail with a
//     release barrier; consumer observes tail-then-payload with an acquire
//     barrier. head/tail are single word-aligned (naturally atomic on x86).
typedef struct
{
    volatile LONG head;             // CONSUMER-owned: index of oldest queued point (dequeue advances)
    volatile LONG tail;             // PRODUCER-owned: index of next free slot (enqueue advances)
    // ASYNC-SAFE FLUSH REQUEST: set TRUE by any (possibly async, non-quiescing)
    // caller that wants the ring emptied; ACTIONED (and cleared) exclusively by
    // the consumer at the top of Ros_MotionControl_AddToIncQueueProcess. The
    // async setter touches NEITHER head NOR tail, so the single-writer-each SPSC
    // invariant is preserved even from the IO-status monitor task. Init FALSE at
    // CtrlGroup construction (bzero of point_q).
    volatile BOOL flushRequested;   // CONSUMER-cleared flush request; setter writes neither index
    UINT32 guard_pre;               // == POINT_QUEUE_GUARD_MAGIC (pre-buffer sentinel)
    JointMotionData data[POINT_QUEUE_SLOTS];
    UINT32 guard_post;              // == POINT_QUEUE_GUARD_MAGIC (post-buffer sentinel)
} PointQueue_q;

// CtrlGroup embeds a PointQueue_q and is defined just below this #include in
// CtrlGroup.h; forward-declare the tag so the ring prototypes below (which take
// CtrlGroup*) name the same struct type rather than a prototype-scoped one.
struct _CtrlGroup;
typedef struct _CtrlGroup CtrlGroup;

// Point-queue ring primitives (point-queue mode). LOCK-FREE SPSC: enqueue is
// producer-only (single enqueue-caller task), dequeue is consumer-only (single
// dequeue-caller task). See the load-bearing SPSC invariant at PointQueue_q.
extern LONG Ros_MotionControl_PointQueueCount(CtrlGroup* ctrlGroup);          // derived depth; ERROR on guard corruption.
extern BOOL Ros_MotionControl_PointQueueEnqueue(CtrlGroup* ctrlGroup, JointMotionData* pt); // producer append; FALSE if full/corrupt.
extern BOOL Ros_MotionControl_PointQueueDequeue(CtrlGroup* ctrlGroup, JointMotionData* out); // consumer pop oldest; FALSE if empty/corrupt.

// DIRECT flush: reset the ring to empty (head == tail == 0), discarding queued
// points. QUIESCED-CALLER-ONLY: writes both indices, so it is race-free only when
// the consumer is provably stopped (Ros_MotionControl_StopMotion, which waits for
// !HasDataToProcess() first). Does NOT touch the sticky underran flag (spec 5.5).
extern void Ros_MotionControl_PointQueueFlush(CtrlGroup* ctrlGroup);

// ASYNC-SAFE flush: request that the consumer empty the ring. Sets the per-group
// flushRequested flag and returns; touches NEITHER head NOR tail. Safe from any
// context, including async callers that do not quiesce the consumer (IO-status
// monitor). The consumer actions it (head = tail) at the top of its loop. Used by
// Ros_MotionControl_ClearQ_All. Does NOT touch the sticky underran flag (spec 5.5).
extern void Ros_MotionControl_PointQueueRequestFlush(CtrlGroup* ctrlGroup);

#endif  // MOTOROS2_POINT_QUEUE_H
