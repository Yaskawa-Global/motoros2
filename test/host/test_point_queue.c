// test_point_queue.c — host-gcc unit tests for the point-queue feature.
//
// TIER 1 (ring logic, REAL runtime — bodies verbatim from MotionControl.c):
//   - enqueue/dequeue FIFO order
//   - wraparound past POINT_QUEUE_DEPTH
//   - count correctness
//   - full rejects enqueue; empty rejects dequeue
//
// TIER 2 (admission policy + underran latch, REAL runtime for the ring +
//         policy branches; the underran latch is a MODEL of Task 5 semantics):
//   - ONE_DEEP: first enqueue SUCCESS (depth 1), second returns BUSY
//   - FIFO: fills to POINT_QUEUE_DEPTH (SUCCESS, increasing depth), next QUEUE_FULL
//   - underran latch: starts TRUE; ReadAndClear TRUE then FALSE; consumer-empty
//     re-latches TRUE; read-and-clear semantics hold
//   - legacy-equivalence: ONE_DEEP yields SUCCESS then BUSY, never QUEUE_FULL
//
// Build+run via test/host/Makefile or test/host/run_tests.sh. Exits non-zero on
// any failure. Uses plain asserts (custom ASSERT that keeps counting).

#include <stdio.h>
#include "admission_model.h"

static int failures = 0;
#define ASSERT(cond) do { \
    if (!(cond)) { printf("FAIL: %s (line %d)\n", #cond, __LINE__); failures++; } \
    else { printf("ok:   %s\n", #cond); } \
} while (0)

static void new_group(CtrlGroup* g)
{
    memset(g, 0, sizeof(*g));
    g->point_q.q_lock = mpSemBCreate(SEM_Q_FIFO, SEM_FULL);
    // mirror CtrlGroup.c init: bracket guards set to MAGIC after bzero
    g->point_q.guard_pre = POINT_QUEUE_GUARD_MAGIC;
    g->point_q.guard_post = POINT_QUEUE_GUARD_MAGIC;
}

//======================= TIER 1 : ring logic ==============================

// Brief's canonical FIFO-order test.
static void test_point_queue_fifo_order(void)
{
    printf("== TIER1 test_point_queue_fifo_order ==\n");
    CtrlGroup g; new_group(&g);
    JointMotionData a; memset(&a, 0, sizeof(a)); a.time = 10;
    JointMotionData b; memset(&b, 0, sizeof(b)); b.time = 20;
    ASSERT(Ros_MotionControl_PointQueueEnqueue(&g, &a));
    ASSERT(Ros_MotionControl_PointQueueEnqueue(&g, &b));
    ASSERT(Ros_MotionControl_PointQueueCount(&g) == 2);
    JointMotionData out;
    ASSERT(Ros_MotionControl_PointQueueDequeue(&g, &out) && out.time == 10);
    ASSERT(Ros_MotionControl_PointQueueDequeue(&g, &out) && out.time == 20);
    ASSERT(!Ros_MotionControl_PointQueueDequeue(&g, &out));
}

// empty rejects dequeue.
static void test_point_queue_empty_rejects_dequeue(void)
{
    printf("== TIER1 test_point_queue_empty_rejects_dequeue ==\n");
    CtrlGroup g; new_group(&g);
    JointMotionData out;
    ASSERT(Ros_MotionControl_PointQueueCount(&g) == 0);
    ASSERT(!Ros_MotionControl_PointQueueDequeue(&g, &out));
}

// full rejects enqueue + count correctness at the boundary.
static void test_point_queue_full_rejects_enqueue(void)
{
    printf("== TIER1 test_point_queue_full_rejects_enqueue ==\n");
    CtrlGroup g; new_group(&g);
    for (int i = 0; i < POINT_QUEUE_DEPTH; i++) {
        JointMotionData p; memset(&p, 0, sizeof(p)); p.time = (UINT64)(1000 + i);
        ASSERT(Ros_MotionControl_PointQueueEnqueue(&g, &p));
        ASSERT(Ros_MotionControl_PointQueueCount(&g) == i + 1);
    }
    ASSERT(Ros_MotionControl_PointQueueCount(&g) == POINT_QUEUE_DEPTH);
    JointMotionData overflow; memset(&overflow, 0, sizeof(overflow)); overflow.time = 9999;
    ASSERT(!Ros_MotionControl_PointQueueEnqueue(&g, &overflow));
    ASSERT(Ros_MotionControl_PointQueueCount(&g) == POINT_QUEUE_DEPTH);
}

// guard sentinels: corrupting either bracket word makes Enqueue/Dequeue fail
// safe (return FALSE) and Count report ERROR. Mirrors the fail-safe guard
// validation in MotionControl.c's ring primitives.
static void test_point_queue_guard_corruption_fails_safe(void)
{
    printf("== TIER1 test_point_queue_guard_corruption_fails_safe ==\n");

    // pre-guard corruption
    CtrlGroup g1; new_group(&g1);
    JointMotionData p; memset(&p, 0, sizeof(p)); p.time = 7;
    ASSERT(Ros_MotionControl_PointQueueEnqueue(&g1, &p)); // healthy
    g1.point_q.guard_pre = 0xDEADBEEFu;                   // corrupt
    JointMotionData out;
    ASSERT(!Ros_MotionControl_PointQueueEnqueue(&g1, &p));
    ASSERT(!Ros_MotionControl_PointQueueDequeue(&g1, &out));
    ASSERT(Ros_MotionControl_PointQueueCount(&g1) == ERROR);

    // post-guard corruption
    CtrlGroup g2; new_group(&g2);
    ASSERT(Ros_MotionControl_PointQueueEnqueue(&g2, &p)); // healthy
    g2.point_q.guard_post = 0xDEADBEEFu;                  // corrupt
    ASSERT(!Ros_MotionControl_PointQueueEnqueue(&g2, &p));
    ASSERT(!Ros_MotionControl_PointQueueDequeue(&g2, &out));
    ASSERT(Ros_MotionControl_PointQueueCount(&g2) == ERROR);
}

// wraparound past POINT_QUEUE_DEPTH: drain half, refill, forcing idx to wrap.
static void test_point_queue_wraparound(void)
{
    printf("== TIER1 test_point_queue_wraparound ==\n");
    CtrlGroup g; new_group(&g);

    for (int i = 0; i < POINT_QUEUE_DEPTH; i++) {
        JointMotionData p; memset(&p, 0, sizeof(p)); p.time = (UINT64)(1000 + i);
        ASSERT(Ros_MotionControl_PointQueueEnqueue(&g, &p));
    }
    ASSERT(Ros_MotionControl_PointQueueCount(&g) == POINT_QUEUE_DEPTH);

    // drain half in FIFO order
    for (int i = 0; i < POINT_QUEUE_DEPTH / 2; i++) {
        JointMotionData out;
        ASSERT(Ros_MotionControl_PointQueueDequeue(&g, &out) && out.time == (UINT64)(1000 + i));
    }
    ASSERT(Ros_MotionControl_PointQueueCount(&g) == POINT_QUEUE_DEPTH / 2);

    // refill half -> idx wraps past end of the backing buffer
    for (int i = 0; i < POINT_QUEUE_DEPTH / 2; i++) {
        JointMotionData p; memset(&p, 0, sizeof(p)); p.time = (UINT64)(2000 + i);
        ASSERT(Ros_MotionControl_PointQueueEnqueue(&g, &p));
    }
    ASSERT(Ros_MotionControl_PointQueueCount(&g) == POINT_QUEUE_DEPTH);

    // drain all, verify order: remaining originals then new ones
    for (int i = POINT_QUEUE_DEPTH / 2; i < POINT_QUEUE_DEPTH; i++) {
        JointMotionData out;
        ASSERT(Ros_MotionControl_PointQueueDequeue(&g, &out) && out.time == (UINT64)(1000 + i));
    }
    for (int i = 0; i < POINT_QUEUE_DEPTH / 2; i++) {
        JointMotionData out;
        ASSERT(Ros_MotionControl_PointQueueDequeue(&g, &out) && out.time == (UINT64)(2000 + i));
    }
    ASSERT(Ros_MotionControl_PointQueueCount(&g) == 0);
    JointMotionData out;
    ASSERT(!Ros_MotionControl_PointQueueDequeue(&g, &out));
}

//================ TIER 2 : admission policy + underran latch ===============

// ONE_DEEP: first SUCCESS (depth 1), second BUSY.
static void test_admit_one_deep(void)
{
    printf("== TIER2 test_admit_one_deep ==\n");
    CtrlGroup g; new_group(&g);
    JointMotionData p1; memset(&p1, 0, sizeof(p1)); p1.time = 1;
    JointMotionData p2; memset(&p2, 0, sizeof(p2)); p2.time = 2;
    UINT16 depth = 0xFFFF;
    ASSERT(PointQueue_AdmitOne(&g, POINT_QUEUE_ADMIT_ONE_DEEP, &p1, &depth) == QRE_SUCCESS);
    ASSERT(depth == 1);
    ASSERT(PointQueue_AdmitOne(&g, POINT_QUEUE_ADMIT_ONE_DEEP, &p2, &depth) == QRE_BUSY);
    ASSERT(Ros_MotionControl_PointQueueCount(&g) == 1); // BUSY did not enqueue
}

// FIFO: fills to POINT_QUEUE_DEPTH (SUCCESS, increasing depth), next QUEUE_FULL.
static void test_admit_fifo_fill_then_full(void)
{
    printf("== TIER2 test_admit_fifo_fill_then_full ==\n");
    CtrlGroup g; new_group(&g);
    for (int i = 0; i < POINT_QUEUE_DEPTH; i++) {
        JointMotionData p; memset(&p, 0, sizeof(p)); p.time = (UINT64)i;
        UINT16 depth = 0xFFFF;
        ASSERT(PointQueue_AdmitOne(&g, POINT_QUEUE_ADMIT_FIFO, &p, &depth) == QRE_SUCCESS);
        ASSERT(depth == (UINT16)(i + 1)); // depth increases 1..DEPTH
    }
    JointMotionData over; memset(&over, 0, sizeof(over)); over.time = 999;
    UINT16 depth = 0xFFFF;
    ASSERT(PointQueue_AdmitOne(&g, POINT_QUEUE_ADMIT_FIFO, &over, &depth) == QRE_QUEUE_FULL);
    ASSERT(Ros_MotionControl_PointQueueCount(&g) == POINT_QUEUE_DEPTH); // FULL did not enqueue
}

// underran latch: starts TRUE; ReadAndClear TRUE then FALSE; consumer-empty
// re-latches TRUE; read-and-clear semantics hold.
static void test_underran_latch(void)
{
    printf("== TIER2 test_underran_latch ==\n");
    UnderranLatch_Init();
    ASSERT(UnderranLatch_ReadAndClear() == TRUE);   // starts TRUE
    ASSERT(UnderranLatch_ReadAndClear() == FALSE);  // cleared by previous read
    UnderranLatch_SetConsumerEmpty();               // simulated consumer-empty
    ASSERT(UnderranLatch_ReadAndClear() == TRUE);   // latched again
    ASSERT(UnderranLatch_ReadAndClear() == FALSE);  // and cleared again
}

// legacy-equivalence: ONE_DEEP path yields SUCCESS then BUSY, and NEVER
// QUEUE_FULL (across many attempts against a one-deep ring).
static void test_legacy_equivalence(void)
{
    printf("== TIER2 test_legacy_equivalence ==\n");
    CtrlGroup g; new_group(&g);
    JointMotionData p; memset(&p, 0, sizeof(p)); p.time = 42;
    ASSERT(PointQueue_AdmitOne(&g, POINT_QUEUE_ADMIT_ONE_DEEP, &p, NULL) == QRE_SUCCESS);
    for (int i = 0; i < 100; i++) {
        JointMotionData q; memset(&q, 0, sizeof(q)); q.time = (UINT64)i;
        UINT16 r = PointQueue_AdmitOne(&g, POINT_QUEUE_ADMIT_ONE_DEEP, &q, NULL);
        ASSERT(r == QRE_BUSY);
        ASSERT(r != QRE_QUEUE_FULL);
    }
}

int main(void)
{
    // Tier 1
    test_point_queue_fifo_order();
    test_point_queue_empty_rejects_dequeue();
    test_point_queue_full_rejects_enqueue();
    test_point_queue_wraparound();
    test_point_queue_guard_corruption_fails_safe();
    // Tier 2
    test_admit_one_deep();
    test_admit_fifo_fill_then_full();
    test_underran_latch();
    test_legacy_equivalence();

    if (failures == 0) { printf("\nALL PASS\n"); return 0; }
    printf("\n%d FAILURE(S)\n", failures);
    return 1;
}
