// motoplus_stubs.h — minimal host-gcc stubs for the point-queue unit tests.
//
// PURPOSE
// -------
// Lets the point-queue ring + admission logic be exercised with plain gcc on
// Linux, with NO MotoPlus toolchain and NO ROS. Only the symbols the tested
// code touches are provided here, and every stub is deliberately a no-op or a
// verbatim copy of the corresponding real declaration.
//
// WHAT IS REAL vs STUBBED (read carefully)
// ----------------------------------------
//   REAL (copied verbatim from motoros2/src/CtrlGroup.h):
//     - Q_LOCK_TIMEOUT, Q_OFFSET_IDX, POINT_QUEUE_DEPTH macros
//     - JointMotionData, PointQueue_q struct layouts
//   REAL (copied verbatim from the libmicroros-generated
//   motoros2_interfaces/.../queue_result_enum__struct.h):
//     - QueueResultEnum constant *values* (SUCCESS=1, BUSY=4,
//       INVALID_JOINT_LIST=5, UNABLE_TO_PROCESS_POINT=6, QUEUE_FULL=7)
//   STUBBED (host no-ops / minimal fakes, NOT real behavior):
//     - MotoPlus base typedefs (BOOL/LONG/UINT16/UINT64/UCHAR/SEM_ID)
//     - mpSemBCreate/mpSemTake/mpSemGive — always succeed, no locking
//     - a minimal CtrlGroup that carries only point_q
//
// The tested ring primitive *bodies* live in point_queue_ring.c and are
// byte-identical to those committed in motoros2/src/MotionControl.c (see the
// header banner in that file). The admission decision predicate lives in
// admission_model.c and mirrors, branch-for-branch, the admission checks in
// Ros_MotionControl_EnqueueTrajectoryPoint (also called out there).

#ifndef MOTOROS2_TEST_MOTOPLUS_STUBS_H
#define MOTOROS2_TEST_MOTOPLUS_STUBS_H

#include <string.h>

// --- MotoPlus base typedefs (host stubs) ---
typedef int                 BOOL;
typedef long                LONG;
typedef unsigned short      UINT16;
typedef unsigned long long  UINT64;
typedef unsigned char       UCHAR;
typedef void*               SEM_ID;

#ifndef TRUE
#define TRUE  1
#endif
#ifndef FALSE
#define FALSE 0
#endif
#ifndef OK
#define OK    0
#endif
#ifndef ERROR
#define ERROR (-1)
#endif

// semaphore ops: no-ops on host (always succeed; no real mutual exclusion)
#define SEM_Q_FIFO 0
#define SEM_FULL   1
static inline SEM_ID mpSemBCreate(int a, int b) { (void)a; (void)b; return (SEM_ID)1; }
static inline int    mpSemTake(SEM_ID s, int t) { (void)s; (void)t; return OK; }
static inline int    mpSemGive(SEM_ID s)        { (void)s; return OK; }

// --- copied verbatim from motoros2/src/CtrlGroup.h ---
#define Q_LOCK_TIMEOUT 5000
#define Q_OFFSET_IDX( a, b, c ) (((a)+(b)) >= (c) ) ? ((a)+(b)-(c)) \
                : ( (((a)+(b)) < 0 ) ? ((a)+(b)+(c)) : ((a)+(b)) )
#define POINT_QUEUE_DEPTH 32

#define MP_GRP_AXES_NUM 8

// JointMotionData — verbatim layout from CtrlGroup.h
typedef struct
{
    BOOL valid;                     // this point has valid data
    UINT64 time;                    // time in millisecond
    double pos[MP_GRP_AXES_NUM];    // position in radians
    double vel[MP_GRP_AXES_NUM];    // velocity in radians/s
} JointMotionData;

// PointQueue_q — verbatim layout from CtrlGroup.h
typedef struct
{
    SEM_ID q_lock;
    LONG cnt;                       // number of points currently queued
    LONG idx;                       // index of the oldest queued point
    JointMotionData data[POINT_QUEUE_DEPTH];
} PointQueue_q;

// Minimal CtrlGroup: only the member the tested code touches (point_q).
typedef struct
{
    PointQueue_q point_q;
} CtrlGroup;

// --- Admission policy enum (verbatim from motoros2/src/MotionControl.h) ---
typedef enum
{
    POINT_QUEUE_ADMIT_ONE_DEEP,
    POINT_QUEUE_ADMIT_FIFO
} PointQueueAdmitPolicy;

// --- QueueResultEnum constant values (verbatim from the generated header
//     motoros2_interfaces/msg/detail/queue_result_enum__struct.h) ---
enum {
    QRE_SUCCESS                 = 1,
    QRE_BUSY                    = 4,
    QRE_INVALID_JOINT_LIST      = 5,
    QRE_UNABLE_TO_PROCESS_POINT = 6,
    QRE_QUEUE_FULL              = 7
};

#endif // MOTOROS2_TEST_MOTOPLUS_STUBS_H
