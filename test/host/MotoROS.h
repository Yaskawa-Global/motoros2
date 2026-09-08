// MotoROS.h — HOST-ONLY shim so the REAL production src/PointQueue.c can be
// compiled/#included on plain gcc (no MotoPlus toolchain, no micro-ROS).
//
// WHY THIS EXISTS
// ---------------
// src/PointQueue.c begins with `#include "MotoROS.h"` (the project umbrella
// header) exactly like every other production .c. On the controller that drags
// in motoPlus.h + the full micro-ROS header set, which cannot be satisfied by
// plain gcc. Rather than DUPLICATE the ring bodies into the test tree (the old
// point_queue_ring.c approach), we shim just the two things the ring leaf code
// actually needs from MotoROS.h and let the host build pull the REAL bodies:
//   - the base types + PointQueue_q/CtrlGroup/JointMotionData layouts
//     (provided verbatim by motoplus_stubs.h, shared with the other host TUs so
//      all translation units agree on the struct types)
//   - the fail-safe alarm hook the guard-check calls on corruption
//
// The host build compiles src/PointQueue.c with `-iquote <this dir> -I-` so this
// shim (not src/MotoROS.h) resolves for that one translation unit. See
// test/host/Makefile / run_tests.sh.
//
// This is a TEST shim only; it ships nothing and changes no production behavior.

#ifndef MOTOROS2_TEST_HOST_MOTOROS_SHIM_H
#define MOTOROS2_TEST_HOST_MOTOROS_SHIM_H

#include "motoplus_stubs.h"

// --- minimal ErrorHandling.h surface used by the ring guard-check ---
// Production raises mpSetAlarm(ALARM_ASSERTION_FAIL, msg,
// SUBCODE_POINT_QUEUE_GUARD_CORRUPTION) on a guard mismatch before failing safe.
// On host there is no controller alarm subsystem; the hook is a no-op so the
// EXACT fail-safe RETURN behavior (FALSE / ERROR) of the real bodies is exercised.
enum { ALARM_ASSERTION_FAIL = 8011 };
enum { SUBCODE_POINT_QUEUE_GUARD_CORRUPTION = 1 };
static inline void mpSetAlarm(int alarmCode, const char* msg, int subCode)
{
    (void)alarmCode; (void)msg; (void)subCode;
}

#endif // MOTOROS2_TEST_HOST_MOTOROS_SHIM_H
