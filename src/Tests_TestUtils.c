// Tests_TestUtils.c

// SPDX-FileCopyrightText: 2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifdef MOTOROS2_TESTING_ENABLE

#include "MotoROS.h"

BOOL Ros_Testing_CompareDouble(double a, double b)
{
    BOOL ok = fabs(a - b) < EPSILON_TOLERANCE_DOUBLE; //this approach has potential flaws, but it works just fine for these simple tests

    if (!ok)
        Ros_Debug_BroadcastMsg("Fail: %.04f != %.04f", a, b);

    return ok;
}

// This is necessary because rounding errors can cause the converted positions to be wrong by a pulse-count or two
BOOL Ros_Testing_CompareLong(long a, long b)
{
    BOOL ok = fabs(a - b) < EPSILON_TOLERANCE_PULSECOUNT; //this approach has potential flaws, but it works just fine for these simple tests

    if (!ok)
        Ros_Debug_BroadcastMsg("Fail: %d != %d", a, b);

    return ok;
}

BOOL Ros_Testing_INT64_Equals(INT64 a, INT64 b)
{
    BOOL ok = a == b;

    if (!ok)
        Ros_Debug_BroadcastMsg("Fail: %lld != %lld", a, b);

    return ok;
}

BOOL Ros_Testing_Timespec_Equals(const struct timespec* lhs, const struct timespec* rhs)
{
    BOOL ok = lhs->tv_sec == rhs->tv_sec;

    if (!ok)
    {
        Ros_Debug_BroadcastMsg("Fail timespec seconds: %d != %d", lhs->tv_sec, rhs->tv_sec);
    }

    ok &= lhs->tv_nsec == rhs->tv_nsec;

    if (!ok)
    {
        Ros_Debug_BroadcastMsg("Fail timespec nanos: %d != %d", lhs->tv_nsec, rhs->tv_nsec);
    }
    return ok;
}

#endif //MOTOROS2_TESTING_ENABLE
