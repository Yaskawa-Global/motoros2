//TimeConversionUtils.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_TIME_CONVERSION_UTILS_H
#define MOTOROS2_TIME_CONVERSION_UTILS_H


#include "MotoROS.h"


static inline INT64 Ros_Duration_Msg_To_Millis(builtin_interfaces__msg__Duration const* const x)
{
    return ((INT64)x->sec * 1000) + (INT64)(x->nanosec * 0.000001);
}

static inline INT64 Ros_Duration_Msg_To_Nanos(builtin_interfaces__msg__Duration const* const x)
{
    return ((INT64)x->sec * 1000000000LL) + (INT64)x->nanosec;
}

static inline void Ros_Millis_To_Duration_Msg(INT64 x, builtin_interfaces__msg__Duration* const y)
{
    y->sec = x / 1000;
    y->nanosec = (x % 1000) * 1000000;
}

static inline void Ros_Nanos_To_Duration_Msg(INT64 x, builtin_interfaces__msg__Duration* const y)
{
    y->sec = x / 1000000000LL;
    y->nanosec = (x % 1000000000LL);
}

static inline INT64 Ros_Time_Msg_To_Millis(builtin_interfaces__msg__Time const* const x)
{
    return ((INT64)x->sec * 1000) + (INT64)(x->nanosec * 0.000001);
}

static inline INT64 Ros_Time_Msg_To_Nanos(builtin_interfaces__msg__Time const* const x)
{
    return ((INT64)x->sec * 1000000000LL) + (INT64)x->nanosec;
}

static inline void Ros_Millis_To_Time_Msg(INT64 x, builtin_interfaces__msg__Time* const y)
{
    y->sec = x / 1000;
    y->nanosec = (x % 1000) * 1000000;
}


static inline void Ros_Nanos_To_Time_Msg(INT64 x, builtin_interfaces__msg__Time* const y)
{
    y->sec = x / 1000000000LL;
    y->nanosec = (x % 1000000000LL);
}

static inline void Ros_Nanos_To_Timespec(INT64 x, struct timespec* const y)
{
    y->tv_sec = x / 1000000000LL;
    y->tv_nsec = (x % 1000000000LL);
}


#endif // MOTOROS2_TIME_CONVERSION_UTILS_H
