//TimeConversionUtils.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_TIME_CONVERSION_UTILS_H
#define MOTOROS2_TIME_CONVERSION_UTILS_H


#include "MotoROS.h"

static inline void floor_divide(INT64 dividend, INT64 divisor, INT64* quotient, INT64* remainder)
{
    *quotient = dividend / divisor;
    // If the division has a remainder, and the result is negative,
    // we need to adjust to floor the result
    if (dividend % divisor != 0 && ((dividend < 0) != (divisor < 0)))
    {
        *quotient -= 1;
    }

    *remainder = abs(dividend - *quotient * divisor);
}

static inline INT64 Ros_Duration_Msg_To_Millis(builtin_interfaces__msg__Duration const* const x)
{
    return (x->sec * (INT64)1e3) + (x->nanosec / (INT64)1e6);
}

static inline INT64 Ros_Duration_Msg_To_Nanos(builtin_interfaces__msg__Duration const* const x)
{
    return (x->sec * (INT64)1e9) + (x->nanosec);
}

static inline void Ros_Millis_To_Duration_Msg(INT64 x, builtin_interfaces__msg__Duration* const y)
{
    INT64 remainder, quotient;
    floor_divide(x, (INT64)1e3, &quotient, &remainder);
    y->sec = quotient;
    y->nanosec = remainder * (INT64)1e6;
}

static inline void Ros_Nanos_To_Duration_Msg(INT64 x, builtin_interfaces__msg__Duration* const y)
{
    INT64 remainder, quotient;
    floor_divide(x, (INT64)1e9, &quotient, &remainder);
    y->sec = quotient;
    y->nanosec = remainder;
}

static inline INT64 Ros_Time_Msg_To_Millis(builtin_interfaces__msg__Time const* const x)
{
    return (x->sec * (INT64)1e3) + (x->nanosec / ((INT64)1e6));
}

static inline INT64 Ros_Time_Msg_To_Nanos(builtin_interfaces__msg__Time const* const x)
{
    return (x->sec * (INT64)1e9) + x->nanosec;
}

static inline void Ros_Millis_To_Time_Msg(INT64 x, builtin_interfaces__msg__Time* const y)
{
    INT64 remainder, quotient;
    floor_divide(x, (INT64)1e3, &quotient, &remainder);
    y->sec = quotient;
    y->nanosec = remainder * (INT64)1e6;
}

static inline void Ros_Nanos_To_Time_Msg(INT64 x, builtin_interfaces__msg__Time* const y)
{
    INT64 remainder, quotient;
    floor_divide(x, (INT64)1e9, &quotient, &remainder);
    y->sec = quotient;
    y->nanosec = remainder;
}

static inline void Ros_Nanos_To_Timespec(INT64 x, struct timespec* const y)
{
    INT64 remainder, quotient;
    floor_divide(x, (INT64)1e9, &quotient, &remainder);
    y->tv_sec = quotient;
    y->tv_nsec = remainder;
}


#endif // MOTOROS2_TIME_CONVERSION_UTILS_H
