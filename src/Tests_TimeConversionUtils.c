// Tests_TimeConversionUtils.c

// SPDX-FileCopyrightText: 2025, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2025, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifdef MOTOROS2_TESTING_ENABLE

#include "MotoROS.h"

BOOL Ros_Testing_Ros_Duration_Msg_To_Millis()
{
    BOOL bSuccess = TRUE;
    INT64 millis;

    const builtin_interfaces__msg__Duration M1 = { 0, 0 };
    millis = Ros_Duration_Msg_To_Millis(&M1);
    bSuccess &= Ros_Testing_INT64_Equals(0, millis);

    const builtin_interfaces__msg__Duration M2 = { 0, 499999999 };
    millis = Ros_Duration_Msg_To_Millis(&M2);
    bSuccess &= Ros_Testing_INT64_Equals(499, millis);

    const builtin_interfaces__msg__Duration M3 = {0, 500000000};
    millis = Ros_Duration_Msg_To_Millis(&M3);
    bSuccess &= Ros_Testing_INT64_Equals(500, millis);

    const builtin_interfaces__msg__Duration M4 = { 1, 864 };
    millis = Ros_Duration_Msg_To_Millis(&M4);
    bSuccess &= Ros_Testing_INT64_Equals(1000, millis);

    const builtin_interfaces__msg__Duration M5 = { 999, 1000000 };
    millis = Ros_Duration_Msg_To_Millis(&M5);
    bSuccess &= Ros_Testing_INT64_Equals(999001, millis);

    const builtin_interfaces__msg__Duration M6 = { -1, 0 };
    millis = Ros_Duration_Msg_To_Millis(&M6);
    bSuccess &= Ros_Testing_INT64_Equals(-1000, millis);

    const builtin_interfaces__msg__Duration M7 = { -1, 1000000 };
    millis = Ros_Duration_Msg_To_Millis(&M7);
    bSuccess &= Ros_Testing_INT64_Equals(-999, millis);

    const builtin_interfaces__msg__Duration M8 = { INT_MIN, 0 };
    millis = Ros_Duration_Msg_To_Millis(&M8);
    bSuccess &= Ros_Testing_INT64_Equals(INT_MIN * 1000LL, millis);

    const builtin_interfaces__msg__Duration M9 = { INT_MIN, 999999999 };
    millis = Ros_Duration_Msg_To_Millis(&M9);
    bSuccess &= Ros_Testing_INT64_Equals(INT_MIN * 1000LL + 999, millis);

    Ros_Debug_BroadcastMsg("Testing Ros_Duration_Msg_To_Millis: %s", bSuccess ? "PASS" : "FAIL");
    return bSuccess;
}

BOOL Ros_Testing_Ros_Duration_Msg_To_Nanos()
{
    BOOL bSuccess = TRUE;
    INT64 nanos;

    const builtin_interfaces__msg__Duration M1 = { 0, 0 };
    nanos = Ros_Duration_Msg_To_Nanos(&M1);
    bSuccess &= Ros_Testing_INT64_Equals(0, nanos);

    const builtin_interfaces__msg__Duration M2 = { 0, 499999999 };
    nanos = Ros_Duration_Msg_To_Nanos(&M2);
    bSuccess &= Ros_Testing_INT64_Equals(499999999, nanos);

    const builtin_interfaces__msg__Duration M3 = { 0, 500000000 };
    nanos = Ros_Duration_Msg_To_Nanos(&M3);
    bSuccess &= Ros_Testing_INT64_Equals(500000000, nanos);

    const builtin_interfaces__msg__Duration M4 = { 1, 864 };
    nanos = Ros_Duration_Msg_To_Nanos(&M4);
    bSuccess &= Ros_Testing_INT64_Equals(1000000864LL, nanos);

    const builtin_interfaces__msg__Duration M5 = { 999, 1000000 };
    nanos = Ros_Duration_Msg_To_Nanos(&M5);
    bSuccess &= Ros_Testing_INT64_Equals(999001000000, nanos);

    const builtin_interfaces__msg__Duration M6 = { -1, 499999999 };
    nanos = Ros_Duration_Msg_To_Nanos(&M6);
    bSuccess &= Ros_Testing_INT64_Equals(-500000001, nanos);

    const builtin_interfaces__msg__Duration M7= { -1, 0 };
    nanos = Ros_Duration_Msg_To_Nanos(&M7);
    bSuccess &= Ros_Testing_INT64_Equals(-1000000000, nanos);

    const builtin_interfaces__msg__Duration M8 = { INT_MIN, 864 };
    nanos = Ros_Duration_Msg_To_Nanos(&M8);
    bSuccess &= Ros_Testing_INT64_Equals(INT_MIN * 1000LL * 1000LL * 1000LL + 864, nanos);

    const builtin_interfaces__msg__Duration M9 = { INT_MIN, 0 };
    nanos = Ros_Duration_Msg_To_Nanos(&M9);
    bSuccess &= Ros_Testing_INT64_Equals(INT_MIN * 1000LL * 1000LL * 1000LL, nanos);

    Ros_Debug_BroadcastMsg("Testing Ros_Duration_Msg_To_Nanos: %s", bSuccess ? "PASS" : "FAIL");
    return bSuccess;
}

BOOL Ros_Testing_Ros_Millis_To_Duration_Msg()
{
    BOOL bSuccess = TRUE;
    INT64 millis;

    builtin_interfaces__msg__Duration M1, M2;

    M1.sec = 0;
    M1.nanosec = 0;
    millis = 0;
    Ros_Millis_To_Duration_Msg(millis, &M2);
    bSuccess &= builtin_interfaces__msg__Duration__are_equal(&M1, &M2);

    M1.sec = 0;
    M1.nanosec = 999000000;
    millis = 999;
    Ros_Millis_To_Duration_Msg(millis, &M2);
    bSuccess &= builtin_interfaces__msg__Duration__are_equal(&M1, &M2);

    M1.sec = 0;
    M1.nanosec = 999999999;
    millis = 999;
    Ros_Millis_To_Duration_Msg(millis, &M2);
    bSuccess &= builtin_interfaces__msg__Duration__are_equal(&M1, &M2);

    M1.sec = 10000000;
    M1.nanosec = 98353503;
    millis = 10000000098;
    Ros_Millis_To_Duration_Msg(millis, &M2);
    bSuccess &= builtin_interfaces__msg__Duration__are_equal(&M1, &M2);

    M1.sec = -1;
    M1.nanosec = 0;
    millis = -1000;
    Ros_Millis_To_Duration_Msg(millis, &M2);
    bSuccess &= builtin_interfaces__msg__Duration__are_equal(&M1, &M2);

    M1.sec = -1;
    M1.nanosec = 1000000;
    millis = -999;
    Ros_Millis_To_Duration_Msg(millis, &M2);
    bSuccess &= builtin_interfaces__msg__Duration__are_equal(&M1, &M2);

    M1.sec = -1;
    M1.nanosec = 999000000;
    millis = -1;
    Ros_Millis_To_Duration_Msg(millis, &M2);
    bSuccess &= builtin_interfaces__msg__Duration__are_equal(&M1, &M2);

    M1.sec = INT_MIN;
    M1.nanosec = 999000000;
    millis = INT_MIN * 1000LL + 999;
    Ros_Millis_To_Duration_Msg(millis, &M2);
    bSuccess &= builtin_interfaces__msg__Duration__are_equal(&M1, &M2);

    Ros_Debug_BroadcastMsg("Testing Ros_Millis_To_Duration_Msg: %s", bSuccess ? "PASS" : "FAIL");
    return bSuccess;
}

BOOL Ros_Testing_Ros_Nanos_To_Duration_Msg()
{
    BOOL bSuccess = TRUE;
    INT64 nanos;

    builtin_interfaces__msg__Duration M1, M2;

    M1.sec = 0;
    M1.nanosec = 0;
    nanos = 0;
    Ros_Nanos_To_Duration_Msg(nanos, &M2);
    bSuccess &= builtin_interfaces__msg__Duration__are_equal(&M1, &M2);

    M1.sec = 0;
    M1.nanosec = 99999999LL;
    nanos = 99999999LL;
    Ros_Nanos_To_Duration_Msg(nanos, &M2);
    bSuccess &= builtin_interfaces__msg__Duration__are_equal(&M1, &M2);

    M1.sec = 8;
    M1.nanosec = 275937194LL;
    nanos = 8275937194LL;
    Ros_Nanos_To_Duration_Msg(nanos, &M2);
    bSuccess &= builtin_interfaces__msg__Duration__are_equal(&M1, &M2);

    M1.sec = 98375198;
    M1.nanosec = 1;
    nanos = 98375198000000001;
    Ros_Nanos_To_Duration_Msg(nanos, &M2);
    bSuccess &= builtin_interfaces__msg__Duration__are_equal(&M1, &M2);

    M1.sec = -1;
    M1.nanosec = 999999999;
    nanos = -1;
    Ros_Nanos_To_Duration_Msg(nanos, &M2);
    bSuccess &= builtin_interfaces__msg__Duration__are_equal(&M1, &M2);

    M1.sec = -1;
    M1.nanosec = 1;
    nanos = -999999999;
    Ros_Nanos_To_Duration_Msg(nanos, &M2);
    bSuccess &= builtin_interfaces__msg__Duration__are_equal(&M1, &M2);

    M1.sec = INT_MIN;
    M1.nanosec = 0;
    nanos = INT_MIN * 1000LL * 1000LL * 1000LL;
    Ros_Nanos_To_Duration_Msg(nanos, &M2);
    bSuccess &= builtin_interfaces__msg__Duration__are_equal(&M1, &M2);

    Ros_Debug_BroadcastMsg("Testing Ros_Nanos_To_Duration_Msg: %s", bSuccess ? "PASS" : "FAIL");
    return bSuccess;
}

BOOL Ros_Testing_Ros_Time_Msg_To_Millis()
{
    BOOL bSuccess = TRUE;
    INT64 millis;

    const builtin_interfaces__msg__Time M1 = { 0, 0 };
    millis = Ros_Time_Msg_To_Millis(&M1);
    bSuccess &= Ros_Testing_INT64_Equals(0, millis);

    const builtin_interfaces__msg__Time M2 = { 0, 499999999 };
    millis = Ros_Time_Msg_To_Millis(&M2);
    bSuccess &= Ros_Testing_INT64_Equals(499, millis);

    const builtin_interfaces__msg__Time M3 = { 0, 500000000 };
    millis = Ros_Time_Msg_To_Millis(&M3);
    bSuccess &= Ros_Testing_INT64_Equals(500, millis);

    const builtin_interfaces__msg__Time M4 = { 1, 864 };
    millis = Ros_Time_Msg_To_Millis(&M4);
    bSuccess &= Ros_Testing_INT64_Equals(1000, millis);

    const builtin_interfaces__msg__Time M5 = { 999, 1000000 };
    millis = Ros_Time_Msg_To_Millis(&M5);
    bSuccess &= Ros_Testing_INT64_Equals(999001, millis);

    const builtin_interfaces__msg__Time M6 = { -1, 0 };
    millis = Ros_Time_Msg_To_Millis(&M6);
    bSuccess &= Ros_Testing_INT64_Equals(-1000, millis);

    const builtin_interfaces__msg__Time M7 = { -1, 1000000 };
    millis = Ros_Time_Msg_To_Millis(&M7);
    bSuccess &= Ros_Testing_INT64_Equals(-999, millis);

    const builtin_interfaces__msg__Time M8 = { INT_MIN, 0 };
    millis = Ros_Time_Msg_To_Millis(&M8);
    bSuccess &= Ros_Testing_INT64_Equals(INT_MIN * 1000LL, millis);

    const builtin_interfaces__msg__Time M9 = { INT_MIN, 999999999 };
    millis = Ros_Time_Msg_To_Millis(&M9);
    bSuccess &= Ros_Testing_INT64_Equals(INT_MIN * 1000LL + 999, millis);

    Ros_Debug_BroadcastMsg("Testing Ros_Time_Msg_To_Millis: %s", bSuccess ? "PASS" : "FAIL");
    return bSuccess;
}

BOOL Ros_Testing_Ros_Time_Msg_To_Nanos()
{
    BOOL bSuccess = TRUE;
    INT64 nanos;

    const builtin_interfaces__msg__Time M1 = { 0, 0 };
    nanos = Ros_Time_Msg_To_Nanos(&M1);
    bSuccess &= Ros_Testing_INT64_Equals(0, nanos);

    const builtin_interfaces__msg__Time M2 = { 0, 499999999 };
    nanos = Ros_Time_Msg_To_Nanos(&M2);
    bSuccess &= Ros_Testing_INT64_Equals(499999999, nanos);

    const builtin_interfaces__msg__Time M3 = { 0, 500000000 };
    nanos = Ros_Time_Msg_To_Nanos(&M3);
    bSuccess &= Ros_Testing_INT64_Equals(500000000, nanos);

    const builtin_interfaces__msg__Time M4 = { 1, 864 };
    nanos = Ros_Time_Msg_To_Nanos(&M4);
    bSuccess &= Ros_Testing_INT64_Equals(1000000864LL, nanos);

    const builtin_interfaces__msg__Time M5 = { 999, 1000000 };
    nanos = Ros_Time_Msg_To_Nanos(&M5);
    bSuccess &= Ros_Testing_INT64_Equals(999001000000, nanos);

    const builtin_interfaces__msg__Time M6 = { -1, 499999999 };
    nanos = Ros_Time_Msg_To_Nanos(&M6);
    bSuccess &= Ros_Testing_INT64_Equals(-500000001, nanos);

    const builtin_interfaces__msg__Time M7 = { -1, 0 };
    nanos = Ros_Time_Msg_To_Nanos(&M7);
    bSuccess &= Ros_Testing_INT64_Equals(-1000000000, nanos);

    const builtin_interfaces__msg__Time M8 = { INT_MIN, 864 };
    nanos = Ros_Time_Msg_To_Nanos(&M8);
    bSuccess &= Ros_Testing_INT64_Equals(INT_MIN * 1000LL * 1000LL * 1000LL + 864, nanos);

    const builtin_interfaces__msg__Time M9 = { INT_MIN, 0 };
    nanos = Ros_Time_Msg_To_Nanos(&M9);
    bSuccess &= Ros_Testing_INT64_Equals(INT_MIN * 1000LL * 1000LL * 1000LL, nanos);

    Ros_Debug_BroadcastMsg("Testing Ros_Time_Msg_To_Nanos: %s", bSuccess ? "PASS" : "FAIL");
    return bSuccess;
}

BOOL Ros_Testing_Ros_Millis_To_Time_Msg()
{
    BOOL bSuccess = TRUE;
    INT64 millis;

    builtin_interfaces__msg__Time M1, M2;

    M1.sec = 0;
    M1.nanosec = 0;
    millis = 0;
    Ros_Millis_To_Time_Msg(millis, &M2);
    bSuccess &= builtin_interfaces__msg__Time__are_equal(&M1, &M2);

    M1.sec = 0;
    M1.nanosec = 1000000;
    millis = 1;
    Ros_Millis_To_Time_Msg(millis, &M2);
    bSuccess &= builtin_interfaces__msg__Time__are_equal(&M1, &M2);

    M1.sec = 0;
    M1.nanosec = 999000000;
    millis = 999;
    Ros_Millis_To_Time_Msg(millis, &M2);
    bSuccess &= builtin_interfaces__msg__Time__are_equal(&M1, &M2);

    M1.sec = 10000000;
    M1.nanosec = 98000000;
    millis = 10000000098;
    Ros_Millis_To_Time_Msg(millis, &M2);
    bSuccess &= builtin_interfaces__msg__Time__are_equal(&M1, &M2);

    M1.sec = -1;
    M1.nanosec = 0;
    millis = -1000;
    Ros_Millis_To_Time_Msg(millis, &M2);
    bSuccess &= builtin_interfaces__msg__Time__are_equal(&M1, &M2);

    M1.sec = -1;
    M1.nanosec = 1000000;
    millis = -999;
    Ros_Millis_To_Time_Msg(millis, &M2);
    bSuccess &= builtin_interfaces__msg__Time__are_equal(&M1, &M2);

    M1.sec = -1;
    M1.nanosec = 999000000;
    millis = -1;
    Ros_Millis_To_Time_Msg(millis, &M2);
    bSuccess &= builtin_interfaces__msg__Time__are_equal(&M1, &M2);

    M1.sec = INT_MIN;
    M1.nanosec = 999000000;
    millis = INT_MIN * 1000LL + 999;
    Ros_Millis_To_Time_Msg(millis, &M2);
    bSuccess &= builtin_interfaces__msg__Time__are_equal(&M1, &M2);

    Ros_Debug_BroadcastMsg("Testing Ros_Millis_To_Time_Msg: %s", bSuccess ? "PASS" : "FAIL");
    return bSuccess;
}

BOOL Ros_Testing_Ros_Nanos_To_Time_Msg()
{
    BOOL bSuccess = TRUE;
    INT64 nanos;

    builtin_interfaces__msg__Time M1, M2;

    M1.sec = 0;
    M1.nanosec = 0;
    nanos = 0;
    Ros_Nanos_To_Time_Msg(nanos, &M2);
    bSuccess &= builtin_interfaces__msg__Time__are_equal(&M1, &M2);

    M1.sec = 0;
    M1.nanosec = 99999999LL;
    nanos = 99999999LL;
    Ros_Nanos_To_Time_Msg(nanos, &M2);
    bSuccess &= builtin_interfaces__msg__Time__are_equal(&M1, &M2);

    M1.sec = 8;
    M1.nanosec = 275937194LL;
    nanos = 8275937194LL;
    Ros_Nanos_To_Time_Msg(nanos, &M2);
    bSuccess &= builtin_interfaces__msg__Time__are_equal(&M1, &M2);

    M1.sec = 98375198;
    M1.nanosec = 1;
    nanos = 98375198000000001;
    Ros_Nanos_To_Time_Msg(nanos, &M2);
    bSuccess &= builtin_interfaces__msg__Time__are_equal(&M1, &M2);

    M1.sec = -1;
    M1.nanosec = 999999999;
    nanos = -1;
    Ros_Nanos_To_Time_Msg(nanos, &M2);
    bSuccess &= builtin_interfaces__msg__Time__are_equal(&M1, &M2);

    M1.sec = -1;
    M1.nanosec = 1;
    nanos = -999999999;
    Ros_Nanos_To_Time_Msg(nanos, &M2);
    bSuccess &= builtin_interfaces__msg__Time__are_equal(&M1, &M2);

    M1.sec = INT_MIN;
    M1.nanosec = 0;
    nanos = INT_MIN * 1000LL * 1000LL * 1000LL;
    Ros_Nanos_To_Time_Msg(nanos, &M2);
    bSuccess &= builtin_interfaces__msg__Time__are_equal(&M1, &M2);

    Ros_Debug_BroadcastMsg("Testing Ros_Nanos_To_Time_Msg: %s", bSuccess ? "PASS" : "FAIL");
    return bSuccess;
}

BOOL Ros_Testing_Ros_Nanos_To_Timespec()
{
    BOOL bSuccess = TRUE;
    INT64 nanos;

    struct timespec M1, M2;

    M1.tv_sec = 0;
    M1.tv_nsec = 0;
    nanos = 0;
    Ros_Nanos_To_Timespec(nanos, &M2);
    bSuccess &= Ros_Testing_Timespec_Equals(&M1, &M2);

    M1.tv_sec = 0;
    M1.tv_nsec = 99999999LL;
    nanos = 99999999LL;
    Ros_Nanos_To_Timespec(nanos, &M2);
    bSuccess &= Ros_Testing_Timespec_Equals(&M1, &M2);

    M1.tv_sec = 8;
    M1.tv_nsec = 275937194LL;
    nanos = 8275937194LL;
    Ros_Nanos_To_Timespec(nanos, &M2);
    bSuccess &= Ros_Testing_Timespec_Equals(&M1, &M2);

    M1.tv_sec = 98375198;
    M1.tv_nsec = 1;
    nanos = 98375198000000001;
    Ros_Nanos_To_Timespec(nanos, &M2);
    bSuccess &= Ros_Testing_Timespec_Equals(&M1, &M2);

    Ros_Debug_BroadcastMsg("Testing Ros_Testing_Timespec_Equals: %s", bSuccess ? "PASS" : "FAIL");
    return bSuccess;
}

BOOL Ros_Testing_Mixed_Time_Testing() 
{
    BOOL bSuccess = TRUE;
    INT64 output;
    builtin_interfaces__msg__Time time_msg, output_time_msg;
    builtin_interfaces__msg__Duration duration_msg, output_duration_msg;

    
    INT64 millisecond_inputs[] = { 0, 1, 99, 1000, 99999, 1743788456057, 2147483647999 };
    for (int i = 0; i < sizeof(millisecond_inputs) / sizeof(INT64); i++) 
    {
        Ros_Millis_To_Duration_Msg(millisecond_inputs[i], &duration_msg);
        output = Ros_Duration_Msg_To_Millis(&duration_msg);
        bSuccess &= Ros_Testing_INT64_Equals(millisecond_inputs[i], output);

        Ros_Millis_To_Time_Msg(millisecond_inputs[i], &time_msg);
        output = Ros_Time_Msg_To_Millis(&time_msg);
        bSuccess &= Ros_Testing_INT64_Equals(millisecond_inputs[i], output);
    }

    INT64 nanosecond_inputs[] = { 0, 1, 999999, 1000000, 1743788456057000000, 2147483647999000000 };
    for (int i = 0; i < sizeof(nanosecond_inputs) / sizeof(INT64); i++)
    {
        Ros_Nanos_To_Duration_Msg(nanosecond_inputs[i], &duration_msg);
        output = Ros_Duration_Msg_To_Nanos(&duration_msg);
        bSuccess &= Ros_Testing_INT64_Equals(nanosecond_inputs[i], output);

        Ros_Nanos_To_Time_Msg(nanosecond_inputs[i], &time_msg);
        output = Ros_Time_Msg_To_Nanos(&time_msg);
        bSuccess &= Ros_Testing_INT64_Equals(nanosecond_inputs[i], output);
    }

    const builtin_interfaces__msg__Duration duration_msg_array_nanos[] =
    {
        { INT_MIN, 0 },
        { INT_MIN, 20 },
        { -1, 0 },
        { -1, 1 },
        { -1, 999999999 },
        { 0, 0 },
        { 0, 1 },
        { 0, 999999 },
        { 0, 1000000 },
        { 0 , 999999999},
        { 1 , 0},
        { 1 , 999999999},
        { INT_MAX , 999999999},
    };

    const builtin_interfaces__msg__Time time_msg_array_nanos[] =
    {
        { INT_MIN, 0 },
        { INT_MIN, 20 },
        { -1, 0 },
        { -1, 1 },
        { -1, 999999999 },
        { 0, 0 },
        { 0, 1 },
        { 0, 999999 },
        { 0, 1000000 },
        { 0 , 999999999},
        { 1 , 0},
        { 1 , 999999999},
        { INT_MAX , 999999999},
    };

    for (int i = 0; i < sizeof(duration_msg_array_nanos) / sizeof(builtin_interfaces__msg__Duration); i++)
    {
        output = Ros_Duration_Msg_To_Nanos(&duration_msg_array_nanos[i]);
        Ros_Nanos_To_Duration_Msg(output, &output_duration_msg);
        bSuccess &= builtin_interfaces__msg__Duration__are_equal(&output_duration_msg , &duration_msg_array_nanos[i]);

        output = Ros_Time_Msg_To_Nanos(&time_msg_array_nanos[i]);
        Ros_Nanos_To_Time_Msg(output, &output_time_msg);
        bSuccess &= builtin_interfaces__msg__Time__are_equal(&output_time_msg, &time_msg_array_nanos[i]);
    }

    const builtin_interfaces__msg__Duration duration_msg_array_millis[] =
    {
        { INT_MIN, 0 },
        { INT_MIN, 2000000 },
        { -1, 0 },
        { -1, 1000000 },
        { -1, 999000000 },
        { 0, 0 },
        { 0, 1000000 },
        { 0, 999000000 },
        { 1, 0 },
        { INT_MAX , 999000000},
    };

    const builtin_interfaces__msg__Time time_msg_array_millis[] =
    {
        { INT_MIN, 0 },
        { INT_MIN, 2000000 },
        { -1, 0 },
        { -1, 1000000 },
        { -1, 999000000 },
        { 0, 0 },
        { 0, 1000000 },
        { 0, 999000000 },
        { 1, 0 },
        { INT_MAX , 999000000},
    };

    for (int i = 0; i < sizeof(duration_msg_array_millis) / sizeof(builtin_interfaces__msg__Duration); i++)
    {
        output = Ros_Duration_Msg_To_Millis(&duration_msg_array_millis[i]);
        Ros_Millis_To_Duration_Msg(output, &output_duration_msg);
        bSuccess &= builtin_interfaces__msg__Duration__are_equal(&output_duration_msg , &duration_msg_array_millis[i]);

        output = Ros_Time_Msg_To_Millis(&time_msg_array_millis[i]);
        Ros_Millis_To_Time_Msg(output, &output_time_msg);
        bSuccess &= builtin_interfaces__msg__Time__are_equal(&output_time_msg, &time_msg_array_millis[i]);
    }
    Ros_Debug_BroadcastMsg("Testing mixed time functions: %s", bSuccess ? "PASS" : "FAIL");
    return bSuccess;
}

BOOL Ros_Testing_TimeConversionUtils()
{
    BOOL bSuccess = TRUE;

    bSuccess &= Ros_Testing_Ros_Duration_Msg_To_Millis();
    bSuccess &= Ros_Testing_Ros_Duration_Msg_To_Nanos();
    bSuccess &= Ros_Testing_Ros_Duration_Msg_To_Millis();
    bSuccess &= Ros_Testing_Ros_Nanos_To_Duration_Msg();
    bSuccess &= Ros_Testing_Ros_Time_Msg_To_Millis();
    bSuccess &= Ros_Testing_Ros_Time_Msg_To_Nanos();
    bSuccess &= Ros_Testing_Ros_Millis_To_Time_Msg();
    bSuccess &= Ros_Testing_Ros_Nanos_To_Time_Msg();
    bSuccess &= Ros_Testing_Ros_Nanos_To_Timespec();
    bSuccess &= Ros_Testing_Mixed_Time_Testing();
    return bSuccess;
}

#endif //MOTOROS2_TESTING_ENABLE
