// Ros_mpGetCalendar.c

// SPDX-FileCopyrightText: 2024, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2024, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"


STATUS Ros_mpGetCalendar(MP_CALENDAR_RSP_DATA* rData)
{
#if defined (DX200) || defined (YRC1000) || defined (YRC1000u)
    // on these controllers we can forward to mpGetCalendar(..)
    return mpGetCalendar(rData);

#elif defined (FS100)
    if (NULL == rData)
    {
        //can't be more specific, this is what mpGetCalendar(..) does
        return NG;
    }

    bzero(rData, sizeof(MP_CALENDAR_RSP_DATA));

    struct tm now;
    struct timespec tp;

    //retrieve time since epoch
    if (0 != clock_gettime(CLOCK_REALTIME, &tp))
    {
        //can't be more specific, this is what mpGetCalendar(..) does
        return NG;
    }

    //convert to local time
    //NOTE: this uses the VxWorks < 6.9 version of localtime_r(..)
    if (OK != localtime_r(&tp.tv_sec, &now))
    {
        //can't be more specific, this is what mpGetCalendar(..) does
        return NG;
    }

    //set output fields
    rData->usYear = now.tm_year + 1900;
    rData->usMonth = now.tm_mon + 1;
    rData->usDay = now.tm_mday;
    rData->usHour = now.tm_hour;
    rData->usMin = now.tm_min;
    rData->usSec = now.tm_sec;

    return OK;
#else
#error Ros_mpGetCalendar: unsupported platform
#endif
}
