//Ros_mpGetCalendar.h

// SPDX-FileCopyrightText: 2024, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2024, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_ROS_MP_GET_CALENDAR_H
#define MOTOROS2_ROS_MP_GET_CALENDAR_H

#include "motoPlus.h"

#if defined (FS100)

// from YRC1000/inc/mpLegApi00.h
typedef struct
{
    USHORT  usYear;
    USHORT  usMonth;
    USHORT  usDay;
    USHORT  usHour;
    USHORT  usMin;
    USHORT  usSec;
    CHAR    reserved[4];
} MP_CALENDAR_RSP_DATA;

#endif

extern STATUS Ros_mpGetCalendar(MP_CALENDAR_RSP_DATA* rData);

#endif  // MOTOROS2_ROS_MP_GET_CALENDAR_H
