// Ros_mpGetRobotCalibrationData.c

// SPDX-FileCopyrightText: 2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"


LONG Ros_mpGetRobotCalibrationData(ULONG file_no, MP_RB_CALIB_DATA *rData)
{
#if defined (YRC1000) || defined (YRC1000u)
    // on these controllers we can forward to mpGetRobotCalibrationData(..)
    return mpGetRobotCalibrationData(file_no, rData);

#elif defined (DX200) || defined (FS100)
    #error Not implemented (yet)

    return ERROR;

#else
    #error Ros_mpGetRobotCalibrationData: unsupported platform

#endif
}
