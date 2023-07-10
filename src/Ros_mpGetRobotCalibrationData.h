// Ros_mpGetRobotCalibrationData.h

// SPDX-FileCopyrightText: 2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_ROS_MP_GET_ROBOT_CALIBRATION_DATA_H
#define MOTOROS2_ROS_MP_GET_ROBOT_CALIBRATION_DATA_H


#include "motoPlus.h"


#if defined (DX200) || defined (FS100)

// from mpSysCtrl.h

#ifdef MP_RC_XYZ_NUM
#error "DX200/FS100 M+ SDK conflicts with MP_RC_XYZ_NUM definition"
#endif

#ifdef MP_RC_CALIB_P_NUM
#error "DX200/FS100 M+ SDK conflicts with MP_RC_CALIB_P_NUM definition"
#endif

#ifdef MP_RC_RB_P_NUM
#error "DX200/FS100 M+ SDK conflicts with MP_RC_RB_P_NUM definition"
#endif

#define MP_RC_XYZ_NUM     (3)
#define MP_RC_CALIB_P_NUM (5)
#define MP_RC_RB_P_NUM    (2)

typedef struct  {
    CHAR  tool_no;
    UCHAR reserved[3];
    LONG  axis_data[MP_GRP_AXES_NUM];
} MP_RB_CALIB_POS;

typedef struct  {
    ULONG           grp_no;
    MP_RB_CALIB_POS calib_pos[MP_RC_CALIB_P_NUM][MP_RC_RB_P_NUM];
} MP_RB_CALIB_INFO;

typedef struct  {
    MP_RB_CALIB_INFO m_rb;
    MP_RB_CALIB_INFO s_rb;
    LONG             pos_uow[MP_RC_XYZ_NUM];
    LONG             ang_uow[MP_RC_XYZ_NUM];
} MP_RB_CALIB_DATA;

#endif

extern void Ros_mpGetRobotCalibrationData_Initialize();
extern void Ros_mpGetRobotCalibrationData_Cleanup();
extern LONG Ros_mpGetRobotCalibrationData(ULONG file_no, MP_RB_CALIB_DATA *rData);


#endif  // MOTOROS2_ROS_MP_GET_ROBOT_CALIBRATION_DATA_H
