/* ParameterTypes.h - Parameter Extraction type definitions header file */

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: BSD-3-Clause

#ifndef MOTOROS2_CMOS_PARAMETER_TYPES_H
#define MOTOROS2_CMOS_PARAMETER_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	double PtoR[MAX_PULSE_AXES]; //Array to store PULSE TO RADIAN conversion factors for each axes
} PULSE_TO_RAD;

typedef struct
{
    double PtoM[MAX_PULSE_AXES]; //Array to store PULSE TO METER conversion factors for each axes
} PULSE_TO_METER;

typedef enum
{
    AXIS_ROTATION,
    AXIS_LINEAR,
    AXIS_INVALID
} AXIS_TYPE;

typedef struct
{
    AXIS_TYPE	type[MAX_PULSE_AXES];	//Array to store whether axis is rotational or linear
} AXIS_MOTION_TYPE;

typedef struct
{
    BOOL  bValid;			//TRUE if ulSourceAxis != 0
    INT32 ulSourceAxis;
    INT32 ulCorrectionAxis;
    double fCorrectionRatio;
} FB_AXIS_CORRECTION;

typedef struct
{
    FB_AXIS_CORRECTION  correction[MAX_PULSE_AXES];
} FB_PULSE_CORRECTION_DATA;

typedef struct
{
    UINT32	qtyOfOutFiles;
    UINT32	qtyOfHighPriorityTasks;
    UINT32	qtyOfNormalPriorityTasks;
} TASK_QTY_INFO;

typedef struct
{
    UINT16 periodInMilliseconds;
} GP_INTERPOLATION_PERIOD;

typedef struct
{
    UINT32	maxIncrement[MAX_PULSE_AXES];
} MAX_INCREMENT_INFO;

typedef struct
{
    int ctrlGrp;				//Robot control group
    int IpCycleInMilliseconds;	//Interpolation Cycle in milliseconds
    MAX_INCREMENT_INFO info;	//Maximum increment per interpolation cycle
} MAX_INC_PIPC;

typedef struct
{
    INT32 maxLimit[MAX_PULSE_AXES];
    INT32 minLimit[MAX_PULSE_AXES];
} JOINT_PULSE_LIMITS;

typedef struct
{
    INT32 maxLimit[MAX_PULSE_AXES];
} JOINT_ANGULAR_VELOCITY_LIMITS;

typedef struct
{
    BOOL bFeedbackSpeedEnabled;
    ULONG cioAddressForAxis[MAX_PULSE_AXES][2];
} JOINT_FEEDBACK_SPEED_ADDRESSES;

typedef struct
{
    float theta;
    float d;
    float a;
    float alpha;
} DH_LINK;

typedef struct
{
    DH_LINK link[MAX_PULSE_AXES];
} DH_PARAMETERS;

typedef enum
{
    MOTION_TYPE_NOT_USED = -1,
    MOTION_TYPE_X,
    MOTION_TYPE_Y,
    MOTION_TYPE_Z,
    MOTION_TYPE_RX,
    MOTION_TYPE_RY,
    MOTION_TYPE_RZ
} BASE_AXIS_MOTION_TYPE;

typedef struct
{
    BASE_AXIS_MOTION_TYPE motionType[MAX_PULSE_AXES];
    MP_COORD offsetFromBaseToRobotOrigin;
} BASE_AXIS_INFO;

typedef enum
{
    ECO_UNIT_MINUTES,
    ECO_UNIT_SECONDS
} ECO_MODE_UNITS;

typedef struct
{
    BOOL bEnabled;
    ECO_MODE_UNITS timeUnit;
    UINT32 timeout;
} ECO_MODE_INFO;

#ifdef __cplusplus
}
#endif

#endif  // MOTOROS2_CMOS_PARAMETER_TYPES_H
