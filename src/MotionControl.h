//MotionControl.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_MOTION_CONTROL_H
#define MOTOROS2_MOTION_CONTROL_H

#define START_MAX_PULSE_DEVIATION           30

#define MOTION_START_TIMEOUT                5000  // in milliseconds
#define MOTION_START_CHECK_PERIOD           50  // in millisecond
#define MOTION_STOP_TIMEOUT                 20

typedef enum
{
    MOTION_MODE_INACTIVE,
    MOTION_MODE_TRAJECTORY,
    MOTION_MODE_POINTQUEUE
} MOTION_MODE;

extern Init_Trajectory_Status Ros_MotionControl_InitTrajectory(control_msgs__action__FollowJointTrajectory_SendGoal_Request* pending_ros_goal_request);
extern void Ros_MotionControl_IncMoveLoopStart();
extern void Ros_MotionControl_AddToIncQueueProcess(CtrlGroup* ctrlGroup);
extern UINT8 Ros_MotionControl_ProcessQueuedTrajectoryPoint(motoros2_interfaces__srv__QueueTrajPoint_Request* request);
extern BOOL Ros_MotionControl_AddPulseIncPointToQ(CtrlGroup* ctrlGroup, Incremental_data const* dataToEnQ);
extern BOOL Ros_MotionControl_HasDataInQueue();
extern BOOL Ros_MotionControl_HasDataToProcess();
extern BOOL Ros_MotionControl_IsRosControllingMotion();
extern int Ros_MotionControl_GetQueueCnt(int groupNo);
extern BOOL Ros_MotionControl_StopMotion(BOOL bKeepJobRunning);
extern BOOL Ros_MotionControl_ClearQ_All();
extern MotionNotReadyCode Ros_MotionControl_StartMotionMode(MOTION_MODE mode, rosidl_runtime_c__String* responseMessage);
extern void Ros_MotionControl_StopTrajMode();

extern BOOL Ros_MotionControl_IsMotionMode_Trajectory();
extern BOOL Ros_MotionControl_IsMotionMode_PointQueue();
extern BOOL Ros_MotionControl_IsMotionMode_RawStreaming();

extern void Ros_MotionControl_ValidateMotionModeIsOk();

#endif  // MOTOROS2_MOTION_CONTROL_H
