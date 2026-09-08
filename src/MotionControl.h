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
#define MOTION_START_ERROR_MESSAGE_LENGTH  256
#define MOTION_STOP_TIMEOUT                 20

typedef enum
{
    MOTION_MODE_INACTIVE,
    MOTION_MODE_TRAJECTORY,
    MOTION_MODE_POINTQUEUE
} MOTION_MODE;

// Admission policy governing how a new point is accepted into the point-queue ring.
//   POINT_QUEUE_ADMIT_ONE_DEEP : legacy behavior — admit iff the ring is empty (BUSY otherwise).
//   POINT_QUEUE_ADMIT_FIFO     : streaming behavior — admit iff the ring is not full (QUEUE_FULL otherwise).
typedef enum
{
    POINT_QUEUE_ADMIT_ONE_DEEP,
    POINT_QUEUE_ADMIT_FIFO
} PointQueueAdmitPolicy;

extern Init_Trajectory_Status Ros_MotionControl_InitTrajectory(control_msgs__action__FollowJointTrajectory_SendGoal_Request* pending_ros_goal_request);
extern void Ros_MotionControl_IncMoveLoopStart();
extern void Ros_MotionControl_AddToIncQueueProcess(CtrlGroup* ctrlGroup);
extern UINT16 Ros_MotionControl_ProcessQueuedTrajectoryPoint(motoros2_interfaces__srv__QueueTrajPoint_Request* request);

// Point-queue ring primitives (point-queue mode) are declared in PointQueue.h:
//   Ros_MotionControl_PointQueueCount / Enqueue / Dequeue / Flush / RequestFlush.

// Shared admission+convert path feeding the point-queue ring for all groups.
// Returns a motoros2_interfaces__msg__QueueResultEnum value; sets *out_depth (if non-NULL)
// to the post-call depth of group 0 (all groups move in lockstep by construction).
extern UINT16 Ros_MotionControl_EnqueueTrajectoryPoint(motoros2_interfaces__srv__QueueTrajPoint_Request* request, PointQueueAdmitPolicy policy, UINT16* out_depth);
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

// Returns the sticky point-queue underran flag and clears it to FALSE.
// This is the ONLY site that clears the flag.
extern BOOL Ros_MotionControl_ReadAndClearPointQueueUnderran(void);

extern void Ros_MotionControl_ValidateMotionModeIsOk();

#endif  // MOTOROS2_MOTION_CONTROL_H
