// ActionServer_FJT.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_ACTION_SERVER_FJT_H
#define MOTOROS2_ACTION_SERVER_FJT_H

#define MAX_NUMBER_OF_POINTS_PER_TRAJECTORY 200
#define MIN_NUMBER_OF_POINTS_PER_TRAJECTORY 2   //current position and destination

#define DEFAULT_FJT_GOAL_POSITION_TOLERANCE  (0.01) //radians per axis or meters per axis
#define DEFAULT_FJT_GOAL_TIME_TOLERANCE      (500000000LL) //nanoseconds (0.5 seconds)

extern rclc_action_server_t g_actionServerFollowJointTrajectory;

extern control_msgs__action__FollowJointTrajectory_SendGoal_Request g_actionServer_FJT_SendGoal_Request;
extern UINT32 g_actionServer_FJT_SendGoal_Request__sizeof;

extern void Ros_ActionServer_FJT_Initialize();
extern void Ros_ActionServer_FJT_Cleanup();

extern rcl_ret_t Ros_ActionServer_FJT_Goal_Received(rclc_action_goal_handle_t* goal_handle, void* context);
extern void Ros_ActionServer_FJT_ProcessFeedback();
extern void Ros_ActionServer_FJT_ProcessResult();
extern bool Ros_ActionServer_FJT_Goal_Cancel(rclc_action_goal_handle_t* goal_handle, void* context);

extern void Ros_ActionServer_FJT_UpdateProgressTracker(MP_EXPOS_DATA* incrementData);


#endif  // MOTOROS2_ACTION_SERVER_FJT_H
