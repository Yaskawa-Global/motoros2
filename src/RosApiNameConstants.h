//RosApiNameConstants.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_ROS_API_NAME_CONSTANTS_H
#define MOTOROS2_ROS_API_NAME_CONSTANTS_H


//============================================
// Topic, service and action server names
//============================================
#define TOPIC_NAME_TF "tf"
#define TOPIC_NAME_ROBOT_STATUS "robot_status"
#define TOPIC_NAME_JOINT_STATES "joint_states"

#define SERVICE_NAME_READ_SINGLE_IO "read_single_io"
#define SERVICE_NAME_READ_GROUP_IO "read_group_io"
#define SERVICE_NAME_WRITE_SINGLE_IO "write_single_io"
#define SERVICE_NAME_WRITE_GROUP_IO "write_group_io"
#define SERVICE_NAME_READ_MREGISTER "read_mregister"
#define SERVICE_NAME_WRITE_MREGISTER "write_mregister"
#define SERVICE_NAME_RESET_ERROR "reset_error"
#define SERVICE_NAME_START_TRAJ_MODE "start_traj_mode"
#define SERVICE_NAME_START_POINT_QUEUE_MODE "start_point_queue_mode"
#define SERVICE_NAME_START_RT_MODE "start_rt_mode"
#define SERVICE_NAME_STOP_TRAJ_MODE "stop_traj_mode"
#define SERVICE_NAME_QUEUE_TRAJ_POINT "queue_traj_point"
#define SERVICE_NAME_SELECT_MOTION_TOOL "select_motion_tool"

#define ACTION_NAME_FOLLOW_JOINT_TRAJECTORY "follow_joint_trajectory"


#endif // MOTOROS2_ROS_API_NAME_CONSTANTS_H
