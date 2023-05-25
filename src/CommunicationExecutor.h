//Communication.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_COMMUNICATION_EXECUTOR_H
#define MOTOROS2_COMMUNICATION_EXECUTOR_H

#define PERIOD_COMMUNICATION_PING_AGENT_MS					5000

// total number of handles =
//		timers +                                            2
//		action_server +                                     1
//		service reset                                       1
//		service start_traj_mode                             1
//		service start_point_queue_mode                      1
//		service stop_traj_mode                              1
//		service queue_traj_point                            1
//		service select_tool                                 1
#define QUANTITY_OF_HANDLES_FOR_MOTION_EXECUTOR             (9)

// total number of handles =
//		service read & write I/O +                          6
#define QUANTITY_OF_HANDLES_FOR_IO_EXECUTOR                 (6)

typedef struct
{
    rcl_init_options_t initOptions;
    rclc_support_t support;
    rcl_node_t node;
} Communication_NodeInfo;
extern Communication_NodeInfo g_microRosNodeInfo;

extern BOOL g_Ros_Communication_AgentIsConnected;

extern void Ros_Communication_ConnectToAgent();
extern void Ros_Communication_StartExecutors(SEM_ID semCommunicationExecutorStatus);

extern void Ros_Communication_Initialize();
extern void Ros_Communication_Cleanup();

#endif  // MOTOROS2_COMMUNICATION_EXECUTOR_H
