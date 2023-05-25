//ServiceStartTrajMode.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_SERVICE_START_TRAJ_MODE_H
#define MOTOROS2_SERVICE_START_TRAJ_MODE_H


extern rcl_service_t g_serviceStartTrajMode;

typedef struct
{
    motoros2_interfaces__srv__StartTrajMode_Request request;
    motoros2_interfaces__srv__StartTrajMode_Response response;
} ServiceStartTrajMode_Messages;
extern ServiceStartTrajMode_Messages g_messages_StartTrajMode;

extern void Ros_ServiceStartTrajMode_Initialize();
extern void Ros_ServiceStartTrajMode_Cleanup();

extern void Ros_ServiceStartTrajMode_Trigger(const void* request_msg, void* response_msg);


#endif  // MOTOROS2_SERVICE_START_TRAJ_MODE_H
