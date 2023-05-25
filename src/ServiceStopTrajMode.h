//ServiceStopTrajMode.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_SERVICE_STOP_TRAJ_MODE_H
#define MOTOROS2_SERVICE_STOP_TRAJ_MODE_H

extern rcl_service_t g_serviceStopTrajMode;

typedef struct
{
    std_srvs__srv__Trigger_Request request;
    std_srvs__srv__Trigger_Response response;
} ServiceStopTrajMode_Messages;
extern ServiceStopTrajMode_Messages g_messages_StopTrajMode;

extern void Ros_ServiceStopTrajMode_Initialize();
extern void Ros_ServiceStopTrajMode_Cleanup();

extern void Ros_ServiceStopTrajMode_Trigger(const void* request_msg, void* response_msg);

#endif  // MOTOROS2_SERVICE_STOP_TRAJ_MODE_H
