//ServiceStartRtMode.h

// SPDX-FileCopyrightText: 2025, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2025, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_SERVICE_START_RT_MODE_H
#define MOTOROS2_SERVICE_START_RT_MODE_H


extern rcl_service_t g_serviceStartRtMode;

typedef struct
{
    motoros2_interfaces__srv__StartRtMode_Request request;
    motoros2_interfaces__srv__StartRtMode_Response response;
} ServiceStartRtMode_Messages;
extern ServiceStartRtMode_Messages g_messages_StartRtMode;

extern void Ros_ServiceStartRtMode_Initialize();
extern void Ros_ServiceStartRtMode_Cleanup();

extern void Ros_ServiceStartRtMode_Trigger(const void* request_msg, void* response_msg);


#endif  // MOTOROS2_SERVICE_START_RT_MODE_H
