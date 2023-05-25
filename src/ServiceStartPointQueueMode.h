//ServiceStartPointQueueMode.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_SERVICE_START_POINT_QUEUE_MODE_H
#define MOTOROS2_SERVICE_START_POINT_QUEUE_MODE_H


extern rcl_service_t g_serviceStartPointQueueMode;

typedef struct
{
    motoros2_interfaces__srv__StartPointQueueMode_Request request;
    motoros2_interfaces__srv__StartPointQueueMode_Response response;
} ServicePointQueueMode_Messages;
extern ServicePointQueueMode_Messages g_messages_StartPointQueueMode;

extern void Ros_ServiceStartPointQueueMode_Initialize();
extern void Ros_ServiceStartPointQueueMode_Cleanup();

extern void Ros_ServiceStartPointQueueMode_Trigger(const void* request_msg, void* response_msg);


#endif  // MOTOROS2_SERVICE_START_POINT_QUEUE_MODE_H
