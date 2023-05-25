//ServiceQueueTrajPoint.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_SERVICE_QUEUE_TRAJ_POINT_H
#define MOTOROS2_SERVICE_QUEUE_TRAJ_POINT_H

extern rcl_service_t g_serviceQueueTrajPoint;

typedef struct
{
    motoros2_interfaces__srv__QueueTrajPoint_Request* request;
    motoros2_interfaces__srv__QueueTrajPoint_Response* response;
} ServiceQueueTrajPoint_Messages;
extern ServiceQueueTrajPoint_Messages g_messages_QueueTrajPoint;

extern void Ros_ServiceQueueTrajPoint_Initialize();
extern void Ros_ServiceQueueTrajPoint_Cleanup();

extern void Ros_ServiceQueueTrajPoint_Trigger(const void* request_msg, void* response_msg);

#endif  // MOTOROS2_SERVICE_QUEUE_TRAJ_POINT_H
