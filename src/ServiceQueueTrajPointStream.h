//ServiceQueueTrajPointStream.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_SERVICE_QUEUE_TRAJ_POINT_STREAM_H
#define MOTOROS2_SERVICE_QUEUE_TRAJ_POINT_STREAM_H

extern rcl_service_t g_serviceQueueTrajPointStream;

typedef struct
{
    motoros2_interfaces__srv__QueueTrajPointStream_Request* request;
    motoros2_interfaces__srv__QueueTrajPointStream_Response* response;
} ServiceQueueTrajPointStream_Messages;
extern ServiceQueueTrajPointStream_Messages g_messages_QueueTrajPointStream;

extern void Ros_ServiceQueueTrajPointStream_Initialize();
extern void Ros_ServiceQueueTrajPointStream_Cleanup();

extern void Ros_ServiceQueueTrajPointStream_Trigger(const void* request_msg, void* response_msg);

#endif  // MOTOROS2_SERVICE_QUEUE_TRAJ_POINT_STREAM_H
