//ServiceGetPositionFK.h

// SPDX-FileCopyrightText: 2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_SERVICE_GET_POSITION_FK_H
#define MOTOROS2_SERVICE_GET_POSITION_FK_H


extern rcl_service_t g_serviceGetPositionFK;

typedef struct
{
    motoros2_interfaces__srv__GetPositionFK_Request request;
    motoros2_interfaces__srv__GetPositionFK_Response response;
} ServiceGetPositionFK_Messages;
extern ServiceGetPositionFK_Messages g_messages_GetPositionFK;

extern void Ros_ServiceGetPositionFK_Initialize();
extern void Ros_ServiceGetPositionFK_Cleanup();

extern void Ros_ServiceGetPositionFK_Trigger(const void* request_msg, void* response_msg);


#endif  // MOTOROS2_SERVICE_GET_POSITION_FK_H
