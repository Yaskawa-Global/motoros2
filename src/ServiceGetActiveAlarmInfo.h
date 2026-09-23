//ServiceGetActiveAlarmInfo.h

// SPDX-FileCopyrightText: 2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_SERVICE_GET_ACTIVE_ALARM_INFO_H
#define MOTOROS2_SERVICE_GET_ACTIVE_ALARM_INFO_H

extern rcl_service_t g_serviceGetActiveAlarmInfo;

typedef struct
{
    motoros2_interfaces__srv__GetActiveAlarmInfo_Request request;
    motoros2_interfaces__srv__GetActiveAlarmInfo_Response response;
} ServiceGetActiveAlarmInfo_Messages;
extern ServiceGetActiveAlarmInfo_Messages g_messages_GetActiveAlarmInfo;

extern void Ros_ServiceGetActiveAlarmInfo_Initialize();
extern void Ros_ServiceGetActiveAlarmInfo_Cleanup();

extern void Ros_ServiceGetActiveAlarmInfo_Trigger(const void* request_msg, void* response_msg);

#endif  // MOTOROS2_SERVICE_GET_ACTIVE_ALARM_INFO_H
