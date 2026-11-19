//ServiceSelectMotionTool.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_SERVICE_SELECT_TOOL_H
#define MOTOROS2_SERVICE_SELECT_TOOL_H

extern rcl_service_t g_serviceSelectMotionTool;

typedef struct
{
    motoros2_interfaces__srv__SelectMotionTool_Request request;
    motoros2_interfaces__srv__SelectMotionTool_Response response;
} ServiceSelectMotionTool_Messages;
extern ServiceSelectMotionTool_Messages g_messages_SelectMotionTool;

extern void Ros_ServiceSelectMotionTool_Initialize();
extern void Ros_ServiceSelectMotionTool_Cleanup();

extern void Ros_ServiceSelectMotionTool_Trigger(const void* request_msg, void* response_msg);

extern void Ros_ServiceSelectMotionTool_InitTcpFrameName(rosidl_runtime_c__String* tcpFrameName, const char* name);
extern void Ros_ServiceSelectMotionTool_UpdateTcpFrameName(rosidl_runtime_c__String* tcpFrameName, int tool);

#endif  // MOTOROS2_SERVICE_SELECT_TOOL_H
