//ServiceResetError.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_SERVICE_RESET_ERROR_H
#define MOTOROS2_SERVICE_RESET_ERROR_H

extern rcl_service_t g_serviceResetError;

typedef struct
{
    motoros2_interfaces__srv__ResetError_Request request;
    motoros2_interfaces__srv__ResetError_Response response;
} ServiceResetError_Messages;
extern ServiceResetError_Messages g_messages_ResetError;

extern void Ros_ServiceResetError_Initialize();
extern void Ros_ServiceResetError_Cleanup();

extern void Ros_ServiceResetError_Trigger(const void* request_msg, void* response_msg);

#endif  // MOTOROS2_SERVICE_RESET_ERROR_H
