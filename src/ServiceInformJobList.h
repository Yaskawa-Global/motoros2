//ServiceInformJobList.h

// SPDX-FileCopyrightText: 2024, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2024, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_SERVICE_INFORM_JOB_LIST_H
#define MOTOROS2_SERVICE_INFORM_JOB_LIST_H

extern rcl_service_t g_serviceListInformJobs;

typedef struct
{
    motoros2_interfaces__srv__ListInformJobs_Request request;
    motoros2_interfaces__srv__ListInformJobs_Response response;
} ServiceListInformJobs_Messages;
extern ServiceListInformJobs_Messages g_messages_ListInformJobs;

extern void Ros_ServiceListInformJobs_Initialize();
extern void Ros_ServiceListInformJobs_Cleanup();

extern void Ros_ServiceListInformJobs_Trigger(const void* request_msg, void* response_msg);

#endif  // MOTOROS2_SERVICE_INFORM_JOB_LIST_H
