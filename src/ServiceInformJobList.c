//ServiceJobList.c

// SPDX-FileCopyrightText: 2024, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2024, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

rcl_service_t g_serviceListInformJobs;

ServiceListInformJobs_Messages g_messages_ListInformJobs;

typedef motoros2_interfaces__srv__ListInformJobs_Response ListInformJobsResponse;


static const size_t RESP_NAMES_LIST_INITIAL_SIZE = 1;


void Ros_ServiceListInformJobs_Initialize()
{
    MOTOROS2_MEM_TRACE_START(svc_list_jobs_init);

    const rosidl_service_type_support_t* type_support =
        ROSIDL_GET_SRV_TYPE_SUPPORT(motoros2_interfaces, srv, ListInformJobs);

    rcl_ret_t ret = rclc_service_init_default(
        &g_serviceListInformJobs, &g_microRosNodeInfo.node, type_support, SERVICE_NAME_LIST_INFORM_JOBS);
    motoRosAssert_withMsg(ret == RCL_RET_OK,
        SUBCODE_FAIL_INIT_SERVICE_LIST_INFORM_JOBS, "Failed to init service (%d)", (int)ret);

    rosidl_runtime_c__String__init(&g_messages_ListInformJobs.response.message);
    //NOTE: if/when needed, this will be resized
    rosidl_runtime_c__String__Sequence__init(
        &g_messages_ListInformJobs.response.names, RESP_NAMES_LIST_INITIAL_SIZE);
    g_messages_ListInformJobs.response.names.size = 0;

    MOTOROS2_MEM_TRACE_REPORT(svc_list_jobs_init);
}

void Ros_ServiceListInformJobs_Cleanup()
{
    rcl_ret_t ret;
    MOTOROS2_MEM_TRACE_START(svc_list_jobs_fini);

    Ros_Debug_BroadcastMsg("Cleanup service " SERVICE_NAME_LIST_INFORM_JOBS);
    ret = rcl_service_fini(&g_serviceListInformJobs, &g_microRosNodeInfo.node);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up " SERVICE_NAME_LIST_INFORM_JOBS " service: %d", ret);
    rosidl_runtime_c__String__fini(&g_messages_ListInformJobs.response.message);
    rosidl_runtime_c__String__Sequence__fini(&g_messages_ListInformJobs.response.names);

    MOTOROS2_MEM_TRACE_REPORT(svc_list_jobs_fini);
}

void Ros_ServiceListInformJobs_Trigger(const void* request_msg, void* response_msg)
{
    RCL_UNUSED(request_msg);
    ListInformJobsResponse* response = (ListInformJobsResponse*)response_msg;

    Ros_Debug_BroadcastMsg("%s: enter", __func__);

DONE:
    Ros_Debug_BroadcastMsg("%s: exit", __func__);
}
