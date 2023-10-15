//ServiceGetActiveAlarmInfo.c

// SPDX-FileCopyrightText: 2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

rcl_service_t g_serviceGetActiveAlarmInfo;

ServiceGetActiveAlarmInfo_Messages g_messages_GetActiveAlarmInfo;

typedef motoros2_interfaces__srv__GetActiveAlarmInfo_Response GetActiveAlarmInfoResponse;


void Ros_ServiceGetActiveAlarmInfo_Initialize()
{
    MOTOROS2_MEM_TRACE_START(svc_get_active_alarm_info_init);

    // TODO: init request and response

    rcl_ret_t ret = rclc_service_init_default(
        &g_serviceGetActiveAlarmInfo, &g_microRosNodeInfo.node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(motoros2_interfaces, srv, GetActiveAlarmInfo),
        SERVICE_NAME_GET_ACTIVE_ALARM_INFO);
    motoRosAssert_withMsg(ret == RCL_RET_OK,
        SUBCODE_FAIL_INIT_SERVICE_GET_ACTIVE_ALARM_INFO, "Failed to init service (%d)",
        (int)ret);

    MOTOROS2_MEM_TRACE_REPORT(svc_get_active_alarm_info_init);
}

void Ros_ServiceGetActiveAlarmInfo_Cleanup()
{
    MOTOROS2_MEM_TRACE_START(svc_get_active_alarm_info_fini);

    Ros_Debug_BroadcastMsg("Cleanup service " SERVICE_NAME_GET_ACTIVE_ALARM_INFO);
    rcl_ret_t ret = rcl_service_fini(
        &g_serviceGetActiveAlarmInfo, &g_microRosNodeInfo.node);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up "
            SERVICE_NAME_GET_ACTIVE_ALARM_INFO " service: %d", ret);

    MOTOROS2_MEM_TRACE_REPORT(svc_get_active_alarm_info_fini);
}

void Ros_ServiceGetActiveAlarmInfo_Trigger(const void* request_msg, void* response_msg)
{
    RCL_UNUSED(request_msg);
    GetActiveAlarmInfoResponse* response = (GetActiveAlarmInfoResponse*)response_msg;

    Ros_Debug_BroadcastMsg("%s: retrieving active alarm & error info", __func__);


    // TODO: implement actual functionality


    // TODO: use proper error codes + strings
    rosidl_runtime_c__String__assign(&response->message, "success");
    response->result_code = 1;

DONE:
    Ros_Debug_BroadcastMsg("%s: exit: '%s' (%lu)", __func__, response->message.data,
        (unsigned long) response->result_code);
}
