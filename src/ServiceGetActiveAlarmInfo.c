//ServiceGetActiveAlarmInfo.c

// SPDX-FileCopyrightText: 2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

rcl_service_t g_serviceGetActiveAlarmInfo;

ServiceGetActiveAlarmInfo_Messages g_messages_GetActiveAlarmInfo;

typedef motoros2_interfaces__srv__GetActiveAlarmInfo_Response GetActiveAlarmInfoResponse;


static micro_ros_utilities_memory_rule_t mem_rules_response_[] =
{
    {"result_message", 50},

    {"alarms", MAX_ALARM_COUNT},
    {"alarms.message", ROS_MAX_ALARM_MSG_LEN},
    {"alarms.contents", 1}, //NOTE: zero-length strings don't appear to work
    {"alarms.description", 1}, //NOTE: zero-length strings don't appear to work
    {"alarms.help", 0},

    {"errors", MAX_ERROR_COUNT},
    {"errors.message", ROS_MAX_ERROR_MSG_LEN},
};
static micro_ros_utilities_memory_conf_t mem_conf_response_ = { 0 };
static const rosidl_message_type_support_t* type_support_response_ = NULL;


void Ros_ServiceGetActiveAlarmInfo_Initialize()
{
    MOTOROS2_MEM_TRACE_START(svc_get_active_alarm_info_init);

    // init request: empty, so nothing more to do after this
    motoRosAssert_withMsg(
        motoros2_interfaces__srv__GetActiveAlarmInfo_Request__init(
            &g_messages_GetActiveAlarmInfo.request),
        SUBCODE_FAIL_INIT_SERVICE_GET_ACTIVE_ALARM_INFO,
        "Failed to init request");

    // init response
    mem_conf_response_.allocator = &g_motoros2_Allocator;
    mem_conf_response_.rules = mem_rules_response_;
    mem_conf_response_.n_rules = sizeof(mem_rules_response_) / sizeof(mem_rules_response_[0]);
    type_support_response_ = ROSIDL_GET_MSG_TYPE_SUPPORT(
        motoros2_interfaces, srv, GetActiveAlarmInfo_Response);

    motoRosAssert_withMsg(
        micro_ros_utilities_create_message_memory(
            type_support_response_, &g_messages_GetActiveAlarmInfo.response, mem_conf_response_),
        SUBCODE_FAIL_INIT_SERVICE_GET_ACTIVE_ALARM_INFO,
        "Failed to init response");

    g_messages_GetActiveAlarmInfo.response.alarms.size = 0;
    g_messages_GetActiveAlarmInfo.response.errors.size = 0;

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

    bool result = micro_ros_utilities_destroy_message_memory(
        type_support_response_, &g_messages_GetActiveAlarmInfo.response, mem_conf_response_);
    Ros_Debug_BroadcastMsg("%s: cleanup response msg memory: %d", __func__, result);

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
