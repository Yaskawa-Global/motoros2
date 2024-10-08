//ServiceStartPointQueueMode.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

rcl_service_t g_serviceStartPointQueueMode;

ServicePointQueueMode_Messages g_messages_StartPointQueueMode;

// shorten the typename a little, locally
typedef motoros2_interfaces__srv__StartPointQueueMode_Response StartPointQueueMode_Response;

void Ros_ServiceStartPointQueueMode_Initialize()
{
    MOTOROS2_MEM_TRACE_START(svc_start_point_queue_mode_init);

    rcl_ret_t ret = rclc_service_init_default(&g_serviceStartPointQueueMode, &g_microRosNodeInfo.node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(motoros2_interfaces, srv, StartPointQueueMode),
        SERVICE_NAME_START_POINT_QUEUE_MODE);
    motoRosAssert_withMsg(ret == RCL_RET_OK, SUBCODE_FAIL_INIT_SERVICE_START_QUEUE_MODE, "Failed to init service (%d)", (int)ret);

    rosidl_runtime_c__String__init(&g_messages_StartPointQueueMode.response.message);

    MOTOROS2_MEM_TRACE_REPORT(svc_start_point_queue_mode_init);
}

void Ros_ServiceStartPointQueueMode_Cleanup()
{
    MOTOROS2_MEM_TRACE_START(svc_start_point_queue_mode_fini);

    rcl_ret_t ret;

    Ros_Debug_BroadcastMsg("Cleanup service " SERVICE_NAME_START_POINT_QUEUE_MODE);
    ret = rcl_service_fini(&g_serviceStartPointQueueMode, &g_microRosNodeInfo.node);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg(
            "Failed cleaning up " SERVICE_NAME_START_POINT_QUEUE_MODE " service: %d", ret);
    rosidl_runtime_c__String__fini(&g_messages_StartPointQueueMode.response.message);

    MOTOROS2_MEM_TRACE_REPORT(svc_start_point_queue_mode_fini);
}

void Ros_ServiceStartPointQueueMode_Trigger(const void* request_msg, void* response_msg)
{
    RCL_UNUSED(request_msg);
    StartPointQueueMode_Response* response = (StartPointQueueMode_Response*) response_msg;

    // trust ..
    response->result_code.value = MOTION_READY;
    rosidl_runtime_c__String__assign(&response->message, "");
    
    MotionNotReadyCode motion_result_code = Ros_MotionControl_StartMotionMode(MOTION_MODE_POINTQUEUE, &response->message);
    if (motion_result_code != MOTION_READY)
    {
        // update response
        response->result_code.value = motion_result_code;

        //If it is a MOTION_NOT_READY_ERROR, then the string was already populated in the Ros_MotionControl_StartMotionMode function
        if (motion_result_code != MOTION_NOT_READY_ERROR) {
            // map to human readable string
            rosidl_runtime_c__String__assign(&response->message,
                Ros_ErrorHandling_MotionNotReadyCode_ToString((MotionNotReadyCode)response->result_code.value));
        }

        Ros_Debug_BroadcastMsg("%s: %s (%d)", __func__,
            response->message.data, response->result_code.value);
    }
    else
    {
        Ros_Debug_BroadcastMsg("%s: activated", __func__);
    }
}
