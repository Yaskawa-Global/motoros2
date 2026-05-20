//ServiceStartRtMode.c

// SPDX-FileCopyrightText: 2025, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2025, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

rcl_service_t g_serviceStartRtMode;

ServiceStartRtMode_Messages g_messages_StartRtMode;

// shorten the typename a little, locally
typedef motoros2_interfaces__srv__StartRtMode_Request StartRtMode_Request;
typedef motoros2_interfaces__srv__StartRtMode_Response StartRtMode_Response;

void Ros_ServiceStartRtMode_Initialize()
{
    MOTOROS2_MEM_TRACE_START(svc_start_rt_mode_init);

    rcl_ret_t ret = rclc_service_init_default(&g_serviceStartRtMode, &g_microRosNodeInfo.node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(motoros2_interfaces, srv, StartRtMode),
        SERVICE_NAME_START_RT_MODE);
    motoRos_RCLAssertOK_withMsg(ret, SUBCODE_FAIL_INIT_SERVICE_START_RT_MODE, "Failed to init service (%d)", (int)ret);

    rosidl_runtime_c__String__init(&g_messages_StartRtMode.response.message);

    MOTOROS2_MEM_TRACE_REPORT(svc_start_rt_mode_init);
}

void Ros_ServiceStartRtMode_Cleanup()
{
    MOTOROS2_MEM_TRACE_START(svc_start_rt_mode_fini);

    rcl_ret_t ret;

    Ros_Debug_BroadcastMsg("Cleanup service " SERVICE_NAME_START_RT_MODE);
    ret = rcl_service_fini(&g_serviceStartRtMode, &g_microRosNodeInfo.node);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up " SERVICE_NAME_START_RT_MODE " service: %d", ret);
    rosidl_runtime_c__String__fini(&g_messages_StartRtMode.response.message);

    MOTOROS2_MEM_TRACE_REPORT(svc_start_rt_mode_fini);
}

void Ros_ServiceStartRtMode_Trigger(const void* request_msg, void* response_msg)
{
    StartRtMode_Request* request = (StartRtMode_Request*)request_msg;
    StartRtMode_Response* response = (StartRtMode_Response*)response_msg;

    response->result_code.value = MOTION_READY;
    rosidl_runtime_c__String__assign(&response->message, "");
    response->period = g_Ros_Controller.interpolPeriod;
    response->timeout_for_rt_msg = g_nodeConfigSettings.timeout_for_rt_msg;
    response->max_sequence_diff_for_rt_msg = g_nodeConfigSettings.max_sequence_diff_for_rt_msg;
    
    MOTION_MODE mm = MOTION_MODE_INACTIVE;
    
    if (request->control_mode.value == motoros2_interfaces__msg__ControlModeEnum__CARTESIAN)
        mm = MOTION_MODE_RT_CARTESIAN;
    else if (request->control_mode.value == motoros2_interfaces__msg__ControlModeEnum__JOINT_ANGLES)
        mm = MOTION_MODE_RT_JOINT;

    if (mm != MOTION_MODE_INACTIVE)
        response->result_code.value = Ros_MotionControl_StartMotionMode(mm, &response->message);
    else
        response->result_code.value = MOTION_NOT_READY_INVALID_SELECTION;

    if (response->result_code.value != MOTION_READY)
    {
        // update response

        //If it is a MOTION_NOT_READY_ERROR, then the string was already populated in the Ros_MotionControl_StartMotionMode function
        if (response->result_code.value != MOTION_NOT_READY_ERROR)
        {
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
