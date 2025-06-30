//ServiceStartRtMode.c

// SPDX-FileCopyrightText: 2025, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2025, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

rcl_service_t g_serviceStartRtMode;

ServiceStartRtMode_Messages g_messages_StartRtMode;

// shorten the typename a little, locally
typedef std_srvs__srv__Trigger_Response StartRtMode_Response;

void Ros_ServiceStartRtMode_Initialize()
{
    MOTOROS2_MEM_TRACE_START(svc_start_rt_mode_init);

    rcl_ret_t ret = rclc_service_init_default(&g_serviceStartRtMode, &g_microRosNodeInfo.node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
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
        Ros_Debug_BroadcastMsg(
            "Failed cleaning up " SERVICE_NAME_START_RT_MODE " service: %d", ret);
    rosidl_runtime_c__String__fini(&g_messages_StartRtMode.response.message);

    MOTOROS2_MEM_TRACE_REPORT(svc_start_rt_mode_fini);
}

void Ros_ServiceStartRtMode_Trigger(const void* request_msg, void* response_msg)
{
    RCL_UNUSED(request_msg);
    StartRtMode_Response* response = (StartRtMode_Response*)response_msg;

    // trust ..
    //response->result_code.value = MOTION_READY;
    response->success = TRUE;
    rosidl_runtime_c__String__assign(&response->message, "");

    Ros_Debug_BroadcastMsg("Creating new task: IncMoveTask");

    g_Ros_Controller.tidIncMoveThread = mpCreateTask(MP_PRI_IP_CLK_TAKE, MP_STACK_SIZE,
        (FUNCPTR)Ros_MotionControl_IncMoveLoopStart,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    MotionNotReadyCode motion_result_code = Ros_MotionControl_StartMotionMode(MOTION_MODE_RT, &response->message);
    if (motion_result_code != MOTION_READY)
    {
        // update response
        //response->result_code.value = motion_result_code;
        response->success = FALSE;

        //If it is a MOTION_NOT_READY_ERROR, then the string was already populated in the Ros_MotionControl_StartMotionMode function
        if (motion_result_code != MOTION_NOT_READY_ERROR) 
        {
            // map to human readable string
            //rosidl_runtime_c__String__assign(&response->message,
            //    Ros_ErrorHandling_MotionNotReadyCode_ToString((MotionNotReadyCode)response->result_code.value));
            rosidl_runtime_c__String__assign(&response->message, "nope!");
        }

        //Ros_Debug_BroadcastMsg("%s: %s (%d)", __func__,
        //    response->message.data, response->result_code.value);
        Ros_Debug_BroadcastMsg("%s: %s (%d)", __func__,
            response->message.data, FALSE);
    }
    else
    {
        Ros_Debug_BroadcastMsg("%s: activated", __func__);
    }
}
