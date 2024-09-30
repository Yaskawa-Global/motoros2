//ServiceStopTrajMode.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

rcl_service_t g_serviceStopTrajMode;

ServiceStopTrajMode_Messages g_messages_StopTrajMode;

void Ros_ServiceStopTrajMode_Initialize()
{
    MOTOROS2_MEM_TRACE_START(svc_stop_traj_mode_init);

    const rosidl_service_type_support_t* type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger);

    rcl_ret_t ret = rclc_service_init_default(&g_serviceStopTrajMode, &g_microRosNodeInfo.node, type_support, SERVICE_NAME_STOP_TRAJ_MODE);
    motoRos_ASSERT_EQ_INT_MESSAGE(ret, RCL_RET_OK, SUBCODE_FAIL_INIT_SERVICE_STOP_TRAJ_MODE, "Failed to init service (%d)", (int)ret);

    rosidl_runtime_c__String__init(&g_messages_StopTrajMode.response.message);

    MOTOROS2_MEM_TRACE_REPORT(svc_stop_traj_mode_init);
}

void Ros_ServiceStopTrajMode_Cleanup()
{
    MOTOROS2_MEM_TRACE_START(svc_stop_traj_mode);

    rcl_ret_t ret;

    Ros_Debug_BroadcastMsg("Cleanup service stop_traj_mode");
    ret = rcl_service_fini(&g_serviceStopTrajMode, &g_microRosNodeInfo.node);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up stop_traj_mode service: %d", ret);
    rosidl_runtime_c__String__fini(&g_messages_StopTrajMode.response.message);

    MOTOROS2_MEM_TRACE_REPORT(svc_stop_traj_mode);
}

void Ros_ServiceStopTrajMode_Trigger(const void* request_msg, void* response_msg)
{
    Ros_Debug_BroadcastMsg("stop_traj_mode: attempting to stop trajectory mode");

    std_srvs__srv__Trigger_Response* response = (std_srvs__srv__Trigger_Response*)response_msg;

    // Don't change mode if queue is not empty
    if(Ros_MotionControl_HasDataInQueue())
    {
        Ros_Debug_BroadcastMsg("stop_traj_mode: increment queue not empty");
        rosidl_runtime_c__String__assign(&response->message, "Can't disable trajectory mode: increment queue not empty");
        response->success = FALSE;
        return;
    }

    // Stop motion
    if(!Ros_MotionControl_StopMotion(/*bKeepJobRunning = */ FALSE))
    {
        Ros_Debug_BroadcastMsg("stop_traj_mode: StopMotion failed (unknown error)");
        rosidl_runtime_c__String__assign(&response->message, "Couldn't stop trajectory mode: unknown error");
        response->success = FALSE;
        return;
    }

    Ros_MotionControl_StopTrajMode();

    // Set I/O signal
    Ros_Controller_SetIOState(IO_FEEDBACK_MP_INCMOVE_DONE, TRUE);

    Ros_Debug_BroadcastMsg("stop_traj_mode: stopped trajectory mode");
    rosidl_runtime_c__String__assign(&response->message, "");
    response->success = TRUE;
}
