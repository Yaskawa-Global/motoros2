//ServiceResetError.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

#define RESET_ERROR_CHECK_TIMEOUT         5000  // ms
#define RESET_ERROR_CHECK_PERIOD            50  // ms

rcl_service_t g_serviceResetError;

ServiceResetError_Messages g_messages_ResetError;

typedef motoros2_interfaces__srv__ResetError_Response ResetErrorResponse;


void Ros_ServiceResetError_Initialize()
{
    MOTOROS2_MEM_TRACE_START(svc_reset_error_init);

    const rosidl_service_type_support_t* type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(motoros2_interfaces, srv, ResetError);
    
    rcl_ret_t ret = rclc_service_init_default(&g_serviceResetError, &g_microRosNodeInfo.node, type_support, SERVICE_NAME_RESET_ERROR);
    motoRosAssert_withMsg(ret == RCL_RET_OK, SUBCODE_FAIL_INIT_SERVICE_RESET_ERROR, "Failed to init service (%d)", (int)ret);

    rosidl_runtime_c__String__init(&g_messages_ResetError.response.message);

    MOTOROS2_MEM_TRACE_REPORT(svc_reset_error_init);
}

void Ros_ServiceResetError_Cleanup()
{
    rcl_ret_t ret;
    MOTOROS2_MEM_TRACE_START(svc_reset_error_fini);

    Ros_Debug_BroadcastMsg("Cleanup service reset");
    ret = rcl_service_fini(&g_serviceResetError, &g_microRosNodeInfo.node);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up reset service: %d", ret);
    rosidl_runtime_c__String__fini(&g_messages_ResetError.response.message);

    MOTOROS2_MEM_TRACE_REPORT(svc_reset_error_fini);
}

void Ros_ServiceResetError_Trigger(const void* request_msg, void* response_msg)
{
    int ret;
    MP_STD_RSP_DATA rData;

    RCL_UNUSED(request_msg);
    ResetErrorResponse* response = (ResetErrorResponse*)response_msg;

    Ros_Debug_BroadcastMsg("reset: attempting to reset controller");

    // we do not allow resetting the controller unless in remote mode
    if (!Ros_Controller_IsRemote())
    {
        rosidl_runtime_c__String__assign(&response->message, "Pendant is not in REMOTE mode");
        response->result_code.value = MOTION_NOT_READY_NOT_REMOTE;
        goto DONE;
    }

    // not 'real' errors/alarms, but internal error flags
    Ros_Controller_Reset_PflDuringRosMove();
    Ros_Controller_Reset_MpIncMoveError();

    // Check for condition that can be fixed remotely
    if (Ros_Controller_IsError())
    {
        // Cancel error
        bzero(&rData, sizeof(rData));
        ret = mpCancelError(&rData);

        // if mpCancelError failed, we can't recover
        if (ret != 0)
        {
            rosidl_runtime_c__String__assign(&response->message, "Robot has an active ERROR");
            response->result_code.value = MOTION_NOT_READY_ERROR;
            goto DONE;
        }
    }

    // Major alarms cannot be reset remotely
    if (Ros_Controller_IsMajorAlarm())
    {
        rosidl_runtime_c__String__assign(&response->message, "Major alarm active. Cannot be reset. Check teach pendant");
        response->result_code.value = MOTION_NOT_READY_MAJOR_ALARM;
        goto DONE;
    }

    // Other alarm types can be reset remotely
    if (Ros_Controller_IsAlarm())
    {
        // Reset alarm
        bzero(&rData, sizeof(rData));
        ret = mpResetAlarm(&rData);

        // if mpResetAlarm failed, we can't recover
        if (ret != 0)
        {
            rosidl_runtime_c__String__assign(&response->message, "Robot has an active ALARM");
            response->result_code.value = MOTION_NOT_READY_ALARM;
            goto DONE;
        }

        // wait for the Alarm reset confirmation
        int checkCount;
        for (checkCount = 0; checkCount < RESET_ERROR_CHECK_TIMEOUT; checkCount += RESET_ERROR_CHECK_PERIOD)
        {
            // Update status
            Ros_Controller_IoStatusUpdate();

            if (Ros_Controller_IsAlarm() == FALSE)
                break;

            Ros_Sleep(RESET_ERROR_CHECK_PERIOD);
        }
        if (Ros_Controller_IsAlarm())
        {
            rosidl_runtime_c__String__assign(&response->message, "Robot has an active ALARM");
            response->result_code.value = MOTION_NOT_READY_ALARM;
            goto DONE;
        }
    }

    // at this point alarms/errors should be cleared, so report to caller
    rosidl_runtime_c__String__assign(&response->message, "success");
    response->result_code.value = MOTION_READY;

DONE:
    Ros_Debug_BroadcastMsg("reset: %s", response->message.data);
}
