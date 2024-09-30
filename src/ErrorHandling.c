// ErrorHandling.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"


//-------------------------------------------------------------------
// Convert error code to string
//-------------------------------------------------------------------
const char* const Ros_ErrorHandling_ErrNo_ToString(int errNo)
{
    switch (errNo)
    {
    //Note: returning literals here as 'const char*' is OK, as they are stored
    //in an anonymous array with static storage.
    case 0x2010: return "Robot is in operation";
    case 0x2030: return "In HOLD status (PP)";
    case 0x2040: return "In HOLD status (External)";
    case 0x2050: return "In HOLD status (Command)";
    case 0x2060: return "In ERROR/ALARM status";
    case 0x2070: return "In SERVO OFF status";
    case 0x2080: return "Wrong operation mode";
    case 0x3040: return "The home position is not registered";
    case 0x3050: return "Out of range (ABSO data)";
    case 0x3400: return "Cannot operate MASTER JOB";
    case 0x3410: return "The JOB name is already registered in another task";
    case 0x4040: return "Specified JOB not found";
    case 0x5200: return "Over data range";
    default:     return "Unspecified reason";
    }
}

const char* const Ros_ErrorHandling_MotionNotReadyCode_ToString(MotionNotReadyCode code)
{
    //messages defined in motoros2_interfaces/msg/MotionReadyEnum.msg
    switch (code)
    {
    case MOTION_READY:
        return motoros2_interfaces__msg__MotionReadyEnum__READY_STR;
    case MOTION_NOT_READY_UNSPECIFIED:
        return motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_UNSPECIFIED_STR;
    case MOTION_NOT_READY_ALARM:
        return motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_ALARM_STR;
    case MOTION_NOT_READY_ERROR:
        return motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_ERROR_STR;
    case MOTION_NOT_READY_ESTOP:
        return motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_ESTOP_STR;
    case MOTION_NOT_READY_NOT_PLAY:
        return motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_NOT_PLAY_STR; //REMOTE-TEACH is possible
    case MOTION_NOT_READY_NOT_REMOTE:
        return motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_NOT_REMOTE_STR;
    case MOTION_NOT_READY_SERVO_OFF:
        return motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_SERVO_OFF_STR;
    case MOTION_NOT_READY_HOLD:
        return motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_HOLD_STR;
    case MOTION_NOT_READY_NOT_STARTED:
        return motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_JOB_NOT_STARTED_STR;
    case MOTION_NOT_READY_WAITING_ROS:
        return motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_NOT_ON_WAIT_CMD_STR;
    case MOTION_NOT_READY_PFL_ACTIVE:
        return motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_PFL_ACTIVE_STR;
    case MOTION_NOT_READY_INC_MOVE_ERROR:
        return motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_INC_MOVE_ERROR_STR;
    case MOTION_NOT_READY_OTHER_PROGRAM_RUNNING:
        return motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_OTHER_PROGRAM_RUNNING_STR;
    case MOTION_NOT_READY_OTHER_TRAJ_MODE_ACTIVE:
        return motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_OTHER_TRAJ_MODE_ACTIVE_STR;
    case MOTION_NOT_READY_NOT_CONT_CYCLE_MODE:
        return motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_NOT_CONT_CYCLE_MODE_STR;
    default:
        return motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_UNSPECIFIED_STR;
    }
}

void motoRosAssert(BOOL mustBeTrue, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse)
{
    motoRosAssert_withMsg(mustBeTrue, subCodeIfFalse, APPLICATION_NAME ": Fatal Error");
}

void motoRosAssert_withMsg(BOOL mustBeTrue, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse, char* msgFmtIfFalse, ...)
{
    char msg[ERROR_MSG_MAX_SIZE];
    va_list va;

    if (mustBeTrue)
    {
        return;
    }

    bzero(msg, ERROR_MSG_MAX_SIZE);

    va_start(va, msgFmtIfFalse);
    vsnprintf(msg, ERROR_MSG_MAX_SIZE, msgFmtIfFalse, va);
    va_end(va);

    Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
    Ros_Controller_SetIOState(IO_FEEDBACK_INITIALIZATION_DONE, FALSE);

    mpSetAlarm(ALARM_ASSERTION_FAIL, msg, subCodeIfFalse);

    FOREVER
    {
        Ros_Debug_BroadcastMsg("motoRosAssert: %s (subcode: %d)", msg, subCodeIfFalse);
        Ros_Sleep(5000);
    }
}

void motoRos_RCLAssertOK(int code, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse)
{
    motoRos_RCLAssertOK_withMsg(code, subCodeIfFalse, APPLICATION_NAME ": Fatal Error");
}

void motoRos_RCLAssertOK_withMsg(int code, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse, char* msgFmtIfFalse, ...)
{
    char rcl_api_msg[ERROR_MSG_MAX_SIZE];
    char assert_msg[ERROR_MSG_MAX_SIZE];

    va_list va;
    if (code == RCL_RET_OK)
        return;

    bzero(rcl_api_msg, ERROR_MSG_MAX_SIZE);
    bzero(assert_msg, ERROR_MSG_MAX_SIZE);

    snprintf(rcl_api_msg, ERROR_MSG_MAX_SIZE, "RCL(C) API error: %d", code);

    va_start(va, msgFmtIfFalse);
    vsnprintf(assert_msg, ERROR_MSG_MAX_SIZE, msgFmtIfFalse, va);
    va_end(va);

    Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
    Ros_Controller_SetIOState(IO_FEEDBACK_INITIALIZATION_DONE, FALSE);

    mpSetAlarm(ALARM_RCL_RCLC_FAIL, rcl_api_msg, SUBCODE_RCL_RCLC_API_ERROR);
    mpSetAlarm(ALARM_ASSERTION_FAIL, assert_msg, subCodeIfFalse);

    FOREVER
    {
        Ros_Debug_BroadcastMsg("motoRos_RCLAssertOK: %s (alarm code: %d) (subcode: %d)", rcl_api_msg, ALARM_RCL_RCLC_FAIL, SUBCODE_RCL_RCLC_API_ERROR);
        Ros_Debug_BroadcastMsg("motoRos_RCLAssertOK: %s (alarm code: %d) (subcode: %d)", assert_msg, ALARM_ASSERTION_FAIL, subCodeIfFalse);
        Ros_Sleep(5000);
    }
}
