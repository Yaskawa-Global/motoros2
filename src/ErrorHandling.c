// ErrorHandling.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"


//-------------------------------------------------------------------
// Convert error code to string
//-------------------------------------------------------------------
void Ros_ErrorHandling_ErrNo_ToString(int errNo, char* const errMsg, int errMsgSize)
{
    switch (errNo)
    {
    case 0x2010: strncpy(errMsg, "Robot is in operation", errMsgSize); break;
    case 0x2030: strncpy(errMsg, "In HOLD status (PP)", errMsgSize); break;
    case 0x2040: strncpy(errMsg, "In HOLD status (External)", errMsgSize); break;
    case 0x2050: strncpy(errMsg, "In HOLD status (Command)", errMsgSize); break;
    case 0x2060: strncpy(errMsg, "In ERROR/ALARM status", errMsgSize); break;
    case 0x2070: strncpy(errMsg, "In SERVO OFF status", errMsgSize); break;
    case 0x2080: strncpy(errMsg, "Wrong operation mode", errMsgSize); break;
    case 0x3040: strncpy(errMsg, "The home position is not registered", errMsgSize); break;
    case 0x3050: strncpy(errMsg, "Out of range (ABSO data)", errMsgSize); break;
    case 0x3400: strncpy(errMsg, "Cannot operate MASTER JOB", errMsgSize); break;
    case 0x3410: strncpy(errMsg, "The JOB name is already registered in another task", errMsgSize); break;
    case 0x4040: strncpy(errMsg, "Specified JOB not found", errMsgSize); break;
    case 0x5200: strncpy(errMsg, "Over data range", errMsgSize); break;
    default: strncpy(errMsg, "Unspecified reason", errMsgSize); break;
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
    default:
        return motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_UNSPECIFIED_STR;
    }
}

void motoRosAssert(BOOL mustBeTrue, ASSERTION_SUBCODE subCodeIfFalse)
{
    motoRosAssert_withMsg(mustBeTrue, subCodeIfFalse, APPLICATION_NAME ": Fatal Error");
}

void motoRosAssert_withMsg(BOOL mustBeTrue, ASSERTION_SUBCODE subCodeIfFalse, char* msgFmtIfFalse, ...)
{
    const int MAX_MSG_LEN = 32;
    char msg[MAX_MSG_LEN];
    va_list va;

    if (mustBeTrue)
    {
        return;
    }

    bzero(msg, MAX_MSG_LEN);

    va_start(va, msgFmtIfFalse);
    vsnprintf(msg, MAX_MSG_LEN, msgFmtIfFalse, va);
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
