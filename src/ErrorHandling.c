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
    case 0x3450: return "Servo power cannot be turned on";
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
    case MOTION_NOT_READY_ECO_MODE:
        return motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_ECO_MODE_STR;
    case MOTION_NOT_READY_SERVO_ON_TIMEOUT:
        return motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_SERVO_ON_TIMEOUT_STR;
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
