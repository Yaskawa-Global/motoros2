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
    case MOTION_NOT_READY_MAJOR_ALARM:
        return motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_MAJOR_ALARM_STR;
    case MOTION_NOT_READY_ECO_MODE:
        return motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_ECO_MODE_STR;
    case MOTION_NOT_READY_SERVO_ON_TIMEOUT:
        return motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_SERVO_ON_TIMEOUT_STR;
    default:
        return motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_UNSPECIFIED_STR;
    }
}

const char* const Ros_ErrorHandling_Init_Trajectory_Status_ToString(Init_Trajectory_Status code)
{
    //messages defined in motoros2_interfaces/msg/InitTrajEnum.msg
    switch (code)
    {
    case INIT_TRAJ_OK:
        return motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_OK_STR;
    case INIT_TRAJ_UNSPECIFIED:
        return motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_UNSPECIFIED_STR;
    case INIT_TRAJ_TOO_SMALL:
        return motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_TOO_SMALL_STR;
    case INIT_TRAJ_TOO_BIG:
        return motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_TOO_BIG_STR;
    case INIT_TRAJ_ALREADY_IN_MOTION:
        return motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_ALREADY_IN_MOTION_STR;
    case INIT_TRAJ_INVALID_STARTING_POS:
        return motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_INVALID_STARTING_POS_STR;
    case INIT_TRAJ_INVALID_VELOCITY:
        return motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_INVALID_VELOCITY_STR;
    case INIT_TRAJ_INVALID_JOINTNAME:
        return motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_INVALID_JOINTNAME_STR;
    case INIT_TRAJ_INCOMPLETE_JOINTLIST:
        return motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_INCOMPLETE_JOINTLIST_STR;
    case INIT_TRAJ_INVALID_TIME:
        return motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_INVALID_TIME_STR;
    case INIT_TRAJ_WRONG_MODE:
        return motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_WRONG_MODE_STR;
    case INIT_TRAJ_BACKWARD_TIME:
        return motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_BACKWARD_TIME_STR;
    case INIT_TRAJ_WRONG_NUMBER_OF_POSITIONS:
        return motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_WRONG_NUMBER_OF_POSITIONS_STR;
    case INIT_TRAJ_WRONG_NUMBER_OF_VELOCITIES:
        return motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_WRONG_NUMBER_OF_VELOCITIES_STR;
    case INIT_TRAJ_INVALID_ENDING_VELOCITY:
        return motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_INVALID_ENDING_VELOCITY_STR;
    case INIT_TRAJ_INVALID_ENDING_ACCELERATION:
        return motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_INVALID_ENDING_ACCELERATION_STR;
    case INIT_TRAJ_DUPLICATE_JOINT_NAME:
        return motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_DUPLICATE_JOINT_NAME_STR;
    default:
        return motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_UNSPECIFIED_STR;
    }
}

void motoRos_ASSERT_FAIL(ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure)
{
    motoRos_ASSERT_FAIL_MESSAGE(subCodeOnFailure, APPLICATION_NAME ": Fatal Error");
}
void motoRos_ASSERT_FAIL_MESSAGE(ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure, char* msgFmtOnFailure, ...)
{
    char msg[ERROR_MSG_MAX_SIZE];
    va_list va;

    bzero(msg, ERROR_MSG_MAX_SIZE);

    va_start(va, msgFmtOnFailure);
    vsnprintf(msg, ERROR_MSG_MAX_SIZE, msgFmtOnFailure, va);
    va_end(va);

    Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
    Ros_Controller_SetIOState(IO_FEEDBACK_INITIALIZATION_DONE, FALSE);

    mpSetAlarm(ALARM_ASSERTION_FAIL, msg, subCodeOnFailure);

    FOREVER
    {
        Ros_Debug_BroadcastMsg("motoRos_ASSERT_FAIL: %s (subcode: %d)", msg, subCodeOnFailure);
        Ros_Sleep(5000);
    }
}

void _motoRos_ASSERT_TRUE(BOOL mustBeTrue, ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure, char* actualName)
{
    _motoRos_ASSERT_TRUE_MESSAGE(mustBeTrue, subCodeOnFailure, actualName, APPLICATION_NAME ": Fatal Error");
}

void _motoRos_ASSERT_TRUE_MESSAGE(BOOL mustBeTrue, ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure, char* actualName, char* msgFmtOnFailure, ...)
{
    char msg[ERROR_MSG_MAX_SIZE];
    va_list va;

    if (mustBeTrue)
    {
        return;
    }

    bzero(msg, ERROR_MSG_MAX_SIZE);

    va_start(va, msgFmtOnFailure);
    vsnprintf(msg, ERROR_MSG_MAX_SIZE, msgFmtOnFailure, va);
    va_end(va);

    Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
    Ros_Controller_SetIOState(IO_FEEDBACK_INITIALIZATION_DONE, FALSE);

    mpSetAlarm(ALARM_ASSERTION_FAIL, msg, subCodeOnFailure);

    FOREVER
    {
        Ros_Debug_BroadcastMsg("motoRos_ASSERT_TRUE: %s (subcode: %d)", msg, subCodeOnFailure);
        Ros_Debug_BroadcastMsg("Actual: %s == %d (FALSE)", actualName, mustBeTrue);
        Ros_Debug_BroadcastMsg("Expected: %s == TRUE", actualName);
        Ros_Sleep(5000);
    }
}

void _motoRos_ASSERT_FALSE(BOOL mustBeFalse, ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure, char* actualName)
{
    _motoRos_ASSERT_FALSE_MESSAGE(mustBeFalse, subCodeOnFailure, actualName, APPLICATION_NAME ": Fatal Error");
}

void _motoRos_ASSERT_FALSE_MESSAGE(BOOL mustBeFalse, ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure, char* actualName, char* msgFmtOnFailure, ...)
{
    char msg[ERROR_MSG_MAX_SIZE];
    va_list va;

    if (!mustBeFalse)
    {
        return;
    }

    bzero(msg, ERROR_MSG_MAX_SIZE);

    va_start(va, msgFmtOnFailure);
    vsnprintf(msg, ERROR_MSG_MAX_SIZE, msgFmtOnFailure, va);
    va_end(va);

    Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
    Ros_Controller_SetIOState(IO_FEEDBACK_INITIALIZATION_DONE, FALSE);

    mpSetAlarm(ALARM_ASSERTION_FAIL, msg, subCodeOnFailure);

    FOREVER
    {
        Ros_Debug_BroadcastMsg("motoRos_ASSERT_FALSE: %s (subcode: %d)", msg, subCodeOnFailure);
        Ros_Debug_BroadcastMsg("Actual: %s == %d (TRUE)", actualName, mustBeFalse);
        Ros_Debug_BroadcastMsg("Expected: %s == FALSE", actualName);
        Ros_Sleep(5000);
    }
}

void _motoRos_ASSERT_EQUAL_INT(int actual, int expected, ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure, char* actualName, char* expectedName)
{
    _motoRos_ASSERT_EQUAL_INT_MESSAGE(actual, expected, subCodeOnFailure, actualName, expectedName, APPLICATION_NAME ": Fatal Error");
}

void _motoRos_ASSERT_EQUAL_INT_MESSAGE(int actual, int expected, ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure, char* actualName, char* expectedName, char* msgFmtOnFailure, ...)
{
    char msg[ERROR_MSG_MAX_SIZE];
    va_list va;

    if (actual == expected)
    {
        return;
    }

    bzero(msg, ERROR_MSG_MAX_SIZE);

    va_start(va, msgFmtOnFailure);
    vsnprintf(msg, ERROR_MSG_MAX_SIZE, msgFmtOnFailure, va);
    va_end(va);

    Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
    Ros_Controller_SetIOState(IO_FEEDBACK_INITIALIZATION_DONE, FALSE);

    mpSetAlarm(ALARM_ASSERTION_FAIL, msg, subCodeOnFailure);

    FOREVER
    {
        Ros_Debug_BroadcastMsg("motoRos_ASSERT_EQUAL_INT: %s (subcode: %d)", msg, subCodeOnFailure);
        Ros_Debug_BroadcastMsg("Actual: %s == %d", actualName, actual);
        Ros_Debug_BroadcastMsg("Expected: %s == %d (%s)", actualName, expected, expectedName);
        Ros_Sleep(5000);
    }
}

void _motoRos_ASSERT_NOT_EQUAL_INT(int actual, int invalid, ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure, char* actualName, char* invalidName)
{
    _motoRos_ASSERT_NOT_EQUAL_INT_MESSAGE(actual, invalid, subCodeOnFailure, actualName, invalidName, APPLICATION_NAME ": Fatal Error");
}

void _motoRos_ASSERT_NOT_EQUAL_INT_MESSAGE(int actual, int invalid, ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure, char* actualName, char* invalidName, char* msgFmtOnFailure, ...)
{
    char msg[ERROR_MSG_MAX_SIZE];
    va_list va;

    if (actual != invalid)
    {
        return;
    }

    bzero(msg, ERROR_MSG_MAX_SIZE);

    va_start(va, msgFmtOnFailure);
    vsnprintf(msg, ERROR_MSG_MAX_SIZE, msgFmtOnFailure, va);
    va_end(va);

    Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
    Ros_Controller_SetIOState(IO_FEEDBACK_INITIALIZATION_DONE, FALSE);

    mpSetAlarm(ALARM_ASSERTION_FAIL, msg, subCodeOnFailure);

    FOREVER
    {
        Ros_Debug_BroadcastMsg("motoRos_ASSERT_NOT_EQUAL_INT: %s (subcode: %d)", msg, subCodeOnFailure);
        Ros_Debug_BroadcastMsg("Actual: %s == %d", actualName, actual);
        Ros_Debug_BroadcastMsg("Expected: %s != %d (%s)", actualName, invalid, invalidName);
        Ros_Sleep(5000);
    }
}

void _motoRos_ASSERT_GREATER_THAN_OR_EQUAL_TO_INT(int actual, int threshold, ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure, char* actualName, char* thresholdName)
{
    _motoRos_ASSERT_GREATER_THAN_OR_EQUAL_TO_INT_MESSAGE(actual, threshold, subCodeOnFailure, actualName, thresholdName, APPLICATION_NAME ": Fatal Error");
}

void _motoRos_ASSERT_GREATER_THAN_OR_EQUAL_TO_INT_MESSAGE(int actual, int threshold, ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure, char* actualName, char* thresholdName, char* msgFmtOnFailure, ...)
{
    char msg[ERROR_MSG_MAX_SIZE];
    va_list va;

    if (actual >= threshold)
    {
        return;
    }

    bzero(msg, ERROR_MSG_MAX_SIZE);

    va_start(va, msgFmtOnFailure);
    vsnprintf(msg, ERROR_MSG_MAX_SIZE, msgFmtOnFailure, va);
    va_end(va);

    Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
    Ros_Controller_SetIOState(IO_FEEDBACK_INITIALIZATION_DONE, FALSE);

    mpSetAlarm(ALARM_ASSERTION_FAIL, msg, subCodeOnFailure);

    FOREVER
    {
        Ros_Debug_BroadcastMsg("motoRos_ASSERT_GREATER_THAN_OR_EQUAL_TO_INT: %s (subcode: %d)", msg, subCodeOnFailure);
        Ros_Debug_BroadcastMsg("Actual: %s == %d", actualName, actual);
        Ros_Debug_BroadcastMsg("Expected: %s >= %d (%s)", actualName, threshold, thresholdName);
        Ros_Sleep(5000);
    }
}

void _motoRos_ASSERT_GREATER_THAN_INT(int actual, int threshold, ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure, char* actualName, char* thresholdName)
{
    _motoRos_ASSERT_GREATER_THAN_INT_MESSAGE(actual, threshold, subCodeOnFailure, actualName, thresholdName, APPLICATION_NAME ": Fatal Error");
}

void _motoRos_ASSERT_GREATER_THAN_INT_MESSAGE(int actual, int threshold, ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure, char* actualName, char* thresholdName, char* msgFmtOnFailure, ...)
{
    char msg[ERROR_MSG_MAX_SIZE];
    va_list va;

    if (actual > threshold)
    {
        return;
    }

    bzero(msg, ERROR_MSG_MAX_SIZE);

    va_start(va, msgFmtOnFailure);
    vsnprintf(msg, ERROR_MSG_MAX_SIZE, msgFmtOnFailure, va);
    va_end(va);

    Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
    Ros_Controller_SetIOState(IO_FEEDBACK_INITIALIZATION_DONE, FALSE);

    mpSetAlarm(ALARM_ASSERTION_FAIL, msg, subCodeOnFailure);

    FOREVER
    {
        Ros_Debug_BroadcastMsg("motoRos_ASSERT_GREATER_THAN_INT: %s (subcode: %d)", msg, subCodeOnFailure);
        Ros_Debug_BroadcastMsg("Actual: %s == %d", actualName, actual);
        Ros_Debug_BroadcastMsg("Expected: %s > %d (%s)", actualName, threshold, thresholdName);
        Ros_Sleep(5000);
    }
}

void _motoRos_ASSERT_LESS_THAN_OR_EQUAL_TO_INT(int actual, int threshold, ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure, char* actualName, char* thresholdName)
{
    _motoRos_ASSERT_LESS_THAN_OR_EQUAL_TO_INT_MESSAGE(actual, threshold, subCodeOnFailure, actualName, thresholdName, APPLICATION_NAME ": Fatal Error");
}

void _motoRos_ASSERT_LESS_THAN_OR_EQUAL_TO_INT_MESSAGE(int actual, int threshold, ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure, char* actualName, char* thresholdName, char* msgFmtOnFailure, ...)
{
    char msg[ERROR_MSG_MAX_SIZE];
    va_list va;

    if (actual <= threshold)
    {
        return;
    }

    bzero(msg, ERROR_MSG_MAX_SIZE);

    va_start(va, msgFmtOnFailure);
    vsnprintf(msg, ERROR_MSG_MAX_SIZE, msgFmtOnFailure, va);
    va_end(va);

    Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
    Ros_Controller_SetIOState(IO_FEEDBACK_INITIALIZATION_DONE, FALSE);

    mpSetAlarm(ALARM_ASSERTION_FAIL, msg, subCodeOnFailure);

    FOREVER
    {
        Ros_Debug_BroadcastMsg("motoRos_ASSERT_LESS_THAN_OR_EQUAL_TO_INT: %s (subcode: %d)", msg, subCodeOnFailure);
        Ros_Debug_BroadcastMsg("Actual: %s == %d", actualName, actual);
        Ros_Debug_BroadcastMsg("Expected: %s <= %d (%s)", actualName, threshold, thresholdName);
        Ros_Sleep(5000);
    }
}

void _motoRos_ASSERT_LESS_THAN_INT(int actual, int threshold, ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure, char* actualName, char* thresholdName)
{
    _motoRos_ASSERT_LESS_THAN_INT_MESSAGE(actual, threshold, subCodeOnFailure, actualName, thresholdName, APPLICATION_NAME ": Fatal Error");
}

void _motoRos_ASSERT_LESS_THAN_INT_MESSAGE(int actual, int threshold, ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure, char* actualName, char* thresholdName, char* msgFmtOnFailure, ...)
{
    char msg[ERROR_MSG_MAX_SIZE];
    va_list va;

    if (actual < threshold)
    {
        return;
    }

    bzero(msg, ERROR_MSG_MAX_SIZE);

    va_start(va, msgFmtOnFailure);
    vsnprintf(msg, ERROR_MSG_MAX_SIZE, msgFmtOnFailure, va);
    va_end(va);

    Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
    Ros_Controller_SetIOState(IO_FEEDBACK_INITIALIZATION_DONE, FALSE);

    mpSetAlarm(ALARM_ASSERTION_FAIL, msg, subCodeOnFailure);

    FOREVER
    {
        Ros_Debug_BroadcastMsg("motoRos_ASSERT_LESS_THAN_INT: %s (subcode: %d)", msg, subCodeOnFailure);
        Ros_Debug_BroadcastMsg("Actual: %s == %d", actualName, actual);
        Ros_Debug_BroadcastMsg("Expected: %s < %d (%s)", actualName, threshold, thresholdName);
        Ros_Sleep(5000);
    }
}

void _motoRos_ASSERT_NULL(void *ptr, ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure, char* actualName)
{
    _motoRos_ASSERT_NULL_MESSAGE(ptr, subCodeOnFailure, actualName, APPLICATION_NAME ": Fatal Error");
}

void _motoRos_ASSERT_NULL_MESSAGE(void* ptr, ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure, char* actualName, char* msgFmtOnFailure, ...)
{
    char msg[ERROR_MSG_MAX_SIZE];
    va_list va;

    if (ptr == NULL)
    {
        return;
    }

    bzero(msg, ERROR_MSG_MAX_SIZE);

    va_start(va, msgFmtOnFailure);
    vsnprintf(msg, ERROR_MSG_MAX_SIZE, msgFmtOnFailure, va);
    va_end(va);

    Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
    Ros_Controller_SetIOState(IO_FEEDBACK_INITIALIZATION_DONE, FALSE);

    mpSetAlarm(ALARM_ASSERTION_FAIL, msg, subCodeOnFailure);

    FOREVER
    {
        Ros_Debug_BroadcastMsg("motoRos_ASSERT_NULL: %s (subcode: %d)", msg, subCodeOnFailure);
        Ros_Debug_BroadcastMsg("Actual: %s == %p", actualName, ptr);
        Ros_Debug_BroadcastMsg("Expected: %s == NULL", actualName);
        Ros_Sleep(5000);
    }
}


void _motoRos_ASSERT_NOT_NULL(void* ptr, ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure, char* actualName)
{
    _motoRos_ASSERT_NOT_NULL_MESSAGE(ptr, subCodeOnFailure, actualName, APPLICATION_NAME ": Fatal Error");
}

void _motoRos_ASSERT_NOT_NULL_MESSAGE(void* ptr, ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure, char* actualName, char* msgFmtOnFailure, ...)
{
    char msg[ERROR_MSG_MAX_SIZE];
    va_list va;

    if (ptr != NULL)
    {
        return;
    }

    bzero(msg, ERROR_MSG_MAX_SIZE);

    va_start(va, msgFmtOnFailure);
    vsnprintf(msg, ERROR_MSG_MAX_SIZE, msgFmtOnFailure, va);
    va_end(va);

    Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
    Ros_Controller_SetIOState(IO_FEEDBACK_INITIALIZATION_DONE, FALSE);

    mpSetAlarm(ALARM_ASSERTION_FAIL, msg, subCodeOnFailure);

    FOREVER
    {
        Ros_Debug_BroadcastMsg("motoRos_ASSERT_NOT_NULL: %s (subcode: %d)", msg, subCodeOnFailure);
        Ros_Debug_BroadcastMsg("Actual: %s == NULL", actualName);
        Ros_Debug_BroadcastMsg("Expected: %s != NULL", actualName);
        Ros_Sleep(5000);
    }
}

void motoRos_RCLAssertOK(int rcl_return_code, ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure)
{
    motoRos_RCLAssertOK_withMsg(rcl_return_code, subCodeOnFailure, APPLICATION_NAME ": Fatal Error");
}

void motoRos_RCLAssertOK_withMsg(int rcl_return_code, ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure, char* msgFmtOnFailure, ...)
{
    char rcl_api_msg[ERROR_MSG_MAX_SIZE];
    char assert_msg[ERROR_MSG_MAX_SIZE];

    va_list va;
    if (rcl_return_code == RCL_RET_OK)
        return;

    bzero(rcl_api_msg, ERROR_MSG_MAX_SIZE);
    bzero(assert_msg, ERROR_MSG_MAX_SIZE);

    snprintf(rcl_api_msg, ERROR_MSG_MAX_SIZE, "RCL(C) API error: %d", rcl_return_code);

    va_start(va, msgFmtOnFailure);
    vsnprintf(assert_msg, ERROR_MSG_MAX_SIZE, msgFmtOnFailure, va);
    va_end(va);

    Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
    Ros_Controller_SetIOState(IO_FEEDBACK_INITIALIZATION_DONE, FALSE);

    mpSetAlarm(ALARM_RCL_RCLC_FAIL, rcl_api_msg, SUBCODE_RCL_RCLC_API_ERROR);
    mpSetAlarm(ALARM_ASSERTION_FAIL, assert_msg, subCodeOnFailure);

    FOREVER
    {
        Ros_Debug_BroadcastMsg("motoRos_RCLAssertOK: %s (alarm code: %d) (subcode: %d)", rcl_api_msg, ALARM_RCL_RCLC_FAIL, SUBCODE_RCL_RCLC_API_ERROR);
        Ros_Debug_BroadcastMsg("motoRos_RCLAssertOK: %s (alarm code: %d) (subcode: %d)", assert_msg, ALARM_ASSERTION_FAIL, subCodeOnFailure);
        Ros_Sleep(5000);
    }
}
