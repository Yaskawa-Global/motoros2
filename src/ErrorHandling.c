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

void Ros_ErrorHandling_Container_rcl_rclc_fail_info_Populate(int errNo,Container_rcl_rclc_fail_info* info) {
    switch (errNo)
    {
    case RCL_RET_OK:
        rosidl_runtime_c__String__assign(&info->message, "RCL/RCLC - Success");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_OK;
        break;
    case RCL_RET_ERROR:
        rosidl_runtime_c__String__assign(&info->message, "RCL/RCLC - Unspecified error");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_ERROR;
        break;
    case RCL_RET_TIMEOUT:
        rosidl_runtime_c__String__assign(&info->message, "RCL/RCLC - Timeout occurred");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_TIMEOUT;
        break;
    case RCL_RET_BAD_ALLOC:
        rosidl_runtime_c__String__assign(&info->message, "RCL/RCLC failed to alloc memory");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_BAD_ALLOC;
        break;
    case RCL_RET_INVALID_ARGUMENT:
        rosidl_runtime_c__String__assign(&info->message, "RCL/RCLC - Invalid argument");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_INVALID_ARGUMENT;
        break;
    case RCL_RET_UNSUPPORTED:
        rosidl_runtime_c__String__assign(&info->message, "RCL/RCLC- unsupported ret. code");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_UNSUPPORTED;
        break;
    case RCL_RET_ALREADY_INIT:
        rosidl_runtime_c__String__assign(&info->message, "rcl_init() already called");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_ALREADY_INIT;
        break;
    case RCL_RET_NOT_INIT:
        rosidl_runtime_c__String__assign(&info->message, "rcl_init() not yet called");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_NOT_INIT;
        break;
    case RCL_RET_MISMATCHED_RMW_ID:
        rosidl_runtime_c__String__assign(&info->message, "RCL/RCLC mismatched rmw ident.");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_MISMATCHED_RMW_ID;
        break;
    case RCL_RET_TOPIC_NAME_INVALID:
        rosidl_runtime_c__String__assign(&info->message, "RCL/RCLC - topic name invalid");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_TOPIC_NAME_INVALID;
        break;
    case RCL_RET_SERVICE_NAME_INVALID:
        rosidl_runtime_c__String__assign(&info->message, "RCL/RCLC - service name invalid");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_SERVICE_NAME_INVALID;
        break;
    case RCL_RET_UNKNOWN_SUBSTITUTION:
        rosidl_runtime_c__String__assign(&info->message, "RCL/RCLC topic name sub unknown");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_UNKNOWN_SUBSTITUTION;
        break;
    case RCL_RET_ALREADY_SHUTDOWN:
        rosidl_runtime_c__String__assign(&info->message, "rclc_shutdown() already called");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_ALREADY_SHUTDOWN;
        break;
    case RCL_RET_NODE_INVALID:
        rosidl_runtime_c__String__assign(&info->message, "Invalid rcl_node_t given");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_NODE_INVALID;
        break;
    case RCL_RET_NODE_INVALID_NAME:
        rosidl_runtime_c__String__assign(&info->message, "RCL/RCLC invalid node name");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_NODE_INVALID_NAME;
        break;
    case RCL_RET_NODE_INVALID_NAMESPACE:
        rosidl_runtime_c__String__assign(&info->message, "RCL/RCLC invalid node namespace");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_NODE_INVALID_NAMESPACE;
        break;
    case RCL_RET_NODE_NAME_NON_EXISTENT:
        rosidl_runtime_c__String__assign(&info->message, "RCL/RCLC cannot find node name");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_NODE_NAME_NON_EXISTENT;
        break;
    case RCL_RET_PUBLISHER_INVALID:
        rosidl_runtime_c__String__assign(&info->message, "Invalid rcl_publisher_t given");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_PUBLISHER_INVALID;
        break;
    case RCL_RET_SUBSCRIPTION_INVALID:
        rosidl_runtime_c__String__assign(&info->message, "Invalid rcl_subscription_t");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_SUBSCRIPTION_INVALID;
        break;
    case RCL_RET_SUBSCRIPTION_TAKE_FAILED:
        rosidl_runtime_c__String__assign(&info->message, "RCL/RCLC Take msg from sub fail");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_SUBSCRIPTION_TAKE_FAILED;
        break;
    case RCL_RET_CLIENT_INVALID:
        rosidl_runtime_c__String__assign(&info->message, "Invalid rcl_client_t given");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_CLIENT_INVALID;
        break;
    case RCL_RET_CLIENT_TAKE_FAILED:
        rosidl_runtime_c__String__assign(&info->message, "RCL/RCLC client take res failed");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_CLIENT_TAKE_FAILED;
        break;
    case RCL_RET_SERVICE_INVALID:
        rosidl_runtime_c__String__assign(&info->message, "RCL/RCLC invalid rcl_service_t");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_SERVICE_INVALID;
        break;
    case RCL_RET_SERVICE_TAKE_FAILED:
        rosidl_runtime_c__String__assign(&info->message, "RCL/RCL get service req. failed");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_SERVICE_TAKE_FAILED;
        break;
    case RCL_RET_TIMER_INVALID:
        rosidl_runtime_c__String__assign(&info->message, "RCL/RCLC - invalid rcl_timer_t ");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_TIMER_INVALID;
        break;
    case RCL_RET_TIMER_CANCELED:
        rosidl_runtime_c__String__assign(&info->message, "RCL/RCLC - given timer canceled");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_TIMER_CANCELED;
        break;
    case RCL_RET_WAIT_SET_INVALID:
        rosidl_runtime_c__String__assign(&info->message, "Invalid rcl_wait_set_t given");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_WAIT_SET_INVALID;
        break;
    case RCL_RET_WAIT_SET_EMPTY:
        rosidl_runtime_c__String__assign(&info->message, "rcl_wait_set_t is empty");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_WAIT_SET_EMPTY;
        break;
    case RCL_RET_WAIT_SET_FULL:
        rosidl_runtime_c__String__assign(&info->message, "rcl_wait_set_t is full");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_WAIT_SET_FULL;
        break;
    case RCL_RET_INVALID_REMAP_RULE:
        rosidl_runtime_c__String__assign(&info->message, "RCL/RCLC not a valid remap rule");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_INVALID_REMAP_RULE;
        break;
    case RCL_RET_WRONG_LEXEME:
        rosidl_runtime_c__String__assign(&info->message, "RCL/RCLC wrong lexeme");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_WRONG_LEXEME;
        break;
    case RCL_RET_INVALID_ROS_ARGS:
        rosidl_runtime_c__String__assign(&info->message, "RCL/RCLC found invalid ros arg");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_INVALID_ROS_ARGS;
        break;
    case RCL_RET_INVALID_PARAM_RULE:
        rosidl_runtime_c__String__assign(&info->message, "RCL/RCLC Arg not valid param");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_INVALID_PARAM_RULE;
        break;
    case RCL_RET_INVALID_LOG_LEVEL_RULE:
        rosidl_runtime_c__String__assign(&info->message, "RCL/RCLC Arg not valid log lvl");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_INVALID_LOG_LEVEL_RULE;
        break;
    case RCL_RET_EVENT_INVALID:
        rosidl_runtime_c__String__assign(&info->message, "RCL/RCLC - Invalid rcl_event_t");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_EVENT_INVALID;
        break;
    case RCL_RET_EVENT_TAKE_FAILED:
        rosidl_runtime_c__String__assign(&info->message, "RCL/RCLC Failed to get event");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_EVENT_TAKE_FAILED;
        break;
    case RCL_RET_LIFECYCLE_STATE_REGISTERED:
        rosidl_runtime_c__String__assign(&info->message, "rcl_lifecycle state registered");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_LIFECYCLE_STATE_REGISTERED;
        break;
    case RCL_RET_LIFECYCLE_STATE_NOT_REGISTERED:
        rosidl_runtime_c__String__assign(&info->message, "rcl_lifecycle state not reg.");
        info->subcode = SUBCODE_RCL_RCLC_FAIL_LIFECYCLE_STATE_NOT_REGISTERED;
        break;
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

void motoRosAssert_RCLAssertOK(rcl_ret_t code)
{
    if (code == RCL_RET_OK)
        return;

    Container_rcl_rclc_fail_info info;

    Ros_ErrorHandling_Container_rcl_rclc_fail_info_Populate(code, &info);

    Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
    Ros_Controller_SetIOState(IO_FEEDBACK_INITIALIZATION_DONE, FALSE);

    mpSetAlarm(ALARM_RCL_RCLC_FAIL, info.message.data, info.subcode);

    FOREVER
    {
        Ros_Debug_BroadcastMsg("motoRos_Rcl_Rclc_AssertOK: %s (subcode: %d)", info.message.data, info.subcode);
        Ros_Sleep(5000);
    }
}

void motoRosAssert_RCLAssertOK_withAlarmAssertionFailSubcode(rcl_ret_t code, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse)
{
    motoRosAssert_RCLAssertOK_withAlarmAssertionFailSubcode_withMsg(code, subCodeIfFalse, APPLICATION_NAME ": Fatal Error");
}

void motoRosAssert_RCLAssertOK_withAlarmAssertionFailSubcode_withMsg(rcl_ret_t code, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse, char* msgFmtIfFalse, ...)
{
    const int MAX_MSG_LEN = 32;
    char msg[MAX_MSG_LEN];
    va_list va;
    Container_rcl_rclc_fail_info rcl_ret_info;

    if (code == RCL_RET_OK)
        return;

    bzero(msg, MAX_MSG_LEN);

    va_start(va, msgFmtIfFalse);
    vsnprintf(msg, MAX_MSG_LEN, msgFmtIfFalse, va);
    va_end(va);

    Ros_ErrorHandling_Container_rcl_rclc_fail_info_Populate(code, &rcl_ret_info);

    Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
    Ros_Controller_SetIOState(IO_FEEDBACK_INITIALIZATION_DONE, FALSE);

    mpSetAlarm(ALARM_RCL_RCLC_FAIL, rcl_ret_info.message.data, rcl_ret_info.subcode);
    mpSetAlarm(ALARM_ASSERTION_FAIL, msg, subCodeIfFalse);


    FOREVER
    {
        Ros_Debug_BroadcastMsg("motoRosAssert_RCLAssertOK_withAlarmAssertionFailSubcode_withMsg: %s (subcode: %d)", msg, subCodeIfFalse);
        Ros_Debug_BroadcastMsg("motoRosAssert_RCLAssertOK_withAlarmAssertionFailSubcode_withMsg: %s (subcode: %d)", rcl_ret_info.message.data, rcl_ret_info.subcode);
        Ros_Sleep(5000);
    }
}
