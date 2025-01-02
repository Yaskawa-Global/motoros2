// ErrorHandling.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_ERROR_HANDLING_H
#define MOTOROS2_ERROR_HANDLING_H

//**********************************************************************
//**********************************************************************
// MOTION FAILURE ERROR CODES
//**********************************************************************
//**********************************************************************

typedef enum
{
    MOTION_READY = motoros2_interfaces__msg__MotionReadyEnum__READY,
    MOTION_NOT_READY_UNSPECIFIED = motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_UNSPECIFIED,
    MOTION_NOT_READY_ALARM = motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_ALARM,
    MOTION_NOT_READY_ERROR = motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_ERROR,
    MOTION_NOT_READY_ESTOP = motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_ESTOP,
    MOTION_NOT_READY_NOT_PLAY = motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_NOT_PLAY,
    MOTION_NOT_READY_NOT_REMOTE = motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_NOT_REMOTE,
    MOTION_NOT_READY_SERVO_OFF = motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_SERVO_OFF,
    MOTION_NOT_READY_HOLD = motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_HOLD,
    MOTION_NOT_READY_NOT_STARTED = motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_JOB_NOT_STARTED,
    MOTION_NOT_READY_WAITING_ROS = motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_NOT_ON_WAIT_CMD,
    MOTION_NOT_READY_PFL_ACTIVE = motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_PFL_ACTIVE,
    MOTION_NOT_READY_INC_MOVE_ERROR = motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_INC_MOVE_ERROR,
    MOTION_NOT_READY_OTHER_PROGRAM_RUNNING = motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_OTHER_PROGRAM_RUNNING,
    MOTION_NOT_READY_OTHER_TRAJ_MODE_ACTIVE = motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_OTHER_TRAJ_MODE_ACTIVE,
    MOTION_NOT_READY_NOT_CONT_CYCLE_MODE = motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_NOT_CONT_CYCLE_MODE,
    MOTION_NOT_READY_MAJOR_ALARM = motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_MAJOR_ALARM,
    MOTION_NOT_READY_ECO_MODE = motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_ECO_MODE,
    MOTION_NOT_READY_SERVO_ON_TIMEOUT = motoros2_interfaces__msg__MotionReadyEnum__NOT_READY_SERVO_ON_TIMEOUT,
} MotionNotReadyCode;

typedef enum
{
    INIT_TRAJ_OK = motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_OK,
    INIT_TRAJ_UNSPECIFIED = motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_UNSPECIFIED,
    INIT_TRAJ_TOO_SMALL = motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_TOO_SMALL,
    INIT_TRAJ_TOO_BIG = motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_TOO_BIG,
    INIT_TRAJ_ALREADY_IN_MOTION = motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_ALREADY_IN_MOTION,
    INIT_TRAJ_INVALID_STARTING_POS = motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_INVALID_STARTING_POS,
    INIT_TRAJ_INVALID_VELOCITY = motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_INVALID_VELOCITY,
    INIT_TRAJ_INVALID_JOINTNAME = motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_INVALID_JOINTNAME,
    INIT_TRAJ_INCOMPLETE_JOINTLIST = motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_INCOMPLETE_JOINTLIST,
    INIT_TRAJ_INVALID_TIME = motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_INVALID_TIME,
    INIT_TRAJ_WRONG_MODE = motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_WRONG_MODE,
    INIT_TRAJ_BACKWARD_TIME = motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_BACKWARD_TIME,
    INIT_TRAJ_WRONG_NUMBER_OF_POSITIONS = motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_WRONG_NUMBER_OF_POSITIONS,
    INIT_TRAJ_WRONG_NUMBER_OF_VELOCITIES = motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_WRONG_NUMBER_OF_VELOCITIES,
    INIT_TRAJ_INVALID_ENDING_VELOCITY = motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_INVALID_ENDING_VELOCITY,
    INIT_TRAJ_INVALID_ENDING_ACCELERATION = motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_INVALID_ENDING_ACCELERATION,
    INIT_TRAJ_DUPLICATE_JOINT_NAME = motoros2_interfaces__msg__InitTrajEnum__INIT_TRAJ_DUPLICATE_JOINT_NAME,
} Init_Trajectory_Status;

typedef enum
{
    FAIL_TRAJ_CANCEL = 300,
    FAIL_TRAJ_POSITION,
    FAIL_TRAJ_TIME,
    FAIL_TRAJ_ALARM,
    FAIL_TRAJ_TOLERANCE_PARSE,
} Failed_Trajectory_Status;

//**********************************************************************
//**********************************************************************
// TEACH-PENDANT ALARM CODES
//**********************************************************************
//**********************************************************************

#define ERROR_MSG_MAX_SIZE 32
#define DEBUG_MSG_MAX_SIZE 256
//===================================
//Main Codes
//===================================
typedef enum
{
    ALARM_TASK_CREATE_FAIL = 8010,
    ALARM_ASSERTION_FAIL,
    ALARM_ALLOCATION_FAIL,
    ALARM_CONFIGURATION_FAIL,
    ALARM_INFORM_JOB_FAIL,
    ALARM_DAT_FILE_PARSE_FAIL,
    ALARM_OPERATION_FAIL,
    ALARM_RCL_RCLC_FAIL
} ALARM_MAIN_CODE;


//===================================
//Sub Codes
//===================================
typedef enum
{
    SUBCODE_INITIALIZATION,
    SUBCODE_EXECUTOR,
    SUBCODE_INCREMENTAL_MOTION,
    SUBCODE_ADD_TO_INC_Q,
} ALARM_TASK_CREATE_FAIL_SUBCODE; //8010

typedef enum
{
    SUBCODE_FAIL_ROS_CONTROLLER_INIT,
    SUBCODE_INVALID_AXIS_TYPE,
    SUBCODE_FAIL_OPTIONS_INIT,
    SUBCODE_FAIL_RMW_OPTIONS_INIT,
    SUBCODE_FAIL_SUPPORT_INIT,
    SUBCODE_FAIL_NODE_INIT,
    SUBCODE_FAIL_MEM_ALLOC_CFG,
    SUBCODE_FAIL_CREATE_PUBLISHER_JOINT_STATE,
    SUBCODE_FAIL_CREATE_PUBLISHER_JOINT_STATE_ALL,
    SUBCODE_FAIL_CREATE_PUBLISHER_TRANSFORM,

    SUBCODE_FAIL_CREATE_PUBLISHER_ROBOT_STATUS,
    SUBCODE_FAIL_CREATE_TIMER,
    SUBCODE_FAIL_ALLOCATE_TRANSFORM,
    SUBCODE_FAIL_IO_STATUS_UPDATE,
    SUBCODE_FAIL_OPTIONS_INIT_DOMAIN_ID,
    SUBCODE_CONFIGURATION_INVALID_DOMAIN_ID,
    SUBCODE_CONFIGURATION_MISSING_AGENT_IP,
    SUBCODE_CONFIGURATION_INVALID_AGENT_SUBNET,
    SUBCODE_FAIL_ROS_CONTROLLER_INIT_TOO_MANY_GROUPS,
    SUBCODE_FAIL_MP_NICDATA_INIT0,

    SUBCODE_MULTIPLE_INSTANCES_DETECTED,
    SUBCODE_CONFIGURATION_INVALID_NODE_NAME,
    SUBCODE_CONFIGURATION_INVALID_JOB_NAME,
    SUBCODE_FAIL_CREATE_SERVICE_QUEUE_POINT,
    SUBCODE_FAIL_TIMER_INIT_PING,
    SUBCODE_FAIL_TIMER_INIT_ROBOT_FB,
    SUBCODE_FAIL_TIMER_INIT_ACTION_FB,
    SUBCODE_FAIL_CREATE_MOTION_EXECUTOR,
    SUBCODE_FAIL_TIMER_ADD_PING,
    SUBCODE_FAIL_TIMER_ADD_ROBOT_FB,

    SUBCODE_FAIL_TIMER_ADD_ACTION_FB,
    SUBCODE_FAIL_ADD_FJT_SERVER,
    SUBCODE_FAIL_ADD_SERVICE_STOP_TRAJ,
    SUBCODE_FAIL_ADD_SERVICE_READ_SINGLE_IO,
    SUBCODE_FAIL_ADD_SERVICE_READ_GROUP_IO,
    SUBCODE_FAIL_ADD_SERVICE_WRITE_SINGLE_IO,
    SUBCODE_FAIL_ADD_SERVICE_WRITE_GROUP_IO,
    SUBCODE_FAIL_ADD_SERVICE_READ_M_REG,
    SUBCODE_FAIL_ADD_SERVICE_WRITE_M_REG,
    SUBCODE_FAIL_ADD_SERVICE_RESET_ERROR,

    SUBCODE_FAIL_ADD_SERVICE_START_TRAJ_MODE,
    SUBCODE_FAIL_ADD_SERVICE_START_QUEUE_MODE,
    SUBCODE_FAIL_ADD_SERVICE_QUEUE_POINT,
    SUBCODE_FAIL_INIT_SERVICE_READ_SINGLE_IO,
    SUBCODE_FAIL_INIT_SERVICE_READ_GROUP_IO,
    SUBCODE_FAIL_INIT_SERVICE_WRITE_SINGLE_IO,
    SUBCODE_FAIL_INIT_SERVICE_WRITE_GROUP_IO,
    SUBCODE_FAIL_INIT_SERVICE_READ_M_REG,
    SUBCODE_FAIL_INIT_SERVICE_WRITE_M_REG,
    SUBCODE_FAIL_INIT_SERVICE_RESET_ERROR,

    SUBCODE_FAIL_INIT_SERVICE_START_TRAJ_MODE,
    SUBCODE_FAIL_INIT_SERVICE_START_QUEUE_MODE,
    SUBCODE_FAIL_INIT_SERVICE_STOP_TRAJ_MODE,
    SUBCODE_FAIL_ADD_SERVICE_SELECT_MOTION_TOOL,
    SUBCODE_FAIL_INIT_SERVICE_SELECT_MOTION_TOOL,
    SUBCODE_CONFIGURATION_EMPTY_JOINT_NAME,
    SUBCODE_FAIL_CREATE_IO_EXECUTOR,
    SUBCODE_FAIL_TIMER_INIT_USERLAN_MONITOR,
    SUBCODE_FAIL_TIMER_ADD_USERLAN_MONITOR,
    SUBCODE_CONFIGURATION_AGENT_ON_NET_CHECK,

    SUBCODE_CONFIGURATION_FAIL_MP_NICDATA0,
    SUBCODE_CONFIGURATION_FAIL_MP_NICDATA1,
    SUBCODE_FAIL_MP_NICDATA_INIT1,
    SUBCODE_FAIL_INVALID_BASE_TRACK_MOTION_TYPE,
    SUBCODE_DEBUG_INIT_FAIL_MP_NICDATA,

    SUBCODE_CONFIGURATION_MISSING_AGENT_PORT,
    SUBCODE_FAIL_OPTIONS_INIT_DOMAIN_ID_TOO_SMALL,
    SUBCODE_FAIL_OPTIONS_INIT_DOMAIN_ID_TOO_LARGE
} ALARM_ASSERTION_FAIL_SUBCODE; //8011

typedef enum
{
    SUBCODE_ALLOCATION_MALLOC,
    SUBCODE_ALLOCATION_CALLOC,
    SUBCODE_ALLOCATION_REALLOC,
} ALARM_ALLOCATION_FAIL_SUBCODE; //8012

typedef enum
{
    SUBCODE_CONFIGURATION_MISSINGFILE,
    SUBCODE_CONFIGURATION_SRAM_ACCESS_FAILURE,
    SUBCODE_CONFIGURATION_SRAM_WRITE_FAILURE,
    SUBCODE_CONFIGURATION_INVALID_BOOLEAN_VALUE,
    SUBCODE_CONFIGURATION_INVALID_QOS_VALUE,
    SUBCODE_CONFIGURATION_INVALID_EXECUTOR_PERIOD,
    SUBCODE_CONFIGURATION_INVALID_FEEDBACK_PERIOD,
    SUBCODE_CONFIGURATION_INVALID_IO_PERIOD,
    SUBCODE_CONFIGURATION_TOO_MANY_REMAP_RULES1,
    SUBCODE_CONFIGURATION_TOO_MANY_REMAP_RULES2,

    SUBCODE_CONFIGURATION_INVALID_REMAP_RULE_FORMAT,
    SUBCODE_CONFIGURATION_FAIL_NODE_INIT_ARG_PARSE,
    SUBCODE_CONFIGURATION_INVALID_CUSTOM_JOINT_NAME,
    SUBCODE_CONFIGURATION_INVALID_USERLAN_MONITOR_PORT,
    SUBCODE_CONFIGURATION_USERLAN_MONITOR_AUTO_DETECT_FAILED,
    SUBCODE_CONFIGURATION_RUNTIME_USERLAN_LINKUP_ERR,
    SUBCODE_CONFIGURATION_NO_CALIB_FILES_LOADED,
    SUBCODE_CONFIGURATION_INVALID_DEBUG_BROADCAST_PORT,
} ALARM_CONFIGURATION_FAIL_SUBCODE; //8013

typedef enum
{
    SUBCODE_INFORM_FAIL_TO_OPEN_DRAM,
    SUBCODE_INFORM_INVALID_JOB,
    SUBCODE_INFORM_FAIL_TO_CREATE_JOB,
    SUBCODE_INFORM_FAIL_TO_LOAD_JOB,
} ALARM_INFORM_JOB_FAIL_SUBCODE; //8014

typedef enum
{
    SUBCODE_DAT_FAIL_OPEN,
    SUBCODE_DAT_FAIL_PARSE_RBCALIB,
    SUBCODE_DAT_FAIL_PARSE_MGROUP,
    SUBCODE_DAT_FAIL_PARSE_SGROUP,
    SUBCODE_DAT_FAIL_PARSE_SRANG
} ALARM_DAT_FILE_PARSE_FAIL_SUBCODE; //8015

typedef enum
{
    SUBCODE_OPERATION_SET_CYCLE,
} ALARM_OPERATION_FAIL_SUBCODE; //8016

typedef enum
{
    SUBCODE_RCL_RCLC_API_ERROR,
} ALARM_RCL_RCLC_FAIL_SUBCODE; //8017


#define motoRos_ASSERT_GENERIC_COMPARISON_INT_MESSAGE(actual, benchmark, subCodeOnFailure, actualName, benchmarkName, op, msgFmtOnFailure, ...) \
    do { \
        if (!((actual) op (benchmark))) \
        { \
            char motoRos_error_handling_assert_msg[ERROR_MSG_MAX_SIZE] = {0}; \
            snprintf(motoRos_error_handling_assert_msg, ERROR_MSG_MAX_SIZE, msgFmtOnFailure, ##__VA_ARGS__); \
            Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE); \
            Ros_Controller_SetIOState(IO_FEEDBACK_INITIALIZATION_DONE, FALSE); \
            mpSetAlarm(ALARM_ASSERTION_FAIL, motoRos_error_handling_assert_msg, subCodeOnFailure); \
            FOREVER \
            { \
                Ros_Debug_BroadcastMsg("MotoROS2 Assertion Failure: %s (subcode: %d)", motoRos_error_handling_assert_msg, subCodeOnFailure); \
                Ros_Debug_BroadcastMsg("Actual: %s == %d", actualName, (actual)); \
                Ros_Debug_BroadcastMsg("Expected: %s %s %d (%s)", actualName, #op, (benchmark), benchmarkName); \
                Ros_Sleep(5000); \
            } \
        } \
    } while (0)

#define motoRos_ASSERT_GENERIC_COMPARISON_POINTER_MESSAGE(actual, benchmark, subCodeOnFailure, actualName, benchmarkName, op, msgFmtOnFailure, ...) \
    do { \
        if (!((actual) op (benchmark))) \
        { \
            char motoRos_error_handling_assert_msg[ERROR_MSG_MAX_SIZE] = {0}; \
            snprintf(motoRos_error_handling_assert_msg, ERROR_MSG_MAX_SIZE, msgFmtOnFailure, ##__VA_ARGS__); \
            Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE); \
            Ros_Controller_SetIOState(IO_FEEDBACK_INITIALIZATION_DONE, FALSE); \
            mpSetAlarm(ALARM_ASSERTION_FAIL, motoRos_error_handling_assert_msg, subCodeOnFailure); \
            FOREVER \
            { \
                Ros_Debug_BroadcastMsg("MotoROS2 Assertion Failure: %s (subcode: %d)", motoRos_error_handling_assert_msg, subCodeOnFailure); \
                Ros_Debug_BroadcastMsg("Actual: %s == %p", actualName, (actual)); \
                Ros_Debug_BroadcastMsg("Expected: %s %s %p (%s)", actualName, #op, (benchmark), benchmarkName); \
                Ros_Sleep(5000); \
            } \
        } \
    } while (0)

/// <summary>
/// Validate that a value is truthy. If the value is falsy, then raise a fatal alarm on the pendant 
/// and block further execution. The alarm will indicate a context-specific MotoROS2 code.
/// </summary>
/// <param name="mustBeTrue">Value to verify is truthy. Alarm will occur if not truthy.</param>
/// <param name="subCodeOnFailure">Context-specific alarm [subcode] to display if the assertion fails.</param>
#define motoRos_ASSERT_TRUE(mustBeTrue,subCodeOnFailure) \
    motoRos_ASSERT_GENERIC_COMPARISON_INT_MESSAGE(mustBeTrue, TRUE, subCodeOnFailure, #mustBeTrue, "TRUE", ==, APPLICATION_NAME ": Fatal Error")

/// <summary>
/// Validate that a value is truthy. If the value is falsy, then raise a fatal alarm on the pendant 
/// and block further execution. The alarm will indicate a context-specific MotoROS2 code.
/// An additional message will be displayed to help the user understand the error.
/// </summary>
/// <param name="mustBeTrue">Value to verify is truthy. Alarm will occur if not truthy.</param>
/// <param name="subCodeOnFailure">Context-specific alarm [subcode] to display if the assertion fails.</param>
/// <param name="msgFmtOnFailure">Format-string msg to display to the user if the assertion fails.</param>
#define motoRos_ASSERT_TRUE_MESSAGE(mustBeTrue, subCodeOnFailure, msgFmtOnFailure, ...) \
    motoRos_ASSERT_GENERIC_COMPARISON_INT_MESSAGE(mustBeTrue, TRUE, subCodeOnFailure, #mustBeTrue, "TRUE", ==, msgFmtOnFailure, ##__VA_ARGS__)

/// <summary>
/// Validate that a value is falsy. If the value is truthy, then raise a fatal alarm on the pendant 
/// and block further execution. The alarm will indicate a context-specific MotoROS2 code.
/// </summary>
/// <param name="mustBeFalse">Value to verify is falsy. Alarm will occur if not falsy.</param>
/// <param name="subCodeOnFailure">Context-specific alarm [subcode] to display if the assertion fails.</param>
#define motoRos_ASSERT_FALSE(mustBeFalse, subCodeOnFailure) \
    motoRos_ASSERT_GENERIC_COMPARISON_INT_MESSAGE(mustBeFalse, FALSE, subCodeOnFailure, #mustBeFalse, "FALSE", ==, APPLICATION_NAME ": Fatal Error")

/// <summary>
/// Validate that a value is falsy. If the value is truthy, then raise a fatal alarm on the pendant 
/// and block further execution. The alarm will indicate a context-specific MotoROS2 code.
/// An additional message will be displayed to help the user understand the error.
/// </summary>
/// <param name="mustBeFalse">Value to verify is falsy. Alarm will occur if not falsy.</param>
/// <param name="subCodeOnFailure">Context-specific alarm [subcode] to display if the assertion fails.</param>
/// <param name="msgFmtOnFailure">Format-string msg to display to the user if the assertion fails.</param>
#define motoRos_ASSERT_FALSE_MESSAGE(mustBeFalse, subCodeOnFailure, msgFmtOnFailure, ...) \
    motoRos_ASSERT_GENERIC_COMPARISON_INT_MESSAGE(mustBeFalse, FALSE, subCodeOnFailure, #mustBeFalse, "FALSE", ==, msgFmtOnFailure, ##__VA_ARGS__)

/// <summary>
/// Validate that a value is equal to the expected value. If the value is not equal to the expected value,
/// then raise a fatal alarm on the pendant and block further execution. The alarm will indicate a 
/// context-specific MotoROS2 code.
/// </summary>
/// <param name="actual">Value to check for equality with "expected".</param>
/// <param name="expected">Expected value that is being checked for equality with "actual" value.</param>
/// <param name="subCodeOnFailure">Context-specific alarm [subcode] to display if the assertion fails.</param>
#define motoRos_ASSERT_EQUAL_INT(actual, expected, subCodeOnFailure) \
    motoRos_ASSERT_GENERIC_COMPARISON_INT_MESSAGE(actual, expected, subCodeOnFailure, #actual, #expected, ==, APPLICATION_NAME ": Fatal Error")

/// <summary>
/// Validate that a value is equal to the expected value. If the value is not equal to the expected value,
/// then raise a fatal alarm on the pendant and block further execution. The alarm will indicate a 
/// context-specific MotoROS2 code.
/// An additional message will be displayed to help the user understand the error.
/// </summary>
/// <param name="actual">Value to check for equality with "expected".</param>
/// <param name="expected">Expected value that is being checked for equality with "actual" value.</param>
/// <param name="subCodeOnFailure">Context-specific alarm [subcode] to display if the assertion fails.</param>
/// <param name="msgFmtOnFailure">Format-string msg to display to the user if the assertion fails.</param>
#define motoRos_ASSERT_EQUAL_INT_MESSAGE(actual, expected, subCodeOnFailure, msgFmtOnFailure, ...) \
    motoRos_ASSERT_GENERIC_COMPARISON_INT_MESSAGE(actual, expected, subCodeOnFailure, #actual, #expected, ==, msgFmtOnFailure, ##__VA_ARGS__)

/// <summary>
/// Validate that a value is NOT equal to a given invalid value. If the value is equal to the invalid value,
/// then raise a fatal alarm on the pendant and block further execution. The alarm will indicate a 
/// context-specific MotoROS2 code.
/// </summary>
/// <param name="actual">Value to validate is not equal to "invalid" value.</param>
/// <param name="invalid">Invalid value that is being checked against "actual" value.</param>
/// <param name="subCodeOnFailure">Context-specific alarm [subcode] to display if the assertion fails.</param>
#define motoRos_ASSERT_NOT_EQUAL_INT(actual, invalid, subCodeOnFailure) \
    motoRos_ASSERT_GENERIC_COMPARISON_INT_MESSAGE(actual, invalid, subCodeOnFailure, #actual, #invalid, !=, APPLICATION_NAME ": Fatal Error")

/// <summary>
/// Validate that a value is NOT equal to a given invalid value. If the value is equal to the invalid value,
/// then raise a fatal alarm on the pendant and block further execution. The alarm will indicate a 
/// context-specific MotoROS2 code.
/// An additional message will be displayed to help the user understand the error.
/// </summary>
/// <param name="actual">Value to validate is not equal to "invalid" value.</param>
/// <param name="invalid">Invalid value that is being checked against "actual" value.</param>
/// <param name="subCodeOnFailure">Context-specific alarm [subcode] to display if the assertion fails.</param>
/// <param name="msgFmtOnFailure">Format-string msg to display to display if the assertion fails".</param>
#define motoRos_ASSERT_NOT_EQUAL_INT_MESSAGE(actual, invalid, subCodeOnFailure, msgFmtOnFailure, ...) \
    motoRos_ASSERT_GENERIC_COMPARISON_INT_MESSAGE(actual, invalid, subCodeOnFailure, #actual, #invalid, !=, msgFmtOnFailure, ##__VA_ARGS__)


/// <summary>
/// Validate that an integer value is greater than or equal to a given threshold value. If the value is not 
/// greater than or equal to the threshold value, then raise a fatal alarm on the pendant and 
/// block further execution. The alarm will indicate a context-specific MotoROS2 code.
/// </summary>
/// <param name="actual">Value being evaluated to check if it is greater than or equal to "threshold".</param>
/// <param name="threshold">Minimum value that "actual" must be equal to or greater than.</param>
/// <param name="subCodeOnFailure">Context-specific alarm [subcode] to display if the assertion fails.</param>
#define motoRos_ASSERT_GREATER_THAN_OR_EQUAL_TO_INT(actual, threshold, subCodeOnFailure) \
    motoRos_ASSERT_GENERIC_COMPARISON_INT_MESSAGE(actual, threshold, subCodeOnFailure, #actual, #threshold, >=, APPLICATION_NAME ": Fatal Error")

/// <summary>
/// Validate that an integer value is greater than or equal to a given threshold value. If the value is not 
/// greater than or equal to the threshold value, then raise a fatal alarm on the pendant and 
/// block further execution. The alarm will indicate a context-specific MotoROS2 code.
/// An additional message will be displayed to help the user understand the error.
/// </summary>
/// <param name="actual">Value being evaluated to check if it is greater than or equal to "threshold".</param>
/// <param name="threshold">Minimum value that "actual" must be equal to or greater than.</param>
/// <param name="subCodeOnFailure">Context-specific alarm [subcode] to display if the assertion fails.</param>
/// <param name="msgFmtOnFailure">Format-string msg to display if the assertion fails.</param>
#define motoRos_ASSERT_GREATER_THAN_OR_EQUAL_TO_INT_MESSAGE(actual, threshold, subCodeOnFailure, msgFmtOnFailure, ...) \
    motoRos_ASSERT_GENERIC_COMPARISON_INT_MESSAGE(actual, threshold, subCodeOnFailure, #actual, #threshold, >=, msgFmtOnFailure, ##__VA_ARGS__)

/// <summary>
/// Validate that an integer value is greater than a given threshold value. If the value is not 
/// greater than the threshold value, then raise a fatal alarm on the pendant and 
/// block further execution. The alarm will indicate a context-specific MotoROS2 code.
/// </summary>
/// <param name="actual">Value being evaluated to check if it is greater than "threshold".</param>
/// <param name="threshold">Minimum value that "actual" must be greater than.</param>
/// <param name="subCodeOnFailure">Context-specific alarm [subcode] to display if the assertion fails.</param>
#define motoRos_ASSERT_GREATER_THAN_INT(actual, threshold, subCodeOnFailure) \
    motoRos_ASSERT_GENERIC_COMPARISON_INT_MESSAGE(actual, threshold, subCodeOnFailure, #actual, #threshold, >, APPLICATION_NAME ": Fatal Error")

/// <summary>
/// Validate that an integer value is greater than a given threshold value. If the value is not 
/// greater than the threshold value, then raise a fatal alarm on the pendant and 
/// block further execution. The alarm will indicate a context-specific MotoROS2 code.
/// An additional message will be displayed to help the user understand the error.
/// </summary>
/// <param name="actual">Value being evaluated to check if it is greater than "threshold".</param>
/// <param name="threshold">Minimum value that "actual" must be greater than.</param>
/// <param name="subCodeOnFailure">Context-specific alarm [subcode] to display if the assertion fails.</param>
/// <param name="msgFmtOnFailure">Format-string msg to display if the assertion fails.</param>
#define motoRos_ASSERT_GREATER_THAN_INT_MESSAGE(actual, threshold, subCodeOnFailure, msgFmtOnFailure, ...) \
    motoRos_ASSERT_GENERIC_COMPARISON_INT_MESSAGE(actual, threshold, subCodeOnFailure, #actual, #threshold, >, msgFmtOnFailure, ##__VA_ARGS__)

/// <summary>
/// Validate that an integer value is less than or equal to a given threshold value. If the value is not 
/// less than or equal to the threshold value, then raise a fatal alarm on the pendant and 
/// block further execution. The alarm will indicate a context-specific MotoROS2 code.
/// </summary>
/// <param name="actual">Value being evaluated to check if it is less than or equal to "threshold".</param>
/// <param name="threshold">Maximum value that "actual" must be equal to or less than.</param>
/// <param name="subCodeOnFailure">Context-specific alarm [subcode] to display if the assertion fails.</param>
#define motoRos_ASSERT_LESS_THAN_OR_EQUAL_TO_INT(actual, threshold, subCodeOnFailure) \
    motoRos_ASSERT_GENERIC_COMPARISON_INT_MESSAGE(actual, threshold, subCodeOnFailure, #actual, #threshold, <=, APPLICATION_NAME ": Fatal Error")

/// <summary>
/// Validate that an integer value is less than or equal to a given threshold value. If the value is not 
/// less than or equal to the threshold value, then raise a fatal alarm on the pendant and 
/// block further execution. The alarm will indicate a context-specific MotoROS2 code.
/// An additional message will be displayed to help the user understand the error.
/// </summary>
/// <param name="actual">Value being evaluated to check if it is less than or equal to "threshold".</param>
/// <param name="threshold">Maximum value that "actual" must be equal to or less than.</param>
/// <param name="subCodeOnFailure">Context-specific alarm [subcode] to display if the assertion fails.</param>
/// <param name="msgFmtOnFailure">Format-string msg to display if the assertion fails.</param>
#define motoRos_ASSERT_LESS_THAN_OR_EQUAL_TO_INT_MESSAGE(actual, threshold, subCodeOnFailure, msgFmtOnFailure, ...) \
    motoRos_ASSERT_GENERIC_COMPARISON_INT_MESSAGE(actual, threshold, subCodeOnFailure, #actual, #threshold, <=, msgFmtOnFailure, ##__VA_ARGS__)

/// <summary>
/// Validate that an integer value is less than a given threshold value. If the value is not 
/// less than the threshold value, then raise a fatal alarm on the pendant and 
/// block further execution. The alarm will indicate a context-specific MotoROS2 code.
/// </summary>
/// <param name="actual">Value being evaluated to check if it is less than "threshold".</param>
/// <param name="threshold">Maximum value that "actual" must be less than.</param>
/// <param name="subCodeOnFailure">Context-specific alarm [subcode] to display if if the assertion fails.</param>
#define motoRos_ASSERT_LESS_THAN_INT(actual, threshold, subCodeOnFailure) \
    motoRos_ASSERT_GENERIC_COMPARISON_INT_MESSAGE(actual, threshold, subCodeOnFailure, #actual, #threshold, <, APPLICATION_NAME ": Fatal Error")

/// <summary>
/// Validate that an integer value is less than a given threshold value. If the value is not 
/// less than the threshold value, then raise a fatal alarm on the pendant and 
/// block further execution. The alarm will indicate a context-specific MotoROS2 code.
/// An additional message will be displayed to help the user understand the error.
/// </summary>
/// <param name="actual">Value being evaluated to check if it is less than "threshold".</param>
/// <param name="threshold">Maximum value that "actual" must be less than.</param>
/// <param name="subCodeOnFailure">Context-specific alarm [subcode] to display if the assertion fails.</param>
/// <param name="msgFmtOnFailure">Format-string msg to display if the assertion fails.</param>
#define motoRos_ASSERT_LESS_THAN_INT_MESSAGE(actual, threshold, subCodeOnFailure, msgFmtOnFailure, ...) \
    motoRos_ASSERT_GENERIC_COMPARISON_INT_MESSAGE(actual, threshold, subCodeOnFailure, #actual, #threshold, <, msgFmtOnFailure, ##__VA_ARGS__)

/// <summary>
/// Validate that a value is NULL. If the value is not NULL then raise a fatal alarm on the 
/// pendant and block further execution. The alarm will indicate a context-specific MotoROS2 code.
/// </summary>
/// <param name="ptr">Value to confirm is NULL.</param>
/// <param name="subCodeOnFailure">Context-specific alarm [subcode] to display if the assertion fails.</param>
#define motoRos_ASSERT_NULL(ptr, subCodeOnFailure) \
    motoRos_ASSERT_GENERIC_COMPARISON_POINTER_MESSAGE(ptr, NULL, subCodeOnFailure, #ptr, "NULL", ==, APPLICATION_NAME ": Fatal Error")

/// <summary>
/// Validate that a value is NULL. If the value is not NULL then raise a fatal alarm on the 
/// pendant and block further execution. The alarm will indicate a context-specific MotoROS2 code.
/// An additional message will be displayed to help the user understand the error.
/// </summary>
/// <param name="ptr">Value to confirm is NULL.</param>
/// <param name="subCodeOnFailure">Context-specific alarm [subcode] to display if the assertion fails.</param>
/// <param name="msgFmtOnFailure">Format-string msg to display to the user if the assertion fails.</param>
#define motoRos_ASSERT_NULL_MESSAGE(ptr, subCodeOnFailure, msgFmtOnFailure, ...) \
    motoRos_ASSERT_GENERIC_COMPARISON_POINTER_MESSAGE(ptr, NULL, subCodeOnFailure,  #ptr, "NULL", ==, msgFmtOnFailure, ##__VA_ARGS__)

/// <summary>
/// Validate that a value is not NULL. If the value is NULL then raise a fatal alarm on the 
/// pendant and block further execution. The alarm will indicate a context-specific MotoROS2 code.
/// </summary>
/// <param name="ptr">Value to confirm is not NULL.</param>
/// <param name="subCodeOnFailure">Context-specific alarm [subcode] to display if the assertion fails.</param>
#define motoRos_ASSERT_NOT_NULL(ptr, subCodeOnFailure) \
    motoRos_ASSERT_GENERIC_COMPARISON_POINTER_MESSAGE(ptr, NULL, subCodeOnFailure,  #ptr, "NULL", !=, APPLICATION_NAME ": Fatal Error")

/// <summary>
/// Validate that a value is not NULL. If the value is NULL then raise a fatal alarm on the 
/// pendant and block further execution. The alarm will indicate a context-specific MotoROS2 code.
/// An additional message will be displayed to help the user understand the error.
/// </summary>
/// <param name="ptr">Value to confirm is not NULL.</param>
/// <param name="subCodeOnFailure">Context-specific alarm [subcode] to display if the assertion fails.</param>
/// <param name="msgFmtOnFailure">Format-string msg to display to the user if the assertion fails.</param>
#define motoRos_ASSERT_NOT_NULL_MESSAGE(ptr, subCodeOnFailure, msgFmtOnFailure, ...) \
    motoRos_ASSERT_GENERIC_COMPARISON_POINTER_MESSAGE(ptr, NULL, subCodeOnFailure,  #ptr, "NULL", !=, msgFmtOnFailure, ##__VA_ARGS__)

/// <summary>
/// Raise a fatal alarm on the pendant and block further execution. The alarm will indicate a context-specific MotoROS2 code.
/// </summary>
/// <param name="subCodeOnFailure">Context-specific alarm [subcode] to display.</param>
extern void motoRos_ASSERT_FAIL(ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure);

/// <summary>
/// Raise a fatal alarm on the pendant and block further execution. The alarm will indicate a context-specific MotoROS2 code.
/// An additional message will be displayed to help the user understand the error.
/// </summary>
/// <param name="subCodeOnFailure">Context-specific alarm [subcode] to display.</param>
/// <param name="msgFmtOnFailure">Format-string msg to display.</param>
extern void motoRos_ASSERT_FAIL_MESSAGE(ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure, char* msgFmtOnFailure, ...);

/// <summary>
/// Validate that an RCL return value is OK. If the return code is anything other than OK,
/// then raise a fatal alarm on the pendant and block further execution. The alarm will
/// indicate both a context-specific MotoROS2 code and also the RCL return code.
/// </summary>
/// <param name="rcl_return_code">RCL return code to verify is OK. Assertion will occur if not OK.</param>
/// <param name="subCodeOnFailure">Context-specific alarm [subcode] to display in addition to the RCL return code.</param>
extern void motoRos_RCLAssertOK(int rcl_return_code, ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure);

/// <summary>
/// Validate that an RCL return value is OK. If the return code is anything other than OK,
/// then raise a fatal alarm on the pendant and block further execution. The alarm will
/// indicate both a context-specific MotoROS2 code and also the RCL return code. An additional
/// message will be displayed to help the user understand the error.
/// </summary>
/// <param name="rcl_return_code">RCL return code to verify is OK. Assertion will occur if not OK.</param>
/// <param name="subCodeOnFailure">Context-specific alarm [subcode] to display in addition to the RCL return code.</param>
/// <param name="msgFmtOnFailure">Format-string msge to display to the user.</param>
extern void motoRos_RCLAssertOK_withMsg(int rcl_return_code, ALARM_ASSERTION_FAIL_SUBCODE subCodeOnFailure, char* msgFmtOnFailure, ...);

extern const char* const Ros_ErrorHandling_ErrNo_ToString(int errNo);
extern const char* const Ros_ErrorHandling_MotionNotReadyCode_ToString(MotionNotReadyCode code);
extern const char* const Ros_ErrorHandling_Init_Trajectory_Status_ToString(Init_Trajectory_Status code);

#endif  // MOTOROS2_ERROR_HANDLING_H
