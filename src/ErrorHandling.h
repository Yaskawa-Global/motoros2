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

#define motoRos_ASSERT_TRUE(mustBeTrue,subCodeIfFalse) motoRos_complete_ASSERT_TRUE(mustBeTrue, subCodeIfFalse, #mustBeTrue)
#define motoRos_ASSERT_TRUE_MESSAGE(mustBeTrue, subCodeIfFalse,msgFmtIfFalse, ...) motoRos_complete_ASSERT_TRUE_MESSAGE(mustBeTrue, subCodeIfFalse, #mustBeTrue, msgFmtIfFalse, ##__VA_ARGS__)
#define motoRos_ASSERT_FALSE(mustBeFalse, subCodeIfTrue) motoRos_complete_ASSERT_FALSE(mustBeFalse, subCodeIfTrue, #mustBeFalse)
#define motoRos_ASSERT_FALSE_MESSAGE(mustBeFalse, subCodeIfTrue, msgFmtIfTrue, ...) motoRos_complete_ASSERT_FALSE_MESSAGE(mustBeFalse, subCodeIfTrue, #mustBeFalse, msgFmtIfTrue, ##__VA_ARGS__)
#define motoRos_ASSERT_EQ_INT(actual, expected, subCodeIfFalse) motoRos_complete_ASSERT_EQ_INT(actual, expected, subCodeIfFalse, #actual, #expected)
#define motoRos_ASSERT_EQ_INT_MESSAGE(actual, expected, subCodeIfFalse, msgFmtIfFalse, ...) motoRos_complete_ASSERT_EQ_INT_MESSAGE(actual, expected, subCodeIfFalse, #actual, #expected, msgFmtIfFalse, ##__VA_ARGS__)
#define motoRos_ASSERT_NE_INT(actual, expected, subCodeIfFalse) motoRos_complete_ASSERT_NE_INT(actual, expected, subCodeIfFalse, #actual, #expected)
#define motoRos_ASSERT_NE_INT_MESSAGE(actual, expected, subCodeIfFalse, msgFmtIfFalse, ...) motoRos_complete_ASSERT_NE_INT_MESSAGE(actual, expected, subCodeIfFalse, #actual, #expected, msgFmtIfFalse, ##__VA_ARGS__)
#define motoRos_ASSERT_GE_INT(actual, expected, subCodeIfFalse) motoRos_complete_ASSERT_GE_INT(actual, expected, subCodeIfFalse, #actual, #expected)
#define motoRos_ASSERT_GE_INT_MESSAGE(actual, expected, subCodeIfFalse, msgFmtIfFalse, ...) motoRos_complete_ASSERT_GE_INT_MESSAGE(actual, expected, subCodeIfFalse, #actual, #expected, msgFmtIfFalse, ##__VA_ARGS__)
#define motoRos_ASSERT_GT_INT(actual, expected, subCodeIfFalse) motoRos_complete_ASSERT_GT_INT(actual, expected, subCodeIfFalse, #actual, #expected)
#define motoRos_ASSERT_GT_INT_MESSAGE(actual, expected, subCodeIfFalse, msgFmtIfFalse, ...) motoRos_complete_ASSERT_GT_INT_MESSAGE(actual, expected, subCodeIfFalse, #actual, #expected, msgFmtIfFalse, ##__VA_ARGS__)
#define motoRos_ASSERT_LE_INT(actual, expected, subCodeIfFalse) motoRos_complete_ASSERT_LE_INT(actual, expected, subCodeIfFalse, #actual, #expected)
#define motoRos_ASSERT_LE_INT_MESSAGE(actual, expected, subCodeIfFalse, msgFmtIfFalse, ...) motoRos_complete_ASSERT_LE_INT_MESSAGE(actual, expected, subCodeIfFalse, #actual, #expected, msgFmtIfFalse, ##__VA_ARGS__)
#define motoRos_ASSERT_LT_INT(actual, expected, subCodeIfFalse) motoRos_complete_ASSERT_LT_INT(actual, expected, subCodeIfFalse, #actual, #expected)
#define motoRos_ASSERT_LT_INT_MESSAGE(actual, expected, subCodeIfFalse, msgFmtIfFalse, ...) motoRos_complete_ASSERT_LT_INT_MESSAGE(actual, expected, subCodeIfFalse, #actual, #expected, msgFmtIfFalse, ##__VA_ARGS__)
#define motoRos_ASSERT_NULL(ptr, subCodeIfNotNull) motoRos_complete_ASSERT_NULL(ptr, subCodeIfNotNull, #ptr)
#define motoRos_ASSERT_NULL_MESSAGE(ptr, subCodeIfNotNull, msgFmtIfNotNull, ...) motoRos_complete_ASSERT_NULL(ptr, subCodeIfNotNull, #ptr, msgFmtIfNotNull, ##__VA_ARGS__)
#define motoRos_ASSERT_NOT_NULL(ptr, subCodeIfNull) motoRos_complete_ASSERT_NOT_NULL(ptr, subCodeIfNull, #ptr)
#define motoRos_ASSERT_NOT_NULL_MESSAGE(ptr, subCodeIfNull, msgFmtIfNull, ...) motoRos_complete_ASSERT_NOT_NULL_MESSAGE(ptr, subCodeIfNull, #ptr, msgFmtIfNull, ##__VA_ARGS__)

extern void motoRos_ASSERT_FAIL(ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse);
extern void motoRos_ASSERT_FAIL_MESSAGE(ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse, char* msgFmtIfFalse, ...);
extern void motoRos_complete_ASSERT_TRUE(BOOL mustBeTrue, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse, char* actualName);
extern void motoRos_complete_ASSERT_TRUE_MESSAGE(BOOL mustBeTrue, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse, char* actualName, char* msgFmtIfFalse, ...);
extern void motoRos_complete_ASSERT_FALSE(BOOL mustBeFalse, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse, char* actualName);
extern void motoRos_complete_ASSERT_FALSE_MESSAGE(BOOL mustBeFalse, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse, char* actualName, char* msgFmtIfFalse, ...);
extern void motoRos_complete_ASSERT_EQ_INT(int actual, int expected, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse, char* actualName, char* expectedName);
extern void motoRos_complete_ASSERT_EQ_INT_MESSAGE(int actual, int expected, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse, char* actualName, char* expectedName, char* msgFmtIfFalse, ...);
extern void motoRos_complete_ASSERT_NE_INT(int actual, int expected, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse, char* actualName, char* expectedName);
extern void motoRos_complete_ASSERT_NE_INT_MESSAGE(int actual, int expected, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse, char* actualName, char* expectedName, char* msgFmtIfFalse, ...);
extern void motoRos_complete_ASSERT_GE_INT(int actual, int expected, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse, char* actualName, char* expectedName);
extern void motoRos_complete_ASSERT_GE_INT_MESSAGE(int actual, int expected, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse, char* actualName, char* expectedName, char* msgFmtIfFalse, ...);
extern void motoRos_complete_ASSERT_GT_INT(int actual, int expected, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse, char* actualName, char* expectedName);
extern void motoRos_complete_ASSERT_GT_INT_MESSAGE(int actual, int expected, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse, char* actualName, char* expectedName, char* msgFmtIfFalse, ...);
extern void motoRos_complete_ASSERT_LE_INT(int actual, int expected, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse, char* actualName, char* expectedName);
extern void motoRos_complete_ASSERT_LE_INT_MESSAGE(int actual, int expected, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse, char* actualName, char* expectedName, char* msgFmtIfFalse, ...);
extern void motoRos_complete_ASSERT_LT_INT(int actual, int expected, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse, char* actualName, char* expectedName);
extern void motoRos_complete_ASSERT_LT_INT_MESSAGE(int actual, int expected, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse, char* actualName, char* expectedName, char* msgFmtIfFalse, ...);
extern void motoRos_complete_ASSERT_NULL(void* ptr, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse, char* actualName);
extern void motoRos_complete_ASSERT_NULL_MESSAGE(void* ptr, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse, char* actualName, char* msgFmtIfFalse, ...);
extern void motoRos_complete_ASSERT_NOT_NULL(void* ptr, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse, char* actualName);
extern void motoRos_complete_ASSERT_NOT_NULL_MESSAGE(void* ptr, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse, char* actualName, char* msgFmtIfFalse, ...);
typedef enum
{
    SUBCODE_RCL_RCLC_API_ERROR,
} ALARM_RCL_RCLC_FAIL_SUBCODE; //8017

/// <summary>
/// Validate that an RCL return value is OK. If the return code is anything other than OK,
/// then raise a fatal alarm on the pendant and block further execution. The alarm will
/// indicate both a context-specific MotoROS2 code and also the RCL return code.
/// </summary>
/// <param name="rcl_return_code">RCL return code to verify is OK. Assertion will occur if not OK.</param>
/// <param name="subCodeIfFalse">Context-specific alarm [subcode] to display in addition to the RCL return code.</param>
extern void motoRos_RCLAssertOK(int rcl_return_code, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse);

/// <summary>
/// Validate that an RCL return value is OK. If the return code is anything other than OK,
/// then raise a fatal alarm on the pendant and block further execution. The alarm will
/// indicate both a context-specific MotoROS2 code and also the RCL return code. An additional
/// message will be displayed to help the user understand the error.
/// </summary>
/// <param name="rcl_return_code">RCL return code to verify is OK. Assertion will occur if not OK.</param>
/// <param name="subCodeIfFalse">Context-specific alarm [subcode] to display in addition to the RCL return code.</param>
/// <param name="msgFmtIfFalse">Format-string msge to display to the user.</param>
extern void motoRos_RCLAssertOK_withMsg(int rcl_return_code, ALARM_ASSERTION_FAIL_SUBCODE subCodeIfFalse, char* msgFmtIfFalse, ...);

extern const char* const Ros_ErrorHandling_ErrNo_ToString(int errNo);
extern const char* const Ros_ErrorHandling_MotionNotReadyCode_ToString(MotionNotReadyCode code);
extern const char* const Ros_ErrorHandling_Init_Trajectory_Status_ToString(Init_Trajectory_Status code);

#endif  // MOTOROS2_ERROR_HANDLING_H
