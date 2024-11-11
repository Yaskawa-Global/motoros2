//Controller.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_CONTROLLER_STATUS_IO_H
#define MOTOROS2_CONTROLLER_STATUS_IO_H

#define IO_FEEDBACK_WAITING_MP_INCMOVE      11120  //output# 889
#define IO_FEEDBACK_MP_INCMOVE_DONE         11121  //output# 890
#define IO_FEEDBACK_INITIALIZATION_DONE     11122  //output# 891
#define IO_FEEDBACK_CONTROLLERRUNNING       11123  //output# 892
#define IO_FEEDBACK_AGENTCONNECTED          11124  //output# 893
#define IO_FEEDBACK__                       11125  //output# 894
#define IO_FEEDBACK___                      11126  //output# 895
#define IO_FEEDBACK_FAILURE                 11127  //output# 896

#define IO_FEEDBACK_RESERVED_1              11130  //output# 897
#define IO_FEEDBACK_RESERVED_2              11131  //output# 898
#define IO_FEEDBACK_RESERVED_3              11132  //output# 899
#define IO_FEEDBACK_RESERVED_4              11133  //output# 900
#define IO_FEEDBACK_RESERVED_5              11134  //output# 901
#define IO_FEEDBACK_RESERVED_6              11135  //output# 902
#define IO_FEEDBACK_RESERVED_7              11136  //output# 903
#define IO_FEEDBACK_RESERVED_8              11137  //output# 904

#define INVALID_TASK                        -1

#define MAX_CONTROLLABLE_GROUPS             8

#define MASK_ISALARM_ACTIVEALARM            0x02
#define MASK_ISALARM_ACTIVEERROR            0x01

#define MAX_ROBOT_CALIBRATION_FILES         32

#define MIN_VALID_TOOL_INDEX                0
#define MAX_VALID_TOOL_INDEX                63

typedef enum
{
    IO_ROBOTSTATUS_ALARM_MAJOR = 0,
    IO_ROBOTSTATUS_ALARM_MINOR,
    IO_ROBOTSTATUS_ALARM_SYSTEM,
    IO_ROBOTSTATUS_ALARM_USER,
    IO_ROBOTSTATUS_ERROR,
    IO_ROBOTSTATUS_PLAY,
    IO_ROBOTSTATUS_TEACH,
    IO_ROBOTSTATUS_REMOTE,
    IO_ROBOTSTATUS_OPERATING,
    IO_ROBOTSTATUS_HOLD,
    IO_ROBOTSTATUS_SERVO,
    IO_ROBOTSTATUS_ESTOP_EX,
    IO_ROBOTSTATUS_ESTOP_PP,
    IO_ROBOTSTATUS_ESTOP_CTRL,
    IO_ROBOTSTATUS_WAITING_ROS,
    IO_ROBOTSTATUS_INECOMODE,
    IO_ROBOTSTATUS_CONT_CYC_MODE,
#if (YRC1000||YRC1000u)
    IO_ROBOTSTATUS_PFL_STOP,
    IO_ROBOTSTATUS_PFL_ESCAPE,
    IO_ROBOTSTATUS_PFL_AVOIDING,
    IO_ROBOTSTATUS_PFL_AVOID_JOINT,
    IO_ROBOTSTATUS_PFL_AVOID_TRANS,
#endif
    IO_ROBOTSTATUS_MAX
} IoStatusIndex;

typedef struct
{
    UINT16 interpolPeriod;                                  // Interpolation period of the controller
    int numGroup;                                           // Actual number of defined group
    //int numRobot;                                         // Actual number of defined robot
    int totalAxesCount;                                     // Number of axes attached to the controller (all groups)
    CtrlGroup* ctrlGroups[MAX_CONTROLLABLE_GROUPS];         // Array of the controller control group

    // Controller Status
    MP_IO_INFO ioStatusAddr[IO_ROBOTSTATUS_MAX];            // Array of Specific Input Address representing the I/O status
    USHORT ioStatus[IO_ROBOTSTATUS_MAX];                    // Array storing the current status of the controller
    int alarmCode;                                          // Alarm number currently active
    BOOL bStopMotion;                                       // Flag to stop motion
    BOOL bPFLEnabled;                                       // Flag indicating that the controller has the PFL option enabled
    BOOL bPFLduringRosMove;                                 // Flag to keep track PFL activation during RosMotion
    BOOL bMpIncMoveError;                                   // Flag indicating that the incremental motion API failed
    BOOL bPrevAlarmState;                                   // Flag indicating if there was an active ALARM during the last I/O cycle

    int tidIncMoveThread;                                   // ThreadId for sending the incremental move to the controller
} Controller;

typedef struct
{
    rcl_publisher_t robotStatus;
} ControllerStatus_Publishers;
extern ControllerStatus_Publishers g_publishers_RobotStatus;

typedef struct
{
    industrial_msgs__msg__RobotStatus* msgRobotStatus;
} ControllerStatus_Messages;
extern ControllerStatus_Messages g_messages_RobotStatus;

extern Controller g_Ros_Controller;

extern BOOL Ros_Controller_Initialize();
extern void Ros_Controller_Cleanup();

extern BOOL Ros_Controller_IsValidGroupNo(int groupNo);

extern void Ros_Controller_StatusInit();
extern BOOL Ros_Controller_StatusRead(USHORT ioStatus[IO_ROBOTSTATUS_MAX]);
extern BOOL Ros_Controller_IoStatusUpdate();
extern BOOL Ros_Controller_IsAlarm();
extern BOOL Ros_Controller_IsMajorAlarm();
extern BOOL Ros_Controller_IsError();
extern BOOL Ros_Controller_IsPlay();
extern BOOL Ros_Controller_IsTeach();
extern BOOL Ros_Controller_IsRemote();
extern BOOL Ros_Controller_IsOperating();
extern BOOL Ros_Controller_IsHold();
extern BOOL Ros_Controller_IsServoOn();
extern BOOL Ros_Controller_IsEcoMode();
extern BOOL Ros_Controller_IsEStop();
extern BOOL Ros_Controller_IsWaitingRos();
extern BOOL Ros_Controller_IsContinuousCycle();
extern BOOL Ros_Controller_IsMotionReady();
extern BOOL Ros_Controller_IsInMotion();
extern BOOL Ros_Controller_IsPflActive();
extern BOOL Ros_Controller_IsMpIncMoveErrorActive();
extern BOOL Ros_Controller_IsAnyFaultActive();
extern BOOL Ros_Controller_MasterTaskIsJobName(const char* const jobName);
extern MotionNotReadyCode Ros_Controller_GetNotReadySubcode(bool ignoreTractableProblems);

//reset internal flag indicating whether PFL became active during a move
extern void Ros_Controller_Reset_PflDuringRosMove();

//reset internal flag indicating whether mpIncMove(..) has errored
extern void Ros_Controller_Reset_MpIncMoveError();

extern BOOL Ros_Controller_GetIOState(ULONG signal);
extern void Ros_Controller_SetIOState(ULONG signal, BOOL status);

extern int Ros_Controller_GetAlarmCode();

//retrieve all active alarms (and a possible error) and store them in 'active_alarms'
extern int Ros_Controller_GetActiveAlarmCodes(USHORT active_alarms[MAX_ALARM_COUNT + MAX_ERROR_COUNT]);

//TODO(gavanderhoorn): make static, see comment on definition
extern BOOL Ros_Controller_ShouldWarnNoCalibDataLoaded(Controller const* controller, BOOL bCalibLoadedOk, BOOL bPublishTfEnabled);

//#define DUMMY_SERVO_MODE 1    // Dummy servo mode is used for testing with Yaskawa debug controllers
#ifdef DUMMY_SERVO_MODE
#warning Dont forget to disable DUMMY_SERO_MODE when done testing
#endif

#endif  // MOTOROS2_CONTROLLER_STATUS_IO_H
