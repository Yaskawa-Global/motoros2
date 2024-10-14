// Controller.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

Controller g_Ros_Controller;
ControllerStatus_Publishers g_publishers_RobotStatus;
ControllerStatus_Messages g_messages_RobotStatus;

//'private' APIs
static BOOL Ros_Controller_LoadGroupCalibrationData(Controller* const controller);

//-------------------------------------------------------------------
// Wait for the controller to be ready to start initialization
//-------------------------------------------------------------------
BOOL Ros_Controller_WaitInitReady()
{
    do  //minor alarms can be delayed briefly after bootup
    {
        Ros_Debug_BroadcastMsg("Waiting for robot alarms to clear...");
        Ros_Sleep(2500);
        Ros_Controller_StatusRead(g_Ros_Controller.ioStatus);

    } while (Ros_Controller_IsAlarm());

    return TRUE;
}

//-------------------------------------------------------------------
// Initialize the controller structure
// This should be done before the controller is used for anything
//-------------------------------------------------------------------
BOOL Ros_Controller_Initialize()
{
    int groupIndex;
    BOOL bInitOk;
    STATUS status;

    MOTOROS2_MEM_TRACE_START(ctrlr_init);

    Ros_Debug_BroadcastMsg("Initializing controller");

    //==================================
    // Init variables and controller status
    bInitOk = TRUE;
    g_Ros_Controller.bStopMotion = FALSE;
    status = GP_isPflEnabled(&g_Ros_Controller.bPFLEnabled);
    if (status != OK)
        bInitOk = FALSE;
    if (g_Ros_Controller.bPFLEnabled)
        Ros_Debug_BroadcastMsg("System has PFL Enabled");
    g_Ros_Controller.bPFLduringRosMove = FALSE;
    g_Ros_Controller.bMpIncMoveError = FALSE;
    g_Ros_Controller.bPrevAlarmState = FALSE;

    //==================================
    // Get the interpolation clock
    status = GP_getInterpolationPeriod(&g_Ros_Controller.interpolPeriod);
    if(status!=OK)
        bInitOk = FALSE;

    // Get the number of groups
    g_Ros_Controller.numGroup = GP_getNumberOfGroups();
    Ros_Debug_BroadcastMsg("Number of motion groups: %d (max supported: %d)",
        g_Ros_Controller.numGroup, MAX_CONTROLLABLE_GROUPS);

    // too few groups is not OK
    if(g_Ros_Controller.numGroup < 1)
        bInitOk = FALSE;

    // but too many is also not OK
    motoRosAssert(g_Ros_Controller.numGroup <= MAX_CONTROLLABLE_GROUPS,
        SUBCODE_FAIL_ROS_CONTROLLER_INIT_TOO_MANY_GROUPS);

    BOOL bShouldSetJointNamesToDefaultValues = (strlen(g_nodeConfigSettings.joint_names[0]) == 0);

    //==================================
    //Create control groups
    g_Ros_Controller.totalAxesCount = 0;
    // Check for each group
    for(groupIndex=0; groupIndex < MAX_CONTROLLABLE_GROUPS; groupIndex++)
    {
        if(groupIndex < g_Ros_Controller.numGroup)
        {
            // Determine if specific group exists and allocate memory for it
            g_Ros_Controller.ctrlGroups[groupIndex] = Ros_CtrlGroup_Create(groupIndex,                       //Zero based index of the group number(0 - 3)
                                                                (groupIndex==(g_Ros_Controller.numGroup-1)), //TRUE if this is the final group that is being initialized. FALSE if you plan to call this function again.
                                                                g_Ros_Controller.interpolPeriod);            //Value of the interpolation period (ms) for the robot controller.
            if(g_Ros_Controller.ctrlGroups[groupIndex] != NULL)
            {
                Ros_CtrlGroup_GetPulsePosCmd(g_Ros_Controller.ctrlGroups[groupIndex], g_Ros_Controller.ctrlGroups[groupIndex]->prevPulsePos); // set the current commanded pulse
                g_Ros_Controller.totalAxesCount += g_Ros_Controller.ctrlGroups[groupIndex]->numAxes;

                if (bShouldSetJointNamesToDefaultValues) //joint names were NOT specified in the yaml config file
                {
                    for (int jointIndex = 0; jointIndex < g_Ros_Controller.ctrlGroups[groupIndex]->numAxes; jointIndex += 1)
                    {
                        sprintf(g_nodeConfigSettings.joint_names[(groupIndex * MP_GRP_AXES_NUM) + jointIndex], DEFAULT_JOINT_NAME_FMT, groupIndex + 1, jointIndex + 1);
                    }
                }

                //ensure that the default values have been set before calling this function
                Ros_CtrlGroup_UpdateJointNamesInMotoOrder(g_Ros_Controller.ctrlGroups[groupIndex]);
            }
            else
                bInitOk = FALSE;

            Ros_Debug_BroadcastMsg("Created ctrl group %d, memfree = (%d) bytes", groupIndex, mpNumBytesFree());
        }
        else
            g_Ros_Controller.ctrlGroups[groupIndex] = NULL;
    }

    //get the robot calibration data for multi-robot systems
    const BOOL bCalibLoadedOk = Ros_Controller_LoadGroupCalibrationData(&g_Ros_Controller);
    //see whether the user should be notified about failures (it's OK to not
    //have/load any calibration in some cases)
    const BOOL bNeedToWarnAboutCalib = Ros_Controller_ShouldWarnNoCalibDataLoaded(
        &g_Ros_Controller, bCalibLoadedOk, g_nodeConfigSettings.publish_tf);
    Ros_Debug_BroadcastMsg(
        "%s: calib loaded ok: %s, should warn: %s", __func__,
            bCalibLoadedOk ? "yes" : "no", bNeedToWarnAboutCalib ? "yes" : "no");
    if (bNeedToWarnAboutCalib)
    {
        if (g_nodeConfigSettings.ignore_missing_calib_data)
        {
            Ros_Debug_BroadcastMsg(
                "%s: ignoring absence of calibration data (or load failure) as configured",
                __func__);
        }
        else
        {
            mpSetAlarm(ALARM_CONFIGURATION_FAIL, "No calibration: invalid TF",
                SUBCODE_CONFIGURATION_NO_CALIB_FILES_LOADED);
            Ros_Debug_BroadcastMsg(
                "%s: no calibration files loaded: TF potentially incorrect, posting warning "
                "(disable with 'ignore_missing_calib_data')", __func__);
        }
    }

#ifdef DEBUG
    Ros_Debug_BroadcastMsg("g_Ros_Controller.numRobot = %d", g_Ros_Controller.numRobot);
#endif

    //==================================
    //create publisher for robot status
    const rmw_qos_profile_t* qos_profile = Ros_ConfigFile_To_Rmw_Qos_Profile(g_nodeConfigSettings.qos_robot_status);
    rcl_ret_t ret;
    ret = rclc_publisher_init(
        &g_publishers_RobotStatus.robotStatus,
        &g_microRosNodeInfo.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(industrial_msgs, msg, RobotStatus),
        TOPIC_NAME_ROBOT_STATUS,
        qos_profile);
    motoRosAssert(ret == RCL_RET_OK, SUBCODE_FAIL_CREATE_PUBLISHER_ROBOT_STATUS);

    //==================================
    //create message for robot status
    //TODO(gavanderhoorn): use micro_ros_utilities_create_message_memory(..) instead
    g_messages_RobotStatus.msgRobotStatus = industrial_msgs__msg__RobotStatus__create();
    rosidl_runtime_c__int32__Sequence__init(&g_messages_RobotStatus.msgRobotStatus->error_codes, MAX_ALARM_COUNT + 1);

    //==================================
    // If not started, start the IncMoveTask (there should be only one instance of this thread)
    if (g_Ros_Controller.tidIncMoveThread == INVALID_TASK)
    {
        Ros_Debug_BroadcastMsg("Creating new task: IncMoveTask");

        g_Ros_Controller.tidIncMoveThread = mpCreateTask(MP_PRI_IP_CLK_TAKE, MP_STACK_SIZE,
            (FUNCPTR)Ros_MotionControl_IncMoveLoopStart,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        if (g_Ros_Controller.tidIncMoveThread == ERROR)
        {
            Ros_Debug_BroadcastMsg("Failed to create task for incremental-motion.  Check robot parameters.");
            g_Ros_Controller.tidIncMoveThread = INVALID_TASK;
            Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
            mpSetAlarm(ALARM_TASK_CREATE_FAIL, APPLICATION_NAME " FAILED TO CREATE TASK", SUBCODE_INCREMENTAL_MOTION);

            return FALSE;
        }
    }

    //==================================
    // Check and report eco-mode settings
    ECO_MODE_INFO eco_mode_info;
    if (GP_getEcoModesettings(&eco_mode_info) == OK)
    {
        Ros_Debug_BroadcastMsg("Eco-mode: %sabled", eco_mode_info.bEnabled ? "en" : "dis");
        if (eco_mode_info.bEnabled)
        {
            Ros_Debug_BroadcastMsg("Eco-mode: timeout: %u %s",
                eco_mode_info.timeout,
                eco_mode_info.timeUnit == ECO_UNIT_SECONDS ? "sec" : "min");
        }
    }
    else
    {
        Ros_Debug_BroadcastMsg("Couldn't retrieve eco-mode settings");
        // this is not fatal, just unfortunate
    }

    //==================================
    if(bInitOk)
    {
        // Turn on initialization done I/O signal
        Ros_Controller_SetIOState(IO_FEEDBACK_INITIALIZATION_DONE, TRUE);
    }
    else
    {
        Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
        Ros_Debug_BroadcastMsg("Failure to initialize controller");
    }

    MOTOROS2_MEM_TRACE_REPORT(ctrlr_init);

    return bInitOk;
}

void Ros_Controller_Cleanup()
{
    rcl_ret_t ret;

    MOTOROS2_MEM_TRACE_START(ctrlr_fini);

    //--------------------------------
    // Cleanup memory
    //

    for (int groupNum = 0; groupNum < MAX_CONTROLLABLE_GROUPS; groupNum += 1)
    {
        if (g_Ros_Controller.ctrlGroups[groupNum] != NULL)
        {
            Ros_Debug_BroadcastMsg("Cleanup control group %d", groupNum + 1);
            Ros_CtrlGrp_Cleanup(g_Ros_Controller.ctrlGroups[groupNum]);
            mpFree(g_Ros_Controller.ctrlGroups[groupNum]);
        }
    }

    mpDeleteTask(g_Ros_Controller.tidIncMoveThread);
    g_Ros_Controller.tidIncMoveThread = INVALID_TASK;

    Ros_Debug_BroadcastMsg("Cleanup publisher robot status");
    ret = rcl_publisher_fini(&g_publishers_RobotStatus.robotStatus, &g_microRosNodeInfo.node);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up robot status publisher: %d", ret);

    industrial_msgs__msg__RobotStatus__destroy(g_messages_RobotStatus.msgRobotStatus);

    MOTOROS2_MEM_TRACE_REPORT(ctrlr_fini);
}

//-------------------------------------------------------------------
// Check the number of inc_move currently in the specified queue
//-------------------------------------------------------------------
BOOL Ros_Controller_IsValidGroupNo(int groupNo)
{
    if((groupNo >= 0) && (groupNo < g_Ros_Controller.numGroup))
        return TRUE;
    else
    {
        Ros_Debug_BroadcastMsg("ERROR: Attempt to access invalid Group No. (%d)", groupNo);
        return FALSE;
    }
}


/**** Controller Status functions ****/

//-------------------------------------------------------------------
// Initialize list of Specific Input to keep track of
//-------------------------------------------------------------------
void Ros_Controller_StatusInit()
{
    g_Ros_Controller.ioStatusAddr[IO_ROBOTSTATUS_ALARM_MAJOR].ulAddr = 50010;       // Alarm
    g_Ros_Controller.ioStatusAddr[IO_ROBOTSTATUS_ALARM_MINOR].ulAddr = 50011;       // Alarm
    g_Ros_Controller.ioStatusAddr[IO_ROBOTSTATUS_ALARM_SYSTEM].ulAddr = 50012;      // Alarm
    g_Ros_Controller.ioStatusAddr[IO_ROBOTSTATUS_ALARM_USER].ulAddr = 50013;        // Alarm
    g_Ros_Controller.ioStatusAddr[IO_ROBOTSTATUS_ERROR].ulAddr = 50014;             // Error
    g_Ros_Controller.ioStatusAddr[IO_ROBOTSTATUS_PLAY].ulAddr = 50054;              // Play
    g_Ros_Controller.ioStatusAddr[IO_ROBOTSTATUS_TEACH].ulAddr = 50053;             // Teach
    g_Ros_Controller.ioStatusAddr[IO_ROBOTSTATUS_REMOTE].ulAddr = 80011; //50056;   // Remote  // Modified E.M. 7/9/2013
    g_Ros_Controller.ioStatusAddr[IO_ROBOTSTATUS_OPERATING].ulAddr = 50070;         // Operating
    g_Ros_Controller.ioStatusAddr[IO_ROBOTSTATUS_HOLD].ulAddr = 50071;              // Hold
    g_Ros_Controller.ioStatusAddr[IO_ROBOTSTATUS_SERVO].ulAddr = 50073;             // Servo ON
    g_Ros_Controller.ioStatusAddr[IO_ROBOTSTATUS_ESTOP_EX].ulAddr = 80025;          // External E-Stop
    g_Ros_Controller.ioStatusAddr[IO_ROBOTSTATUS_ESTOP_PP].ulAddr = 80026;          // Pendant E-Stop
    g_Ros_Controller.ioStatusAddr[IO_ROBOTSTATUS_ESTOP_CTRL].ulAddr = 80027;        // Controller E-Stop
    g_Ros_Controller.ioStatusAddr[IO_ROBOTSTATUS_WAITING_ROS].ulAddr = IO_FEEDBACK_WAITING_MP_INCMOVE; // Job input signaling ready for external motion
    g_Ros_Controller.ioStatusAddr[IO_ROBOTSTATUS_INECOMODE].ulAddr = 50727;         // Energy Saving Mode
    g_Ros_Controller.ioStatusAddr[IO_ROBOTSTATUS_CONT_CYC_MODE].ulAddr = 50052;     // Continuous Cycle Mode
#if (YRC1000||YRC1000u)
    g_Ros_Controller.ioStatusAddr[IO_ROBOTSTATUS_PFL_STOP].ulAddr = 81702;          // PFL function stopped the motion
    g_Ros_Controller.ioStatusAddr[IO_ROBOTSTATUS_PFL_ESCAPE].ulAddr = 81703;        // PFL function escape from clamping motion
    g_Ros_Controller.ioStatusAddr[IO_ROBOTSTATUS_PFL_AVOIDING].ulAddr = 15120;      // PFL function avoidance operating
    g_Ros_Controller.ioStatusAddr[IO_ROBOTSTATUS_PFL_AVOID_JOINT].ulAddr = 15124;   // PFL function avoidance joint enabled
    g_Ros_Controller.ioStatusAddr[IO_ROBOTSTATUS_PFL_AVOID_TRANS].ulAddr = 15125;   // PFL function avoidance translation enabled
#endif
    g_Ros_Controller.alarmCode = 0;

    //==================================
    // wait for controller to be ready for reading parameter
    Ros_Controller_WaitInitReady();
}


BOOL Ros_Controller_IsAlarm()
{
    return ((g_Ros_Controller.ioStatus[IO_ROBOTSTATUS_ALARM_MAJOR]!=0)
        || (g_Ros_Controller.ioStatus[IO_ROBOTSTATUS_ALARM_MINOR]!=0)
        || (g_Ros_Controller.ioStatus[IO_ROBOTSTATUS_ALARM_SYSTEM]!=0)
        || (g_Ros_Controller.ioStatus[IO_ROBOTSTATUS_ALARM_USER]!=0) );
}

BOOL Ros_Controller_IsMajorAlarm()
{
    return ((g_Ros_Controller.ioStatus[IO_ROBOTSTATUS_ALARM_MAJOR] != 0));
}

BOOL Ros_Controller_IsError()
{
    return ((g_Ros_Controller.ioStatus[IO_ROBOTSTATUS_ERROR]!=0));
}

BOOL Ros_Controller_IsPlay()
{
    return ((g_Ros_Controller.ioStatus[IO_ROBOTSTATUS_PLAY]!=0));
}

BOOL Ros_Controller_IsTeach()
{
    return ((g_Ros_Controller.ioStatus[IO_ROBOTSTATUS_TEACH]!=0));
}

BOOL Ros_Controller_IsRemote()
{
    return ((g_Ros_Controller.ioStatus[IO_ROBOTSTATUS_REMOTE]!=0));
}

BOOL Ros_Controller_IsOperating()
{
    return ((g_Ros_Controller.ioStatus[IO_ROBOTSTATUS_OPERATING]!=0));
}

BOOL Ros_Controller_IsHold()
{
    return ((g_Ros_Controller.ioStatus[IO_ROBOTSTATUS_HOLD]!=0));
}

BOOL Ros_Controller_IsServoOn()
{
    return ((g_Ros_Controller.ioStatus[IO_ROBOTSTATUS_SERVO] != 0)
        && !Ros_Controller_IsEcoMode());
}

BOOL Ros_Controller_IsEcoMode()
{
    return (g_Ros_Controller.ioStatus[IO_ROBOTSTATUS_INECOMODE] != 0);
}

BOOL Ros_Controller_IsEStop()
{
    return ((g_Ros_Controller.ioStatus[IO_ROBOTSTATUS_ESTOP_EX]==0)
        || (g_Ros_Controller.ioStatus[IO_ROBOTSTATUS_ESTOP_PP]==0)
        || (g_Ros_Controller.ioStatus[IO_ROBOTSTATUS_ESTOP_CTRL]==0) );
}

BOOL Ros_Controller_IsWaitingRos()
{
    return ((g_Ros_Controller.ioStatus[IO_ROBOTSTATUS_WAITING_ROS]!=0));
}

BOOL Ros_Controller_IsContinuousCycle()
{
    return ((g_Ros_Controller.ioStatus[IO_ROBOTSTATUS_CONT_CYC_MODE] != 0));
}

BOOL Ros_Controller_IsMotionReady()
{
    BOOL bMotionReady;

#ifndef DUMMY_SERVO_MODE
    bMotionReady = Ros_Controller_GetNotReadySubcode(false) == MOTION_READY;
#else
    bMotionReady = Ros_Controller_IsOperating();
#endif

    return bMotionReady;
}


BOOL Ros_Controller_IsPflActive()
{
#if (YRC1000||YRC1000u)
    if (g_Ros_Controller.bPFLEnabled) {
        if (g_Ros_Controller.bPFLduringRosMove || g_Ros_Controller.ioStatus[IO_ROBOTSTATUS_PFL_STOP] || g_Ros_Controller.ioStatus[IO_ROBOTSTATUS_PFL_ESCAPE] ||
            (g_Ros_Controller.ioStatus[IO_ROBOTSTATUS_PFL_AVOIDING]
                && (g_Ros_Controller.ioStatus[IO_ROBOTSTATUS_PFL_AVOID_JOINT] || g_Ros_Controller.ioStatus[IO_ROBOTSTATUS_PFL_AVOID_TRANS])) != 0)
        {
            return TRUE;
        }
    }
#endif
    return FALSE;
}

BOOL Ros_Controller_IsMpIncMoveErrorActive()
{
    return g_Ros_Controller.bMpIncMoveError == TRUE;
}

BOOL Ros_Controller_IsAnyFaultActive()
{
    //we report the controller as being in an error state if there are either
    //active regular alarms or errors, OR an internal MotoROS2-error is active
    return (Ros_Controller_IsAlarm()
        || Ros_Controller_IsError()
        || Ros_Controller_IsMpIncMoveErrorActive()
        || Ros_Controller_IsPflActive());
}


MotionNotReadyCode Ros_Controller_GetNotReadySubcode(bool ignoreTractableProblems)
{
    // Check e-stop
    if(Ros_Controller_IsEStop())
        return MOTION_NOT_READY_ESTOP;

    // Check play mode
    if(!Ros_Controller_IsPlay())
        return MOTION_NOT_READY_NOT_PLAY;

#ifndef DUMMY_SERVO_MODE
    // Check remote
    if(!Ros_Controller_IsRemote())
        return MOTION_NOT_READY_NOT_REMOTE;
#endif

    // Check hold
    if(Ros_Controller_IsHold())
        return MOTION_NOT_READY_HOLD;

    // Check alarm
    if (Ros_Controller_IsAlarm())
        return MOTION_NOT_READY_ALARM;

    // Check PFL active
    if (Ros_Controller_IsPflActive())
        return MOTION_NOT_READY_PFL_ACTIVE;

    // Check if Incremental Motion was rejected
    if (Ros_Controller_IsMpIncMoveErrorActive())
        return MOTION_NOT_READY_INC_MOVE_ERROR;

    // Check error
    if (Ros_Controller_IsError())
        return MOTION_NOT_READY_ERROR;

    if (Ros_Controller_IsOperating() && !Ros_Controller_MasterTaskIsJobName(g_nodeConfigSettings.inform_job_name))
        return MOTION_NOT_READY_OTHER_PROGRAM_RUNNING;

    if (!ignoreTractableProblems)
    {

        // Check if in continuous cycle mode (Here due to being checked before starting servo power)
        if (!Ros_Controller_IsContinuousCycle())
            return MOTION_NOT_READY_NOT_CONT_CYCLE_MODE;
#ifndef DUMMY_SERVO_MODE

        // Check servo power
        if (!Ros_Controller_IsServoOn())
            return MOTION_NOT_READY_SERVO_OFF;
#endif

        // Check operating
        if (!Ros_Controller_IsOperating())
            return MOTION_NOT_READY_NOT_STARTED;

        // Check ready I/O signal (should confirm wait)
        if (!Ros_Controller_IsWaitingRos())
            return MOTION_NOT_READY_WAITING_ROS;
    }

    return MOTION_READY;
}

void Ros_Controller_Reset_PflDuringRosMove()
{
    g_Ros_Controller.bPFLduringRosMove = FALSE;
}

void Ros_Controller_Reset_MpIncMoveError()
{
    g_Ros_Controller.bMpIncMoveError = FALSE;
}

BOOL Ros_Controller_MasterTaskIsJobName(const char* const jobName)
{
    MP_TASK_SEND_DATA taskSendData;
    MP_CUR_JOB_RSP_DATA curJobResponseData;

    taskSendData.sTaskNo = 0; //master task

    mpGetCurJob(&taskSendData, &curJobResponseData);
    return strncmp(curJobResponseData.cJobName, jobName, MAX_JOB_NAME_LEN) == 0;
}

BOOL Ros_Controller_IsInMotion()
{
    int i;
    int groupNo;
    long fbPulsePos[MAX_PULSE_AXES];
    long cmdPulsePos[MAX_PULSE_AXES];
    BOOL bDataInQ;
    CtrlGroup* ctrlGroup;

    bDataInQ = Ros_MotionControl_HasDataInQueue();

    if (bDataInQ == TRUE)
        return TRUE;
    else if (bDataInQ == ERROR)
        return ERROR;
    else
    {
        //for each control group
        for (groupNo = 0; groupNo < g_Ros_Controller.numGroup; groupNo++)
        {
            //Check group number valid
            if (!Ros_Controller_IsValidGroupNo(groupNo))
                continue;

            //Check if the feeback position has caught up to the command position
            ctrlGroup = g_Ros_Controller.ctrlGroups[groupNo];

            Ros_CtrlGroup_GetFBPulsePos(ctrlGroup, fbPulsePos);
            Ros_CtrlGroup_GetPulsePosCmd(ctrlGroup, cmdPulsePos);

            for (i = 0; i < MP_GRP_AXES_NUM; i += 1)
            {
                if (!Ros_CtrlGroup_IsInvalidAxis(ctrlGroup, i))
                {
                    // Check if position matches current command position
                    if (abs(fbPulsePos[i] - cmdPulsePos[i]) > START_MAX_PULSE_DEVIATION)
                        return TRUE;
                }
            }
        }
    }

    return FALSE;
}

//-------------------------------------------------------------------
// Get I/O state on the controller
//-------------------------------------------------------------------
BOOL Ros_Controller_StatusRead(USHORT ioStatus[IO_ROBOTSTATUS_MAX])
{
    return (mpReadIO(g_Ros_Controller.ioStatusAddr, ioStatus, IO_ROBOTSTATUS_MAX) == 0);
}

//-------------------------------------------------------------------
// Update I/O state on the controller
//-------------------------------------------------------------------
BOOL Ros_Controller_IoStatusUpdate()
{
    USHORT ioStatus[IO_ROBOTSTATUS_MAX];
    USHORT active_alarms[MAX_ALARM_COUNT + 1] = { 0 };
    int i;
    BOOL prevReadyStatus;
    INT64 theTime;
    rcl_ret_t ret;

    prevReadyStatus = Ros_Controller_IsMotionReady();

    //Timestamp
    theTime = rmw_uros_epoch_nanos();

    if(Ros_Controller_StatusRead(ioStatus))
    {
        // Check for change of state and potentially react to the change
        for(i=0; i<IO_ROBOTSTATUS_MAX; i++)
        {
            if(g_Ros_Controller.ioStatus[i] != ioStatus[i])
            {
                //Ros_Debug_BroadcastMsg("Change of ioStatus[%d]", i);

                g_Ros_Controller.ioStatus[i] = ioStatus[i];
                switch(i)
                {
                    case IO_ROBOTSTATUS_ALARM_MAJOR: // alarm
                    case IO_ROBOTSTATUS_ALARM_MINOR: // alarm
                    case IO_ROBOTSTATUS_ALARM_SYSTEM: // alarm
                    case IO_ROBOTSTATUS_ALARM_USER: // alarm
                    {
                        if ((ioStatus[IO_ROBOTSTATUS_ALARM_MAJOR] == 0) &&
                            (ioStatus[IO_ROBOTSTATUS_ALARM_MINOR] == 0) &&
                            (ioStatus[IO_ROBOTSTATUS_ALARM_SYSTEM] == 0) &&
                            (ioStatus[IO_ROBOTSTATUS_ALARM_USER] == 0))
                        {
                            g_Ros_Controller.alarmCode = 0;
                            
                            if (g_Ros_Controller.bPrevAlarmState) //manual RESET operation on pendant
                            {
                                //Reset internal error flags
                                //These could also get reset using the ResetError ROS service
                                Ros_Controller_Reset_PflDuringRosMove();
                                Ros_Controller_Reset_MpIncMoveError();
                                g_Ros_Controller.bPrevAlarmState = FALSE;
                            }
                        }
                        else
                        {
                            g_Ros_Controller.alarmCode = Ros_Controller_GetAlarmCode();
                            Ros_MotionControl_ClearQ_All();

                            g_Ros_Controller.bPrevAlarmState = TRUE;
                        }

                        break;
                    }

                    case IO_ROBOTSTATUS_WAITING_ROS: // Job input signaling ready for external motion
                    {
                        if(ioStatus[IO_ROBOTSTATUS_WAITING_ROS] == 0)  // signal turned OFF
                        {
                            // Job execution stopped take action
                            Ros_MotionControl_ClearQ_All();
                        }
                        break;
                    }
#if (YRC1000||YRC1000u)
                    case IO_ROBOTSTATUS_PFL_STOP: // PFL Stop
                    case IO_ROBOTSTATUS_PFL_ESCAPE: //  PFL Escaping
                    case IO_ROBOTSTATUS_PFL_AVOIDING: // PFL Avoidance
                    {
                        if (g_Ros_Controller.bPFLEnabled && Ros_Controller_IsWaitingRos() && Ros_Controller_IsPflActive())
                        {
                            // Job execution stopped by PFL take action
                            g_Ros_Controller.bPFLduringRosMove = TRUE; //force job to be restarted with new ROS_CMD_START_TRAJ_MODE command
                            Ros_MotionControl_ClearQ_All();
                        }
                        break;
                    }
#endif
                }
            }
        }

        if (!prevReadyStatus && Ros_Controller_IsMotionReady())
            Ros_Debug_BroadcastMsg("Robot job is ready for ROS commands.");

        Ros_Nanos_To_Time_Msg(theTime, &g_messages_RobotStatus.msgRobotStatus->header.stamp);

        g_messages_RobotStatus.msgRobotStatus->drives_powered.val = (Ros_Controller_IsServoOn() ? industrial_msgs__msg__TriState__ON : industrial_msgs__msg__TriState__OFF);
        g_messages_RobotStatus.msgRobotStatus->e_stopped.val = (Ros_Controller_IsEStop() ? industrial_msgs__msg__TriState__CLOSED : industrial_msgs__msg__TriState__OPEN);
        g_messages_RobotStatus.msgRobotStatus->in_motion.val = (Ros_Controller_IsInMotion() ? industrial_msgs__msg__TriState__TRUE : industrial_msgs__msg__TriState__FALSE);
        g_messages_RobotStatus.msgRobotStatus->mode.val = (Ros_Controller_IsPlay() ? industrial_msgs__msg__RobotMode__AUTO : industrial_msgs__msg__RobotMode__MANUAL);
        g_messages_RobotStatus.msgRobotStatus->motion_possible.val = (Ros_Controller_IsMotionReady() ? industrial_msgs__msg__TriState__TRUE : industrial_msgs__msg__TriState__FALSE);

        //we report the controller as being in an error state if there are either
        //active regular alarms or errors, OR an internal MotoROS2-error is active
        BOOL in_error = Ros_Controller_IsAnyFaultActive();
        g_messages_RobotStatus.msgRobotStatus->in_error.val = in_error ? industrial_msgs__msg__TriState__TRUE : industrial_msgs__msg__TriState__FALSE;

        // assume there are no active errors
        g_messages_RobotStatus.msgRobotStatus->error_codes.size = 0;
        if (!Ros_Controller_IsMotionReady())
        {
            int num_alarms = Ros_Controller_GetActiveAlarmCodes(active_alarms);
            if (num_alarms < 0)
            {
                Ros_Debug_BroadcastMsg("Error retrieving active alarms: %d", num_alarms);
            }
            else
            {
                g_messages_RobotStatus.msgRobotStatus->error_codes.size = num_alarms;
                // 'msgRobotStatus->error_codes' has been initialised to be of
                // length 'MAX_ALARM_COUNT + 1' in Ros_Controller_Initialize()
                for (int alm = 0; alm < num_alarms; ++alm)
                    g_messages_RobotStatus.msgRobotStatus->error_codes.data[alm] = active_alarms[alm];
            }
        }

        //publish status topic
        ret = rcl_publish(&g_publishers_RobotStatus.robotStatus, g_messages_RobotStatus.msgRobotStatus, NULL);
        // publishing can fail, but we choose to ignore those errors in this implementation
        RCL_UNUSED(ret);

        return TRUE;
    }
    else
        return FALSE;
}



/**** Wrappers on MP standard function ****/

//-------------------------------------------------------------------
// Get I/O state on the controller
//-------------------------------------------------------------------
BOOL Ros_Controller_GetIOState(ULONG signal)
{
    MP_IO_INFO ioInfo;
    USHORT ioState;
    int ret;

    //set feedback signal
    ioInfo.ulAddr = signal;
    ret = mpReadIO(&ioInfo, &ioState, 1);
    if(ret != 0)
        Ros_Debug_BroadcastMsg("mpReadIO failure (%d)", ret);

    return (ioState != 0);
}


//-------------------------------------------------------------------
// Set I/O state on the controller
//-------------------------------------------------------------------
void Ros_Controller_SetIOState(ULONG signal, BOOL status)
{
    MP_IO_DATA ioData;
    int ret;

    //set feedback signal
    ioData.ulAddr = signal;
    ioData.ulValue = status;
    ret = mpWriteIO(&ioData, 1);
    if(ret != 0)
        Ros_Debug_BroadcastMsg("mpWriteIO failure (%d)", ret);
}


//-------------------------------------------------------------------
// Get the code of the first alarm on the controller
//-------------------------------------------------------------------
int Ros_Controller_GetAlarmCode()
{
    MP_ALARM_CODE_RSP_DATA alarmData;
    bzero(&alarmData, sizeof(alarmData));
    if(mpGetAlarmCode(&alarmData) == 0)
    {
        if(alarmData.usAlarmNum > 0)
            return(alarmData.AlarmData.usAlarmNo[0]);
        else if (alarmData.usErrorNo > 0)
            return(alarmData.usErrorNo);
        else
            return 0;
    }
    return -1;
}

int Ros_Controller_GetActiveAlarmCodes(USHORT active_alarms[MAX_ALARM_COUNT + MAX_ERROR_COUNT])
{
    MP_ALARM_CODE_RSP_DATA alarmData;
    bzero(&alarmData, sizeof(alarmData));

    // can't continue if M+ API fails
    if(mpGetAlarmCode(&alarmData) != 0)
        return -1;

    // if the output array is too small, we can't do anything either. This
    // should not happen, but best check for it.
    if (alarmData.usAlarmNum + MAX_ERROR_COUNT > MAX_ALARM_COUNT + MAX_ERROR_COUNT)
        return -2;

    // add all alarms to the output array
    memcpy(active_alarms, alarmData.AlarmData.usAlarmNo, alarmData.usAlarmNum * sizeof(USHORT));
    int num_entries = alarmData.usAlarmNum;

    // if there is an error, also add it to the list
    if (alarmData.usErrorNo > 0)
    {
        active_alarms[num_entries++] = alarmData.usErrorNo;
    }

    return num_entries;
}

/**
 * Attempt to load on-controller extrinsic kinematic calibration data to improve
 * accuracy of TF broadcasts (for frames for which such calibration data exists).
 *
 * NOTE: returns TRUE IFF at least one calibration file loaded successfully,
 *       FALSE in all other cases.
*/
static BOOL Ros_Controller_LoadGroupCalibrationData(Controller* const controller)
{
    BOOL bOneCalibFileLoaded = FALSE;

    for (int i = 0; i < MAX_ROBOT_CALIBRATION_FILES; i += 1)
    {
        MP_RB_CALIB_DATA calibData;
        if (Ros_mpGetRobotCalibrationData(i, &calibData) == OK)
        {
            Ros_Debug_BroadcastMsg("%s: file %d loaded OK", __func__, i);

            //Take note at least one calibration file loaded successfully.
            //
            //NOTE: successful loading of a single calibration file will of
            //course not mean all groups (that would benefit from it) are actually
            //calibrated correctly. But this is the best we can do, as there is
            //no requirement to calibrate every group to every other group, nor
            //will this prevent MotoROS2 from successfully controlling motion.
            //
            //TF frame broadcasts for uncalibrated groups will however be incorrect,
            //which is why posting an alarm in case there are no calibration files
            //at all on a multigroup system is considered a good thing to do.
            bOneCalibFileLoaded = TRUE;

            if (calibData.s_rb.grp_no <= MP_R8_GID && //the slave is a robot
                calibData.m_rb.grp_no <= MP_R8_GID) //the master is another robot's RF
            {
                int groupIndex = mpCtrlGrpId2GrpNo((MP_GRP_ID_TYPE)calibData.s_rb.grp_no);
                MP_COORD* coord = &controller->ctrlGroups[groupIndex]->robotCalibrationToBaseFrame;
                coord->x = calibData.pos_uow[0];
                coord->y = calibData.pos_uow[1];
                coord->z = calibData.pos_uow[2];
                coord->rx = calibData.ang_uow[0];
                coord->ry = calibData.ang_uow[1];
                coord->rz = calibData.ang_uow[2];
            }
        }
    }

    return bOneCalibFileLoaded;
}

//TODO(gavanderhoorn): should be able to make this 'static' once Yaskawa-Global/motoros2#173
//is implemented and we can unit test static functions
BOOL Ros_Controller_ShouldWarnNoCalibDataLoaded(Controller const* controller, BOOL bCalibLoadedOk, BOOL bPublishTfEnabled)
{
    size_t numRobotsOrStations = 0;
    for(size_t i=0; i < controller->numGroup; ++i)
        if (Ros_CtrlGroup_IsRobot(controller->ctrlGroups[i]) || Ros_CtrlGroup_IsStation(controller->ctrlGroups[i]))
            numRobotsOrStations += 1;

    //It's OK if calibration did not load (for single-group systems fi, or systems
    //that haven't been calibrated), but it might be prudent to post an alarm just
    //in case IFF:
    //
    // - this is a multi-group system
    // - there is at least one robot
    // - there are groups other than base groups (so R1+R2, or R1+S1)
    // - TF broadcast is enabled, and
    // - none of the calibration files loaded
    //
    //in that case there could be Robot groups that have not been calibrated (to
    //either other robots and/or Station groups) and this would make their TF
    //broadcasts incorrect.
    return (
        //only groups other than base groups need calibration
        numRobotsOrStations > 1 &&
        //and warn only if TF is enabled AND calib failed to load
        TRUE  == bPublishTfEnabled &&
        FALSE == bCalibLoadedOk);
}
