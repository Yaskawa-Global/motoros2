//MotionControl.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

/// <summary>
/// For each point in an incoming trajectory, process the data for a SINGLE JOINT.
/// The time, pos, and vel are copied from the trajectory structure into the internal
/// buffer for the specific control group object.
/// </summary>
/// <param name="in_jointTrajData">Pointer to the head of the incoming trajectory</param>
/// <param name="incomingAxisIndex">Index of the joint in the in_jointTrajData structure</param>
/// <param name="ctrlGroup">CtrlGroup object where this joint belongs</param>
/// <param name="ctrlGroupAxisIndex">Index of the joint in the CtrlGroup object</param>
/// <param name="out_jointMotionData">Pointer to the head of the array containing the data which will be later broken into increments</param>
/// <returns></returns>
Init_Trajectory_Status Ros_MotionControl_ConvertTrajectoryToJointMotionData(trajectory_msgs__msg__JointTrajectoryPoint__Sequence* in_jointTrajData, 
    int incomingAxisIndex, CtrlGroup* ctrlGroup, int ctrlGroupAxisIndex, JointMotionData* out_jointMotionData);

BOOL Ros_MotionControl_AllGroupsInitComplete = FALSE;

MOTION_MODE Ros_MotionControl_ActiveMotionMode = MOTION_MODE_INACTIVE;

BOOL Ros_MotionControl_MustInitializePointQueue = TRUE; //first point of streaming trajectory must match current-position

Init_Trajectory_Status Ros_MotionControl_Init(rosidl_runtime_c__String__Sequence* sequenceGoalJointNames, trajectory_msgs__msg__JointTrajectoryPoint__Sequence* sequenceOfPoints)
{
    long pulsePos[MAX_PULSE_AXES];
    long curPos[MAX_PULSE_AXES];
    int grpIndex, jointIndexInTraj, pointIndex, checkForDupIndex;

    //Verify we're not already running a trajectory
    for (grpIndex = 0; grpIndex < g_Ros_Controller.numGroup; grpIndex += 1)
    {
        if (g_Ros_Controller.ctrlGroups[grpIndex]->hasDataToProcess)
        {
            Ros_Debug_BroadcastMsg("Already processing trajectory data - Rejecting new trajectory (Group #%d)",
                g_Ros_Controller.ctrlGroups[grpIndex]->groupNo);
            return INIT_TRAJ_ALREADY_IN_MOTION;
        }
    }

    Ros_MotionControl_AllGroupsInitComplete = FALSE;

    //Init internal storage for each group
    for (grpIndex = 0; grpIndex < g_Ros_Controller.numGroup; grpIndex += 1)
    {
        g_Ros_Controller.ctrlGroups[grpIndex]->trajectoryIterator = NULL;
        bzero(g_Ros_Controller.ctrlGroups[grpIndex]->trajectoryToProcess, sizeof(JointMotionData) * MAX_NUMBER_OF_POINTS_PER_TRAJECTORY);
    }

    //------------------------------------------------------------
    //The trajectory contains information for all groups. Determine which groups are used by looking at the 'joint names'.
    BOOL bGroupIsUsed[MAX_CONTROLLABLE_GROUPS];
    bzero(bGroupIsUsed, sizeof(bGroupIsUsed));
    
    if (g_Ros_Controller.totalAxesCount != sequenceGoalJointNames->size)
    {
        Ros_Debug_BroadcastMsg("Trajectory must contain data for all %d joints.", g_Ros_Controller.totalAxesCount);
        return INIT_TRAJ_INCOMPLETE_JOINTLIST;
    }

    //for each point in the trajectory
    for (pointIndex = 0; pointIndex < sequenceOfPoints->size; pointIndex += 1)
    {
        //verify that we have positions for each axis
        if (sequenceOfPoints->data[pointIndex].positions.size != g_Ros_Controller.totalAxesCount)
        {
            Ros_Debug_BroadcastMsg("Each point in the trajectory must have positions for all axes (pt: %d).", pointIndex);
            return INIT_TRAJ_WRONG_NUMBER_OF_POSITIONS;
        }

        //verify that we have velocities for each axis
        if (sequenceOfPoints->data[pointIndex].velocities.size != g_Ros_Controller.totalAxesCount)
        {
            Ros_Debug_BroadcastMsg("Each point in the trajectory must have velocities for all axes (pt: %d).", pointIndex);
            return INIT_TRAJ_WRONG_NUMBER_OF_VELOCITIES;
        }
    }

    //===================================
    //When a goal is received, the all of the points in the trajectory are processed, one joint at a time.
    //For each of those joints, this iterates over all of the CtrlGroup objects and compares the joint names.
    //This allows it to find the correct CtrlGroup object and the joint index (in moto order) in the JointMotionData array.
    //===================================

    //for each joint/axis in a single trajectory point
    for (jointIndexInTraj = 0; jointIndexInTraj < sequenceGoalJointNames->size; jointIndexInTraj += 1)
    {
        int  jointIndexInCtrlGroup;
        CtrlGroup* ctrlGroup;

        //check to ensure there are no duplicate joint names in the list
        for (checkForDupIndex = (jointIndexInTraj + 1); checkForDupIndex < sequenceGoalJointNames->size; checkForDupIndex += 1)
        {
            if (strncmp(sequenceGoalJointNames->data[jointIndexInTraj].data,
                sequenceGoalJointNames->data[checkForDupIndex].data,
                MAX_JOINT_NAME_LENGTH) == 0)
            {
                Ros_Debug_BroadcastMsg("Joint name [%s] is used for multiple joints in the trajectory (indices: %d and %d).", sequenceGoalJointNames->data[jointIndexInTraj].data, jointIndexInTraj, checkForDupIndex);
                return INIT_TRAJ_DUPLICATE_JOINT_NAME;
            }
        }

        //find the ctrlgroup for this joint
        BOOL bFound = FALSE;
        for (grpIndex = 0; grpIndex < MAX_CONTROLLABLE_GROUPS; grpIndex += 1)
        {
            ctrlGroup = g_Ros_Controller.ctrlGroups[grpIndex];
            if (!ctrlGroup)
                continue;

            for (jointIndexInCtrlGroup = 0; jointIndexInCtrlGroup < MP_GRP_AXES_NUM; jointIndexInCtrlGroup += 1)
            {
                char* jointName = ctrlGroup->jointNames_userDefined[jointIndexInCtrlGroup];
                if (strlen(jointName) != 0)
                {
                    if (strcmp(jointName, sequenceGoalJointNames->data[jointIndexInTraj].data) == 0)
                    {
                        bFound = TRUE;
                        break;
                    }
                }
            }

            if (bFound)
                break;
        }

        if (!bFound)
        {
            Ros_Debug_BroadcastMsg("Joint name [%s] is not valid. Check motoros2_config.yaml and update accordingly.", sequenceGoalJointNames->data[jointIndexInTraj].data);
            Ros_Debug_BroadcastMsg("Valid names:");
            for (int groupIndex = 0; groupIndex < MAX_CONTROLLABLE_GROUPS; groupIndex += 1)
            {
                for (int jointIndex = 0; jointIndex < MP_GRP_AXES_NUM; jointIndex += 1)
                {
                    char* configListEntry = g_nodeConfigSettings.joint_names[(groupIndex * MP_GRP_AXES_NUM) + jointIndex];
                    if (strlen(configListEntry) != 0)
                        Ros_Debug_BroadcastMsg(" - %s", configListEntry);
                }
            }

            return INIT_TRAJ_INVALID_JOINTNAME;
        }

        //this processes all points in the trajectory array FOR A SINGLE AXIS at a time
        Init_Trajectory_Status convertStatus = Ros_MotionControl_ConvertTrajectoryToJointMotionData(sequenceOfPoints, jointIndexInTraj, ctrlGroup, jointIndexInCtrlGroup, ctrlGroup->trajectoryToProcess);
        if (convertStatus != INIT_TRAJ_OK)
            return convertStatus;

        bGroupIsUsed[grpIndex] = TRUE;
    } //for each joint in a single trajectory point

    for (grpIndex = 0; grpIndex < g_Ros_Controller.numGroup; grpIndex += 1)
    {
        if (!bGroupIsUsed[grpIndex])
            continue;

        CtrlGroup* ctrlGroup = g_Ros_Controller.ctrlGroups[grpIndex];
        Ros_Debug_BroadcastMsg("Initializing trajectory for group #%d", ctrlGroup->groupNo);

        //---------------
        // For MPL80/100 robot type (SLU-BT): Controller automatically moves the B-axis
        // to maintain orientation as other axes are moved.
        if (ctrlGroup->bIsBaxisSlave)
        {
            for (int i = 0; i < sequenceOfPoints->size; i += 1)
            {
                //This is radians in MOTO joint order
                ctrlGroup->trajectoryToProcess[i].pos[4] += -ctrlGroup->trajectoryToProcess[i].pos[1] + ctrlGroup->trajectoryToProcess[i].pos[2];
                ctrlGroup->trajectoryToProcess[i].vel[4] += -ctrlGroup->trajectoryToProcess[i].vel[1] + ctrlGroup->trajectoryToProcess[i].vel[2];
            }
        }

        ctrlGroup->prevTrajectoryIterator = ctrlGroup->trajectoryToProcess; //reset iterator

        // Assign start position
        ctrlGroup->timeLeftover_ms = 0;
        ctrlGroup->q_time = ctrlGroup->prevTrajectoryIterator->time;

        //Convert start position to pulse format
        // ctrlGroup->prevTrajectoryIterator->pos is already in moto joint order
        Ros_CtrlGroup_ConvertRosUnitsToMotoUnits(ctrlGroup, ctrlGroup->prevTrajectoryIterator->pos, pulsePos);
        Ros_CtrlGroup_GetPulsePosCmd(ctrlGroup, curPos);

        // Initialize prevPulsePos to the current position
        Ros_CtrlGroup_GetPulsePosCmd(ctrlGroup, ctrlGroup->prevPulsePos);

        // Check for each axis
        for (int i = 0; i < MAX_PULSE_AXES; i++)
        {
            // Check if position matches current command position
            if (abs(pulsePos[i] - curPos[i]) > START_MAX_PULSE_DEVIATION)
            {
                Ros_Debug_BroadcastMsg("ERROR: Trajectory start position doesn't match current position (MOTO joint order).");
                Ros_Debug_BroadcastMsg(" - Requested start: %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld",
                    pulsePos[0], pulsePos[1], pulsePos[2],
                    pulsePos[3], pulsePos[4], pulsePos[5],
                    pulsePos[6], pulsePos[7]);
                Ros_Debug_BroadcastMsg(" - Current pos: %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld",
                    curPos[0], curPos[1], curPos[2],
                    curPos[3], curPos[4], curPos[5],
                    curPos[6], curPos[7]);
                Ros_Debug_BroadcastMsg(" - ctrlGroup->prevPulsePos: %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld",
                    ctrlGroup->prevPulsePos[0], ctrlGroup->prevPulsePos[1], ctrlGroup->prevPulsePos[2],
                    ctrlGroup->prevPulsePos[3], ctrlGroup->prevPulsePos[4], ctrlGroup->prevPulsePos[5],
                    ctrlGroup->prevPulsePos[6], ctrlGroup->prevPulsePos[7]);

                for (grpIndex = 0; grpIndex < g_Ros_Controller.numGroup; grpIndex += 1)
                {
                    g_Ros_Controller.ctrlGroups[grpIndex]->hasDataToProcess = FALSE;
                    g_Ros_Controller.ctrlGroups[grpIndex]->trajectoryIterator = NULL;
                    bzero(g_Ros_Controller.ctrlGroups[grpIndex]->trajectoryToProcess, sizeof(JointMotionData) * MAX_NUMBER_OF_POINTS_PER_TRAJECTORY);
                }

                return INIT_TRAJ_INVALID_STARTING_POS;
            }

            // Check maximum velocity limit
            if (abs(ctrlGroup->prevTrajectoryIterator->vel[i]) > ctrlGroup->maxSpeed[i])
            {
                Ros_Debug_BroadcastMsg("ERROR: Command of (%.4f) exceeds the speed limit of (%.4f) for axis %d", ctrlGroup->prevTrajectoryIterator->vel[i], ctrlGroup->maxSpeed[i], i);

                for (grpIndex = 0; grpIndex < g_Ros_Controller.numGroup; grpIndex += 1)
                {
                    g_Ros_Controller.ctrlGroups[grpIndex]->hasDataToProcess = FALSE;
                    g_Ros_Controller.ctrlGroups[grpIndex]->trajectoryIterator = NULL;
                    bzero(g_Ros_Controller.ctrlGroups[grpIndex]->trajectoryToProcess, sizeof(JointMotionData) * MAX_NUMBER_OF_POINTS_PER_TRAJECTORY);
                }

                // excessive speed
                return INIT_TRAJ_INVALID_VELOCITY;
            }
        }

        //Although there are additional groups to process, we'll set this flag to indicate that the point is ready for processing.
        //The Ros_MotionControl_AllGroupsInitComplete flag has not been set yet. That will prevent the AddToIncQueueProcess loop
        //from processing increments before all groups are synchronized.
        for (int i = 0; i < sequenceOfPoints->size; i += 1)
        {
            ctrlGroup->trajectoryToProcess[i].valid = TRUE;
        }

        ctrlGroup->trajectoryIterator = &ctrlGroup->trajectoryToProcess[1];
        ctrlGroup->hasDataToProcess = TRUE;
        Ros_Debug_BroadcastMsg("Group #%d - Trajectory is ready for processing", ctrlGroup->groupNo);

    } //for each group in the controller

    Ros_MotionControl_AllGroupsInitComplete = TRUE;

    return INIT_TRAJ_OK;
}

//-----------------------------------------------------------------------
// Setup the first point of a trajectory
//-----------------------------------------------------------------------
Init_Trajectory_Status Ros_MotionControl_InitTrajectory(control_msgs__action__FollowJointTrajectory_SendGoal_Request* pending_ros_goal_request)
{
    if (pending_ros_goal_request == NULL || pending_ros_goal_request->goal.trajectory.points.size < MIN_NUMBER_OF_POINTS_PER_TRAJECTORY)
        return INIT_TRAJ_TOO_SMALL;

    return Ros_MotionControl_Init(&pending_ros_goal_request->goal.trajectory.joint_names, &pending_ros_goal_request->goal.trajectory.points);
}

Init_Trajectory_Status Ros_MotionControl_InitPointQueue(motoros2_interfaces__srv__QueueTrajPoint_Request* request)
{
    Init_Trajectory_Status status;

    // for point queuing, we create a single-point trajectory, store the incoming
    // point in it and send it off for processing by the trajectory processing
    // pipeline.
    trajectory_msgs__msg__JointTrajectoryPoint__Sequence pointSequence;

    pointSequence.capacity = 1;
    pointSequence.size = 1;
    pointSequence.data = &request->point; //no additional memory is allocated this way

    status = Ros_MotionControl_Init(&request->joint_names, &pointSequence);

    if (status == INIT_TRAJ_OK)
        Ros_MotionControl_MustInitializePointQueue = FALSE;

    return status;
}

Init_Trajectory_Status Ros_MotionControl_ConvertTrajectoryToJointMotionData(trajectory_msgs__msg__JointTrajectoryPoint__Sequence* in_jointTrajData, 
    int incomingAxisIndex, CtrlGroup* ctrlGroup, int ctrlGroupAxisIndex, JointMotionData* out_jointMotionData)
{
    for (int i = 0; i < in_jointTrajData->size; i += 1) //for each point in trajectory
    {
        INT64 millis = Ros_Duration_Msg_To_Millis(&in_jointTrajData->data[i].time_from_start);
        if (millis < 0)
        {
            Ros_Debug_BroadcastMsg("The trajectory [time_from_start] may not be negative (pt: %d).", i);
            return INIT_TRAJ_INVALID_TIME;
        }
        if (millis == 0 && i != 0) //a time of 0 will cause the accel calculations to fail
        {
            Ros_Debug_BroadcastMsg("The trajectory [time_from_start] may only be '0' for the first point in a trajectory (pt: %d).", i);
            return INIT_TRAJ_INVALID_TIME;
        }
        out_jointMotionData[i].time = millis;

        if (i != 0)
        {
            //ensure that the time is greater than the previous point.
            //Check this using out_jointMotionData as time has already been converted
            //to a single scalar per point which is easier to compare against than
            //the msg__Duration struct in in_jointTrajData.
            if (out_jointMotionData[i].time < out_jointMotionData[i - 1].time)
            {
                Ros_Debug_BroadcastMsg("Each point in the trajectory must have a [time_from_start] greater than the previous (pt: %d).", i);
                return INIT_TRAJ_BACKWARD_TIME;
            }
        }

        //Last point in the trajectory. This only applies when receiving an entire trajectory through the FJT action.
        if (Ros_MotionControl_IsMotionMode_Trajectory() && i == (in_jointTrajData->size - 1))
        {
            //verify that the robot is commanded to stop at the end of the trajectory
            if (fabs(in_jointTrajData->data[i].velocities.data[incomingAxisIndex]) > EPSILON_TOLERANCE_DOUBLE) // float version of "!=0"
            {
                Ros_Debug_BroadcastMsg("The final point in a trajectory must specify a target velocity of '0'.");
                return INIT_TRAJ_INVALID_ENDING_VELOCITY;
            }

            //Acceleration is not used. But we want to ensure the trajectory is well behaved and well shaped.
            //The JointTrajectoryController from ros(2)_control can sometimes behave rather strangely when the
            //last point doesn't have zero vel/acc (probably caused by the spline interpolation doing weird 
            //things with non - zero values for velocityand acceleration).
            // ------------------------------------------
            //UPDATE (2023/05/23): It seems that MoveIt doesn't follow this practice. All trajectories from MoveIt have a
            // non-zero acceleration at the end of the trajectory. So we'll remove this check for now.
            // This may be restored in a future update.
            // 
            //if ((in_jointTrajData->data[i].accelerations.size > incomingAxisIndex) &&
            //    fabs(in_jointTrajData->data[i].accelerations.data[incomingAxisIndex]) > EPSILON_TOLERANCE_DOUBLE) // float version of "!=0"
            //{
            //    Ros_Debug_BroadcastMsg("The final point in a trajectory must specify a target acceleration of '0'.");
            //    return INIT_TRAJ_INVALID_ENDING_ACCELERATION;
            //}
        }

        out_jointMotionData[i].pos[ctrlGroupAxisIndex] = in_jointTrajData->data[i].positions.data[incomingAxisIndex];
        out_jointMotionData[i].vel[ctrlGroupAxisIndex] = in_jointTrajData->data[i].velocities.data[incomingAxisIndex];
    }

    return INIT_TRAJ_OK;
}

//-----------------------------------------------------------------------
// Task that handles in the background messages that may have long processing
// time so that they don't block other message from being processed.
//-----------------------------------------------------------------------
void Ros_MotionControl_AddToIncQueueProcess(CtrlGroup* ctrlGroup)
{
    int i;

    while (TRUE)
    {
        if (Ros_MotionControl_AllGroupsInitComplete)
        {
            // if there is no message to process, delay and try again
            if (ctrlGroup->hasDataToProcess && ctrlGroup->trajectoryIterator != NULL && ctrlGroup->trajectoryIterator->valid)
            {
                if (g_Ros_Controller.bStopMotion)
                {
                    bzero(ctrlGroup->trajectoryToProcess, sizeof(ctrlGroup->trajectoryToProcess));
                    ctrlGroup->hasDataToProcess = FALSE;
                    continue;
                }

                Ros_Debug_BroadcastMsg("Processing next point in trajectory [Group #%d - T=%.3f: (%7.4f, %7.4f, %7.4f, %7.4f, %7.4f, %7.4f)]",
                    ctrlGroup->groupNo, (double)ctrlGroup->trajectoryIterator->time * 0.001,
                    ctrlGroup->trajectoryIterator->pos[0], ctrlGroup->trajectoryIterator->pos[1], ctrlGroup->trajectoryIterator->pos[2],
                    ctrlGroup->trajectoryIterator->pos[3], ctrlGroup->trajectoryIterator->pos[4], ctrlGroup->trajectoryIterator->pos[5]);

                //-------------------------------------
                // Check that incoming data is valid
                for (i = 0; i < ctrlGroup->numAxes; i++)
                {
                    // Velocity check
                    if (abs(ctrlGroup->trajectoryIterator->vel[i]) > ctrlGroup->maxSpeed[i])
                    {
                        // excessive speed
                        Ros_Debug_BroadcastMsg("ERROR: Invalid speed in message TrajPointFull data: \n  axis: %d, speed: %f, limit: %f\n",
                            i, ctrlGroup->trajectoryIterator->vel[i], ctrlGroup->maxSpeed[i]);

                        bzero(ctrlGroup->trajectoryToProcess, sizeof(ctrlGroup->trajectoryToProcess));
                        ctrlGroup->hasDataToProcess = FALSE;
                        continue;
                    }
                }

                //-------------------------------------

                JointMotionData _startTrajData;
                JointMotionData* startTrajData;
                JointMotionData* endTrajData;
                JointMotionData* curTrajData;
                double interval;                    // Time between startTime and the new data time
                double accCoef1[MP_GRP_AXES_NUM];   // Acceleration coefficient 1
                double accCoef2[MP_GRP_AXES_NUM];   // Acceleration coefficient 2
                UINT64 timeInc_ms;                  // time increment in millisecond
                UINT64 calculationTime_ms;          // time in ms at which the interpolation takes place
                long newPulsePos[MP_GRP_AXES_NUM];
                Incremental_data incData;

                // Initialization of pointers and memory
                curTrajData = ctrlGroup->prevTrajectoryIterator;
                endTrajData = ctrlGroup->trajectoryIterator;
                startTrajData = &_startTrajData;
                // Set the start of the trajectory interpolation as the current position (which should be the end of last interpolation)
                memcpy(startTrajData, curTrajData, sizeof(JointMotionData));

                // For MPL80/100 robot type (SLUBT): Controller automatically moves the B-axis
                // to maintain orientation as other axes are moved.
                if (ctrlGroup->bIsBaxisSlave)
                {
                    //moto joint order
                    endTrajData->pos[4] += -endTrajData->pos[1] + endTrajData->pos[2];
                    endTrajData->vel[4] += -endTrajData->vel[1] + endTrajData->vel[2];
                }

                bzero(newPulsePos, sizeof(newPulsePos));
                bzero(&incData, sizeof(incData));
                incData.frame = MP_INC_PULSE_DTYPE;
                incData.tool = ctrlGroup->tool;

                // Calculate an acceleration coefficients
                bzero(&accCoef1, sizeof(accCoef1));
                bzero(&accCoef2, sizeof(accCoef2));
                interval = (endTrajData->time - startTrajData->time) / 1000.0;  // time difference in sec
                if (interval > 0.0)
                {
                    for (i = 0; i < ctrlGroup->numAxes; i++)
                    {
                        //Calculate acceleration coefficient (convert interval to seconds
                        accCoef1[i] = (6 * (endTrajData->pos[i] - startTrajData->pos[i]) / (interval * interval))
                            - (2 * (endTrajData->vel[i] + 2 * startTrajData->vel[i]) / interval);
                        accCoef2[i] = (-12 * (endTrajData->pos[i] - startTrajData->pos[i]) / (interval * interval * interval))
                            + (6 * (endTrajData->vel[i] + startTrajData->vel[i]) / (interval * interval));
                    }
                }
                else
                {
                    Ros_Debug_BroadcastMsg("Warning: Group %d - Time difference between endTrajData (%lld) and startTrajData (%lld) is 0 or less.\n", ctrlGroup->groupNo, endTrajData->time, startTrajData->time);
                }

                // Initialize calculation variable before entering while loop
                calculationTime_ms = startTrajData->time;
                if (ctrlGroup->timeLeftover_ms == 0)
                    timeInc_ms = g_Ros_Controller.interpolPeriod;
                else
                    timeInc_ms = ctrlGroup->timeLeftover_ms;

                int iterationCounter = 0;
                // While interpolation time is smaller than new ROS point time
                while ((curTrajData->time < endTrajData->time) && Ros_Controller_IsMotionReady())
                {
                    iterationCounter += 1;
                    //Relinquish CPU control after some number of iterations. Prevent starvation of other tasks.
                    if (iterationCounter >= 15) //15 is an arbitrary number
                    {
                        Ros_Sleep(g_Ros_Controller.interpolPeriod);
                        iterationCounter = 0;
                    }

                    // Increment calculation time by next time increment
                    calculationTime_ms += timeInc_ms;
                    // time increment in second
                    double interpolTime = (calculationTime_ms - startTrajData->time) / 1000.0;

                    if (calculationTime_ms < endTrajData->time)  // Make calculation for full interpolation clock
                    {
                        // Set new interpolation time to calculation time
                        curTrajData->time = calculationTime_ms;

                        // For each axis calculate the new position at the interpolation time
                        for (i = 0; i < ctrlGroup->numAxes; i++)
                        {
                            // Add position change for new interpolation time
                            curTrajData->pos[i] = startTrajData->pos[i]                         // initial position component
                                + startTrajData->vel[i] * interpolTime                          // initial velocity component
                                + accCoef1[i] * interpolTime * interpolTime / 2                 // accCoef1 component
                                + accCoef2[i] * interpolTime * interpolTime * interpolTime / 6; // accCoef2 component

                            // Add velocity change for new interpolation time
                            curTrajData->vel[i] = startTrajData->vel[i]                         // initial velocity component
                                + accCoef1[i] * interpolTime                                    // accCoef1 component
                                + accCoef2[i] * interpolTime * interpolTime / 2;                // accCoef2 component
                        }

                        // Reset the timeInc_ms for the next interpolation cycle
                        if (timeInc_ms < g_Ros_Controller.interpolPeriod)
                        {
                            timeInc_ms = g_Ros_Controller.interpolPeriod;
                            ctrlGroup->timeLeftover_ms = 0;
                        }
                    }
                    else  // Make calculation for partial interpolation cycle
                    {
                        // Set the current trajectory data equal to the end trajectory
                        memcpy(curTrajData, endTrajData, sizeof(JointMotionData));

                        // Set the next interpolation increment to the the remainder to reach the next interpolation cycle
                        if (calculationTime_ms > endTrajData->time)
                        {
                            ctrlGroup->timeLeftover_ms = calculationTime_ms - endTrajData->time;
                        }
                    }

                    // Convert position in motoman pulse joint
                    Ros_CtrlGroup_ConvertRosUnitsToMotoUnits(ctrlGroup, curTrajData->pos, newPulsePos);

                    // Calculate the increment
                    incData.time = curTrajData->time;
                    for (i = 0; i < MP_GRP_AXES_NUM; i++)
                    {
                        if (!Ros_CtrlGroup_IsInvalidAxis(ctrlGroup, i))
                            incData.inc[i] = (newPulsePos[i] - ctrlGroup->prevPulsePos[i]);
                        else
                            incData.inc[i] = 0;
                    }

                    // Add the increment to the queue
                    if (!Ros_MotionControl_AddPulseIncPointToQ(ctrlGroup, &incData))
                    {
                        bzero(ctrlGroup->trajectoryToProcess, sizeof(ctrlGroup->trajectoryToProcess));
                        ctrlGroup->hasDataToProcess = FALSE;
                        continue;
                    }

                    // Copy data to the previous pulse position for next iteration
                    memcpy(ctrlGroup->prevPulsePos, newPulsePos, sizeof(ctrlGroup->prevPulsePos));
                }

                curTrajData->valid = FALSE;

                if (Ros_MotionControl_IsMotionMode_Trajectory())
                {
                    if (ctrlGroup->trajectoryIterator == &ctrlGroup->trajectoryToProcess[MAX_NUMBER_OF_POINTS_PER_TRAJECTORY]) //pointing to last possible entry in the array; don't increment iterator
                    {
                        bzero(ctrlGroup->trajectoryToProcess, sizeof(ctrlGroup->trajectoryToProcess));
                        ctrlGroup->hasDataToProcess = FALSE;
                        Ros_Debug_BroadcastMsg("Done processing final point in trajectory (Group #%d)", ctrlGroup->groupNo);
                    }
                    else
                    {
                        ctrlGroup->prevTrajectoryIterator += 1; // pointer increments sizeof(JointMotionData) bytes
                        ctrlGroup->trajectoryIterator += 1; // pointer increments sizeof(JointMotionData) bytes
                    }
                }
                else if (Ros_MotionControl_IsMotionMode_PointQueue())
                {
                    memcpy(ctrlGroup->prevTrajectoryIterator, ctrlGroup->trajectoryIterator, sizeof(JointMotionData));
                    bzero(ctrlGroup->trajectoryIterator, sizeof(JointMotionData));
                }

            } // IF this group has a point to process
            else
            {
                if (Ros_MotionControl_IsMotionMode_Trajectory())
                {
                    bzero(ctrlGroup->trajectoryToProcess, sizeof(ctrlGroup->trajectoryToProcess));
                    ctrlGroup->hasDataToProcess = FALSE;
                }
            }
        }

        Ros_Sleep(g_Ros_Controller.interpolPeriod);
    } // WHILE (TRUE)
}

//-------------------------------------------------------------------
// Adds pulse increments for one interpolation period to the inc move queue
//-------------------------------------------------------------------
BOOL Ros_MotionControl_AddPulseIncPointToQ(CtrlGroup* ctrlGroup, Incremental_data const* dataToEnQ)
{
    // Set pointer to specified queue
    Incremental_q* q = &ctrlGroup->inc_q;

    while (q->cnt >= Q_SIZE) //queue is full
    {
        //wait for items to be removed from the queue
        Ros_Sleep(g_Ros_Controller.interpolPeriod);

        //make sure we don't get stuck in infinite loop
        if (!Ros_Controller_IsMotionReady()) //<- they probably pressed HOLD or ESTOP
        {
            return FALSE;
        }
    }

    // Lock the q before manipulating it
    if (mpSemTake(q->q_lock, Q_LOCK_TIMEOUT) == OK)
    {
        // Get the index of the end of the queue
        int index = Q_OFFSET_IDX(q->idx, q->cnt, Q_SIZE);
        // Copy data at the end of the queue
        q->data[index] = *dataToEnQ;
        // increase the count of elements in the queue
        q->cnt++;

        // Unlock the q
        mpSemGive(q->q_lock);
    }
    else
    {
        Ros_Debug_BroadcastMsg("ERROR: Unable to add point to queue.  Queue is locked up! (Group #%d)", ctrlGroup->groupNo);
        return FALSE;
    }

    return TRUE;
}

UINT8 Ros_MotionControl_ProcessQueuedTrajectoryPoint(motoros2_interfaces__srv__QueueTrajPoint_Request* request)
{
    if (Ros_MotionControl_MustInitializePointQueue)
    {
        Ros_Debug_BroadcastMsg("Initial point in trajectory queue");

        Init_Trajectory_Status status;
        status = Ros_MotionControl_InitPointQueue(request);

        if (status == INIT_TRAJ_OK)
            return motoros2_interfaces__msg__QueueResultEnum__SUCCESS;
        else
            return motoros2_interfaces__msg__QueueResultEnum__INIT_FAILURE;
    }

    //------------------------------------------------------------
    //The trajectory contains information for all groups. Determine which groups are used by looking at the 'joint names'.
    int grpIndex, jointIndexInTraj;

    if (g_Ros_Controller.totalAxesCount != request->joint_names.size)
    {
        Ros_Debug_BroadcastMsg("Queued point must contain data for all %d joints.", g_Ros_Controller.totalAxesCount);
        return motoros2_interfaces__msg__QueueResultEnum__INVALID_JOINT_LIST;
    }

    //===================================
    //Incoming points are processed one joint at a time.
    //For each of those joints, this iterates over all of the CtrlGroup objects and compares the joint names.
    //This allows it to find the correct CtrlGroup object and the joint index (in moto order) in the JointMotionData array.
    //===================================

    //precheck to ensure all groups are ready to accept a new point
    for (grpIndex = 0; grpIndex < g_Ros_Controller.numGroup; grpIndex += 1)
    {
        CtrlGroup* ctrlGroup = g_Ros_Controller.ctrlGroups[grpIndex];

        if (ctrlGroup->trajectoryIterator != NULL && ctrlGroup->trajectoryIterator->valid)
        {
            //A point is already being processed for this control group.
            //Wait for it to be processed before adding a new point.
            return motoros2_interfaces__msg__QueueResultEnum__BUSY;
        }
    }

    //for each joint/axis in a single trajectory point
    for (jointIndexInTraj = 0; jointIndexInTraj < request->joint_names.size; jointIndexInTraj += 1)
    {
        int  jointIndexInCtrlGroup;
        CtrlGroup* ctrlGroup;


        //find the ctrlgroup for this joint
        BOOL bFound = FALSE;
        for (grpIndex = 0; grpIndex < g_Ros_Controller.numGroup; grpIndex += 1)
        {
            ctrlGroup = g_Ros_Controller.ctrlGroups[grpIndex];
            if (!ctrlGroup)
                continue;

            for (jointIndexInCtrlGroup = 0; jointIndexInCtrlGroup < MP_GRP_AXES_NUM; jointIndexInCtrlGroup += 1)
            {
                char* jointName = ctrlGroup->jointNames_userDefined[jointIndexInCtrlGroup];
                if (strlen(jointName) != 0)
                {
                    if (strcmp(jointName, request->joint_names.data[jointIndexInTraj].data) == 0)
                    {
                        bFound = TRUE;
                        break;
                    }
                }
            }

            if (bFound)
                break;
        }

        if (!bFound)
        {
            Ros_Debug_BroadcastMsg("Joint name [%s] is not valid. Check motoros2_config.yaml and update accordingly.", request->joint_names.data[jointIndexInTraj].data);
            Ros_Debug_BroadcastMsg("Valid names:");
            for (int groupIndex = 0; groupIndex < MAX_CONTROLLABLE_GROUPS; groupIndex += 1)
            {
                for (int jointIndex = 0; jointIndex < MP_GRP_AXES_NUM; jointIndex += 1)
                {
                    char* configListEntry = g_nodeConfigSettings.joint_names[(groupIndex * MP_GRP_AXES_NUM) + jointIndex];
                    if (strlen(configListEntry) != 0)
                        Ros_Debug_BroadcastMsg(" - %s", configListEntry);
                }
            }

            return motoros2_interfaces__msg__QueueResultEnum__INVALID_JOINT_LIST;
        }

        // for point queuing, we create a single-point trajectory, store the incoming
        // point in it and send it off for processing by the trajectory processing
        // pipeline.
        trajectory_msgs__msg__JointTrajectoryPoint__Sequence pointSequence;

        pointSequence.capacity = 1;
        pointSequence.size = 1;
        pointSequence.data = &request->point; //no additional memory is allocated this way

        //NOTE: I'm using the SECOND point in the 200 point buffer to hold the converted data. The `Ros_MotionControl_Init` function
        //      populated the first buffer position with the initial point in the queue. Followup points are placed in the second 
        //      buffer position. As the destination in position 2 is processed, it is moved into position 1 to become the starting
        //      point for the next destination.
        Init_Trajectory_Status status = Ros_MotionControl_ConvertTrajectoryToJointMotionData(&pointSequence, jointIndexInTraj, ctrlGroup, jointIndexInCtrlGroup, ctrlGroup->trajectoryIterator);
        if (status != INIT_TRAJ_OK)
        {
            Ros_Debug_BroadcastMsg("Failed to parse incoming trajectory point.");
            return motoros2_interfaces__msg__QueueResultEnum__UNABLE_TO_PROCESS_POINT;
        }
    }

    for (grpIndex = 0; grpIndex < g_Ros_Controller.numGroup; grpIndex += 1)
    {
        CtrlGroup* ctrlGroup = g_Ros_Controller.ctrlGroups[grpIndex];

        ctrlGroup->trajectoryIterator->valid = TRUE;
    }

    return motoros2_interfaces__msg__QueueResultEnum__SUCCESS;
}

//-------------------------------------------------------------------
// Task to move the robot at each interpolation increment
//-------------------------------------------------------------------
void Ros_MotionControl_IncMoveLoopStart() //<-- IP_CLK priority task
{
    MP_EXPOS_DATA moveData;

    Incremental_q* q;
    int i;
    int ret;
    UINT64 inc_data_time;
    UINT64 q_time;
    int axis;

    MP_CTRL_GRP_SEND_DATA ctrlGrpData;
    MP_PULSE_POS_RSP_DATA prevPulsePosData[MAX_CONTROLLABLE_GROUPS];
    MP_PULSE_POS_RSP_DATA pulsePosData;

    // --- FSU Speed Limit related ---
    // When FSU speed limitation is active, some pulses for an interpolation cycle may not be processed by the controller.
    // To track the true amount of pulses processed, we keep track of the command position and by substracting the
    // the previous position from the current one, we can confirm the amount if pulses precessed.
    // If the amount processed doesn't match the amount sent at the previous cycle, we resend the unprocessed pulses amount.
    // To keep the motion smooth, the 'maximum speed' (max pulses per cycle) is tracked and used to skip reading
    // more pulse increment from the queue if the amount of unprocessed pulses is larger than the detected speed.
    // The 'maximum speed' is also used to prevent exceeding the commanded speed once the FSU Speed Limit is removed.
    //
    // The following set of variables are used to track FSU speed limitation.
    LONG newPulseInc[MAX_CONTROLLABLE_GROUPS][MP_GRP_AXES_NUM];         // Pulse increments that we just retrieved from the incQueue
    LONG toProcessPulses[MAX_CONTROLLABLE_GROUPS][MP_GRP_AXES_NUM];     // Total pulses that still need to be sent to the command
    LONG processedPulses[MP_GRP_AXES_NUM];                              // Amount of pulses from the last command that were actually processed (accepted)
    LONG maxSpeed[MAX_CONTROLLABLE_GROUPS][MP_GRP_AXES_NUM];            // ROS speed (amount of pulses for one cycle from the data queue) that should not be exceeded
    LONG maxSpeedRemain[MAX_CONTROLLABLE_GROUPS][MP_GRP_AXES_NUM];      // Number of pulses (absolute) that remains to be processed at the 'maxSpeed'
    LONG prevMaxSpeed[MAX_CONTROLLABLE_GROUPS][MP_GRP_AXES_NUM];        // Previous data queue reading 'maxSpeed'
    LONG prevMaxSpeedRemain[MAX_CONTROLLABLE_GROUPS][MP_GRP_AXES_NUM];  // Previous data queue reading 'maxSpeedRemain'
    BOOL skipReadingQ[MAX_CONTROLLABLE_GROUPS];                         // Flag indicating to skip reading more data from the increment queue (there is enough unprocessed from previous cycles remaining)
    BOOL queueRead[MAX_CONTROLLABLE_GROUPS];                            // Flag indicating that new increment data was retrieve from the queue on this cycle.
    BOOL isMissingPulse;                                                // Flag that there are pulses send in last cycle that are missing from the command (pulses were not processed)
    BOOL hasUnprocessedData;                                            // Flag that at least one axis (any group) still has unprecessed data. (Used to continue sending data after the queue is empty.)

    bzero(newPulseInc, sizeof(LONG) * MP_GRP_AXES_NUM * MAX_CONTROLLABLE_GROUPS);
    bzero(toProcessPulses, sizeof(LONG) * MP_GRP_AXES_NUM * MAX_CONTROLLABLE_GROUPS);
    bzero(processedPulses, sizeof(LONG) * MP_GRP_AXES_NUM);
    bzero(prevMaxSpeed, sizeof(LONG) * MP_GRP_AXES_NUM * MAX_CONTROLLABLE_GROUPS);
    bzero(prevMaxSpeedRemain, sizeof(LONG) * MP_GRP_AXES_NUM * MAX_CONTROLLABLE_GROUPS);
    bzero(maxSpeed, sizeof(LONG) * MP_GRP_AXES_NUM * MAX_CONTROLLABLE_GROUPS);
    bzero(maxSpeedRemain, sizeof(LONG) * MP_GRP_AXES_NUM * MAX_CONTROLLABLE_GROUPS);
    bzero(skipReadingQ, sizeof(BOOL) * MAX_CONTROLLABLE_GROUPS);
    bzero(queueRead, sizeof(BOOL) * MAX_CONTROLLABLE_GROUPS);

    isMissingPulse = FALSE;
    hasUnprocessedData = FALSE;

    Ros_Debug_BroadcastMsg("IncMoveTask Started");

    bzero(&moveData, sizeof(moveData));

    for (i = 0; i < g_Ros_Controller.numGroup; i++)
    {
        moveData.ctrl_grp |= (0x01 << i);
        moveData.grp_pos_info[i].pos_tag.data[0] = Ros_CtrlGroup_GetAxisConfig(g_Ros_Controller.ctrlGroups[i]);

        ctrlGrpData.sCtrlGrp = g_Ros_Controller.ctrlGroups[i]->groupId;
        mpGetPulsePos(&ctrlGrpData, &prevPulsePosData[i]);
    }

    FOREVER
    {
        mpClkAnnounce(MP_INTERPOLATION_CLK);

        if (Ros_Controller_IsMotionReady()
            && (Ros_MotionControl_HasDataInQueue() || hasUnprocessedData)
            && !g_Ros_Controller.bStopMotion)
        {
            // For each control group, retrieve the new pulse increments for this cycle
            for (i = 0; i < g_Ros_Controller.numGroup; i++)
            {
                queueRead[i] = FALSE;
                if (skipReadingQ[i])
                {
                    // Reset skip flag and set position increment to 0
                    skipReadingQ[i] = FALSE;
                    bzero(&moveData.grp_pos_info[i].pos, sizeof(LONG) * MP_GRP_AXES_NUM);
                }
                else
                {
                    // Retrieve position increment from the queue.
                    q = &g_Ros_Controller.ctrlGroups[i]->inc_q;

                    // Lock the q before manipulating it
                    if (mpSemTake(q->q_lock, Q_LOCK_TIMEOUT) == OK)
                    {
                        if (q->cnt > 0)
                        {
                            // Initialize moveData with the next data from the queue
                            inc_data_time = q->data[q->idx].time;
                            q_time = g_Ros_Controller.ctrlGroups[i]->q_time;
                            moveData.grp_pos_info[i].pos_tag.data[2] = q->data[q->idx].tool;
                            moveData.grp_pos_info[i].pos_tag.data[3] = q->data[q->idx].frame;
                            moveData.grp_pos_info[i].pos_tag.data[4] = q->data[q->idx].user;

                            memcpy(&moveData.grp_pos_info[i].pos, &q->data[q->idx].inc, sizeof(LONG) * MP_GRP_AXES_NUM);
                            queueRead[i] = TRUE;

                            // increment index in the queue and decrease the count
                            q->idx = Q_OFFSET_IDX(q->idx, 1, Q_SIZE);
                            q->cnt--;

                            // Check if complete interpolation period covered.
                            // (Because time period of data received from ROS may not be a multiple of the
                            // controller interpolation clock period, the queue may contain partiel period and
                            // more than one queue increment maybe required to complete the interpolation period)
                            while (q->cnt > 0)
                            {
                                if ((q_time <= q->data[q->idx].time)
                                    && (q->data[q->idx].time - q_time <= g_Ros_Controller.interpolPeriod))
                                {
                                    // next incMove is part of same interpolation period

                                    // check that information is in the same format
                                    if ((moveData.grp_pos_info[i].pos_tag.data[2] != q->data[q->idx].tool)
                                        || (moveData.grp_pos_info[i].pos_tag.data[3] != q->data[q->idx].frame)
                                        || (moveData.grp_pos_info[i].pos_tag.data[4] != q->data[q->idx].user))
                                    {
                                        // Different format can't combine information
                                        break;
                                    }

                                    // add next incMove to current incMove
                                    for (axis = 0; axis < MP_GRP_AXES_NUM; axis++)
                                        moveData.grp_pos_info[i].pos[axis] += q->data[q->idx].inc[axis];
                                    inc_data_time = q->data[q->idx].time;

                                    // increment index in the queue and decrease the count
                                    q->idx = Q_OFFSET_IDX(q->idx, 1, Q_SIZE);
                                    q->cnt--;
                                }
                                else
                                {
                                    // interpolation period complete
                                    break;
                                }
                            }

                            g_Ros_Controller.ctrlGroups[i]->q_time = inc_data_time;
                        }
                        else
                        {
                            // Queue is empty, initialize to 0 pulse increment
                            moveData.grp_pos_info[i].pos_tag.data[2] = 0;
                            moveData.grp_pos_info[i].pos_tag.data[3] = MP_INC_PULSE_DTYPE;
                            moveData.grp_pos_info[i].pos_tag.data[4] = 0;
                            bzero(&moveData.grp_pos_info[i].pos, sizeof(LONG) * MP_GRP_AXES_NUM);
                        }

                        // Unlock the q
                        mpSemGive(q->q_lock);
                    }
                    else
                    {
                        Ros_Debug_BroadcastMsg("ERROR: Can't get data from queue. Queue is locked up (Group #%d)", g_Ros_Controller.ctrlGroups[i]->groupNo);
                        bzero(&moveData.grp_pos_info[i].pos, sizeof(LONG) * MP_GRP_AXES_NUM);
                        continue;
                    }
                }
            }

            hasUnprocessedData = FALSE;
            for (i = 0; i < g_Ros_Controller.numGroup; i++)
            {
                memcpy(newPulseInc[i], moveData.grp_pos_info[i].pos, sizeof(LONG) * MP_GRP_AXES_NUM);

                // record the speed associate with the next amount of pulses
                if (queueRead[i])
                {
                    for (axis = 0; axis < MP_GRP_AXES_NUM; axis++)
                    {
                        maxSpeed[i][axis] = abs(newPulseInc[i][axis]);
                        maxSpeedRemain[i][axis] = abs(newPulseInc[i][axis]);
                    }
                }

                // Check if pulses are missing from last increment.
                // Get the current controller command position and substract the previous command position
                // and check if it matches the amount if increment sent last cycle.  If it doesn't then
                // some pulses are missing and the amount of unprocessed pulses needs to be added to this cycle.
                ctrlGrpData.sCtrlGrp = g_Ros_Controller.ctrlGroups[i]->groupId;
                mpGetPulsePos(&ctrlGrpData, &pulsePosData);
                isMissingPulse = FALSE;                
                for (axis = 0; axis < MP_GRP_AXES_NUM; axis++)
                {
                    // Check how many pulses we processed from last increment
                    processedPulses[axis] = pulsePosData.lPos[axis] - prevPulsePosData[i].lPos[axis];
                    prevPulsePosData[i].lPos[axis] = pulsePosData.lPos[axis];

                    // Remove those pulses from the amount to process.  
                    // If everything was processed, then there should by 0 pulses left. Otherwise FSU Speed limit prevented processing
                    toProcessPulses[i][axis] -= processedPulses[axis];
                    if (toProcessPulses[i][axis] != 0)
                        isMissingPulse = TRUE;

                    // Add the new pulses to be processed for this iteration 
                    toProcessPulses[i][axis] += newPulseInc[i][axis];

                    if (toProcessPulses[i][axis] != 0)
                        hasUnprocessedData = TRUE;
                }

                // Check if pulses are missing which means that the FSU speed limit is enabled
                if (isMissingPulse)
                {
                    UINT64 max_inc;

                    // Prevent going faster than original requested speed once speed limit turns off
                    // Check if the speed (inc) of previous interation should be considered by checking 
                    // if the unprocessed pulses from that speed setting still remains.
                    // If all the pulses of previous increment were processed, then transfer the current 
                    // speed and process the next increment from the increment queue.
                    for (axis = 0; axis < MP_GRP_AXES_NUM; axis++)
                    {
                        // Check if has pulses to process
                        if (toProcessPulses[i][axis] == 0)
                            prevMaxSpeedRemain[i][axis] = 0;
                        else
                            prevMaxSpeedRemain[i][axis] = abs(prevMaxSpeedRemain[i][axis]) - abs(processedPulses[axis]);
                    }

                    // Check if still have data to process from previous iteration
                    skipReadingQ[i] = FALSE;
                    for (axis = 0; axis < MP_GRP_AXES_NUM; axis++)
                    {
                        if (prevMaxSpeedRemain[i][axis] > 0)
                            skipReadingQ[i] = TRUE;
                    }

                    if (!skipReadingQ[i]) {
                        for (axis = 0; axis < MP_GRP_AXES_NUM; axis++) {
                            // Transfer the current speed as the new prevSpeed
                            prevMaxSpeed[i][axis] = maxSpeed[i][axis];
                            prevMaxSpeedRemain[i][axis] += maxSpeedRemain[i][axis];
                        }
                    }

                    // Set the number of pulse that can be sent without exceeding speed
                    for (axis = 0; axis < MP_GRP_AXES_NUM; axis++)
                    {
                        // Check if has pulses to process
                        if (toProcessPulses[i][axis] == 0)
                            continue;

                        // Maximum inc that should be send ()
                        if (prevMaxSpeed[i][axis] > 0)
                            // if previous speed is defined use it
                            max_inc = prevMaxSpeed[i][axis];
                        else
                        {
                            if (maxSpeed[i][axis] > 0)
                                // else fallback on current speed if defined
                                max_inc = maxSpeed[i][axis];
                            else if (newPulseInc[i][axis] != 0)
                                // use the current speed if none zero.
                                max_inc = abs(newPulseInc[i][axis]);
                            else
                                // otherwise use the axis max speed
                                max_inc = g_Ros_Controller.ctrlGroups[i]->maxInc.maxIncrement[axis];

                            if(max_inc > 1)
                                Ros_Debug_BroadcastMsg("Warning undefined speed: Axis %d Defaulting Max Inc: %d (prevSpeed: %d curSpeed %d)",
                                axis, max_inc, prevMaxSpeed[i][axis], maxSpeed[i][axis]);

                        }

                        // Set new increment and recalculate unsent pulses
                        if (abs(toProcessPulses[i][axis]) <= max_inc)
                        {
                            // Pulses to send is small than max, so send everything
                            moveData.grp_pos_info[i].pos[axis] = toProcessPulses[i][axis];
                        }
                        else {
                            // Pulses to send is too high, so send the amount matching the maximum speed
                            if (toProcessPulses[i][axis] >= 0)
                                moveData.grp_pos_info[i].pos[axis] = max_inc;
                            else
                                moveData.grp_pos_info[i].pos[axis] = -max_inc;
                        }
                    }
                }
                else
                {
                    // No PFL Speed Limit detected
                    for (axis = 0; axis < MP_GRP_AXES_NUM; axis++)
                    {
                        prevMaxSpeed[i][axis] = abs(moveData.grp_pos_info[i].pos[axis]);
                        prevMaxSpeedRemain[i][axis] = abs(moveData.grp_pos_info[i].pos[axis]);
                    }
                }
            }

            // Make sure motion / goal has not been cancelled in the meantime.
            // Additionally, if the Agent PC is disconnected, check to see if
            // motion should continue.
            if (!g_Ros_Controller.bStopMotion && (g_Ros_Communication_AgentIsConnected || !g_nodeConfigSettings.stop_motion_on_disconnect))
            {
                // Send pulse increment to the controller command position
                ret = mpExRcsIncrementMove(&moveData);

                Ros_ActionServer_FJT_UpdateProgressTracker(&moveData);
            }
            else
                ret = 0;

            if (ret != 0)
            {
                // Failure: command rejected by controller.
                // Update controller status to help identify cause
                Ros_Controller_IoStatusUpdate();

                if (ret == E_EXRCS_CTRL_GRP)
                    Ros_Debug_BroadcastMsg("mpExRcsIncrementMove returned: %d (ctrl_grp = %d)", ret, moveData.ctrl_grp);
                else if (ret == E_EXRCS_IMOV_UNREADY && g_Ros_Controller.bPFLEnabled)
                {
                    // Check if this is caused by a known cause (E-Stop, Hold, Alarm, Error)
                    if (!Ros_Controller_IsEStop() && !Ros_Controller_IsHold()
                        && !Ros_Controller_IsAlarm() && !Ros_Controller_IsError()) {
                        Ros_Debug_BroadcastMsg("mpExRcsIncrementMove returned UNREADY: %d (Could be PFL Active)", E_EXRCS_IMOV_UNREADY);
                        g_Ros_Controller.bPFLduringRosMove = TRUE;
                    }
                }
                else if (ret == E_EXRCS_PFL_FUNC_BUSY && g_Ros_Controller.bPFLEnabled)
                {
                    Ros_Debug_BroadcastMsg("mpExRcsIncrementMove returned PFL Active");
                    g_Ros_Controller.bPFLduringRosMove = TRUE;
                }
                else if (ret == E_EXRCS_UNDER_ENERGY_SAVING)
                {
                    // retry until servos turn on and motion is accepted
                    int checkCount;
                    for (checkCount = 0; checkCount < MOTION_START_TIMEOUT; checkCount += MOTION_START_CHECK_PERIOD)
                    {
                        ret = mpExRcsIncrementMove(&moveData);
                        if (ret != E_EXRCS_UNDER_ENERGY_SAVING)
                            break;

                        Ros_Sleep(MOTION_START_CHECK_PERIOD);
                    }
                    if (g_Ros_Controller.bMpIncMoveError)
                        Ros_Debug_BroadcastMsg("mpExRcsIncrementMove returned Eco mode enabled");
                }
                else if(ret == E_EXRCS_IMOV_UNREADY)
                {
                    Ros_Debug_BroadcastMsg("mpExRcsIncrementMove returned -1 (Not executing WAIT instruction)");
                }
                else
                    Ros_Debug_BroadcastMsg("mpExRcsIncrementMove returned: %d", ret);

                // Stop motion if motion was rejected
                if (ret != 0)
                {
                    // Flag to prevent further motion until Trajectory mode is reenabled
                    g_Ros_Controller.bMpIncMoveError = TRUE;
                    Ros_MotionControl_StopMotion(/*bKeepJobRunning = */ FALSE);
                    Ros_Debug_BroadcastMsg("Stopping all motion");
                }
            }
        }
        else
        {
            // Reset previous position in case the robot is moved externally
            bzero(toProcessPulses, sizeof(LONG) * MP_GRP_AXES_NUM * MAX_CONTROLLABLE_GROUPS);
            hasUnprocessedData = FALSE;
            for (i = 0; i < g_Ros_Controller.numGroup; i++)
            {
                ctrlGrpData.sCtrlGrp = g_Ros_Controller.ctrlGroups[i]->groupId;
                mpGetPulsePos(&ctrlGrpData, &prevPulsePosData[i]);
            }
        }
    }
}

//-------------------------------------------------------------------
// Check the number of inc_move currently in the specified queue
//-------------------------------------------------------------------
int Ros_MotionControl_GetQueueCnt(int groupNo)
{
    Incremental_q* q;

    // Check group number valid
    if (!Ros_Controller_IsValidGroupNo(groupNo))
        return -1;

    // Set pointer to specified queue
    q = &g_Ros_Controller.ctrlGroups[groupNo]->inc_q;

    // Lock the q before manipulating it
    if (mpSemTake(q->q_lock, Q_LOCK_TIMEOUT) == OK)
    {
        int count = q->cnt;

        // Unlock the q
        mpSemGive(q->q_lock);

        return count;
    }

    Ros_Debug_BroadcastMsg("ERROR: Unable to access queue count.  Queue is locked up! (Group #%d)", groupNo);
    return ERROR;
}

//-------------------------------------------------------------------
// Check that at least one control group of the controller has data in queue
//-------------------------------------------------------------------
BOOL Ros_MotionControl_HasDataInQueue()
{
    for (int groupNo = 0; groupNo < g_Ros_Controller.numGroup; groupNo++)
    {
        int qCnt = Ros_MotionControl_GetQueueCnt(groupNo);
        if (qCnt > 0)
            return TRUE;
        else if (qCnt == ERROR)
            return ERROR;
    }

    return FALSE;
}

//-------------------------------------------------------------------
// Check that at least one control group of the controller is breaking up data
//-------------------------------------------------------------------
BOOL Ros_MotionControl_HasDataToProcess()
{
    for (int groupIndex = 0; groupIndex < g_Ros_Controller.numGroup; groupIndex++)
    {
        if (g_Ros_Controller.ctrlGroups[groupIndex]->hasDataToProcess)
        {
            return TRUE;
        }
    }

    return FALSE;
}

//-------------------------------------------------------------------
// Determine whether MotoROS2 is commanding motion or not
//-------------------------------------------------------------------
BOOL Ros_MotionControl_IsRosControllingMotion()
{
    return (Ros_Controller_IsInMotion() && 
        (Ros_MotionControl_ActiveMotionMode != MOTION_MODE_INACTIVE));
}

//-----------------------------------------------------------------------
// Stop motion by stopping message processing and clearing the queue
//-----------------------------------------------------------------------
BOOL Ros_MotionControl_StopMotion(BOOL bKeepJobRunning)
{
    // NOTE: for the time being, stop motion will stop all motion for all control group
    BOOL bRet;
    BOOL bStopped;
    int checkCnt;

    MP_HOLD_SEND_DATA holdSendData;
    MP_STD_RSP_DATA stdRspData;
    MP_START_JOB_SEND_DATA startJobData;

    // Stop any motion from being processed further
    // NOTE: order is important here:
    //
    //  - first raise bStopMotion (which signals to IncMoveLoop to not process
    //    any more increments)
    //  - then hold the INIT_ROS job, so robot comes to a smooth stop
    //
    // Otherwise mpExRcsIncrementMove(..) will fail trying to submit an increment
    // while INIT_ROS has already been suspended.
    g_Ros_Controller.bStopMotion = TRUE;

    holdSendData.sHold = ON;
    mpHold(&holdSendData, &stdRspData);

    bStopped = FALSE;
    // Check that background processing of message has been stopped
    for (checkCnt = 0; checkCnt < MOTION_STOP_TIMEOUT; checkCnt++)
    {
        BOOL bAtLeastOne = FALSE;
        for (int i = 0; i < g_Ros_Controller.numGroup; i += 1)
        {
            if (g_Ros_Controller.ctrlGroups[i]->hasDataToProcess)
                bAtLeastOne = TRUE;
        }

        if (!bAtLeastOne)
        {
            bStopped = TRUE;
            break;
        }
        else
            Ros_Sleep(1);
    }

    // Clear queues
    bRet = Ros_MotionControl_ClearQ_All();

    // All motion should be stopped at this point, so turn of the flag
    g_Ros_Controller.bStopMotion = FALSE;

    holdSendData.sHold = OFF;
    mpHold(&holdSendData, &stdRspData);

    if (bKeepJobRunning)
    {
        //zero struct, so cJobName is empty, which results in a 'resume from HOLD'
        //NOTE: mpStartJob(..) docs are a bit ambiguous about this
        bzero(&startJobData, sizeof(startJobData));
        mpStartJob(&startJobData, &stdRspData);
        if (stdRspData.err_no != 0)
        {
            Ros_Debug_BroadcastMsg("WARNING: mpStartJob error: %d", stdRspData.err_no);
        }
    }

    if (checkCnt >= MOTION_STOP_TIMEOUT)
        Ros_Debug_BroadcastMsg("WARNING: Message processing not stopped before clearing queue");

    return(bStopped && bRet);
}

//-------------------------------------------------------------------
// Clears the inc move queue
//-------------------------------------------------------------------
BOOL Ros_MotionControl_ClearQ_All()
{
    BOOL bRet = TRUE;

    for (int groupNo = 0; groupNo < g_Ros_Controller.numGroup; groupNo++)
    {
        // Stop addtional items from being added to the queue
        g_Ros_Controller.ctrlGroups[groupNo]->hasDataToProcess = FALSE;

        // Set pointer to specified queue
        Incremental_q* q = &g_Ros_Controller.ctrlGroups[groupNo]->inc_q;

        // Lock the q before manipulating it
        if (mpSemTake(q->q_lock, Q_LOCK_TIMEOUT) == OK)
        {
            // Reset the queue.  No need to modify index or delete data
            q->cnt = 0;

            // Unlock the q
            mpSemGive(q->q_lock);
        }
    }

    return bRet;
}

// only for this compilation unit for now
// TODO(gavanderhoorn): refactor
static STATUS Ros_Controller_DisableEcoMode()
{
#define DISABLE_ECO_MODE_TIMEOUT      5000  // in milliseconds
#define DISABLE_ECO_MODE_CHECK_PERIOD   50  // in millisecond

    MP_SERVO_POWER_SEND_DATA sServoData;
    MP_STD_RSP_DATA rData;
    int ret;

#ifdef DUMMY_SERVO_MODE
    return OK;
#endif

    // eco mode is "power on, but off due to energy conservation mode",
    // so request servos to be powered off, which also exits energy
    // conservation mode.
    if (Ros_Controller_IsEcoMode() == TRUE)
    {
        //toggle servos to disable energy-savings mode
        sServoData.sServoPower = 0;  // OFF
        bzero(&sServoData, sizeof(sServoData));
        bzero(&rData, sizeof(rData));
        ret = mpSetServoPower(&sServoData, &rData);
        if ((ret == 0) && (rData.err_no == 0))
        {
            // wait for the Servo/Eco OFF confirmation
            int checkCount;
            for (checkCount = 0; checkCount < DISABLE_ECO_MODE_TIMEOUT; checkCount += DISABLE_ECO_MODE_CHECK_PERIOD)
            {
                // Update status
                Ros_Controller_IoStatusUpdate();

                if (Ros_Controller_IsEcoMode() == FALSE)
                    break;

                Ros_Sleep(DISABLE_ECO_MODE_CHECK_PERIOD);
            }
        }
        else
        {
            return NG;
        }
    }

    if (Ros_Controller_IsEcoMode() == FALSE)
        return OK;
    else
        return NG;
}

//-----------------------------------------------------------------------
// Attempts to start playback of a job to put the controller in RosMotion mode
//
// NOTE: only attempts to start job if necessary, does not reset errors, alarms.
//       Does attempt to enable servo power (if not on)
//       Does attempt to set the cycle mode to AUTO (if not set)
//-----------------------------------------------------------------------
MotionNotReadyCode Ros_MotionControl_StartMotionMode(MOTION_MODE mode, rosidl_runtime_c__String* responseMessage)
{
    int ret;
    MP_STD_RSP_DATA rData;
    MP_START_JOB_SEND_DATA sStartData;
    int checkCount, grpNo, alarmcode;
    MotionNotReadyCode motion_readiness_code;
    char output[MOTION_START_ERROR_MESSAGE_LENGTH] = { 0 };

    Ros_Debug_BroadcastMsg("%s: enter", __func__);

    if (Ros_MotionControl_ActiveMotionMode != MOTION_MODE_INACTIVE &&
        Ros_MotionControl_ActiveMotionMode != mode)
    {
        Ros_Debug_BroadcastMsg("Another trajectory mode (%d) is already active.", mode);
        return MOTION_NOT_READY_OTHER_TRAJ_MODE_ACTIVE;
    }

    Ros_Controller_IoStatusUpdate();
    // Check if already in the proper mode
    if (Ros_Controller_IsMotionReady())
    {
        Ros_Debug_BroadcastMsg("Already active");
        return MOTION_READY;
    }
    motion_readiness_code = Ros_Controller_GetNotReadySubcode(true);

    //Return with code if any of the problems are intractable
    switch (motion_readiness_code)
    {
        case MOTION_NOT_READY_ESTOP:
        case MOTION_NOT_READY_NOT_PLAY:
        case MOTION_NOT_READY_NOT_REMOTE:
        case MOTION_NOT_READY_HOLD:
            return motion_readiness_code;
        case MOTION_NOT_READY_ERROR: 
            //the responseMessage gets populated only in this case or for other MOTION_NOT_READY_ERROR
            //returns because it has the most information about the error in this situtation. 
            //in other cases, the responseMessage is populated upstream/with only the MotionNotReadyCode
            alarmcode = Ros_Controller_GetAlarmCode();
            snprintf(output, MOTION_START_ERROR_MESSAGE_LENGTH, "%s: '%s' (0x%04X)",
                Ros_ErrorHandling_MotionNotReadyCode_ToString(MOTION_NOT_READY_ERROR),
                Ros_ErrorHandling_ErrNo_ToString(alarmcode),
                alarmcode);
            Ros_Debug_BroadcastMsg(output);
            rosidl_runtime_c__String__assign(responseMessage, output);
            Ros_Debug_BroadcastMsg("Controller is in a fault state. Please call /reset_error");
            return motion_readiness_code;
        case MOTION_NOT_READY_ALARM:
        case MOTION_NOT_READY_PFL_ACTIVE:
        case MOTION_NOT_READY_INC_MOVE_ERROR:
            Ros_Debug_BroadcastMsg("Controller is in a fault state. Please call /reset_error");
            return motion_readiness_code;
        case MOTION_NOT_READY_OTHER_PROGRAM_RUNNING:
            Ros_Debug_BroadcastMsg("%s: robot is running another job (expected: '%s')",
                __func__, g_nodeConfigSettings.inform_job_name);
            return MOTION_NOT_READY_OTHER_PROGRAM_RUNNING;
        default: //Only here to get rid of warnings while compiling...
            ;
    }
    if (Ros_Controller_IsOperating()) 
    {
        //the current call to StarTrajMode is likely intended to get the
        //servos out of eco mode. In that case, servo power will be turned
        //ON again below, but in order for things to work, we need to stop
        //the currently running job first. It will be (re)started at the
        //end of StartTrajMode.
        MP_HOLD_SEND_DATA holdSendData;
        MP_STD_RSP_DATA stdRspData;
        holdSendData.sHold = ON;
        mpHold(&holdSendData, &stdRspData);
        Ros_Sleep(MOTION_START_CHECK_PERIOD);
        holdSendData.sHold = OFF;
        mpHold(&holdSendData, &stdRspData);
        Ros_Sleep(MOTION_START_CHECK_PERIOD);
    }

    // Check if in continous cycle mode
    if (!Ros_Controller_IsContinuousCycle())
    {
        // set the cycle mode to auto if not currently
        MP_CYCLE_SEND_DATA sCycleData;
        bzero(&sCycleData, sizeof(sCycleData));
        bzero(&rData, sizeof(rData));
        sCycleData.sCycle = MP_CYCLE_MODE_AUTO;
        ret = mpSetCycle(&sCycleData, &rData);
        if ((ret != 0) || (rData.err_no != 0))
        {
            snprintf(output, MOTION_START_ERROR_MESSAGE_LENGTH, 
                "%s: Can't set cycle mode to AUTO because: '%s' (0x%04X)",
                Ros_ErrorHandling_MotionNotReadyCode_ToString(MOTION_NOT_READY_ERROR),
                Ros_ErrorHandling_ErrNo_ToString(rData.err_no),
                rData.err_no);
            Ros_Debug_BroadcastMsg(output);
            rosidl_runtime_c__String__assign(responseMessage, output);

            mpSetAlarm(ALARM_OPERATION_FAIL, "Set job-cycle to AUTO", SUBCODE_OPERATION_SET_CYCLE);
            return MOTION_NOT_READY_ERROR;
        }

        Ros_Sleep(g_Ros_Controller.interpolPeriod); //give CIO time to potentially overwrite the cycle (Ladder scan time is smaller than the interpolPeriod)
        Ros_Controller_IoStatusUpdate(); //verify the cycle got set and wasn't forced back due to CIO logic

        if (!Ros_Controller_IsContinuousCycle())
        {
            Ros_Debug_BroadcastMsg("Can't set cycle mode. Check CIOPRG.LST for OUT #40050 - #40052");
            mpSetAlarm(ALARM_OPERATION_FAIL, "Set job-cycle to AUTO", SUBCODE_OPERATION_SET_CYCLE);
            return MOTION_NOT_READY_NOT_CONT_CYCLE_MODE;
        }
    }

#ifndef DUMMY_SERVO_MODE
    if (!Ros_Controller_IsServoOn())
    {
        // if servos are "off" due to eco mode, attempt to disable it
        if (Ros_Controller_IsEcoMode())
        {
            if (Ros_Controller_DisableEcoMode() == NG)
            {
                Ros_Debug_BroadcastMsg("Couldn't disable eco mode");
                return MOTION_NOT_READY_ECO_MODE;
            }
        }

        // servos are off, eco mode is not active any more (if it was), so
        // request power to be turned (back) on
        MP_SERVO_POWER_SEND_DATA sServoData;
        bzero(&sServoData, sizeof(sServoData));
        bzero(&rData, sizeof(rData));
        sServoData.sServoPower = 1;  // ON
        ret = mpSetServoPower(&sServoData, &rData);
        if ((ret == 0) && (rData.err_no == 0))
        {
            // wait for the Servo On confirmation
            for (checkCount = 0; checkCount < MOTION_START_TIMEOUT; checkCount += MOTION_START_CHECK_PERIOD)
            {
                // Update status
                Ros_Controller_IoStatusUpdate();

                if (Ros_Controller_IsServoOn() == TRUE)
                    break;

                Ros_Sleep(MOTION_START_CHECK_PERIOD);
            }
            if (Ros_Controller_IsServoOn() == FALSE)
            {
                Ros_Debug_BroadcastMsg("Timed out waiting for servo on");
                return MOTION_NOT_READY_SERVO_ON_TIMEOUT;
            }
        }
        else
        {
            //TODO(gavanderhoorn): should this be reported to user, or are causes
            //covered by errors in MotionNotReadyCode?
            snprintf(output, MOTION_START_ERROR_MESSAGE_LENGTH, 
                "%s: Can't turn on servo because: '%s' (0x%04X)",
                Ros_ErrorHandling_MotionNotReadyCode_ToString(MOTION_NOT_READY_ERROR),
                Ros_ErrorHandling_ErrNo_ToString(rData.err_no),
                rData.err_no);
            Ros_Debug_BroadcastMsg(output);
            rosidl_runtime_c__String__assign(responseMessage, output);
            return MOTION_NOT_READY_ERROR;
        }
    }
#endif

    // make sure that there is no data in the queues
    if (Ros_MotionControl_HasDataInQueue())
    {
        Ros_Debug_BroadcastMsg("%s: clearing leftover data in queue", __func__);
        Ros_MotionControl_ClearQ_All();

        if (Ros_MotionControl_HasDataInQueue())
            Ros_Debug_BroadcastMsg("%s: WARNING: still data in queue", __func__);
    }

    // have to initialize the prevPulsePos that will be used when interpolating the traj
    for(grpNo = 0; grpNo < g_Ros_Controller.numGroup; ++grpNo)
    {
        if(g_Ros_Controller.ctrlGroups[grpNo] != NULL)
        {
            Ros_CtrlGroup_GetPulsePosCmd(g_Ros_Controller.ctrlGroups[grpNo], g_Ros_Controller.ctrlGroups[grpNo]->prevPulsePos);
        }
    }

    // Start Job
    bzero(&rData, sizeof(rData));
    bzero(&sStartData, sizeof(sStartData));
    sStartData.sTaskNo = 0;
    strncpy(sStartData.cJobName, g_nodeConfigSettings.inform_job_name, MAX_JOB_NAME_LEN);
    ret = mpStartJob(&sStartData, &rData);
    if( (ret != 0) || (rData.err_no !=0) )
    {
        //TODO(gavanderhoorn): special check for "job is not loaded"
        snprintf(output, MOTION_START_ERROR_MESSAGE_LENGTH, 
            "%s: Can't start '%s' because: '%s' (0x%04X)",
            Ros_ErrorHandling_MotionNotReadyCode_ToString(MOTION_NOT_READY_ERROR),
            g_nodeConfigSettings.inform_job_name,
            Ros_ErrorHandling_ErrNo_ToString(rData.err_no),
            rData.err_no);
        Ros_Debug_BroadcastMsg(output);
        rosidl_runtime_c__String__assign(responseMessage, output);
        return MOTION_NOT_READY_ERROR;
    }

    // wait for the Motion Ready
    for(checkCount = 0; checkCount < MOTION_START_TIMEOUT; checkCount += MOTION_START_CHECK_PERIOD)
    {
        // Update status
        Ros_Controller_IoStatusUpdate();

        if(Ros_Controller_IsMotionReady())
            break;

        Ros_Sleep(MOTION_START_CHECK_PERIOD);
    }

    Ros_Controller_IoStatusUpdate();

    Ros_Debug_BroadcastMsg("%s: exit", __func__);

    //Required to allow motion api to work (Potential race condition)
    Ros_Sleep(200);

    // This is essentially a call to Ros_Controller_IsMotionReady(), but the return code is manually being 
    // checked so that if it fails, then it will return the proper subcode so the message is useful. The 
    // first call to Ros_Controller_GetNotReadySubcode() earlier in this function ignored "tractable problems", 
    // but those should all be fixed by now. If they are not then somehting went wrong. 
    motion_readiness_code = Ros_Controller_GetNotReadySubcode(false);

    if (motion_readiness_code == MOTION_READY)
    {
        //set an indicator of which motion mode is now active
        Ros_MotionControl_ActiveMotionMode = mode;
        Ros_Debug_BroadcastMsg("Ros_MotionControl_ActiveMotionMode = %d", Ros_MotionControl_ActiveMotionMode);

        //This indicates that the next incoming point will be the FIRST point in
        //the queue. As such, it will need to go through an initialization routine
        if (Ros_MotionControl_IsMotionMode_PointQueue())
            Ros_MotionControl_MustInitializePointQueue = TRUE;
    }
    return motion_readiness_code;
}

void Ros_MotionControl_StopTrajMode()
{
    Ros_MotionControl_AllGroupsInitComplete = FALSE;
    Ros_MotionControl_ActiveMotionMode = MOTION_MODE_INACTIVE;

    //Race condition: If HOLD is pressed on the pendant, this output is stays ON. Then when you
    //attempt to re-activate traj_mode, Ros_MotionControl_StartMotionMode blows past the motion-ready
    //checks. But, the init-ros job briefly turns this signal OFF. This triggers Ros_Controller_IoStatusUpdate
    //to detect that motion isn't ready, so it deactivates the traj_mode immediately after it was activated.
    //Turning this signal OFF prevents the issue.
    MP_IO_DATA ioWriteData;
    ioWriteData.ulAddr = g_Ros_Controller.ioStatusAddr[IO_ROBOTSTATUS_WAITING_ROS].ulAddr;
    ioWriteData.ulValue = 0;
    mpWriteIO(&ioWriteData, 1);
}

BOOL Ros_MotionControl_IsMotionMode_Trajectory()
{
    return (Ros_MotionControl_ActiveMotionMode == 
        MOTION_MODE_TRAJECTORY);
}

BOOL Ros_MotionControl_IsMotionMode_PointQueue()
{
    return (Ros_MotionControl_ActiveMotionMode == 
        MOTION_MODE_POINTQUEUE);
}

BOOL Ros_MotionControl_IsMotionMode_RawStreaming()
{
    return FALSE;
    
    //TODO
    // 
    //return (Ros_MotionControl_ActiveMotionMode ==
    //    STREAMING_RAW_INCREMENTS);
}

void Ros_MotionControl_ValidateMotionModeIsOk()
{
    if (!g_messages_RobotStatus.msgRobotStatus->motion_possible.val)
    {
        if (Ros_MotionControl_IsMotionMode_PointQueue())
        {
            //If we are in a point-queue-mode and the motion_possible flag drops,
            //then we need to abort the motion mode and force them to initialize the 
            //mode again. This allows the system to detect if the robot is not starting back up
            //from the expected location. (System could have been estopped during the trajectory,
            //or the user could have switched to TEACH and moved the arm around.)
            Ros_Debug_BroadcastMsg("Stopping point-queue motion mode. Please call '%s' to start a new queue.", SERVICE_NAME_START_POINT_QUEUE_MODE);
            Ros_MotionControl_StopTrajMode();
        }

        //TODO: Determine if this should be done for Trajecotry-Mode too
    }
}
