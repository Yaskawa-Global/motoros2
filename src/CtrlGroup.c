// CtrlGroup.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

const char* Ros_CtrlGroup_GRP_ID_String[] =
{
    "r1",
    "r2",
    "r3",
    "r4",
    "r5",
    "r6",
    "r7",
    "r8",

    "b1",
    "b2",
    "b3",
    "b4",
    "b5",
    "b6",
    "b7",
    "b8",

    "s1",
    "s2",
    "s3",
    "s4",
    "s5",
    "s6",
    "s7",
    "s8",
    "s9",
    "s10",
    "s11",
    "s12",
    "s13",
    "s14",
    "s15",
    "s16",
    "s17",
    "s18",
    "s19",
    "s20",
    "s21",
    "s22",
    "s23",
    "s24",
};

CtrlGroup* Ros_CtrlGroup_Ctor()
{
    return (CtrlGroup*)mpMalloc(sizeof(CtrlGroup));
}

void Ros_CtrlGroup_Dtor(CtrlGroup* grp)
{
    mpFree(grp);
}

//-------------------------------------------------------------------
// Create a CtrlGroup data structure for existing group otherwise
// return NULL
//-------------------------------------------------------------------
CtrlGroup* Ros_CtrlGroup_Create(int groupIndex, BOOL bIsLastGrpToInit, float interpolPeriod)
{
    CtrlGroup* ctrlGroup;
    int numAxes;
    int i;
    long maxSpeedPulse[MP_GRP_AXES_NUM];
    STATUS status;
    BOOL bInitOk;
    BOOL slaveAxis;
    MP_GET_TOOL_NO_RSP_DATA retToolData;

    // Check if group is defined
    numAxes = GP_getNumberOfAxes(groupIndex);
#ifdef DEBUG
    Ros_Debug_BroadcastMsg("Group %d: Num Axes %d", groupIndex, numAxes);
#endif
    if (numAxes > 0)
    {
        bInitOk = TRUE;
        // Allocate and initialize memory
        ctrlGroup = (CtrlGroup*)mpMalloc(sizeof(CtrlGroup));
        bzero(ctrlGroup, sizeof(CtrlGroup));

        // Populate values
        ctrlGroup->groupNo = groupIndex;
        ctrlGroup->numAxes = numAxes;
        ctrlGroup->groupId = Ros_mpCtrlGrpNo2GrpId(groupIndex);

        if (Ros_CtrlGroup_IsRobot(ctrlGroup))
        {
            mpGetToolNo(ctrlGroup->groupId, &retToolData);
            //TODO: need to update this value when selected tool changes
            ctrlGroup->tool = retToolData.sToolNo;

            int baseIdOffset = (int)ctrlGroup->groupId - (int)MP_R1_GID;
            ctrlGroup->baseTrackGroupIndex = mpCtrlGrpId2GrpNo((MP_GRP_ID_TYPE)baseIdOffset + MP_B1_GID);
            if (Ros_CtrlGroup_HasBaseTrack(ctrlGroup))
            {
                ctrlGroup->baseTrackGroupId = (MP_GRP_ID_TYPE)(baseIdOffset + MP_B1_GID);
                GP_getBaseAxisInfo(ctrlGroup->baseTrackGroupIndex, &ctrlGroup->baseTrackInfo);
            }
            else
                ctrlGroup->baseTrackGroupId = (MP_GRP_ID_TYPE)-1;
        }
        else
        {
            ctrlGroup->tool = 0;
            ctrlGroup->baseTrackGroupId = (MP_GRP_ID_TYPE)-1;
            ctrlGroup->baseTrackGroupIndex = -1;
        }

        status = GP_getAxisMotionType(groupIndex, &ctrlGroup->axisType);
        if (status != OK)
            bInitOk = FALSE;

        status = GP_getPulseToRad(groupIndex, &ctrlGroup->pulseToRad);
        if (status != OK)
            bInitOk = FALSE;

        status = GP_getPulseToMeter(groupIndex, &ctrlGroup->pulseToMeter);
        if (status != OK)
            bInitOk = FALSE;

        status = GP_getFBPulseCorrection(groupIndex, &ctrlGroup->correctionData);
        if(status!=OK)
            bInitOk = FALSE;

        status = GP_getMaxIncPerIpCycle(groupIndex, interpolPeriod, &ctrlGroup->maxInc);
        if (status != OK)
            bInitOk = FALSE;

        status = GP_isBaxisSlave(groupIndex, &slaveAxis);
        if (status != OK)
            bInitOk = FALSE;

        status = GP_getFeedbackSpeedMRegisterAddresses(groupIndex, //zero based index of the control group
                                                        TRUE, //If the register-speed-feedback is not enabled, automatically modify the SC.PRM file to enable this feature.
                                                        bIsLastGrpToInit, //If activating the reg-speed-feedback feature, delay the alarm until all the groups have been processed.
                                                        &ctrlGroup->speedFeedbackRegisterAddress); //[OUT] Index of the M registers containing the feedback speed values.
        if (status != OK)
        {
            ctrlGroup->speedFeedbackRegisterAddress.bFeedbackSpeedEnabled = FALSE;
        }

        ctrlGroup->bIsBaxisSlave = (numAxes == 5) && slaveAxis;

        //adjust the axisType field to account for robots with non-contiguous axes (such as delta or palletizing which use SLU--T axes)
        for (i = 0; i < MP_GRP_AXES_NUM; i += 1)
        {
            if (ctrlGroup->maxInc.maxIncrement[i] == 0) //if the axis can't move, then I assume it's invalid
                ctrlGroup->axisType.type[i] = AXIS_INVALID;
        }

        bzero(&ctrlGroup->inc_q, sizeof(Incremental_q));
        ctrlGroup->inc_q.q_lock = mpSemBCreate(SEM_Q_FIFO, SEM_FULL);

        // Calculate maximum speed in radian per second
        bzero(maxSpeedPulse, sizeof(maxSpeedPulse));
        for(i=0; i<MP_GRP_AXES_NUM; i++)
            maxSpeedPulse[i] = ctrlGroup->maxInc.maxIncrement[i] * 1000.0 / interpolPeriod;
        Ros_CtrlGroup_ConvertToRosPos(ctrlGroup, maxSpeedPulse, ctrlGroup->maxSpeed);

        char startupMessage[256];
        sprintf(startupMessage, "axisType[%d]: ", groupIndex);
        for (i = 0; i < MP_GRP_AXES_NUM; i++)
        {
            if (ctrlGroup->axisType.type[i] == AXIS_LINEAR)
                strcat(startupMessage, "Lin\t");
            else if (ctrlGroup->axisType.type[i] == AXIS_ROTATION)
                strcat(startupMessage, "Rot\t");
            else
            {
                strcat(startupMessage, "---\t");
            }
        }
        Ros_Debug_BroadcastMsg(startupMessage);

        sprintf(startupMessage, "pulse->unit[%d]: ", groupIndex);
        for (i = 0; i < MP_GRP_AXES_NUM; i++)
        {
            if (ctrlGroup->axisType.type[i] == AXIS_LINEAR)
                sprintf(startupMessage, "%s%.4f\t", startupMessage, ctrlGroup->pulseToMeter.PtoM[i]);
            else if (ctrlGroup->axisType.type[i] == AXIS_ROTATION)
                sprintf(startupMessage, "%s%.4f\t", startupMessage, ctrlGroup->pulseToRad.PtoR[i]);
            else
                strcat(startupMessage, "--\t");
        }
        Ros_Debug_BroadcastMsg(startupMessage);

        Ros_Debug_BroadcastMsg("maxInc[%d] (in motoman joint order): %d, %d, %d, %d, %d, %d, %d, %d",
            groupIndex,
            ctrlGroup->maxInc.maxIncrement[0],ctrlGroup->maxInc.maxIncrement[1],ctrlGroup->maxInc.maxIncrement[2],
            ctrlGroup->maxInc.maxIncrement[3],ctrlGroup->maxInc.maxIncrement[4],ctrlGroup->maxInc.maxIncrement[5],
            ctrlGroup->maxInc.maxIncrement[6],ctrlGroup->maxInc.maxIncrement[7]);

        Ros_Debug_BroadcastMsg("maxSpeed[%d] (in ros joint order): %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
            groupIndex,
            ctrlGroup->maxSpeed[0],ctrlGroup->maxSpeed[1],ctrlGroup->maxSpeed[2],
            ctrlGroup->maxSpeed[3],ctrlGroup->maxSpeed[4],ctrlGroup->maxSpeed[5],
            ctrlGroup->maxSpeed[6],ctrlGroup->maxSpeed[7]);

        //----------------------------------------------------------------
        if(bInitOk == FALSE)
        {
            mpFree(ctrlGroup);
            ctrlGroup = NULL;
        }
    }
    else
    {
        ctrlGroup = NULL;
    }

    if (ctrlGroup)
    {
        ctrlGroup->hasDataToProcess = FALSE;

        Ros_Debug_BroadcastMsg("Creating new task: Add To Inc Q (Group %d)", groupIndex + 1);

        ctrlGroup->tidAddToIncQueue = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE,
                                                    (FUNCPTR)Ros_MotionControl_AddToIncQueueProcess,
                                                    (int)ctrlGroup, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        if (ctrlGroup->tidAddToIncQueue == ERROR)
        {
            Ros_Debug_BroadcastMsg("Failed to create task for interpolating increments.  Check robot parameters.");
            ctrlGroup->tidAddToIncQueue = INVALID_TASK;
            Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
            mpSetAlarm(ALARM_TASK_CREATE_FAIL, APPLICATION_NAME " FAILED TO CREATE TASK", SUBCODE_ADD_TO_INC_Q);

            ctrlGroup = NULL;
        }
    }

    return ctrlGroup;
}

void Ros_CtrlGrp_Cleanup(CtrlGroup* ctrlGroup)
{
    mpDeleteTask(ctrlGroup->tidAddToIncQueue);
    ctrlGroup->tidAddToIncQueue = INVALID_TASK;

    mpSemDelete(ctrlGroup->inc_q.q_lock);
}


//-------------------------------------------------------------------
// Search through the control group to find the GroupId that matches
// the group number
//-------------------------------------------------------------------
MP_GRP_ID_TYPE Ros_mpCtrlGrpNo2GrpId(int groupNo)
{
#if defined (YRC1000) || defined (YRC1000u)
    return mpCtrlGrpNo2GrpId(groupNo);

#elif defined (FS100) || defined (DX200)
    MP_GRP_ID_TYPE grp_id;

    for(grp_id = MP_R1_GID; grp_id < MP_S24_GID; ++grp_id)
    {
        if(groupNo == mpCtrlGrpId2GrpNo(grp_id))
            return grp_id;
    }

    return -1;

#else
#error "Ros_mpCtrlGrpNo2GrpId: unsupported platform"

#endif
}


//-------------------------------------------------------------------
// Get the commanded pulse position in pulse (in motoman joint order)
// Used for MOTION SERVER connection for positional planning calculations.
//-------------------------------------------------------------------
BOOL Ros_CtrlGroup_GetPulsePosCmd(CtrlGroup* ctrlGroup, long pulsePos[MAX_PULSE_AXES])
{
    LONG status = 0;
    MP_CTRL_GRP_SEND_DATA sData;
    MP_PULSE_POS_RSP_DATA pulse_data;
    int i;

    bzero(pulsePos, MAX_PULSE_AXES*sizeof(long));  // clear result, in case of error

    // Set the control group
    sData.sCtrlGrp = ctrlGroup->groupId;

    // get the command joint positions
    status = mpGetPulsePos (&sData,&pulse_data);
    if (0 != status)
    {
        Ros_Debug_BroadcastMsg("Failed to get pulse position (command): %u", status);
        return FALSE;
    }

    // assign return value
    for (i=0; i<MAX_PULSE_AXES; ++i)
        pulsePos[i] = pulse_data.lPos[i];

    // For MPL80/100 robot type (SLUBT): Controller automatically moves the B-axis
    // to maintain orientation as other axes are moved.
    if (ctrlGroup->bIsBaxisSlave)
    {
        //temporary storage for B axis compensation
        double rosAnglePos[MP_GRP_AXES_NUM];

        //B axis compensation works on the ROS ANGLE positions, not on MOTO PULSE positions
        Ros_CtrlGroup_ConvertToRosPos(ctrlGroup, pulsePos, rosAnglePos);
        rosAnglePos[3] += -rosAnglePos[1] + rosAnglePos[2];
        Ros_CtrlGroup_ConvertToMotoPos_FromSequentialOrdering(ctrlGroup, rosAnglePos, pulsePos);
    }

    return TRUE;
}


//-------------------------------------------------------------------
// Get the corrected feedback pulse position in pulse.
// Used exclusively for STATE SERVER connection to report position.
//-------------------------------------------------------------------
BOOL Ros_CtrlGroup_GetFBPulsePos(CtrlGroup* ctrlGroup, long pulsePos[MAX_PULSE_AXES])
{
    MP_CTRL_GRP_SEND_DATA sData;
#ifndef DUMMY_SERVO_MODE
    MP_FB_PULSE_POS_RSP_DATA pulse_data;
#else
    MP_PULSE_POS_RSP_DATA pulse_data;
#endif
    int i;

    bzero(pulsePos, MAX_PULSE_AXES*sizeof(long));  // clear result, in case of error

    // Set the control group
    sData.sCtrlGrp = ctrlGroup->groupId;

#ifndef DUMMY_SERVO_MODE
    // get raw (uncorrected/unscaled) joint positions
    LONG status = mpGetFBPulsePos (&sData,&pulse_data);

    //TODO: Consider using mpGetFBPulsePosEx. The `ex` version automatically applies
    //      any needed corrections, such as gravity compensation and cross-axis
    //      coupling. We're already (manually) applying those corrections, so we don't
    //      need the `ex` version. But if the next controller generation adds some new
    //      feature, then we should transition so that we don't have to worry about it.
    //      See also yaskawa-global/motoros2#199.

    if (0 != status)
    {
        Ros_Debug_BroadcastMsg("Failed to get pulse feedback position: %u", status);
        return FALSE;
    }

    // apply correction to account for cross-axis coupling
    // Note: this is only required for feedback position
    // controller handles this correction internally when
    // dealing with command positon.
    for (i=0; i<MAX_PULSE_AXES; ++i)
    {
        FB_AXIS_CORRECTION *corr = &ctrlGroup->correctionData.correction[i];
        if (corr->bValid)
        {
            int src_axis = corr->ulSourceAxis;
            int dest_axis = corr->ulCorrectionAxis;
            pulse_data.lPos[dest_axis] -= (int)(pulse_data.lPos[src_axis] * corr->fCorrectionRatio);
        }
    }
#else
    mpGetPulsePos(&sData, &pulse_data);
#endif

    // assign return value
    for (i=0; i<MAX_PULSE_AXES; ++i)
        pulsePos[i] = pulse_data.lPos[i];

    //--------------------------------------------------------------------
    //NOTE: Do NOT apply any B axis compensation here.
    //      This is actual feedback which is reported to the state server.
    //--------------------------------------------------------------------

    return TRUE;
}

//-------------------------------------------------------------------
// Get the corrected feedback pulse speed in pulse for each axis.
//-------------------------------------------------------------------
BOOL Ros_CtrlGroup_GetFBServoSpeed(CtrlGroup* ctrlGroup, long pulseSpeed[MAX_PULSE_AXES])
{
    int i;

#ifndef DUMMY_SERVO_MODE
    LONG status;
    MP_IO_INFO registerInfo[MAX_PULSE_AXES * 2]; //values are 4 bytes, which consumes 2 registers
    USHORT registerValues[MAX_PULSE_AXES * 2];
    UINT32 registerValuesLong[MAX_PULSE_AXES * 2];

    bzero(pulseSpeed, sizeof(long[MAX_PULSE_AXES]));

    if (!ctrlGroup->speedFeedbackRegisterAddress.bFeedbackSpeedEnabled)
        return FALSE;

    for (i = 0; i < MAX_PULSE_AXES; i += 1)
    {
        registerInfo[i * 2].ulAddr = ctrlGroup->speedFeedbackRegisterAddress.cioAddressForAxis[i][0];
        registerInfo[(i * 2) + 1].ulAddr = ctrlGroup->speedFeedbackRegisterAddress.cioAddressForAxis[i][1];
    }

    // get raw (uncorrected/unscaled) joint speeds
    status = mpReadIO(registerInfo, registerValues, MAX_PULSE_AXES * 2);
    if (status != OK)
    {
        Ros_Debug_BroadcastMsg("Failed to get pulse feedback speed: %u", status);
        return FALSE;
    }

    for (i = 0; i < MAX_PULSE_AXES; i += 1)
    {
        //move to 32 bit storage
        registerValuesLong[i * 2] = registerValues[i * 2];
        registerValuesLong[(i * 2) + 1] = registerValues[(i * 2) + 1];

        //combine both registers into single 4 byte value (0.0001 deg/sec or 1 um/sec)
        double dblRegister = (registerValuesLong[(i * 2) + 1] << 16) | registerValuesLong[i * 2];

        //convert to pulse/sec
        if (ctrlGroup->axisType.type[i] == AXIS_ROTATION)
        {
            dblRegister /= 1.0E4; //deg/sec
            dblRegister *= RAD_PER_DEGREE; //rad/sec
            dblRegister *= ctrlGroup->pulseToRad.PtoR[i]; //pulse/sec
        }
        else if (ctrlGroup->axisType.type[i] == AXIS_LINEAR)
        {
            dblRegister /= 1.0E6; //m/sec
            dblRegister *= ctrlGroup->pulseToMeter.PtoM[i]; //pulse/sec
        }

        pulseSpeed[i] = (long)dblRegister;
    }

#else //dummy-servo mode for testing
    MP_CTRL_GRP_SEND_DATA sData;
    MP_SERVO_SPEED_RSP_DATA pulse_data;

    mpGetServoSpeed(&sData, &pulse_data);

    // assign return value
    for (i = 0; i<MAX_PULSE_AXES; ++i)
        pulseSpeed[i] = pulse_data.lSpeed[i];
#endif

    return TRUE;
}

//-------------------------------------------------------------------
// Retrieves the absolute value (Nm) of the maximum current servo torque.
//-------------------------------------------------------------------
BOOL Ros_CtrlGroup_GetTorque(CtrlGroup* ctrlGroup, double torqueValues[MAX_PULSE_AXES])
{
    MP_GRP_AXES_T dst_vel;
    MP_TRQ_CTL_VAL dst_trq;
    LONG status = 0;
    int i;

    bzero(torqueValues, sizeof(double [MAX_PULSE_AXES])); // clear result, in case of error
    bzero(dst_trq.data, sizeof(MP_TRQCTL_DATA));
    dst_trq.unit = TRQ_NEWTON_METER; //request data in Nm

    bzero(&dst_vel, sizeof(MP_GRP_AXES_T));

    status = mpSvsGetVelTrqFb(dst_vel, &dst_trq);
    if (status != OK)
        return FALSE;

    for (i = 0; i < MAX_PULSE_AXES; i += 1)
    {
        torqueValues[i] = (double)dst_trq.data[ctrlGroup->groupNo][i] * 0.000001; //Use double.  Float only good for 6 sig digits.
    }

    return TRUE;
}

//-------------------------------------------------------------------
// Retrieves the temperatures of all encoders in the specified group (integer, degrees C)
//-------------------------------------------------------------------
BOOL Ros_CtrlGroup_GetEncoderTemperature(CtrlGroup const* const ctrlGroup, long encoderTemp[MAX_PULSE_AXES])
{
    MP_CTRL_GRP_SEND_DATA sData;
    MP_ENCODER_TEMP_RSP_DATA rData;
    LONG status = 0;

    bzero(&sData, sizeof(sData));
    bzero(&rData, sizeof(rData));

    sData.sCtrlGrp = ctrlGroup->groupId;

    // get encoder temperatures
    status = mpGetEncoderTemp(&sData, &rData);
    if (0 != status)
    {
        Ros_Debug_BroadcastMsg("Failed to get encoder temperature: %ld", status);
        return FALSE;
    }

    memcpy(encoderTemp, rData.lTemp, MAX_PULSE_AXES * sizeof(long));

    return TRUE;
}

//Convert the Motoman position units (pulses) to ROS position units (radians/meters).
//This function must be called BEFORE calling Ros_CtrlGroup_ConvertMotoJointOrderToSequentialJointOrder.
//The joints must be in Motoman (non-sequential) ordering.
void Ros_CtrlGroup_ConvertMotoUnitsToRosUnits(CtrlGroup* ctrlGroup, long const motopulsePos[MAX_PULSE_AXES], double rosPos[MAX_PULSE_AXES])
{
    int i;
    double conversion = 1;

    bzero(rosPos, sizeof(double) * MAX_PULSE_AXES);

    //Delta: (SLU--T-) All rotary axes
    //Scara: (SLUR---) U-axis is linear
    //Large Palletizing: (SLU--T-) All rotary axes
    //High Speed Picking: (SLU-BT-) All rotary axes
    for (i = 0; i < MAX_PULSE_AXES; i += 1)
    {
        if (Ros_CtrlGroup_IsInvalidAxis(ctrlGroup, i))
        {
            continue;
        }

        if (ctrlGroup->axisType.type[i] == AXIS_ROTATION)
            conversion = ctrlGroup->pulseToRad.PtoR[i];
        else if (ctrlGroup->axisType.type[i] == AXIS_LINEAR)
            conversion = ctrlGroup->pulseToMeter.PtoM[i];
        else
            conversion = 1.0;

        rosPos[i] = motopulsePos[i] / conversion;
    }

}

void Ros_CtrlGroup_ConvertMotoJointOrderToSequentialJointOrder(CtrlGroup* ctrlGroup, double const motoPos[MAX_PULSE_AXES], double rosPos[MAX_PULSE_AXES])
{
    int i;

    if ((ctrlGroup->numAxes == 7) && Ros_CtrlGroup_IsRobot(ctrlGroup)) //is robot, and is 7 axis
    {
        // Adjust joint order for 7 axis robot (SLURBTE > SLEURBT); All rotary axes

        for (i = 0; i < ctrlGroup->numAxes; i++)
        {
            if (i < 2)
                rosPos[i] = motoPos[i];
            else if (i == 2)
                rosPos[2] = motoPos[6];
            else
                rosPos[i] = motoPos[i - 1];
        }
    }
    else if (Ros_CtrlGroup_IsRobot(ctrlGroup) && ctrlGroup->numAxes < 6)
    {
        //Delta: (SLU--T- > SLUT---) All rotary axes
        //Scara: (SLUR--- > SLUR---) U-axis is linear
        //Large Palletizing: (SLU--T- > SLUT---) All rotary axes
        //High Speed Picking: (SLU-BT- > SLUBT--) All rotary axes

        int rpi = 0; //rosPos index
        int mpi = 0; //motopos index

        for (i = 0; i < ctrlGroup->numAxes; i += 1, rpi += 1, mpi += 1)
        {
            while (Ros_CtrlGroup_IsInvalidAxis(ctrlGroup, mpi))
            {
                mpi += 1;
                if (mpi >= MAX_PULSE_AXES)
                    return;
            }

            rosPos[rpi] = motoPos[mpi];
        }
    }
    else
    {
        for (i = 0; i < MAX_PULSE_AXES; i++)
        {
            rosPos[i] = motoPos[i];
        }
    }
}

// Convert Motoman position in pulse to Ros position in radian/meters
// In the case of a 7, 4, or 5 axis robot, adjust the order to match
// the physical axis sequence
//-------------------------------------------------------------------
void Ros_CtrlGroup_ConvertToRosPos(CtrlGroup* ctrlGroup, long const motopulsePos[MAX_PULSE_AXES], double rosPos[MAX_PULSE_AXES])
{
    double rosUnitsWithMotoOrder[MAX_PULSE_AXES];

    //call this first, due to expected joint ordering 
    Ros_CtrlGroup_ConvertMotoUnitsToRosUnits(ctrlGroup, motopulsePos, rosUnitsWithMotoOrder);

    Ros_CtrlGroup_ConvertMotoJointOrderToSequentialJointOrder(ctrlGroup, rosUnitsWithMotoOrder, rosPos);
}

// Convert Motoman torque to ROS torque by re-ordering the joints
//-------------------------------------------------------------------
void Ros_CtrlGroup_ConvertToRosTorque(CtrlGroup* ctrlGroup, double const motoTorque[MAX_PULSE_AXES], double rosTorque[MAX_PULSE_AXES])
{
    Ros_CtrlGroup_ConvertMotoJointOrderToSequentialJointOrder(ctrlGroup, motoTorque, rosTorque);
}

//Convert the ROS position units (radians/meters) to Motoman position units (pulses).
//This function must be called AFTER calling Ros_CtrlGroup_ConvertSequentialJointOrderToMotoJointOrder.
//The joints must be in Motoman (non-sequential) ordering.
void Ros_CtrlGroup_ConvertRosUnitsToMotoUnits(CtrlGroup* ctrlGroup, double const rosPos[MAX_PULSE_AXES], long motopulsePos[MAX_PULSE_AXES])
{
    double conversion = 1;

    bzero(motopulsePos, sizeof(long) * MAX_PULSE_AXES);

    //Delta: (SLU--T-) All rotary axes
    //Scara: (SLUR---) U-axis is linear
    //Large Palletizing: (SLU--T-) All rotary axes
    //High Speed Picking: (SLU-BT-) All rotary axes
    for (int i = 0; i < MAX_PULSE_AXES; i += 1)
    {
        if (Ros_CtrlGroup_IsInvalidAxis(ctrlGroup, i))
        {
            continue;
        }

        if (ctrlGroup->axisType.type[i] == AXIS_ROTATION)
            conversion = ctrlGroup->pulseToRad.PtoR[i];
        else if (ctrlGroup->axisType.type[i] == AXIS_LINEAR)
            conversion = ctrlGroup->pulseToMeter.PtoM[i];
        else
            conversion = 1.0;

        motopulsePos[i] = (int)(rosPos[i] * conversion);
    }
}

void Ros_CtrlGroup_ConvertSequentialJointOrderToMotoJointOrder(CtrlGroup* ctrlGroup, double const rosPos[MAX_PULSE_AXES], double motoPos[MAX_PULSE_AXES])
{
    int i;

    // Initialize memory space
    bzero(motoPos, sizeof(double) * MAX_PULSE_AXES);

    if ((ctrlGroup->numAxes == 7) && Ros_CtrlGroup_IsRobot(ctrlGroup))
    {
        // Adjust joint order for 7 axis robot (SLEURBT > SLURBTE); All rotary axes

        for (i = 0; i < ctrlGroup->numAxes; i++)
        {
            if (i < 2)
                motoPos[i] = rosPos[i];
            else if (i == 2)
                motoPos[6] = rosPos[2];
            else
                motoPos[i - 1] = rosPos[i];
        }
    }
    else if (Ros_CtrlGroup_IsRobot(ctrlGroup) && ctrlGroup->numAxes < 6)
    {
        //Delta: (SLUT--- > SLU--T-) All rotary axes
        //Scara: (SLUR--- > SLUR---) U-axis is linear
        //Large Palletizing: (SLUT--- > SLU--T-) All rotary axes
        //High Speed Picking: (SLUBT-- > SLU-BT-) All rotary axes

        int rpi = 0; //radpos index
        int mpi = 0; //motopos index

        for (i = 0; i < ctrlGroup->numAxes; i += 1, rpi += 1, mpi += 1)
        {
            while (Ros_CtrlGroup_IsInvalidAxis(ctrlGroup, mpi))
            {
                mpi += 1;
                if (mpi >= MAX_PULSE_AXES)
                    return;
            }

            motoPos[mpi] = rosPos[rpi];
        }
    }
    else
    {
        for (i = 0; i < MAX_PULSE_AXES; i++)
        {
            motoPos[i] = rosPos[i];
        }
    }
}

//-------------------------------------------------------------------
// Convert Ros position in radian to Motoman position in pulse
// In the case of a 7 or 4 axis robot, adjust the order to match
// the motoman axis sequence
//-------------------------------------------------------------------
void Ros_CtrlGroup_ConvertToMotoPos_FromSequentialOrdering(CtrlGroup* ctrlGroup, double const radPos[MAX_PULSE_AXES], long motopulsePos[MAX_PULSE_AXES])
{
    double rosUnitsWithMotoOrder[MAX_PULSE_AXES];

    Ros_CtrlGroup_ConvertSequentialJointOrderToMotoJointOrder(ctrlGroup, radPos, rosUnitsWithMotoOrder);

    //must call this after Ros_CtrlGroup_ConvertSequentialJointOrderToMotoJointOrder due to expected joint ordering
    Ros_CtrlGroup_ConvertRosUnitsToMotoUnits(ctrlGroup, rosUnitsWithMotoOrder, motopulsePos);
}

//-------------------------------------------------------------------
// Returns a bit wise axis configuration for the increment move API
//-------------------------------------------------------------------
UCHAR Ros_CtrlGroup_GetAxisConfig(CtrlGroup* ctrlGroup)
{
    int i;
    int axisConfig = 0;

    for (i = 0; i < MAX_PULSE_AXES; i++)
    {
        if (!Ros_CtrlGroup_IsInvalidAxis(ctrlGroup, i))
            axisConfig |= (0x01 << i);
    }

    return (UCHAR)axisConfig;
}

//-------------------------------------------------------------------
// Returns TRUE is the specified group is defined as a robot
//-------------------------------------------------------------------
BOOL Ros_CtrlGroup_IsRobot(CtrlGroup* ctrlGroup)
{
    return((ctrlGroup->groupId >= MP_R1_GID) && (ctrlGroup->groupId <= MP_R8_GID));
}

//-------------------------------------------------------------------
// Returns TRUE if the specified group is defined as a base
//-------------------------------------------------------------------
BOOL Ros_CtrlGroup_IsBase(CtrlGroup const* const ctrlGroup)
{
    return ((ctrlGroup->groupId >= MP_B1_GID) && (ctrlGroup->groupId <= MP_B8_GID));
}

//-------------------------------------------------------------------
// Returns TRUE if the specified group is defined as a station
//-------------------------------------------------------------------
BOOL Ros_CtrlGroup_IsStation(CtrlGroup const* const ctrlGroup)
{
    return ((ctrlGroup->groupId >= MP_S1_GID) && (ctrlGroup->groupId <= MP_S24_GID));
}

//-------------------------------------------------------------------
// Returns TRUE if the specified axis in the provided group is an
// invalid one (ie: set to AXIS_INVALID)
//-------------------------------------------------------------------
BOOL Ros_CtrlGroup_IsInvalidAxis(CtrlGroup const* const ctrlGroup, size_t axisIdx)
{
    return ctrlGroup->axisType.type[axisIdx] == AXIS_INVALID;
}

// Returns TRUE if the specified group has an associated base track
//-------------------------------------------------------------------
BOOL Ros_CtrlGroup_HasBaseTrack(CtrlGroup const* ctrlGroup)
{
    //according to the comment on CtrlGroup.h::CtrlGroup::baseTrackGroupIndex,
    //checking for this to be != -1 should be sufficient to determine whether
    //or not this group has a base track configured or not
    return (ctrlGroup->baseTrackGroupIndex != -1);
}

//-------------------------------------------------------------------
// Store the user-defined joint names in the CtrlGroup object using
// "motoman" order. This will be used as a lookup table when receiving
// a trajectory request. 
//-------------------------------------------------------------------
void Ros_CtrlGroup_UpdateJointNamesInMotoOrder(CtrlGroup* ctrlGroup)
{
    bzero(ctrlGroup->jointNames_userDefined, sizeof(ctrlGroup->jointNames_userDefined));

    //Iterate over the axis configuration to ensure we account for any non-sequential gaps
    //  6-axis: (SLURBT--)
    //  7-axis: (SLURBTE-)
    //  Delta: (SLU--T--)
    //  Scara: (SLUR----)
    //  Large Palletizing: (SLU--T--)
    //  High Speed Picking: (SLU-BT--)
    for (int axisIndex = 0, configIndex = 0; axisIndex < MP_GRP_AXES_NUM; axisIndex += 1)
    {
        if (!Ros_CtrlGroup_IsInvalidAxis(ctrlGroup, axisIndex))
        {
            int customNameIndex = (ctrlGroup->groupNo * MP_GRP_AXES_NUM) + configIndex;

            //while we're here, check a non-empty name was configured -- this could be
            //a sign of misconfigured custom joint names (fi: a missing entry).
            //We couldn't check for this earlier, as the config parser is driven by
            //content of the configuration file. Entries which are not present are simply
            //not parsed, while here we are trying to match configuration entries to
            //controller groups and axes.
            if (0 == Ros_strnlen(g_nodeConfigSettings.joint_names[customNameIndex], MAX_JOINT_NAME_LENGTH))
            {
                //Raise two alarms so we can post a clearer error msg
                char errMsg[ERROR_MSG_MAX_SIZE];
                //Note: prints axis index 1-based
                snprintf(errMsg, ERROR_MSG_MAX_SIZE, "group: %s, axis: %d",
                    Ros_CtrlGroup_GRP_ID_String[ctrlGroup->groupId], axisIndex + 1);
                mpSetAlarm(ALARM_CONFIGURATION_FAIL, errMsg,
                    SUBCODE_CONFIGURATION_INVALID_CUSTOM_JOINT_NAME);
                motoRosAssert_withMsg(false, SUBCODE_CONFIGURATION_EMPTY_JOINT_NAME,
                    "Empty custom joint name");
            }

            strncpy(ctrlGroup->jointNames_userDefined[axisIndex],
                g_nodeConfigSettings.joint_names[customNameIndex],
                MAX_JOINT_NAME_LENGTH);

            configIndex += 1;
        }
    }
}
