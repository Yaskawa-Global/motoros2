//RealTimeMotionControl.c

// SPDX-FileCopyrightText: 2025, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2025, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

// Based loosely on @adv4ncr's modifications to MotoROS1 for real-time control through ROS2.
// https://github.com/adv4ncr/motoman_ROS2/blob/cdf63a592596ff711df842680a1e4c730fd547a7/controller_driver/RealTimeMotionServer.c

#include "MotoROS.h"

void Ros_RtMotionControl_InitJointSpace(MP_EXPOS_DATA* moveData);
void Ros_RtMotionControl_InitCartesian(MP_EXPOS_DATA* moveData);
bool Ros_RtMotionControl_ParseJointSpace(RtPacket* incomingCommand, MP_EXPOS_DATA* moveData);
bool Ros_RtMotionControl_ParseCartesian(RtPacket* incomingCommand, MP_EXPOS_DATA* moveData);
void Ros_RtMotionControl_Cleanup();
void Ros_RtMotionControl_PopulateReplyMessage(MOTION_MODE mode, RtPacket* command, RtReply* reply);
bool Ros_CheckForFsuInterference(MOTION_MODE mode, int* tools);

static int sockRtCommandListener = -1;

static LONG prevRtCmdPosition[MAX_GROUPS][MAX_AXES];
static LONG howMuchShouldIHaveMoved[MAX_GROUPS][MAX_AXES];

void Ros_RtMotionControl_HyperRobotCommanderX5(MOTION_MODE mode)
{
    MP_EXPOS_DATA moveData;
    int bytes_received;

    bool bFirstRecv = true;
    struct sockaddr_in client_addr;
    struct sockaddr_in previous_client_addr;
    int client_addr_len = sizeof(client_addr);

    RtPacket incomingCommand;
    RtReply outgoingReply;

    UINT32 previousSequenceId = 0;

    struct fd_set fds;
    struct timeval tv;
    struct timeval* timeout;

    bool fsuLimitingDetected;

    //=========================================================================================

    bzero(prevRtCmdPosition, MAX_GROUPS * MAX_AXES * sizeof(LONG));

    if (mode == MOTION_MODE_RT_JOINT)
        Ros_RtMotionControl_InitJointSpace(&moveData);
    else
        Ros_RtMotionControl_InitCartesian(&moveData);

    Ros_Debug_BroadcastMsg("Starting RT session");

    Ros_Debug_BroadcastMsg("Flushing stale packets from socket buffer...");
    //----------------------------
    //mpIoctl(sockRtCommandListener, FIOFLUSH, 1);
    //UPDATE: mpIoctl isn't working! We'll manually purge the buffer with a draining loop.
    //----------------------------
    while (TRUE)
    {
        FD_ZERO(&fds);
        FD_SET(sockRtCommandListener, &fds);

        //no wait
        tv.tv_usec = 0;
        tv.tv_sec = 0;

        if (mpSelect(sockRtCommandListener + 1, &fds, NULL, NULL, &tv) > 0)
        {
            mpRecvFrom(sockRtCommandListener, (char*)&incomingCommand, sizeof(RtPacket), 0, (struct sockaddr*)&client_addr, &client_addr_len);
        }
        else
            break;
    }
    //----------------------------

    //=========================================================================================
    while (TRUE)
    {
        FD_ZERO(&fds);
        FD_SET(sockRtCommandListener, &fds);

        tv.tv_usec = (g_nodeConfigSettings.timeout_for_rt_msg % 1000) * 1000;
        tv.tv_sec = g_nodeConfigSettings.timeout_for_rt_msg / 1000;

        if (g_nodeConfigSettings.timeout_for_rt_msg != -1)
            timeout = &tv;
        else
            timeout = NULL;

        if (mpSelect(sockRtCommandListener + 1, &fds, NULL, NULL, timeout) > 0)
        {
            bzero(&incomingCommand, sizeof(incomingCommand));
            bytes_received = mpRecvFrom(sockRtCommandListener, (char*)&incomingCommand, sizeof(RtPacket), 0, (struct sockaddr*)&client_addr, &client_addr_len);

            if (bFirstRecv)
            {
                previous_client_addr = client_addr; //only allow a single commander
                //flag is cleared down below
            }
            else
            {
                if (memcmp(&client_addr.sin_addr.s_addr, &previous_client_addr.sin_addr.s_addr, sizeof(UINT32)) != 0)
                {
                    Ros_Debug_BroadcastMsg("ERROR: Received command packets from multiple sources (0x%08X and 0x%08X)",
                                            (UINT32)previous_client_addr.sin_addr.s_addr,
                                            (UINT32)client_addr.sin_addr.s_addr);
                    break; //drop the connection
                }
            }

            if (bytes_received > 0)
            {
                #warning deal with rollover;
                if (incomingCommand.sequenceId <= previousSequenceId && !bFirstRecv)
                {
                    Ros_Debug_BroadcastMsg("WARN: Received old command packet (seq: %d, new: %d)", previousSequenceId, incomingCommand.sequenceId);

                    //send a copy of the previous reply to trigger next packet
                    mpSendTo(sockRtCommandListener, (char*)&outgoingReply, sizeof(RtReply), 0, (struct sockaddr*)&client_addr, client_addr_len);
                    continue; //drop this packet
                }

                if ((incomingCommand.sequenceId - previousSequenceId) > g_nodeConfigSettings.max_sequence_diff_for_rt_msg)
                {
                    Ros_Debug_BroadcastMsg("ERROR: Missed too many command packets (seq: %d, new: %d)", previousSequenceId, incomingCommand.sequenceId);
                    break; //drop the connection
                }

                if (mode == MOTION_MODE_RT_JOINT)
                {
                    if (!Ros_RtMotionControl_ParseJointSpace(&incomingCommand, &moveData))
                        break; //drop the connection
                }
                else
                {
                    if (!Ros_RtMotionControl_ParseCartesian(&incomingCommand, &moveData))
                        break; //drop the connection
                }

                fsuLimitingDetected = Ros_CheckForFsuInterference(mode, incomingCommand.toolIndex);

                // Send increment to robot
                int ret = mpExRcsIncrementMove(&moveData);
                if (ret != OK)
                {
                    Ros_Debug_BroadcastMsg("WARN: mpExRcsIncrementMove returned %d", ret);
                    break; //drop the connection
                }

                bFirstRecv = false;
            }
            else
            {
                Ros_Debug_BroadcastMsg("ERROR: recvFrom returned an error");
                break;
            }

            // Wait for next interpolation cycle
            mpClkAnnounce(MP_INTERPOLATION_CLK);

            //send status back to the PC and notify it that I'm ready for another packet
            previousSequenceId = incomingCommand.sequenceId;
            Ros_RtMotionControl_PopulateReplyMessage(mode, &incomingCommand, &outgoingReply);
            outgoingReply.fsuInterferenceDetected = fsuLimitingDetected;
            mpSendTo(sockRtCommandListener, (char*)&outgoingReply, sizeof(RtReply), 0, (struct sockaddr*)&client_addr, client_addr_len);

            //track how big the increment SHOULD have been
            //we'll compare next cycle to see what actually happened
            bzero(howMuchShouldIHaveMoved, MAX_GROUPS * MAX_AXES * sizeof(LONG));
            for (int groupIndex = 0; groupIndex < g_Ros_Controller.numGroup; groupIndex += 1)
            {
                for (int axis = 0; axis < MAX_AXES; axis += 1)
                {
                    howMuchShouldIHaveMoved[groupIndex][axis] = moveData.grp_pos_info[groupIndex].pos[axis];
                }
            }
        }
        else
        {
            Ros_Debug_BroadcastMsg("No packets received for %d milliseconds", g_nodeConfigSettings.timeout_for_rt_msg);
            break;
        }
    }

    Ros_Debug_BroadcastMsg("Ending Rt Session");
}


void Ros_RtMotionControl_InitJointSpace(MP_EXPOS_DATA* moveData)
{
    int i;
    MP_CTRL_GRP_SEND_DATA ctrlGroup;
    MP_PULSE_POS_RSP_DATA cmdPulse;

    bzero(moveData, sizeof(MP_EXPOS_DATA));

    for (i = 0; i < g_Ros_Controller.numGroup; i++)
    {
        moveData->ctrl_grp |= (0x01 << i);
        moveData->grp_pos_info[i].pos_tag.data[0] = Ros_CtrlGroup_GetAxisConfig(g_Ros_Controller.ctrlGroups[i]);
        moveData->grp_pos_info[i].pos_tag.data[3] = MP_INC_PULSE_DTYPE;


        ctrlGroup.sCtrlGrp = i;
        mpGetPulsePos(&ctrlGroup, &cmdPulse);
        memcpy(prevRtCmdPosition[i], cmdPulse.lPos, sizeof(cmdPulse.lPos));
    }
}

void Ros_RtMotionControl_InitCartesian(MP_EXPOS_DATA* moveData)
{
    int i;
    MP_CARTPOS_EX_SEND_DATA cartSendData;
    MP_CART_POS_RSP_DATA_EX cartRespData;
    MP_GET_TOOL_NO_RSP_DATA getToolResp;

    bzero(moveData, sizeof(MP_EXPOS_DATA));

    moveData->m_ctrl_grp = 0;
    moveData->s_ctrl_grp = 0;
    
    for (i = 0; i < g_Ros_Controller.numGroup; i++)
    {
        moveData->ctrl_grp |= (1 << i);
        moveData->grp_pos_info[i].pos_tag.data[0] = Ros_CtrlGroup_GetAxisConfig(g_Ros_Controller.ctrlGroups[i]);
        moveData->grp_pos_info[i].pos_tag.data[3] = MP_INC_RF_DTYPE;

        //NOTE: This isn't the best method for this. During testing, I had tool #2 selected
        //      on the pendant, but I was commanding increments on tool #0. Because of this,
        //      the first motion on each axis would trigger the FSU detection mechanism. But
        //      it immediately recovers after one cycle.
        mpGetToolNo(MP_R1_GID + i, &getToolResp);

        cartSendData.sRobotNo = i;
        cartSendData.sFrame = 1; //1 = RF
        cartSendData.sToolNo = getToolResp.sToolNo;
        mpGetCartPosEx(&cartSendData, &cartRespData);
        memcpy(prevRtCmdPosition[i], cartRespData.lPos, sizeof(LONG) * MAX_AXES);
    }
}

bool Ros_RtMotionControl_ParseJointSpace(RtPacket* incomingCommand, MP_EXPOS_DATA* moveData)
{
    int i, groupNo;

    long pulse_increments[MAX_PULSE_AXES];

    // For each control group, convert radians to pulses and prepare moveData
    for (groupNo = 0; groupNo < g_Ros_Controller.numGroup; groupNo += 1)
    {
        CtrlGroup* ctrlGroup = g_Ros_Controller.ctrlGroups[groupNo];

        //joints must be in moto-order
        Ros_CtrlGroup_ConvertRosUnitsToMotoUnits(ctrlGroup, incomingCommand->delta[groupNo], pulse_increments);

        // Copy pulse increments to moveData
        for (i = 0; i < ctrlGroup->numAxes; i++)
        {
            moveData->grp_pos_info[groupNo].pos[i] = pulse_increments[i];
            
            if (pulse_increments[i] > ctrlGroup->maxInc.maxIncrement[i])
            {
                Ros_Debug_BroadcastMsg("ERROR: The increment for axis [%d] exceeds the maximum limit of [%d] pulse counts", pulse_increments[i], ctrlGroup->maxInc.maxIncrement[i]);
                return false;
            }
        }

        moveData->grp_pos_info[groupNo].pos_tag.data[2] = incomingCommand->toolIndex[groupNo];
    }

    return true;
}

bool Ros_RtMotionControl_ParseCartesian(RtPacket* incomingCommand, MP_EXPOS_DATA* moveData)
{
    int groupNo;

    // For each control group, convert radians to pulses and prepare moveData
    for (groupNo = 0; groupNo < g_Ros_Controller.numGroup; groupNo += 1)
    {
        moveData->grp_pos_info[groupNo].pos_tag.data[2] = incomingCommand->toolIndex[groupNo];

        moveData->grp_pos_info[groupNo].pos[TCP_X] = METERS_TO_MICROMETERS(incomingCommand->delta[groupNo][TCP_X]);
        moveData->grp_pos_info[groupNo].pos[TCP_Y] = METERS_TO_MICROMETERS(incomingCommand->delta[groupNo][TCP_Y]);
        moveData->grp_pos_info[groupNo].pos[TCP_Z] = METERS_TO_MICROMETERS(incomingCommand->delta[groupNo][TCP_Z]);

        moveData->grp_pos_info[groupNo].pos[TCP_Rx] = RAD_TO_DEG_0001(incomingCommand->delta[groupNo][TCP_Rx]);
        moveData->grp_pos_info[groupNo].pos[TCP_Ry] = RAD_TO_DEG_0001(incomingCommand->delta[groupNo][TCP_Ry]);
        moveData->grp_pos_info[groupNo].pos[TCP_Rz] = RAD_TO_DEG_0001(incomingCommand->delta[groupNo][TCP_Rz]);

        moveData->grp_pos_info[groupNo].pos[TCP_Re] = RAD_TO_DEG_0001(incomingCommand->delta[groupNo][TCP_Re]);

        moveData->grp_pos_info[groupNo].pos[TCP_8] = incomingCommand->delta[groupNo][TCP_8]; //pulse or micron (no known manipulators use this axis)

        double vector = sqrt(pow(incomingCommand->delta[groupNo][TCP_X], 2) + //x^2
                             pow(incomingCommand->delta[groupNo][TCP_Y], 2) + //y^2
                             pow(incomingCommand->delta[groupNo][TCP_Z], 2)); //z^2
        if (vector > 6.0) //1500 mm/sec == 6 mm per 4 milliseconds
        {
            Ros_Debug_BroadcastMsg("ERROR: The increment for the TCP exceeds the maximum limit of 1500 mm/sec");
            return false;
        }

        //Ros_Debug_BroadcastMsg("moveData = %d, incomingCommand = %.5f", moveData->grp_pos_info[groupNo].pos[0], incomingCommand->delta[groupNo][0] * 1000.0);
    }

    return true;
}

void Ros_RtMotionControl_Cleanup()
{
    //Do not close sockRtCommandListener. Allow it to persist
    //indefinitely and be reused.

    if (g_Ros_Controller.tidIncMoveThread != INVALID_TASK)
    {
        mpDeleteTask(g_Ros_Controller.tidIncMoveThread);
        g_Ros_Controller.tidIncMoveThread = INVALID_TASK;
        Ros_Debug_BroadcastMsg("Deleting old R/T task");
    }
}

bool Ros_RtMotionControl_OpenSocket()
{
    struct sockaddr_in server_addr;

    sockRtCommandListener = mpSocket(AF_INET, SOCK_DGRAM, 0);
    if (sockRtCommandListener < 0)
    {
        Ros_Debug_BroadcastMsg("ERROR: Could not allocate socket for RT interface");
        return false;
    }

    // Bind socket to port
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = mpHtons(atoi(g_nodeConfigSettings.rt_udp_port_number));

    if (mpBind(sockRtCommandListener, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
    {
        Ros_Debug_BroadcastMsg("ERROR: Failed to bind UDP socket for real-time motion control");
        mpClose(sockRtCommandListener);
        sockRtCommandListener = -1;
        return false;
    }

    return true;
}

void Ros_RtMotionControl_PopulateReplyMessage(MOTION_MODE mode, RtPacket* command, RtReply* reply)
{
    long pulsePos_moto[MAX_PULSE_AXES];
    long degrees[MP_GRP_AXES_NUM];
    BITSTRING figure;
    MP_COORD coord;
    MP_CTRL_GRP_SEND_DATA ctrlGroup;
    MP_PULSE_POS_RSP_DATA cmdPulse;

    bzero(reply, sizeof(RtReply));
    bzero(degrees, sizeof(long) * MP_GRP_AXES_NUM);

    reply->sequenceEcho = command->sequenceId;

    for (int groupIndex = 0; groupIndex < g_Ros_Controller.numGroup; groupIndex += 1)
    {
        CtrlGroup* group = g_Ros_Controller.ctrlGroups[groupIndex];

        //================================================================================
        //FB pos
        //================================================================================
        Ros_CtrlGroup_GetFBPulsePos(group, pulsePos_moto);

        //Angles
        Ros_CtrlGroup_ConvertMotoUnitsToRosUnits(group, pulsePos_moto, reply->feedbackPositionJoints[groupIndex]);
        
        for (int axis = 0; axis < MP_GRP_AXES_NUM; axis += 1)
            degrees[axis] = RAD_TO_DEG_0001(reply->feedbackPositionJoints[groupIndex][axis]);

        //Cart
        mpConvAxesToCartPos(groupIndex, degrees, command->toolIndex[groupIndex], &figure, &coord);

        reply->feedbackPositionCartesian[groupIndex][TCP_X] = MICROMETERS_TO_METERS(coord.x);
        reply->feedbackPositionCartesian[groupIndex][TCP_Y] = MICROMETERS_TO_METERS(coord.y);
        reply->feedbackPositionCartesian[groupIndex][TCP_Z] = MICROMETERS_TO_METERS(coord.z);

        reply->feedbackPositionCartesian[groupIndex][TCP_Rx] = DEG_0001_TO_RAD(coord.rx);
        reply->feedbackPositionCartesian[groupIndex][TCP_Ry] = DEG_0001_TO_RAD(coord.ry);
        reply->feedbackPositionCartesian[groupIndex][TCP_Rz] = DEG_0001_TO_RAD(coord.rz);
        reply->feedbackPositionCartesian[groupIndex][TCP_Re] = DEG_0001_TO_RAD(coord.ex1);

        //================================================================================
        //CMD pos
        //================================================================================
        //Should this be Ros_CtrlGroup_GetPulsePosCmd?
        //Answer: No, it should not. That should only be used when converting incoming
        //        positional commands that contain an absolute position.
        //        See https://github.com/Yaskawa-Global/motoros2/discussions/455
        ctrlGroup.sCtrlGrp = groupIndex;
        mpGetPulsePos(&ctrlGroup, &cmdPulse);

        //rad
        Ros_CtrlGroup_ConvertMotoUnitsToRosUnits(group, cmdPulse.lPos, reply->previousCommandPositionJoints[groupIndex]);
        
        //deg
        for (int axis = 0; axis < MP_GRP_AXES_NUM; axis += 1)
            degrees[axis] = RAD_TO_DEG_0001(reply->previousCommandPositionJoints[groupIndex][axis]);

        //Cart
        mpConvAxesToCartPos(groupIndex, degrees, command->toolIndex[groupIndex], &figure, &coord);

        reply->previousCommandPositionCartesian[groupIndex][TCP_X] = MICROMETERS_TO_METERS(coord.x);
        reply->previousCommandPositionCartesian[groupIndex][TCP_Y] = MICROMETERS_TO_METERS(coord.y);
        reply->previousCommandPositionCartesian[groupIndex][TCP_Z] = MICROMETERS_TO_METERS(coord.z);

        reply->previousCommandPositionCartesian[groupIndex][TCP_Rx] = DEG_0001_TO_RAD(coord.rx);
        reply->previousCommandPositionCartesian[groupIndex][TCP_Ry] = DEG_0001_TO_RAD(coord.ry);
        reply->previousCommandPositionCartesian[groupIndex][TCP_Rz] = DEG_0001_TO_RAD(coord.rz);
        reply->previousCommandPositionCartesian[groupIndex][TCP_Re] = DEG_0001_TO_RAD(coord.ex1);
    }
}

bool Ros_CheckForFsuInterference(MOTION_MODE mode, int* tools)
{
    MP_CTRL_GRP_SEND_DATA ctrlGroup;
    MP_PULSE_POS_RSP_DATA cmdPulse;
    MP_CARTPOS_EX_SEND_DATA cartSendData;
    MP_CART_POS_RSP_DATA_EX cartRespData;

    for (int groupIndex = 0; groupIndex < g_Ros_Controller.numGroup; groupIndex += 1)
    {
        //================================================================================
        //FSU speed limit
        //================================================================================

        LONG difference;
        LONG howMuchDidIActuallyMove[MAX_AXES];
        bzero(howMuchDidIActuallyMove, sizeof(LONG) * MAX_AXES);

        if (mode == MOTION_MODE_RT_JOINT)
        {
            //Should this be Ros_CtrlGroup_GetPulsePosCmd?
            //Answer: No, it should not. That should only be used when converting incoming
            //        positional commands that contain an absolute position.
            //        See https://github.com/Yaskawa-Global/motoros2/discussions/455
            ctrlGroup.sCtrlGrp = groupIndex;
            mpGetPulsePos(&ctrlGroup, &cmdPulse);
        }
        else if (mode == MOTION_MODE_RT_CARTESIAN)
        {
            cartSendData.sRobotNo = groupIndex;
            cartSendData.sFrame = 1; //1 = RF
            cartSendData.sToolNo = tools[groupIndex];
            mpGetCartPosEx(&cartSendData, &cartRespData);
        }

        // Check if pulses (or mm's) are missing from last increment.
        // Get the current controller command position and substract the previous command position
        // and check if it matches the amount if increment sent last cycle
        for (int axis = 0; axis < MP_GRP_AXES_NUM; axis += 1)
        {
            if (howMuchShouldIHaveMoved[groupIndex][axis] != 0)
            {
                if (mode == MOTION_MODE_RT_JOINT)
                {
                    howMuchDidIActuallyMove[axis] = cmdPulse.lPos[axis] - prevRtCmdPosition[groupIndex][axis];
                    prevRtCmdPosition[groupIndex][axis] = cmdPulse.lPos[axis];
                }
                else if (mode == MOTION_MODE_RT_CARTESIAN)
                {
                    //When working in cartesian space, we're only going to monitor the translation.
                    //1. There is no FSU speed limit for rotation. So it's moot.
                    //2. When rotating by some increment, that rotation gets 'spread out' over multiple
                    //   axes. Even if I put all of my commanded increment into a single axis, all 
                    //   three of them are going to react. So, the cmd-value of my intended axis may
                    //   not be the value I expect.
                    if (axis >= TCP_Rx)
                        break;

                    howMuchDidIActuallyMove[axis] = cartRespData.lPos[axis] - prevRtCmdPosition[groupIndex][axis];
                    prevRtCmdPosition[groupIndex][axis] = cartRespData.lPos[axis];
                }

                difference = howMuchShouldIHaveMoved[groupIndex][axis] - howMuchDidIActuallyMove[axis];
                if (abs(difference) > MAX_INCREMENT_DEVIATION_FOR_FSU_DETECTION)
                {
                    //Ros_Debug_BroadcastMsg("howMuchShouldIHaveMoved[%d][%d] = %d", groupIndex, axis, howMuchShouldIHaveMoved[groupIndex][axis]);
                    //Ros_Debug_BroadcastMsg("howMuchDidIActuallyMove[%d] = %d", axis, howMuchDidIActuallyMove[axis]);
                    //Ros_Debug_BroadcastMsg("difference = %d", difference);
                    //Ros_Debug_BroadcastMsg("---------");

                    return TRUE;
                }
            }
        }
    }
    return FALSE;
}
