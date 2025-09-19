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
bool Ros_RtMotionControl_OpenSocket(int* sockServer);
void Ros_RtMotionControl_PopulateReplyMessage(int sequenceId, RtReply* reply);


static int sockRtCommandListener = -1;

void Ros_RtMotionControl_HyperRobotCommanderX5(MOTION_MODE mode)
{
    MP_EXPOS_DATA moveData;
    int i, groupNo, bytes_received;

    bool bFirstRecv = true;
    struct sockaddr_in client_addr;
    struct sockaddr_in previous_client_addr;
    int client_addr_len = sizeof(client_addr);

    RtPacket incomingCommand;
    RtReply outgoingReply;

    UINT32 sequenceId = 0;

    struct fd_set fds;
    struct timeval tv;
    struct timeval* timeout;

    //=========================================================================================

    if (mode == MOTION_MODE_RT_JOINT)
        Ros_RtMotionControl_InitJointSpace(&moveData);
    else
        Ros_RtMotionControl_InitCartesian(&moveData);


    if (!Ros_RtMotionControl_OpenSocket(&sockRtCommandListener))
        mpDeleteSelf;

    Ros_Debug_BroadcastMsg("Starting RT session");

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
                if (memcmp(&client_addr, &previous_client_addr, sizeof(struct sockaddr_in)) != 0)
                {
                    Ros_Debug_BroadcastMsg("ERROR: Received command packets from multiple sources");
                    break; //drop the connection
                }
            }

            if (bytes_received > 0)
            {
                #warning deal with rollover;
                if (incomingCommand.sequenceId <= sequenceId && !bFirstRecv)
                {
                    Ros_Debug_BroadcastMsg("WARN: Received old command packet (seq: %d, new: %d)", sequenceId, incomingCommand.sequenceId);
                    continue; //drop this packet
                }

                if ((incomingCommand.sequenceId - sequenceId) >= MAX_SEQUENCE_DIFFERENCE)
                {
                    Ros_Debug_BroadcastMsg("ERROR: Missed too many command packets (seq: %d, new: %d)", sequenceId, incomingCommand.sequenceId);
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

                // Send increment to robot
                int ret = mpExRcsIncrementMove(&moveData);
                if (ret != OK)
                    Ros_Debug_BroadcastMsg("WARN: mpExRcsIncrementMove returned %d", ret);

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
            sequenceId = incomingCommand.sequenceId;
            Ros_RtMotionControl_PopulateReplyMessage(sequenceId, &outgoingReply);
            mpSendTo(sockRtCommandListener, (char*)&outgoingReply, sizeof(RtReply), 0, (struct sockaddr*)&client_addr, client_addr_len);
        }
        else
        {
            Ros_Debug_BroadcastMsg("No packets received for %d milliseconds", g_nodeConfigSettings.timeout_for_rt_msg);
            break;
        }
    }

    mpClose(sockRtCommandListener);
    sockRtCommandListener = -1;

    Ros_Debug_BroadcastMsg("Ending Rt Session");

    g_Ros_Controller.tidIncMoveThread = INVALID_TASK;
    mpDeleteSelf;
}


void Ros_RtMotionControl_InitJointSpace(MP_EXPOS_DATA* moveData)
{
    int i;

    bzero(moveData, sizeof(MP_EXPOS_DATA));

    for (i = 0; i < g_Ros_Controller.numGroup; i++)
    {
        moveData->ctrl_grp |= (0x01 << i);
        moveData->grp_pos_info[i].pos_tag.data[0] = Ros_CtrlGroup_GetAxisConfig(g_Ros_Controller.ctrlGroups[i]);
        moveData->grp_pos_info[i].pos_tag.data[3] = MP_INC_PULSE_DTYPE;
    }
}

void Ros_RtMotionControl_InitCartesian(MP_EXPOS_DATA* moveData)
{
    int i;

    bzero(moveData, sizeof(MP_EXPOS_DATA));

#warning how to specify multi group? ;;;
    moveData->ctrl_grp = 1; //R1 independent operation
    moveData->m_ctrl_grp = 0;
    moveData->s_ctrl_grp = 0;

    for (i = 0; i < g_Ros_Controller.numGroup; i++)
    {
        moveData->grp_pos_info[i].pos_tag.data[0] = Ros_CtrlGroup_GetAxisConfig(g_Ros_Controller.ctrlGroups[i]);
        #warning how to specify tool ? ;;;
        moveData->grp_pos_info[i].pos_tag.data[2] = 0;
        moveData->grp_pos_info[i].pos_tag.data[3] = MP_INC_RF_DTYPE;
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
    }

    return true;
}

bool Ros_RtMotionControl_ParseCartesian(RtPacket* incomingCommand, MP_EXPOS_DATA* moveData)
{
    int i, groupNo;

    // For each control group, convert radians to pulses and prepare moveData
    for (groupNo = 0; groupNo < g_Ros_Controller.numGroup; groupNo += 1)
    {
        CtrlGroup* ctrlGroup = g_Ros_Controller.ctrlGroups[groupNo];

        moveData->grp_pos_info[groupNo].pos[0] = METERS_TO_MICROMETERS(incomingCommand->delta[groupNo][0]);
        moveData->grp_pos_info[groupNo].pos[1] = METERS_TO_MICROMETERS(incomingCommand->delta[groupNo][1]);
        moveData->grp_pos_info[groupNo].pos[2] = METERS_TO_MICROMETERS(incomingCommand->delta[groupNo][2]);

        moveData->grp_pos_info[groupNo].pos[3] = RAD_TO_DEG_0001(incomingCommand->delta[groupNo][3]);
        moveData->grp_pos_info[groupNo].pos[4] = RAD_TO_DEG_0001(incomingCommand->delta[groupNo][4]);
        moveData->grp_pos_info[groupNo].pos[5] = RAD_TO_DEG_0001(incomingCommand->delta[groupNo][5]);

        moveData->grp_pos_info[groupNo].pos[6] = RAD_TO_DEG_0001(incomingCommand->delta[groupNo][6]);

        moveData->grp_pos_info[groupNo].pos[7] = incomingCommand->delta[groupNo][7]; //pulse or micron (no known manipulators use this axis)

        double vector = sqrt(pow(incomingCommand->delta[groupNo][0], 2) + //x^2
                             pow(incomingCommand->delta[groupNo][1], 2) + //y^2
                             pow(incomingCommand->delta[groupNo][2], 2)); //z^2
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
    if (sockRtCommandListener != -1)
    {
        mpClose(sockRtCommandListener);
        sockRtCommandListener = -1;
    }

    if (g_Ros_Controller.tidIncMoveThread != INVALID_TASK)
    {
        mpDeleteTask(g_Ros_Controller.tidIncMoveThread);
        g_Ros_Controller.tidIncMoveThread = INVALID_TASK;
    }
}

bool Ros_RtMotionControl_OpenSocket(int* sockServer)
{
    struct sockaddr_in server_addr;
    int optval = 1;

    *sockServer = mpSocket(AF_INET, SOCK_DGRAM, 0);
    if (*sockServer < 0)
    {
        Ros_Debug_BroadcastMsg("ERROR: Could not allocate socket for RT interface");
        return false;
    }

    Ros_setsockopt(*sockServer, SOL_SOCKET, SO_REUSEADDR, (char*)&optval, sizeof(optval));

    // Bind socket to port
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = mpHtons(atoi(g_nodeConfigSettings.rt_udp_port_number));

    if (mpBind(*sockServer, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
    {
        Ros_Debug_BroadcastMsg("ERROR: Failed to bind UDP socket for real-time motion control");
        mpClose(*sockServer);
        return false;
    }

    return true;
}

void Ros_RtMotionControl_PopulateReplyMessage(int sequenceId, RtReply* reply)
{
    long pulsePos_moto[MAX_CONTROLLABLE_GROUPS][MAX_PULSE_AXES];

    reply->sequenceEcho = sequenceId;

    for (int groupIndex = 0; groupIndex < g_Ros_Controller.numGroup; groupIndex += 1)
    {
        CtrlGroup* group = g_Ros_Controller.ctrlGroups[groupIndex];

        Ros_CtrlGroup_GetFBPulsePos(group, pulsePos_moto[groupIndex]);
        Ros_CtrlGroup_ConvertMotoUnitsToRosUnits(group, pulsePos_moto[groupIndex], reply->feedbackPosition[groupIndex]);
    }
}
