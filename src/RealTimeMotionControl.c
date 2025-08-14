//RealTimeMotionControl.c

// SPDX-FileCopyrightText: 2025, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2025, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

// Based loosely on @adv4ncr's modifications to MotoROS1 for real-time control through ROS2.
// https://github.com/adv4ncr/motoman_ROS2/blob/cdf63a592596ff711df842680a1e4c730fd547a7/controller_driver/RealTimeMotionServer.c

#include "MotoROS.h"

void Ros_RtMotionControl_JointSpace();
void Ros_RtMotionControl_Cartesian();
void Ros_RtMotionControl_Cleanup();
bool Ros_RtMotionControl_OpenSocket(int* sockServer);
void Ros_RtMotionControl_PopulateReplyMessage(int sequenceId, RtReply* reply);


static int sockRtCommandListener = -1;

extern MOTION_MODE Ros_MotionControl_ActiveMotionMode;

void Ros_RtMotionControl_RtIncMoveLoopStart(MOTION_MODE mode)
{
    if (mode == MOTION_MODE_RT_JOINT)
        Ros_RtMotionControl_JointSpace();
    else
        Ros_RtMotionControl_Cartesian();
}

void Ros_RtMotionControl_JointSpace()
{
    MP_EXPOS_DATA moveData;
    int i, groupNo, bytes_received;

    struct sockaddr_in client_addr;
    int client_addr_len = sizeof(client_addr);

    MP_CTRL_GRP_SEND_DATA ctrlGrpData;
    MP_PULSE_POS_RSP_DATA prevPulsePosData[MAX_CONTROLLABLE_GROUPS];
    long pulse_increments[MAX_PULSE_AXES];

    int sockServer;

    RtPacket incomingCommand;
    RtReply outgoingReply;

    UINT32 sequenceId = 0;

    struct fd_set fds;
    struct timeval tv;

    //=========================================================================================

    bzero(&moveData, sizeof(moveData));

    for (i = 0; i < g_Ros_Controller.numGroup; i++)
    {
        moveData.ctrl_grp |= (0x01 << i);
        moveData.grp_pos_info[i].pos_tag.data[0] = Ros_CtrlGroup_GetAxisConfig(g_Ros_Controller.ctrlGroups[i]);
        moveData.grp_pos_info[i].pos_tag.data[3] = MP_INC_PULSE_DTYPE;

        ctrlGrpData.sCtrlGrp = g_Ros_Controller.ctrlGroups[i]->groupId;
        mpGetPulsePos(&ctrlGrpData, &prevPulsePosData[i]);
    }

    if (!Ros_RtMotionControl_OpenSocket(&sockServer))
        mpDeleteSelf;

    //=========================================================================================
    while (TRUE)
    {

        //-------------------------------------------------------------------------------------
        FD_ZERO(&fds);
        FD_SET(sockServer, &fds);

        tv.tv_usec = 0;
        tv.tv_sec = g_nodeConfigSettings.timeout_for_rt_msg;

        if (mpSelect(sockServer + 1, &fds, NULL, NULL, &tv) > 0)
        {
            bzero(&incomingCommand, sizeof(incomingCommand));
            bytes_received = mpRecvFrom(sockServer, (char*)&incomingCommand, sizeof(RtPacket), 0, (struct sockaddr*)&client_addr, &client_addr_len);

            if (bytes_received > 0)
            {
                #warning deal with rollover;
                if (incomingCommand.sequenceId < sequenceId)
                {
                    Ros_Debug_BroadcastMsg("WARN: Received old command packet (seq: %d, new: %d)", sequenceId, incomingCommand.sequenceId);
                    continue; //drop this packet
                }

                if ((incomingCommand.sequenceId - sequenceId) >= MAX_SEQUENCE_DIFFERENCE)
                {
                    Ros_Debug_BroadcastMsg("ERROR: Missed too many command packets (seq: %d, new: %d)", sequenceId, incomingCommand.sequenceId);
                    break; //drop the connection
                }

                // For each control group, convert radians to pulses and prepare moveData
                for (groupNo = 0; groupNo < g_Ros_Controller.numGroup; groupNo += 1)
                {
                    CtrlGroup* ctrlGroup = g_Ros_Controller.ctrlGroups[groupNo];

                    Ros_CtrlGroup_ConvertRosUnitsToMotoUnits(ctrlGroup, incomingCommand.delta_rad[groupNo], pulse_increments);

                    // Copy pulse increments to moveData
                    for (i = 0; i < ctrlGroup->numAxes; i++)
                    {
                        moveData.grp_pos_info[groupNo].pos[i] = pulse_increments[i];
                    }
                }

                // Send increment to robot
                int ret = mpExRcsIncrementMove(&moveData);
                if (ret != OK)
                    Ros_Debug_BroadcastMsg("WARN: mpExRcsIncrementMove returned %d", ret);
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
            mpSendTo(sockServer, (char*)&outgoingReply, sizeof(RtReply), 0, (struct sockaddr*)&client_addr, client_addr_len);
        }
        else
        {
            Ros_Debug_BroadcastMsg("No packets received for %d seconds", g_nodeConfigSettings.timeout_for_rt_msg);
            break;
        }
    }

    mpClose(sockRtCommandListener);
    sockRtCommandListener = -1;
    Ros_MotionControl_ActiveMotionMode = MOTION_MODE_INACTIVE;

    Ros_Debug_BroadcastMsg("Ending Rt Session");

    g_Ros_Controller.tidIncMoveThread = INVALID_TASK;
    mpDeleteSelf;
}

void Ros_RtMotionControl_Cartesian()
{
    Ros_MotionControl_StopTrajMode();
    Ros_Debug_BroadcastMsg("ERROR: Cartesian interface not yet implemented");
    mpSetAlarm(ALARM_OPERATION_FAIL, "Cartesian not yet implemented", SUBCODE_NOT_IMPLEMENTED);
    mpDeleteSelf;
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

    *sockServer = mpSocket(AF_INET, SOCK_DGRAM, 0);
    if (*sockServer < 0)
    {
        Ros_Debug_BroadcastMsg("ERROR: Could not allocate socket for RT interface");
        return false;
    }

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
    reply->sequenceId = sequenceId;
}
