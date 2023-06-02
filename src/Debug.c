//Debug.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"


#define DEBUG_UDP_PORT_NUMBER   21789

#define MAX_DEBUG_MESSAGE_SIZE  1024

int ros_DebugSocket = -1;
struct sockaddr_in ros_debug_destAddr1;

void Ros_Debug_Init()
{
    ULONG ip_be;
    ULONG subnetmask_be;
    ULONG gateway_be;
    int broadcastVal = 1;
    UCHAR mac[6];

    ros_DebugSocket = mpSocket(AF_INET, SOCK_DGRAM, 0);
    Ros_setsockopt(ros_DebugSocket, SOL_SOCKET, SO_BROADCAST, (char*)&broadcastVal, sizeof(broadcastVal));

    Ros_mpNICData(ROS_USER_LAN1, &ip_be, &subnetmask_be, mac, &gateway_be);

    ros_debug_destAddr1.sin_addr.s_addr = ip_be | (~subnetmask_be);
    ros_debug_destAddr1.sin_family = AF_INET;
    ros_debug_destAddr1.sin_port = mpHtons(DEBUG_UDP_PORT_NUMBER);
}

void Ros_Debug_BroadcastMsg(char* fmt, ...)
{
    char str[MAX_DEBUG_MESSAGE_SIZE];
    va_list va;

    bzero(str, MAX_DEBUG_MESSAGE_SIZE);

    va_start(va, fmt);
    vsnprintf(str, MAX_DEBUG_MESSAGE_SIZE, fmt, va);
    va_end(va);

    if (ros_DebugSocket == -1)
        Ros_Debug_Init();

    mpSendTo(ros_DebugSocket, str, strlen(str), 0, (struct sockaddr*) &ros_debug_destAddr1, sizeof(struct sockaddr_in));

    if (g_nodeConfigSettings.log_to_stdout)
        puts(str);
}

void Ros_Debug_LogToConsole(char* fmt, ...)
{
    char str[MAX_DEBUG_MESSAGE_SIZE];
    va_list va;

    bzero(str, MAX_DEBUG_MESSAGE_SIZE);

    va_start(va, fmt);
    vsnprintf(str, MAX_DEBUG_MESSAGE_SIZE, fmt, va);
    va_end(va);

    printf(APPLICATION_NAME ": %s\n", str);
}
