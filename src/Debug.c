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
    char Formatted_Time[300];
    va_list va;

    bzero(str, MAX_DEBUG_MESSAGE_SIZE);

    va_start(va, fmt);
    vsnprintf(str, MAX_DEBUG_MESSAGE_SIZE, fmt, va);
    va_end(va);

    if (ros_DebugSocket == -1)
        Ros_Debug_Init();
    
    // Timestamp

    time_t DEBUG_MSG_TIMESTAMP;
    struct tm ts;
    struct timeval tv;


    time(&DEBUG_MSG_TIMESTAMP);

    ts = *localtime(&DEBUG_MSG_TIMESTAMP);

    strftime(Formatted_Time, sizeof(Formatted_Time), "%a %Y-%m-%d %H:%M:%S", &ts);

    gettimeofday(&tv, NULL);
    long msec = tv.tv_usec / 1000;

    snprintf(Formatted_Time, sizeof(Formatted_Time), "%s.%03ld", Formatted_Time, msec);

    puts(Formatted_Time);

    size_t Timestamp_Length = strlen(Formatted_Time);
    size_t Debug_Message_Length = strlen(str);
   
    if (Timestamp_Length + Debug_Message_Length + 1 < MAX_DEBUG_MESSAGE_SIZE)
    {
        memmove(str + Timestamp_Length, str, Debug_Message_Length + 1);
        memcpy(str, Formatted_Time, Timestamp_Length);
    }

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
