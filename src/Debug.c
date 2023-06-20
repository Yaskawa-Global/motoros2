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
    struct tm synced_time;
    struct timeval tv;
    int64_t nanosecs = rmw_uros_epoch_nanos();
    builtin_interfaces__msg__Time debug_msg_timestamp;

    if (g_Ros_Communication_AgentIsConnected)
    {
        //get synchronized time from the agent
        int64_t nanosecs = rmw_uros_epoch_nanos(); 
        Ros_Nanos_To_Time_Msg(nanosecs, &debug_msg_timestamp);
        strftime(timestamp, FORMATTED_TIME_SIZE, "%a %Y-%m-%d %H:%M:%S", localtime_r(&debug_msg_timestamp.sec, &synced_time));
        snprintf(timestamp + strlen(timestamp), FORMATTED_TIME_SIZE - strlen(timestamp), ".%03d", (int)debug_msg_timestamp.nanosec / 1000000);
    }
    else
    {
        //rmw_uros_epoch_nanos cannot sync with agent because it's not connected
        gettimeofday(&tv, NULL);
        strftime(timestamp, FORMATTED_TIME_SIZE, "%a %Y-%m-%d %H:%M:%S", localtime_r(&tv.tv_sec, &synced_time));
        snprintf(timestamp + strlen(timestamp), FORMATTED_TIME_SIZE - strlen(timestamp), ".%03d", tv.tv_usec / 1000);
    }
  
    // Pre - pending the timestamp to the debug message
    size_t timestamp_length = strnlen(timestamp, FORMATTED_TIME_SIZE);
    size_t debug_message_length = strnlen(str, MAX_DEBUG_MESSAGE_SIZE);
    if (timestamp_length + debug_message_length + 1 < MAX_DEBUG_MESSAGE_SIZE)
    {
        // Move existing contents of str buffer to the end by Timestamp_Length to make space 
        //for the timestamp and avoiding overwriting the debug message during the move 
        memmove(str + timestamp_length, str, debug_message_length + 1); 
        // Copy the timestamp stored in Formatted_time buffer to the beginning of str buffer
        memcpy(str, timestamp, timestamp_length);         
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
