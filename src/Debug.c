//Debug.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"


#define DEBUG_UDP_PORT_NUMBER   21789

#define MAX_DEBUG_MESSAGE_SIZE  1024

#define FORMATTED_TIME_SIZE  1024

typedef struct
{
    UINT8 enabledPortCount;
    int debugSocket[MAX_NETWORK_PORTS];
    struct sockaddr_in destAddr[MAX_NETWORK_PORTS];
} userLanInfo;

userLanInfo ros_debugPorts = {0};

void Ros_Debug_Init()
{
    ULONG ip_be;
    ULONG subnetmask_be;
    ULONG gateway_be;
    int broadcastVal = 1;
    UCHAR mac[6];
    STATUS status;
    int count = 0;
    int* socket = ros_debugPorts.debugSocket;
    struct sockaddr_in* sin = ros_debugPorts.destAddr;

    bzero(&ros_debugPorts, sizeof(ros_debugPorts));

    for (int i = 1; i <= MAX_NETWORK_PORTS; i++)
    {

        status = Ros_mpNICData(i, &ip_be, &subnetmask_be, mac, &gateway_be);

        if (status == OK)
        {
            socket[count] = mpSocket(AF_INET, SOCK_DGRAM, 0);
            Ros_setsockopt(socket[count], SOL_SOCKET, SO_BROADCAST, (char*)&broadcastVal, sizeof(broadcastVal));
            sin[count].sin_addr.s_addr = ip_be | (~subnetmask_be);
            sin[count].sin_family = AF_INET;
            sin[count].sin_port = mpHtons(DEBUG_UDP_PORT_NUMBER);
            count++;
        }
    }
    ros_debugPorts.enabledPortCount = count;
}

void Ros_Debug_SetFromConfig()
{
    ULONG ip_be;
    ULONG subnetmask_be;
    ULONG gateway_be;
    int broadcastVal = 1;
    UCHAR mac[6];
    STATUS status;
    char message[ERROR_MSG_MAX_SIZE];
    int* socket = ros_debugPorts.debugSocket;
    struct sockaddr_in* sin = ros_debugPorts.destAddr;

    bzero(&ros_debugPorts, sizeof(ros_debugPorts));

    status = Ros_mpNICData(g_nodeConfigSettings.userlan_debug_broadcast_port, &ip_be, &subnetmask_be, mac, &gateway_be);

    if (status == OK)
    {
        socket[0] = mpSocket(AF_INET, SOCK_DGRAM, 0);
        Ros_setsockopt(socket[0], SOL_SOCKET, SO_BROADCAST, (char*)&broadcastVal, sizeof(broadcastVal));
        sin[0].sin_addr.s_addr = ip_be | (~subnetmask_be);
        sin[0].sin_family = AF_INET;
        sin[0].sin_port = mpHtons(DEBUG_UDP_PORT_NUMBER);
        ros_debugPorts.enabledPortCount = 1;
    }
    else
    {
        int ret = snprintf(message, ERROR_MSG_MAX_SIZE, "Enable LAN port %d for debug", g_nodeConfigSettings.userlan_debug_broadcast_port);
        if (0 < ret && ret <= 32)
            mpSetAlarm(ALARM_ASSERTION_FAIL, message, SUBCODE_DEBUG_INIT_FAIL_MP_NICDATA_SPECIFIC);
        else
            mpSetAlarm(ALARM_ASSERTION_FAIL, "Enable debug LAN port from cfg", SUBCODE_DEBUG_INIT_FAIL_MP_NICDATA_SPECIFIC);
        g_nodeConfigSettings.userlan_debug_broadcast_enabled = FALSE;
    }
}

void Ros_Debug_BroadcastMsg(char* fmt, ...)
{
    char str[MAX_DEBUG_MESSAGE_SIZE];
    va_list va;

    if (g_nodeConfigSettings.userlan_debug_broadcast_enabled && ros_debugPorts.enabledPortCount == 0)
        Ros_Debug_Init();

    if (!g_nodeConfigSettings.userlan_debug_broadcast_enabled && !g_nodeConfigSettings.log_to_stdout)
        return;

    bzero(str, MAX_DEBUG_MESSAGE_SIZE);

    va_start(va, fmt);
    vsnprintf(str, MAX_DEBUG_MESSAGE_SIZE, fmt, va);
    va_end(va);

    // Timestamp
    //The timestamp for the message "Found Micro-Ros PC Agent" will be the epoch time (THU 1970-01-01 00:00:00.000) as the global flags 
    //are set to indicate that the Micro-Ros PC Agent is connected but the first sync of the host time using the micro-ROS agent is yet to occur
    struct tm synced_time;
    struct timespec tp;
    char timestamp[FORMATTED_TIME_SIZE];
    if (g_Ros_Communication_AgentIsConnected)
    {
        //get synchronized time from the agent
        int64_t nanosecs = rmw_uros_epoch_nanos();
        Ros_Nanos_To_Timespec(nanosecs, &tp);
    }
    else
    {
        //rmw_uros_epoch_nanos cannot sync with agent because it's not connected
        clock_gettime(CLOCK_REALTIME, &tp);
    }
    localtime_r(&tp.tv_sec, &synced_time);
    strftime(timestamp, FORMATTED_TIME_SIZE, "%Y-%m-%d %H:%M:%S", &synced_time);
    snprintf(timestamp + strlen(timestamp), FORMATTED_TIME_SIZE - strlen(timestamp), ".%06d ", (int)tp.tv_nsec / 1000);

    // Pre - pending the timestamp to the debug message
    size_t timestamp_length = Ros_strnlen(timestamp, FORMATTED_TIME_SIZE);
    size_t debug_message_length = Ros_strnlen(str, MAX_DEBUG_MESSAGE_SIZE);
    if (timestamp_length + debug_message_length + 1 < MAX_DEBUG_MESSAGE_SIZE)
    {
        // Move existing contents of str buffer to the end by Timestamp_Length to make space 
        //for the timestamp and avoiding overwriting the debug message during the move 
        memmove(str + timestamp_length, str, debug_message_length); 
        // Copy the timestamp stored in Formatted_time buffer to the beginning of str buffer
        memcpy(str, timestamp, timestamp_length);         
    }

    for (int i = 0; i < ros_debugPorts.enabledPortCount; i++)
        mpSendTo(ros_debugPorts.debugSocket[i], str, strlen(str), 0, (struct sockaddr*)&(ros_debugPorts.destAddr[i]), sizeof(struct sockaddr_in));


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
