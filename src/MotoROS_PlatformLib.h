// MotoROS_PlatformLib.h

// SPDX-FileCopyrightText: 2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS_PLATFORM_LIB_H
#define MOTOROS_PLATFORM_LIB_H


// Define the MotoROS_PlatformLib version number as a base-10 integer.
//
// Format: XXXYYYZZ, where:
//
// - XXX: major version
// - YYY: minor version, and
// -  ZZ: patch version nr
//
// Note: no leading zeros, to avoid octal parsing
#define MOTOROS_PLATFORM_LIB_VERSION 212

#define MOTOROS_PLATFORM_LIB_MAJOR (MOTOROS_PLATFORM_LIB_VERSION / 100000)
#define MOTOROS_PLATFORM_LIB_MINOR (MOTOROS_PLATFORM_LIB_VERSION / 100 % 1000)
#define MOTOROS_PLATFORM_LIB_PATCH (MOTOROS_PLATFORM_LIB_VERSION % 100)


// Returns the PlatformLib version number
extern UINT32 MotoROS_PlatformLib_GetVersion();


extern BOOL Ros_IsOtherInstanceRunning();


// Attempt to read MAC address from the specified LAN interface
// IFF return value is 'OK', 'macAddress' will contain the MAC. In all other
// cases, 'macAddress' may be left uninitialised.
#define ROS_GET_MAC_ADDR_INVALID_IF_NO -10
#define ROS_GET_MAC_ADDR_GENERIC_FAILURE -11
extern STATUS Ros_GetMacAddress(USHORT if_no, UINT8 macAddress[6]);


#define ROS_USER_LAN1 1   /* general LAN interface1 */
#define ROS_USER_LAN2 2   /* general LAN interface2(only YRC1000) */

// Retrieves the IP address, subnet mask, MAC address and default gateway for the specified network interface.
// NOTE: If this is called immediately upon startup, the default_gw may not be available yet. In this case, the default_gw will be set to 0.
#define ROS_MP_NIC_DATA_INVALID_IF_NO -10
#define ROS_MP_NIC_DATA_ERR_IP_ADDR -11
#define ROS_MP_NIC_DATA_ERR_IP_ADDR_SZ -12
#define ROS_MP_NIC_DATA_ERR_INADDR -13
#define ROS_MP_NIC_DATA_ERR_MASK -14
// allow room for reporting Ros_GetMacAddress(..) error codes
#define ROS_MP_NIC_DATA_ERR_MAC -100
// allow room for reporting Ros_GetGateway(..) error codes
#define ROS_MP_NIC_DATA_ERR_GW -200
extern STATUS Ros_mpNICData(USHORT if_no, ULONG* ip_addr, ULONG* subnet_mask, UCHAR* mac_addr, ULONG* default_gw);


extern size_t Ros_strnlen(const char *s, size_t maxlen);


extern ULONG tickGet();


#define SO_BROADCAST    0x0020
extern STATUS Ros_setsockopt(int s, int level, int optname, char* optval, int optlen);

#if defined (DX100) || defined (FS100)
// VxWorks 5.5/6.8
extern int localtime_r(const time_t* timer, struct tm* timeBuffer);
#elif defined (YRC1000) || defined (YRC1000u) || defined (DX200)
// >= VxWorks 6.9
struct tm* localtime_r(const time_t* timep, struct tm* result);
#else
#error localtime_r: unsupported platform
#endif


#if defined (YRC1000) || defined (YRC1000u) || defined (DX200) || defined (FS100) || defined (DX100)
// from clockLib
struct timespec
{
    time_t tv_sec;  /* seconds */
    long   tv_nsec; /* nanoseconds (0 -1,000,000,000) */
};
#ifndef CLOCK_REALTIME
#define CLOCK_REALTIME 0
#endif
typedef int clockid_t;
extern int clock_gettime(clockid_t clock_id, /* clock ID (always CLOCK_REALTIME) */
                         struct timespec* tp /* where to store current time */);
#else
#error clock_gettime: unsupported platform
#endif


// Attempts to determine whether the specific link is UP (ie: cable is connected).
// IFF return value is 'OK', 'is_up' will reflect link state. In all other cases,
// 'is_up' may be left uninitialised.
#define ROS_USERLAN_STATE_INVALID_IF_NO -10
#define ROS_USERLAN_STATE_NO_SUCH_IF -11
#define ROS_USERLAN_STATE_GET_STATE_ERROR -12
#define ROS_USERLAN_STATE_GENERIC_FAILURE -13
extern STATUS Ros_UserLan_IsLinkUp(USHORT if_no, BOOL* const is_up /*out*/);


#endif  // MOTOROS_PLATFORM_LIB_H
