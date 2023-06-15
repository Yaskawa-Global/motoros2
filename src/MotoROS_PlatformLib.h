// MotoROS_PlatformLib.h

// SPDX-FileCopyrightText: 2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS_PLATFORM_LIB_H
#define MOTOROS_PLATFORM_LIB_H


// Define the MotoROS_PlatformLib version number as a base-10 integer.
//
// Format: XXXYYZZZ, where:
//
// - XXX: major version
// -  YY: minor version, and
// -  ZZ: patch version nr
//
// Note: no leading zeros, to avoid octal parsing
#define MOTOROS_PLATFORM_LIB_VERSION 202

#define MOTOROS_PLATFORM_LIB_MAJOR (MOTOROS_PLATFORM_LIB_VERSION / 100000)
#define MOTOROS_PLATFORM_LIB_MINOR (MOTOROS_PLATFORM_LIB_VERSION / 100 % 1000)
#define MOTOROS_PLATFORM_LIB_PATCH (MOTOROS_PLATFORM_LIB_VERSION % 100)


// Returns the PlatformLib version number
extern UINT32 MotoROS_PlatformLib_GetVersion();


extern BOOL Ros_IsOtherInstanceRunning();


// Attempt to read MAC address from available network cards
// On YRC1000 tries both network cards
// Returns ERROR (-1) in case of failure
extern STATUS Ros_GetMacAddress(UINT8 macAddress[6]);


#define ROS_USER_LAN1 1   /* general LAN interface1 */
#define ROS_USER_LAN2 2   /* general LAN interface2(only YRC1000) */

// Retrieves the IP address, subnet mask, MAC address and default gateway for the specified network interface.
// NOTE: If this is called immediately upon startup, the default_gw may not be available yet. In this case, the default_gw will be set to 0.
extern STATUS Ros_mpNICData(USHORT if_no, ULONG* ip_addr, ULONG* subnet_mask, UCHAR* mac_addr, ULONG* default_gw);


extern size_t Ros_strnlen(const char *s, size_t maxlen);


extern ULONG tickGet();


#define SO_BROADCAST    0x0020
extern STATUS Ros_setsockopt(int s, int level, int optname, char* optval, int optlen);


extern int localtime_r(const time_t* timer, struct tm* timeBuffer);


#endif  // MOTOROS_PLATFORM_LIB_H
