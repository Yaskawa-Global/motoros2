//Debug.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_DEBUG_H
#define MOTOROS2_DEBUG_H
#if defined (YRC1000) || defined (YRC1000u)
#define MAX_NETWORK_PORTS 2
#else
#define MAX_NETWORK_PORTS 1
#endif

extern void Ros_Debug_SetFromConfig();
extern void Ros_Debug_BroadcastMsg(char* fmt, ...);
extern void Ros_Debug_LogToConsole(char* fmt, ...);

#endif  // MOTOROS2_DEBUG_H
