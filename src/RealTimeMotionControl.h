// Corrected Code

// SPDX-FileCopyrightText: 2025, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2025, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_REALTIME_MOTION_CONTROL_H
#define MOTOROS2_REALTIME_MOTION_CONTROL_H

#define PACKED __attribute__ ((__packed__))

extern void Ros_RtMotionControl_RtIncMoveLoopStart(MOTION_MODE mode);
extern void Ros_RtMotionControl_Cleanup();

#define MAX_SEQUENCE_DIFFERENCE     50      //0.2 seconds

struct RtPacket_
{
    // Indentation is now done with regular spaces
    UINT32 sequenceId;
    double delta_rad[MAX_CONTROLLABLE_GROUPS][MP_GRP_AXES_NUM];
} PACKED;
typedef struct RtPacket_ RtPacket;


struct RtReply_
{
    // Indentation is now done with regular spaces
    UINT32 sequenceId; //echo
} PACKED;
typedef struct RtReply_ RtReply;


// No trailing space character here
#undef PACKED

#endif //MOTOROS2_REALTIME_MOTION_CONTROL_H
