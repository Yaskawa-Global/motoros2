// Corrected Code

// SPDX-FileCopyrightText: 2025, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2025, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_REALTIME_MOTION_CONTROL_H
#define MOTOROS2_REALTIME_MOTION_CONTROL_H

#define PACKED __attribute__ ((__packed__))

extern void Ros_RtMotionControl_HyperRobotCommanderX5(MOTION_MODE mode);
extern void Ros_RtMotionControl_Cleanup();

struct RtPacket_
{
    UINT32 sequenceId;
    
    //The order of the joints must be in the order of [S L U R B T E 8].
    //Please note that for seven axis robots, the 'E' joint is phyically
    //mounted in the middle of the arm. But it must be sent at the end
    //of the joint array.
    //
    //For joint-space, this will be radians of each joint.
    //
    //For cartesian, this will be meters and radians of the TCP.
    //The order of the joints must be in the order of [X Y Z Rx Ry Rz Re 8].
    //Rotations are applied in the order of ZYX.
    double delta[MAX_CONTROLLABLE_GROUPS][MP_GRP_AXES_NUM];
} PACKED;
typedef struct RtPacket_ RtPacket;


struct RtReply_
{
    UINT32 sequenceEcho;

    double feedbackPosition[MAX_CONTROLLABLE_GROUPS][MP_GRP_AXES_NUM];

} PACKED;
typedef struct RtReply_ RtReply;

#undef PACKED

#endif //MOTOROS2_REALTIME_MOTION_CONTROL_H
