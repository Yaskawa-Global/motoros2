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

typedef enum
{
    Group_1 = 0,
    Group_2,
    Group_3,
    Group_4,
    Group_5,
    Group_6,
    Group_7,
    Group_8,

    MAX_GROUPS
} GroupIndeces;

typedef enum
{
    Joint_S = 0,    //radians
    Joint_L,
    Joint_U,
    Joint_R,
    Joint_B,
    Joint_T,
    Joint_E,
    Joint_8,

    MAX_JOINTS
} JointIndeces;

typedef enum
{
    TCP_X = 0,      //meters
    TCP_Y,
    TCP_Z,

    TCP_Rx,         //0.0001 degrees
    TCP_Ry,
    TCP_Rz,
    TCP_Re,

    TCP_8,          //pulse

    MAX_AXES //maxies
} CartesianIndeces;

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
    double delta[MAX_GROUPS][MP_GRP_AXES_NUM];
    
    //Set tool that will be used by motion API (ie: passed by us to mpExRcsIncrementMove(..))
    //NOTE: this will change the 'motion tool' ONLY for those increments which
    //      haven't yet been added to the increment queue. See also the ROS 2
    //      'select_tool' service definition file in motoros2_interfaces.
    int toolIndex[MAX_GROUPS]; //TOOL 0 - 63

} PACKED;
typedef struct RtPacket_ RtPacket;


struct RtReply_
{
    UINT32 sequenceEcho;

    double feedbackPosition[MAX_GROUPS][MP_GRP_AXES_NUM];

} PACKED;
typedef struct RtReply_ RtReply;

#undef PACKED

#endif //MOTOROS2_REALTIME_MOTION_CONTROL_H
