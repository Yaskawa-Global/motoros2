// PositionMonitor.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_POSITION_MONITOR_H
#define MOTOROS2_POSITION_MONITOR_H

typedef enum
{
    tfLink_WorldToBase = 0,
    tfLink_BaseToFlange,
    tfLink_FlangeToTool0,
    tfLink_FlangeToTcp,

    NUMBER_TRANSFORM_LINKS_PER_ROBOT
} TransformLinkIndex;

typedef struct
{
    rcl_publisher_t jointStateAllGroups;
    rcl_publisher_t transform;
} PositionMonitor_Publishers;
extern PositionMonitor_Publishers g_publishers_PositionMonitor;

typedef struct
{
    sensor_msgs__msg__JointState* jointStateAllGroups;
    tf2_msgs__msg__TFMessage* transform;
} PositionMonitor_Messages;
extern PositionMonitor_Messages g_messages_PositionMonitor;

extern void Ros_PositionMonitor_Initialize();
extern void Ros_PositionMonitor_Cleanup();

extern void Ros_PositionMonitor_UpdateLocation();

#endif  // MOTOROS2_POSITION_MONITOR_H
