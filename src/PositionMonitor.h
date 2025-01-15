// PositionMonitor.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_POSITION_MONITOR_H
#define MOTOROS2_POSITION_MONITOR_H

typedef enum
{
    publishIndex_tfLink_WorldToBase = 0,
    publishIndex_tfLink_BaseToFlange,
    //publishIndex_tfLink_FlangeToTool0,
    publishIndex_tfLink_FlangeToTcp,
    PUBLISHED_NUMBER_TRANSFORM_LINKS_PER_ROBOT
} PublishedTransformLinkIndex;

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

typedef struct
{
    MP_FRAME frameTool0ToFlange, frameFlangeToTool0;
    MP_COORD coordFlangeToTool0;
} TF_Static_Data;

extern void Ros_PositionMonitor_Initialize(TF_Static_Data* tf_static_data, rcl_publisher_t* publisher_transform_static, tf2_msgs__msg__TFMessage* msg_transform_static);
extern void Ros_PositionMonitor_Cleanup(rcl_publisher_t* publisher_transform_static, tf2_msgs__msg__TFMessage* msg_transform_static);
extern void Ros_PositionMonitor_UpdateLocation(TF_Static_Data *);
extern bool Ros_PositionMonitor_Send_TF_Static(TF_Static_Data*, rcl_publisher_t* publisher_transform_static, tf2_msgs__msg__TFMessage* msg_transform_static);
void Ros_PositionMonitor_CalculateStaticTransforms(TF_Static_Data*);

#endif  // MOTOROS2_POSITION_MONITOR_H
