// StaticTransformBroadcaster.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_STATIC_TRANSFORM_BROADCASTER_H
#define MOTOROS2_STATIC_TRANSFORM_BROADCASTER_H

extern rcl_publisher_t g_publishers_transform_static;
extern tf2_msgs__msg__TFMessage* g_messages_transform_static;

#define STATIC_TRANSFORM_BROADCASTER_MAX_TF_COUNT 16

extern bool Ros_StaticTransformBroadcaster_Send(geometry_msgs__msg__TransformStamped *msg, int count);
extern void Ros_StaticTransformBroadcaster_Init();
extern void Ros_StaticTransformBroadcaster_Cleanup();

#endif  // MOTOROS2_STATIC_TRANSFORM_BROADCASTER_H
