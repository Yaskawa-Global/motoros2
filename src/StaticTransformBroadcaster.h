// StaticTransformBroadcaster.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_STATIC_TRANSFORM_BROADCASTER_H
#define MOTOROS2_STATIC_TRANSFORM_BROADCASTER_H

#define STATIC_TRANSFORM_BROADCASTER_MAX_TF_COUNT 16

extern bool Ros_StaticTransformBroadcaster_Send(rcl_publisher_t *publisher_transform_static, tf2_msgs__msg__TFMessage *msg_transform_static, geometry_msgs__msg__TransformStamped *msg, int count);
extern void Ros_StaticTransformBroadcaster_Init(rcl_publisher_t *publisher_transform_static, tf2_msgs__msg__TFMessage *msg_transform_static);
extern void Ros_StaticTransformBroadcaster_Cleanup(rcl_publisher_t *publisher_transform_static, tf2_msgs__msg__TFMessage *msg_transform_static);

#endif  // MOTOROS2_STATIC_TRANSFORM_BROADCASTER_H
