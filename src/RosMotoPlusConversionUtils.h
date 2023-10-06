//RosMotoPlusConversionUtils.h

// SPDX-FileCopyrightText: 2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_ROS_MOTOPLUS_CONVERSION_UTILS_H
#define MOTOROS2_ROS_MOTOPLUS_CONVERSION_UTILS_H


/**
 * Converts an M+ `MP_COORD` struct to a ROS `geometry_msgs/msg/Pose`.
 *
 * Unit conversion is also performed by this function:
 *
 *  - `MP_COORD` positions are converted from micro-meters to meters
 *  - `MP_COORD` orientations are converted from milli-degrees to radians (and
 *    subsequently to a `Quaternion`)
 *
 * @param mp_coord  Input M+ `MP_COORD` instance
 * @param ros_pose  Output `geometry_msgs/msg/Pose`
 */
extern void Ros_MpCoord_To_GeomMsgsPose(MP_COORD const* const mp_coord, geometry_msgs__msg__Pose* const ros_pose);


/**
 * Converts an M+ `MP_COORD` struct to a ROS `geometry_msgs/msg/Transform`.
 *
 * Unit conversion is also performed by this function:
 *
 *  - `MP_COORD` positions are converted from micro-meters to meters
 *  - `MP_COORD` orientations are converted from milli-degrees to radians (and
 *    subsequently to a `Quaternion`)
 *
 * @param mp_coord       Input M+ `MP_COORD` instance
 * @param ros_transform  Output `geometry_msgs/msg/Pose`
 */
extern void Ros_MpCoord_To_GeomMsgsTransform(MP_COORD const* const mp_coord, geometry_msgs__msg__Transform* const ros_transform);


#endif  // MOTOROS2_ROS_MOTOPLUS_CONVERSION_UTILS_H
