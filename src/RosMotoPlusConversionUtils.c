//RosMotoPlusConversionUtils.c

// SPDX-FileCopyrightText: 2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"


void Ros_MpCoord_To_GeomMsgsPose(MP_COORD const* const mp_coord, geometry_msgs__msg__Pose* const ros_pose)
{
    ros_pose->position.x = MICROMETERS_TO_METERS(mp_coord->x);
    ros_pose->position.y = MICROMETERS_TO_METERS(mp_coord->y);
    ros_pose->position.z = MICROMETERS_TO_METERS(mp_coord->z);
    QuatConversion_MpCoordOrient_To_GeomMsgsQuaternion(
        mp_coord->rx, mp_coord->ry, mp_coord->rz, &ros_pose->orientation);
}

void Ros_MpCoord_To_GeomMsgsTransform(MP_COORD const* const mp_coord, geometry_msgs__msg__Transform* const ros_transform)
{
    ros_transform->translation.x = MICROMETERS_TO_METERS(mp_coord->x);
    ros_transform->translation.y = MICROMETERS_TO_METERS(mp_coord->y);
    ros_transform->translation.z = MICROMETERS_TO_METERS(mp_coord->z);
    QuatConversion_MpCoordOrient_To_GeomMsgsQuaternion(
        mp_coord->rx, mp_coord->ry, mp_coord->rz, &ros_transform->rotation);
}
