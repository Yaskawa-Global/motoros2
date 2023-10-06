// Tests_RosMotoPlusConversionUtils.c

// SPDX-FileCopyrightText: 2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifdef MOTOROS2_TESTING_ENABLE

#include "MotoROS.h"


BOOL Ros_Testing_Ros_MpCoord_To_GeomMsgsPose()
{
    BOOL bSuccess = TRUE;
    MP_COORD mp_coord_ = { 0 };
    geometry_msgs__msg__Pose ros_pose_ = { { 0 } };

    mp_coord_.x  = 1e6;
    mp_coord_.y  = 2e6;
    mp_coord_.z  = 3e6;
    mp_coord_.rx =   4;
    mp_coord_.ry =   5;
    mp_coord_.rz =   6;

    Ros_MpCoord_To_GeomMsgsPose(&mp_coord_, &ros_pose_);

    // position is in micro-meters originally, so the ROS type should have it
    // in meters
    bSuccess &= Ros_Testing_CompareDouble(1.0, ros_pose_.position.x);
    bSuccess &= Ros_Testing_CompareDouble(2.0, ros_pose_.position.y);
    bSuccess &= Ros_Testing_CompareDouble(3.0, ros_pose_.position.z);

    // orientation is a Quaternion (and 'in' radians)
    // use the inverse (quaternion -> MP_COORD) and compare to the original
    LONG rx_ = 0, ry_ = 0, rz_ = 0;
    QuatConversion_GeomMsgsQuaternion_To_MpCoordOrient(
        &ros_pose_.orientation, &rx_, &ry_, &rz_);

    // NOTE: this is not a strict equals test right now (uses non-zero epsilon)
    bSuccess &= Ros_Testing_CompareLong(mp_coord_.rx, rx_);
    bSuccess &= Ros_Testing_CompareLong(mp_coord_.ry, ry_);
    bSuccess &= Ros_Testing_CompareLong(mp_coord_.rz, rz_);

    Ros_Debug_BroadcastMsg("Testing Ros_MpCoord_To_GeomMsgsPose: %s", bSuccess ? "PASS" : "FAIL");
    return bSuccess;
}

BOOL Ros_Testing_Ros_MpCoord_To_GeomMsgsTransform()
{
    BOOL bSuccess = TRUE;
    MP_COORD mp_coord_ = { 0 };
    geometry_msgs__msg__Transform ros_tf_ = { { 0 } };

    mp_coord_.x  = 4e6;
    mp_coord_.y  = 5e6;
    mp_coord_.z  = 6e6;
    mp_coord_.rx =   1;
    mp_coord_.ry =   2;
    mp_coord_.rz =   3;

    Ros_MpCoord_To_GeomMsgsTransform(&mp_coord_, &ros_tf_);

    // translation is in micro-meters originally, so the ROS type should have it
    // in meters
    bSuccess &= Ros_Testing_CompareDouble(4.0, ros_tf_.translation.x);
    bSuccess &= Ros_Testing_CompareDouble(5.0, ros_tf_.translation.y);
    bSuccess &= Ros_Testing_CompareDouble(6.0, ros_tf_.translation.z);

    // rotation is a Quaternion (and 'in' radians)
    // use the inverse (quaternion -> MP_COORD) and compare to the original
    LONG rx_ = 0, ry_ = 0, rz_ = 0;
    QuatConversion_GeomMsgsQuaternion_To_MpCoordOrient(
        &ros_tf_.rotation, &rx_, &ry_, &rz_);

    // NOTE: this is not a strict equals test right now (uses non-zero epsilon)
    bSuccess &= Ros_Testing_CompareLong(mp_coord_.rx, rx_);
    bSuccess &= Ros_Testing_CompareLong(mp_coord_.ry, ry_);
    bSuccess &= Ros_Testing_CompareLong(mp_coord_.rz, rz_);

    Ros_Debug_BroadcastMsg("Testing Ros_MpCoord_To_GeomMsgsTransform: %s", bSuccess ? "PASS" : "FAIL");
    return bSuccess;
}

BOOL Ros_Testing_RosMotoPlusConversionUtils()
{
    BOOL bSuccess = TRUE;

    bSuccess &= Ros_Testing_Ros_MpCoord_To_GeomMsgsPose();
    bSuccess &= Ros_Testing_Ros_MpCoord_To_GeomMsgsTransform();

    return bSuccess;
}

#endif //MOTOROS2_TESTING_ENABLE
