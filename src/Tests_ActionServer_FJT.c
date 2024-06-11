// Tests_ActionServer_FJT.c

// SPDX-FileCopyrightText: 2024, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2024, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0


#ifdef MOTOROS2_TESTING_ENABLE

#include "MotoROS.h"
#include <control_msgs/msg/joint_tolerance.h>


static BOOL Ros_Testing_Ros_ActionServer_FJT_Parse_GoalPosTolerances_null_args()
{
    BOOL bSuccess = TRUE;

    STATUS status = 0;
    control_msgs__msg__JointTolerance__Sequence joint_tolerances;
    rosidl_runtime_c__String__Sequence joint_names;
    const size_t NUM_JOINTS = 6;
    double posTolerances[NUM_JOINTS];

    status = Ros_ActionServer_FJT_Parse_GoalPosTolerances(NULL, &joint_names, posTolerances, NUM_JOINTS);

    BOOL bT00 = status == -1;
    bSuccess &= bT00;
    Ros_Debug_BroadcastMsg("Testing %s: goal_joint_tolerances NULL: %s", __func__, bT00 ? "PASS" : "FAIL");

    status = 0;
    status = Ros_ActionServer_FJT_Parse_GoalPosTolerances(&joint_tolerances, NULL, posTolerances, NUM_JOINTS);

    BOOL bT01 = status == -1;
    bSuccess &= bT01;
    Ros_Debug_BroadcastMsg("Testing %s: joint_names NULL: %s", __func__, bT01 ? "PASS" : "FAIL");

    status = 0;
    status = Ros_ActionServer_FJT_Parse_GoalPosTolerances(&joint_tolerances, &joint_names, NULL, NUM_JOINTS);

    BOOL bT02 = status == -1;
    bSuccess &= bT02;
    Ros_Debug_BroadcastMsg("Testing %s: posTolerances NULL: %s", __func__, bT02 ? "PASS" : "FAIL");

    Ros_Debug_BroadcastMsg("Testing %s: %s", __func__, bSuccess ? "PASS" : "FAIL");
    return bSuccess;
}

static BOOL Ros_Testing_Ros_ActionServer_FJT_Parse_GoalPosTolerances_empty_jtolerances()
{
    BOOL bSuccess = TRUE;

    STATUS status = 0;
    control_msgs__msg__JointTolerance__Sequence joint_tolerances;
    rosidl_runtime_c__String__Sequence joint_names;
    const size_t NUM_JOINTS = 6;
    double posTolerances[NUM_JOINTS];

    //init capacity to NUM_JOINTS, but don't add any elements
    control_msgs__msg__JointTolerance__Sequence__init(&joint_tolerances, NUM_JOINTS);
    joint_tolerances.size = 0;

    //this should now return OK, and init posTolerances to all defaults
    status = Ros_ActionServer_FJT_Parse_GoalPosTolerances(&joint_tolerances, &joint_names, posTolerances, NUM_JOINTS);

    BOOL bT00 = status == OK;
    bSuccess &= bT00;
    Ros_Debug_BroadcastMsg("Testing %s: call: %s", __func__, bT00 ? "PASS" : "FAIL");

    BOOL bT01 = (posTolerances[0] == DEFAULT_FJT_GOAL_POSITION_TOLERANCE
                    && posTolerances[1] == DEFAULT_FJT_GOAL_POSITION_TOLERANCE
                    && posTolerances[2] == DEFAULT_FJT_GOAL_POSITION_TOLERANCE
                    && posTolerances[3] == DEFAULT_FJT_GOAL_POSITION_TOLERANCE
                    && posTolerances[4] == DEFAULT_FJT_GOAL_POSITION_TOLERANCE
                    && posTolerances[5] == DEFAULT_FJT_GOAL_POSITION_TOLERANCE
                );
    bSuccess &= bT01;
    Ros_Debug_BroadcastMsg("Testing %s: all defaults: %s", __func__, bT01 ? "PASS" : "FAIL");

    control_msgs__msg__JointTolerance__Sequence__fini(&joint_tolerances);
    rosidl_runtime_c__String__Sequence__fini(&joint_names);

    Ros_Debug_BroadcastMsg("Testing %s: %s", __func__, bSuccess ? "PASS" : "FAIL");
    return bSuccess;
}

static BOOL Ros_Testing_Ros_ActionServer_FJT_Parse_GoalPosTolerances_out_array_too_small()
{
    //output array of size == 1, two joints, single JointTolerance instance in goal.
    //success if 'output array too small' return value is returned

    BOOL bSuccess = TRUE;

    STATUS status = 0;
    control_msgs__msg__JointTolerance joint_tolerance;
    control_msgs__msg__JointTolerance__Sequence joint_tolerances;
    rosidl_runtime_c__String__Sequence joint_names;
    const size_t NUM_JOINTS = 6;

    //make output array of size 1
    const size_t OUTPUT_ARRAY_SZ = 1;
    double posTolerances[OUTPUT_ARRAY_SZ];

    //add 2 joint names
    rosidl_runtime_c__String__Sequence__init(&joint_names, NUM_JOINTS);
    rosidl_runtime_c__String__assign(&joint_names.data[0], "joint0");
    rosidl_runtime_c__String__assign(&joint_names.data[1], "joint1");
    joint_names.size = 2;

    control_msgs__msg__JointTolerance__Sequence__init(&joint_tolerances, NUM_JOINTS);
    joint_tolerances.size = 0;

    //add (at least) a single JointTolerance (don't need to init it, just add it)
    bzero(&joint_tolerance, sizeof(joint_tolerance));
    control_msgs__msg__JointTolerance__init(&joint_tolerance);
    joint_tolerances.data[0] = joint_tolerance;
    joint_tolerances.size = 1;

    //call
    status = Ros_ActionServer_FJT_Parse_GoalPosTolerances(&joint_tolerances, &joint_names, posTolerances, OUTPUT_ARRAY_SZ);

    BOOL bT00 = status == -2;
    bSuccess &= bT00;
    Ros_Debug_BroadcastMsg("Testing %s: out array too small: %s", __func__, bT00 ? "PASS" : "FAIL");

    control_msgs__msg__JointTolerance__Sequence__fini(&joint_tolerances);
    rosidl_runtime_c__String__Sequence__fini(&joint_names);

    Ros_Debug_BroadcastMsg("Testing %s: %s", __func__, bSuccess ? "PASS" : "FAIL");
    return bSuccess;
}

static BOOL Ros_Testing_Ros_ActionServer_FJT_Parse_GoalPosTolerances_more_jtol_than_jnames()
{
    //hypothetical controller with zero joints (so zero joint names, normal sized output array),
    //but goal with single JointTolerance instance.
    //success if JointTolerance is 'ignored'.

    BOOL bSuccess = TRUE;

    STATUS status = 0;
    control_msgs__msg__JointTolerance joint_tolerance;
    control_msgs__msg__JointTolerance__Sequence joint_tolerances;
    rosidl_runtime_c__String__Sequence joint_names;
    const size_t NUM_JOINTS = 6;

    //zero jnames, zero output array sz
    double posTolerances[NUM_JOINTS];
    rosidl_runtime_c__String__Sequence__init(&joint_names, NUM_JOINTS);
    joint_names.size = 1;

    control_msgs__msg__JointTolerance__Sequence__init(&joint_tolerances, NUM_JOINTS);
    joint_tolerances.size = 0;

    //add a single JointTolerance
    bzero(&joint_tolerance, sizeof(joint_tolerance));
    control_msgs__msg__JointTolerance__init(&joint_tolerance);
    rosidl_runtime_c__String__assign(&joint_tolerance.name, "joint5");
    joint_tolerance.position = 1.0;
    joint_tolerances.data[0] = joint_tolerance;
    joint_tolerances.size = 1;

    //no jnames, zero len output array, but 1 jtol: that's legal, just means the jtol
    //will essentially be ignored
    status = Ros_ActionServer_FJT_Parse_GoalPosTolerances(&joint_tolerances, &joint_names, posTolerances, NUM_JOINTS);

    BOOL bT00 = status == OK;
    bSuccess &= bT00;
    Ros_Debug_BroadcastMsg("Testing %s: call: %s", __func__, bT00 ? "PASS" : "FAIL");

    //should have everything set to defaults (as JointTolerance is ignored)
    BOOL bT01 = (posTolerances[0] == DEFAULT_FJT_GOAL_POSITION_TOLERANCE
                    && posTolerances[1] == DEFAULT_FJT_GOAL_POSITION_TOLERANCE
                    && posTolerances[2] == DEFAULT_FJT_GOAL_POSITION_TOLERANCE
                    && posTolerances[3] == DEFAULT_FJT_GOAL_POSITION_TOLERANCE
                    && posTolerances[4] == DEFAULT_FJT_GOAL_POSITION_TOLERANCE
                    && posTolerances[5] == DEFAULT_FJT_GOAL_POSITION_TOLERANCE);
    bSuccess &= bT01;
    Ros_Debug_BroadcastMsg("Testing %s: all defaults: %s", __func__, bT01 ? "PASS" : "FAIL");

    control_msgs__msg__JointTolerance__Sequence__fini(&joint_tolerances);
    rosidl_runtime_c__String__Sequence__fini(&joint_names);

    Ros_Debug_BroadcastMsg("Testing %s: %s", __func__, bSuccess ? "PASS" : "FAIL");
    return bSuccess;
}

static BOOL Ros_Testing_Ros_ActionServer_FJT_Parse_GoalPosTolerances_jtol_in_slot0()
{
    //hypothetical controller with single joint, goal with single JointTolerance instance.
    //success if JointTolerance position value == first element in output array.

    BOOL bSuccess = TRUE;

    STATUS status = 0;
    control_msgs__msg__JointTolerance joint_tolerance;
    control_msgs__msg__JointTolerance__Sequence joint_tolerances;
    rosidl_runtime_c__String__Sequence joint_names;
    const size_t NUM_JOINTS = 6;
    const char JOINT_NAME[] = "joint0";
    const double J0_POS_TOL = 1.0;
    double posTolerances[NUM_JOINTS];
    bzero(posTolerances, NUM_JOINTS);

    rosidl_runtime_c__String__Sequence__init(&joint_names, NUM_JOINTS);
    rosidl_runtime_c__String__assign(&joint_names.data[0], JOINT_NAME);
    joint_names.size = 1;

    //add a single JointTolerance
    control_msgs__msg__JointTolerance__Sequence__init(&joint_tolerances, NUM_JOINTS);
    joint_tolerances.size = 0;

    bzero(&joint_tolerance, sizeof(joint_tolerance));
    control_msgs__msg__JointTolerance__init(&joint_tolerance);
    rosidl_runtime_c__String__assign(&joint_tolerance.name, JOINT_NAME);
    joint_tolerance.position = J0_POS_TOL;
    joint_tolerance.velocity = 0.0;
    joint_tolerance.acceleration = 0.0;
    joint_tolerances.data[0] = joint_tolerance;
    joint_tolerances.size = 1;

    status = Ros_ActionServer_FJT_Parse_GoalPosTolerances(&joint_tolerances, &joint_names, posTolerances, NUM_JOINTS);

    //call itself must succeed
    BOOL bT00 = status == OK;
    bSuccess &= bT00;
    Ros_Debug_BroadcastMsg("Testing %s: call: %s", __func__, bT00 ? "PASS" : "FAIL");

    //expect position tolerance for 'joint0' in first pos of output array
    //NOTE: using equals, as position tolerance should have been directly copied
    BOOL bT01 = posTolerances[0] == J0_POS_TOL;
    bSuccess &= bT01;
    Ros_Debug_BroadcastMsg("Testing %s: expected j0 pos tol: %s", __func__, bT01 ? "PASS" : "FAIL");

    control_msgs__msg__JointTolerance__Sequence__fini(&joint_tolerances);
    rosidl_runtime_c__String__Sequence__fini(&joint_names);

    Ros_Debug_BroadcastMsg("Testing %s: %s", __func__, bSuccess ? "PASS" : "FAIL");
    return bSuccess;
}

static BOOL Ros_Testing_Ros_ActionServer_FJT_Parse_GoalPosTolerances_jtol_in_slot2()
{
    //hypothetical controller with single joint, goal with single JointTolerance instance.
    //success if JointTolerance position value == third element in output array.

    BOOL bSuccess = TRUE;

    STATUS status = 0;
    control_msgs__msg__JointTolerance joint_tolerance;
    control_msgs__msg__JointTolerance__Sequence joint_tolerances;
    rosidl_runtime_c__String__Sequence joint_names;
    const size_t NUM_JOINTS = 6;
    const size_t JOINT0_IDX = 2;
    const char JOINT_NAME[] = "joint0";
    const double J0_POS_TOL = 1.0;
    double posTolerances[NUM_JOINTS];
    bzero(posTolerances, NUM_JOINTS);

    rosidl_runtime_c__String__Sequence__init(&joint_names, NUM_JOINTS);
    rosidl_runtime_c__String__assign(&joint_names.data[0], "joint1");
    rosidl_runtime_c__String__assign(&joint_names.data[1], "joint2");
    rosidl_runtime_c__String__assign(&joint_names.data[JOINT0_IDX], JOINT_NAME);
    joint_names.size = 3;

    control_msgs__msg__JointTolerance__Sequence__init(&joint_tolerances, NUM_JOINTS);
    joint_tolerances.size = 0;

    //add a single JointTolerance
    bzero(&joint_tolerance, sizeof(joint_tolerance));
    control_msgs__msg__JointTolerance__init(&joint_tolerance);
    rosidl_runtime_c__String__assign(&joint_tolerance.name, JOINT_NAME);
    joint_tolerance.position = J0_POS_TOL;
    joint_tolerance.velocity = 0.0;
    joint_tolerance.acceleration = 0.0;
    joint_tolerances.data[0] = joint_tolerance;
    joint_tolerances.size = 1;

    status = Ros_ActionServer_FJT_Parse_GoalPosTolerances(&joint_tolerances, &joint_names, posTolerances, NUM_JOINTS);

    //call itself must succeed
    BOOL bT00 = status == OK;
    bSuccess &= bT00;
    Ros_Debug_BroadcastMsg("Testing %s: call: %s", __func__, bT00 ? "PASS" : "FAIL");

    //expect position tolerance for 'joint0' in first pos of output array
    //NOTE: using equals, as position tolerance should have been directly copied
    BOOL bT01 = posTolerances[JOINT0_IDX] == J0_POS_TOL;
    bSuccess &= bT01;
    Ros_Debug_BroadcastMsg("Testing %s: j0 pos tol at [%d]: %s", __func__, JOINT0_IDX, bT01 ? "PASS" : "FAIL");

    control_msgs__msg__JointTolerance__Sequence__fini(&joint_tolerances);
    rosidl_runtime_c__String__Sequence__fini(&joint_names);

    Ros_Debug_BroadcastMsg("Testing %s: %s", __func__, bSuccess ? "PASS" : "FAIL");
    return bSuccess;
}

static BOOL Ros_Testing_Ros_ActionServer_FJT_Parse_GoalPosTolerances_last_setting_wins()
{
    BOOL bSuccess = TRUE;

    STATUS status = 0;
    control_msgs__msg__JointTolerance joint_tolerance;
    control_msgs__msg__JointTolerance__Sequence joint_tolerances;
    rosidl_runtime_c__String__Sequence joint_names;
    const size_t NUM_JOINTS = 3;
    double posTolerances[NUM_JOINTS];
    bzero(posTolerances, NUM_JOINTS);

    rosidl_runtime_c__String__Sequence__init(&joint_names, NUM_JOINTS);
    rosidl_runtime_c__String__assign(&joint_names.data[0], "joint0");
    rosidl_runtime_c__String__assign(&joint_names.data[1], "joint2");
    rosidl_runtime_c__String__assign(&joint_names.data[2], "joint1");
    joint_names.size = 3;

    control_msgs__msg__JointTolerance__Sequence__init(&joint_tolerances, NUM_JOINTS);
    joint_tolerances.size = 0;

    //JointTolerance for joint1
    bzero(&joint_tolerance, sizeof(joint_tolerance));
    control_msgs__msg__JointTolerance__init(&joint_tolerance);
    rosidl_runtime_c__String__assign(&joint_tolerance.name, "joint1");
    joint_tolerance.position = 0.123;
    control_msgs__msg__JointTolerance__copy(&joint_tolerance, &joint_tolerances.data[0]);
    joint_tolerances.size = 1;

    //JointTolerance for joint2
    rosidl_runtime_c__String__assign(&joint_tolerance.name, "joint2");
    joint_tolerance.position = 0.234;
    control_msgs__msg__JointTolerance__copy(&joint_tolerance, &joint_tolerances.data[1]);
    joint_tolerances.size = 2;

    //JointTolerance for joint1 -- NOTE: DUPLICATE
    rosidl_runtime_c__String__assign(&joint_tolerance.name, "joint1");
    joint_tolerance.position = 0.345;
    control_msgs__msg__JointTolerance__copy(&joint_tolerance, &joint_tolerances.data[2]);
    joint_tolerances.size = 3;

    status = Ros_ActionServer_FJT_Parse_GoalPosTolerances(&joint_tolerances, &joint_names, posTolerances, NUM_JOINTS);

    //call itself must succeed
    BOOL bT00 = status == OK;
    bSuccess &= bT00;
    Ros_Debug_BroadcastMsg("Testing %s: call: %s", __func__, bT00 ? "PASS" : "FAIL");

    //check joint tols

    //had no JointTolerance instance, so should be default
    BOOL bT01 = posTolerances[0] == DEFAULT_FJT_GOAL_POSITION_TOLERANCE;
    bSuccess &= bT01;
    Ros_Debug_BroadcastMsg("Testing %s: j0 pos tol at [%d]: %s", __func__, 0, bT01 ? "PASS" : "FAIL");

    //single JointTolerance instance
    BOOL bT02 = posTolerances[1] == 0.234;
    bSuccess &= bT02;
    Ros_Debug_BroadcastMsg("Testing %s: j2 pos tol at [%d]: %s", __func__, 1, bT02 ? "PASS" : "FAIL");

    //joint1 has two JointTolerances, last-setting wins, so should be == the last JointTolerance.position value
    BOOL bT03 = posTolerances[2] == 0.345;
    bSuccess &= bT03;
    Ros_Debug_BroadcastMsg("Testing %s: j1 pos tol at [%d]: %s", __func__, 2, bT03 ? "PASS" : "FAIL");

    control_msgs__msg__JointTolerance__Sequence__fini(&joint_tolerances);
    rosidl_runtime_c__String__Sequence__fini(&joint_names);

    Ros_Debug_BroadcastMsg("Testing %s: %s", __func__, bSuccess ? "PASS" : "FAIL");
    return bSuccess;
}

static BOOL Ros_Testing_Ros_ActionServer_FJT_Reorder_TrajPt_To_Internal_Order_null_args()
{
    BOOL bSuccess = TRUE;

    STATUS status = 0;
    trajectory_msgs__msg__JointTrajectoryPoint traj_point;
    rosidl_runtime_c__String__Sequence traj_point_jnames;
    rosidl_runtime_c__String__Sequence internal_jnames;
    const size_t NUM_JOINTS = 6;
    double trajPtValues[NUM_JOINTS];
    bzero(trajPtValues, sizeof(trajPtValues));

    status = Ros_ActionServer_FJT_Reorder_TrajPt_To_Internal_Order(
        NULL, &traj_point_jnames, &internal_jnames, trajPtValues, NUM_JOINTS);
    BOOL bT00 = status == -1;
    bSuccess &= bT00;
    Ros_Debug_BroadcastMsg("Testing %s: traj_point NULL: %s", __func__, bT00 ? "PASS" : "FAIL");

    status = 0;
    status = Ros_ActionServer_FJT_Reorder_TrajPt_To_Internal_Order(
        &traj_point, NULL, &internal_jnames, trajPtValues, NUM_JOINTS);
    BOOL bT01 = status == -1;
    bSuccess &= bT01;
    Ros_Debug_BroadcastMsg("Testing %s: traj_point_jnames NULL: %s", __func__, bT01 ? "PASS" : "FAIL");

    status = 0;
    status = Ros_ActionServer_FJT_Reorder_TrajPt_To_Internal_Order(
        &traj_point, &traj_point_jnames, NULL, trajPtValues, NUM_JOINTS);
    BOOL bT02 = status == -1;
    bSuccess &= bT02;
    Ros_Debug_BroadcastMsg("Testing %s: internal_jnames NULL: %s", __func__, bT02 ? "PASS" : "FAIL");

    status = 0;
    status = Ros_ActionServer_FJT_Reorder_TrajPt_To_Internal_Order(
        &traj_point, &traj_point_jnames, &internal_jnames, NULL, NUM_JOINTS);
    BOOL bT03 = status == -1;
    bSuccess &= bT03;
    Ros_Debug_BroadcastMsg("Testing %s: trajPtValues NULL: %s", __func__, bT03 ? "PASS" : "FAIL");

    Ros_Debug_BroadcastMsg("Testing %s: %s", __func__, bSuccess ? "PASS" : "FAIL");
    return bSuccess;
}

static BOOL Ros_Testing_Ros_ActionServer_FJT_Reorder_TrajPt_To_Internal_Order_jnames_len_neq()
{
    BOOL bSuccess = TRUE;

    STATUS status = 0;
    trajectory_msgs__msg__JointTrajectoryPoint traj_point;
    rosidl_runtime_c__String__Sequence traj_point_jnames;
    rosidl_runtime_c__String__Sequence internal_jnames;
    const size_t NUM_JOINTS = 6;
    double trajPtValues[NUM_JOINTS];
    bzero(trajPtValues, sizeof(trajPtValues));

    //add 1 traj joint name
    rosidl_runtime_c__String__Sequence__init(&traj_point_jnames, NUM_JOINTS);
    rosidl_runtime_c__String__assign(&traj_point_jnames.data[0], "joint0");
    traj_point_jnames.size = 1;

    //add 2 joint names
    rosidl_runtime_c__String__Sequence__init(&internal_jnames, NUM_JOINTS);
    rosidl_runtime_c__String__assign(&internal_jnames.data[0], "joint0");
    rosidl_runtime_c__String__assign(&internal_jnames.data[1], "joint1");
    internal_jnames.size = 2;

    //call
    status = Ros_ActionServer_FJT_Reorder_TrajPt_To_Internal_Order(
        &traj_point, &traj_point_jnames, &internal_jnames, trajPtValues, NUM_JOINTS);

    BOOL bT00 = status == -2;
    bSuccess &= bT00;
    Ros_Debug_BroadcastMsg("Testing %s: partial pt names: %s", __func__, bT00 ? "PASS" : "FAIL");

    rosidl_runtime_c__String__Sequence__fini(&internal_jnames);
    rosidl_runtime_c__String__Sequence__fini(&traj_point_jnames);

    Ros_Debug_BroadcastMsg("Testing %s: %s", __func__, bSuccess ? "PASS" : "FAIL");
    return bSuccess;
}

static BOOL Ros_Testing_Ros_ActionServer_FJT_Reorder_TrajPt_To_Internal_Order_traj_pt_jnames_neq()
{
    //try to re-order values for a trajectory point which doesn't have a sufficient
    //nr of position values (at least not when compared to the nr of internal joint
    //names).
    //This requires us to setup an equal nr of joint names for the trajectory point
    //as well as the internal joint names, but a different nr of position values in
    //the traj pt 'positions' array. That would be an 'illegal' traj pt (and
    //probably would be caught earlier than this function), but that's on purpose
    //here of course in this test.

    BOOL bSuccess = TRUE;

    STATUS status = 0;
    trajectory_msgs__msg__JointTrajectoryPoint traj_point;
    rosidl_runtime_c__String__Sequence traj_point_jnames;
    rosidl_runtime_c__String__Sequence internal_jnames;
    const size_t NUM_JOINTS = 6;
    double trajPtValues[NUM_JOINTS];
    bzero(trajPtValues, sizeof(trajPtValues));

    //add a single position value: this creates an invalid traj pt, but that's
    //on purpose of course
    bzero(&traj_point, sizeof(traj_point));
    trajectory_msgs__msg__JointTrajectoryPoint__init(&traj_point);
    traj_point.positions.data[0] = 0.0;
    traj_point.positions.size = 1;

    //add 2 traj joint names
    rosidl_runtime_c__String__Sequence__init(&traj_point_jnames, NUM_JOINTS);
    rosidl_runtime_c__String__assign(&traj_point_jnames.data[0], "joint0");
    rosidl_runtime_c__String__assign(&traj_point_jnames.data[1], "joint1");
    traj_point_jnames.size = 2;

    //add 2 joint names
    rosidl_runtime_c__String__Sequence__init(&internal_jnames, NUM_JOINTS);
    rosidl_runtime_c__String__assign(&internal_jnames.data[0], "joint0");
    rosidl_runtime_c__String__assign(&internal_jnames.data[1], "joint1");
    internal_jnames.size = 2;

    //call
    status = Ros_ActionServer_FJT_Reorder_TrajPt_To_Internal_Order(
        &traj_point, &traj_point_jnames, &internal_jnames, trajPtValues, NUM_JOINTS);

    BOOL bT00 = status == -3;
    bSuccess &= bT00;
    Ros_Debug_BroadcastMsg("Testing %s: partial pt pos: %s", __func__, bT00 ? "PASS" : "FAIL");

    rosidl_runtime_c__String__Sequence__fini(&internal_jnames);
    rosidl_runtime_c__String__Sequence__fini(&traj_point_jnames);
    trajectory_msgs__msg__JointTrajectoryPoint__fini(&traj_point);

    Ros_Debug_BroadcastMsg("Testing %s: %s", __func__, bSuccess ? "PASS" : "FAIL");
    return bSuccess;
}

static BOOL Ros_Testing_Ros_ActionServer_FJT_Reorder_TrajPt_To_Internal_Order_out_array_too_small()
{
    //try to re-order values for a trajectory point while the output array is
    //too small.

    BOOL bSuccess = TRUE;

    STATUS status = 0;
    trajectory_msgs__msg__JointTrajectoryPoint traj_point;
    rosidl_runtime_c__String__Sequence traj_point_jnames;
    rosidl_runtime_c__String__Sequence internal_jnames;
    const size_t NUM_JOINTS = 6;

    //make output array of size 1
    const size_t OUTPUT_ARRAY_SZ = 1;
    double trajPtValues[OUTPUT_ARRAY_SZ];
    bzero(trajPtValues, sizeof(trajPtValues));

    bzero(&traj_point, sizeof(traj_point));
    trajectory_msgs__msg__JointTrajectoryPoint__init(&traj_point);
    rosidl_runtime_c__double__Sequence__init(&traj_point.positions, NUM_JOINTS);
    traj_point.positions.data[0] = 0.0;
    traj_point.positions.data[1] = 0.0;
    traj_point.positions.size = 2;

    //add 2 traj joint names
    rosidl_runtime_c__String__Sequence__init(&traj_point_jnames, NUM_JOINTS);
    rosidl_runtime_c__String__assign(&traj_point_jnames.data[0], "joint0");
    rosidl_runtime_c__String__assign(&traj_point_jnames.data[1], "joint1");
    traj_point_jnames.size = 2;

    //add 2 joint names
    rosidl_runtime_c__String__Sequence__init(&internal_jnames, NUM_JOINTS);
    rosidl_runtime_c__String__assign(&internal_jnames.data[0], "joint0");
    rosidl_runtime_c__String__assign(&internal_jnames.data[1], "joint1");
    internal_jnames.size = 2;

    //call
    status = Ros_ActionServer_FJT_Reorder_TrajPt_To_Internal_Order(
        &traj_point, &traj_point_jnames, &internal_jnames, trajPtValues, OUTPUT_ARRAY_SZ);

    BOOL bT00 = status == -4;
    bSuccess &= bT00;
    Ros_Debug_BroadcastMsg("Testing %s: out array too small: %s", __func__, bT00 ? "PASS" : "FAIL");

    rosidl_runtime_c__String__Sequence__fini(&internal_jnames);
    rosidl_runtime_c__String__Sequence__fini(&traj_point_jnames);
    trajectory_msgs__msg__JointTrajectoryPoint__fini(&traj_point);

    Ros_Debug_BroadcastMsg("Testing %s: %s", __func__, bSuccess ? "PASS" : "FAIL");
    return bSuccess;
}

static BOOL Ros_Testing_Ros_ActionServer_FJT_Reorder_TrajPt_To_Internal_Order_single_jt()
{
    //try to re-order values for a trajectory point for a single joint

    BOOL bSuccess = TRUE;

    STATUS status = 0;
    trajectory_msgs__msg__JointTrajectoryPoint traj_point;
    rosidl_runtime_c__String__Sequence traj_point_jnames;
    rosidl_runtime_c__String__Sequence internal_jnames;
    const size_t NUM_JOINTS = 6;
    double trajPtValues[NUM_JOINTS];
    bzero(trajPtValues, sizeof(trajPtValues));

    bzero(&traj_point, sizeof(traj_point));
    trajectory_msgs__msg__JointTrajectoryPoint__init(&traj_point);
    rosidl_runtime_c__double__Sequence__init(&traj_point.positions, NUM_JOINTS);
    traj_point.positions.data[0] = 1.234;
    traj_point.positions.size = 1;

    //add 1 traj joint name
    rosidl_runtime_c__String__Sequence__init(&traj_point_jnames, NUM_JOINTS);
    rosidl_runtime_c__String__assign(&traj_point_jnames.data[0], "joint0");
    traj_point_jnames.size = 1;

    //add 1 joint name
    rosidl_runtime_c__String__Sequence__init(&internal_jnames, NUM_JOINTS);
    rosidl_runtime_c__String__assign(&internal_jnames.data[0], "joint0");
    internal_jnames.size = 1;

    //call
    status = Ros_ActionServer_FJT_Reorder_TrajPt_To_Internal_Order(
        &traj_point, &traj_point_jnames, &internal_jnames, trajPtValues, NUM_JOINTS);

    BOOL bT00 = status == OK;
    bSuccess &= bT00;
    Ros_Debug_BroadcastMsg("Testing %s: call: %s", __func__, bT00 ? "PASS" : "FAIL");

    //expect pos val for 'joint0' in first pos of output array
    BOOL bT01 = trajPtValues[0] == 1.234;
    bSuccess &= bT01;
    Ros_Debug_BroadcastMsg("Testing %s: j0 pos at [%d]: %s", __func__, 0, bT01 ? "PASS" : "FAIL");

    rosidl_runtime_c__String__Sequence__fini(&internal_jnames);
    rosidl_runtime_c__String__Sequence__fini(&traj_point_jnames);
    trajectory_msgs__msg__JointTrajectoryPoint__fini(&traj_point);

    Ros_Debug_BroadcastMsg("Testing %s: %s", __func__, bSuccess ? "PASS" : "FAIL");
    return bSuccess;
}

static BOOL Ros_Testing_Ros_ActionServer_FJT_Reorder_TrajPt_To_Internal_Order_6_jt_reorder()
{
    BOOL bSuccess = TRUE;

    STATUS status = 0;
    trajectory_msgs__msg__JointTrajectoryPoint traj_point;
    rosidl_runtime_c__String__Sequence traj_point_jnames;
    rosidl_runtime_c__String__Sequence internal_jnames;
    const size_t NUM_JOINTS = 6;
    double trajPtValues[NUM_JOINTS];

    bzero(&traj_point, sizeof(traj_point));
    trajectory_msgs__msg__JointTrajectoryPoint__init(&traj_point);
    rosidl_runtime_c__double__Sequence__init(&traj_point.positions, NUM_JOINTS);
    traj_point.positions.data[0] = 0.123;
    traj_point.positions.data[1] = 1.234;
    traj_point.positions.data[2] = 2.345;
    traj_point.positions.data[3] = 3.456;
    traj_point.positions.data[4] = 4.567;
    traj_point.positions.data[5] = 5.678;
    traj_point.positions.size = 6;

    //add 6 traj joint names
    rosidl_runtime_c__String__Sequence__init(&traj_point_jnames, NUM_JOINTS);
    rosidl_runtime_c__String__assign(&traj_point_jnames.data[0], "joint0");
    rosidl_runtime_c__String__assign(&traj_point_jnames.data[1], "joint1");
    rosidl_runtime_c__String__assign(&traj_point_jnames.data[2], "joint2");
    rosidl_runtime_c__String__assign(&traj_point_jnames.data[3], "joint3");
    rosidl_runtime_c__String__assign(&traj_point_jnames.data[4], "joint4");
    rosidl_runtime_c__String__assign(&traj_point_jnames.data[5], "joint5");
    traj_point_jnames.size = 6;

    //add 6 joint names (note: different order)
    rosidl_runtime_c__String__Sequence__init(&internal_jnames, NUM_JOINTS);
    rosidl_runtime_c__String__assign(&internal_jnames.data[0], "joint0");
    rosidl_runtime_c__String__assign(&internal_jnames.data[2], "joint1");
    rosidl_runtime_c__String__assign(&internal_jnames.data[4], "joint2");
    rosidl_runtime_c__String__assign(&internal_jnames.data[1], "joint3");
    rosidl_runtime_c__String__assign(&internal_jnames.data[5], "joint4");
    rosidl_runtime_c__String__assign(&internal_jnames.data[3], "joint5");
    internal_jnames.size = 6;

    //call
    status = Ros_ActionServer_FJT_Reorder_TrajPt_To_Internal_Order(
        &traj_point, &traj_point_jnames, &internal_jnames, trajPtValues, NUM_JOINTS);

    BOOL bT00 = status == OK;
    bSuccess &= bT00;
    Ros_Debug_BroadcastMsg("Testing %s: call: %s", __func__, bT00 ? "PASS" : "FAIL");

    //check values are in the correct order
    BOOL bT01 = (
           trajPtValues[0] == 0.123
        && trajPtValues[1] == 3.456
        && trajPtValues[2] == 1.234
        && trajPtValues[3] == 5.678
        && trajPtValues[4] == 2.345
        && trajPtValues[5] == 4.567);
    bSuccess &= bT01;
    Ros_Debug_BroadcastMsg("Testing %s: value reordering: %s", __func__, bT01 ? "PASS" : "FAIL");

    rosidl_runtime_c__String__Sequence__fini(&internal_jnames);
    rosidl_runtime_c__String__Sequence__fini(&traj_point_jnames);
    trajectory_msgs__msg__JointTrajectoryPoint__fini(&traj_point);

    Ros_Debug_BroadcastMsg("Testing %s: %s", __func__, bSuccess ? "PASS" : "FAIL");
    return bSuccess;
}

BOOL Ros_Testing_ActionServer_FJT()
{
    BOOL bSuccess = TRUE;

    Ros_Debug_BroadcastMsg("~~~");
    bSuccess &= Ros_Testing_Ros_ActionServer_FJT_Parse_GoalPosTolerances_null_args();
    Ros_Debug_BroadcastMsg("~~~");
    bSuccess &= Ros_Testing_Ros_ActionServer_FJT_Parse_GoalPosTolerances_empty_jtolerances();
    Ros_Debug_BroadcastMsg("~~~");
    bSuccess &= Ros_Testing_Ros_ActionServer_FJT_Parse_GoalPosTolerances_out_array_too_small();
    Ros_Debug_BroadcastMsg("~~~");
    bSuccess &= Ros_Testing_Ros_ActionServer_FJT_Parse_GoalPosTolerances_more_jtol_than_jnames();
    Ros_Debug_BroadcastMsg("~~~");
    bSuccess &= Ros_Testing_Ros_ActionServer_FJT_Parse_GoalPosTolerances_jtol_in_slot0();
    Ros_Debug_BroadcastMsg("~~~");
    bSuccess &= Ros_Testing_Ros_ActionServer_FJT_Parse_GoalPosTolerances_jtol_in_slot2();
    Ros_Debug_BroadcastMsg("~~~");
    bSuccess &= Ros_Testing_Ros_ActionServer_FJT_Parse_GoalPosTolerances_last_setting_wins();

    Ros_Debug_BroadcastMsg("~~~");
    bSuccess &= Ros_Testing_Ros_ActionServer_FJT_Reorder_TrajPt_To_Internal_Order_null_args();
    Ros_Debug_BroadcastMsg("~~~");
    bSuccess &= Ros_Testing_Ros_ActionServer_FJT_Reorder_TrajPt_To_Internal_Order_jnames_len_neq();
    Ros_Debug_BroadcastMsg("~~~");
    bSuccess &= Ros_Testing_Ros_ActionServer_FJT_Reorder_TrajPt_To_Internal_Order_traj_pt_jnames_neq();
    Ros_Debug_BroadcastMsg("~~~");
    bSuccess &= Ros_Testing_Ros_ActionServer_FJT_Reorder_TrajPt_To_Internal_Order_out_array_too_small();
    Ros_Debug_BroadcastMsg("~~~");
    bSuccess &= Ros_Testing_Ros_ActionServer_FJT_Reorder_TrajPt_To_Internal_Order_single_jt();
    Ros_Debug_BroadcastMsg("~~~");
    bSuccess &= Ros_Testing_Ros_ActionServer_FJT_Reorder_TrajPt_To_Internal_Order_6_jt_reorder();
    Ros_Debug_BroadcastMsg("~~~");

    return bSuccess;
}

#endif //MOTOROS2_TESTING_ENABLE
