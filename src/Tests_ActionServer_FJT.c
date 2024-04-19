// Tests_ActionServer_FJT.c

// SPDX-FileCopyrightText: 2024, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2024, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0


#ifdef MOTOROS2_TESTING_ENABLE

#include "MotoROS.h"
#include <control_msgs/msg/joint_tolerance.h>


BOOL Ros_Testing_Ros_ActionServer_FJT_Parse_GoalPosTolerances_null_args()
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

BOOL Ros_Testing_Ros_ActionServer_FJT_Parse_GoalPosTolerances_empty_jtolerances()
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

BOOL Ros_Testing_Ros_ActionServer_FJT_Parse_GoalPosTolerances_out_array_too_small()
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

BOOL Ros_Testing_Ros_ActionServer_FJT_Parse_GoalPosTolerances_more_jtol_than_jnames()
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

BOOL Ros_Testing_Ros_ActionServer_FJT_Parse_GoalPosTolerances_jtol_in_slot0()
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

BOOL Ros_Testing_Ros_ActionServer_FJT_Parse_GoalPosTolerances_jtol_in_slot2()
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

BOOL Ros_Testing_Ros_ActionServer_FJT_Parse_GoalPosTolerances_last_setting_wins()
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

    return bSuccess;
}

#endif //MOTOROS2_TESTING_ENABLE
