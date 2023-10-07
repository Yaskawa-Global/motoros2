// Tests_CtrlGroup.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifdef MOTOROS2_TESTING_ENABLE

#include "MotoROS.h"

void Ros_Testing_CtrlGroup_MakeFake6dofRobot(CtrlGroup* group)
{
    bzero(group, sizeof(CtrlGroup));

    group->groupNo = 0;
    group->numAxes = 6;
    group->groupId = MP_R1_GID;

    group->axisType.type[0] = AXIS_ROTATION; //s
    group->axisType.type[1] = AXIS_ROTATION; //l
    group->axisType.type[2] = AXIS_ROTATION; //u
    group->axisType.type[3] = AXIS_ROTATION; //r
    group->axisType.type[4] = AXIS_ROTATION; //b
    group->axisType.type[5] = AXIS_ROTATION; //t
    group->axisType.type[6] = AXIS_INVALID;
    group->axisType.type[7] = AXIS_INVALID;

    group->pulseToRad.PtoR[0] = 9;
    group->pulseToRad.PtoR[1] = 8;
    group->pulseToRad.PtoR[2] = 7;
    group->pulseToRad.PtoR[3] = 6;
    group->pulseToRad.PtoR[4] = 5;
    group->pulseToRad.PtoR[5] = 4;
    group->pulseToRad.PtoR[6] = 3;
    group->pulseToRad.PtoR[7] = 2;
}

void Ros_Testing_CtrlGroup_MakeFakeDeltaRobot(CtrlGroup* group)
{
    bzero(group, sizeof(CtrlGroup));

    group->groupNo = 0;
    group->numAxes = 4;
    group->groupId = MP_R1_GID;

    group->axisType.type[0] = AXIS_ROTATION; //s
    group->axisType.type[1] = AXIS_ROTATION; //l
    group->axisType.type[2] = AXIS_ROTATION; //u
    group->axisType.type[3] = AXIS_INVALID;
    group->axisType.type[4] = AXIS_INVALID;
    group->axisType.type[5] = AXIS_ROTATION; //t
    group->axisType.type[6] = AXIS_INVALID;
    group->axisType.type[7] = AXIS_INVALID;

    group->pulseToRad.PtoR[0] = 9;
    group->pulseToRad.PtoR[1] = 8;
    group->pulseToRad.PtoR[2] = 7;
    group->pulseToRad.PtoR[3] = 6;
    group->pulseToRad.PtoR[4] = 5;
    group->pulseToRad.PtoR[5] = 4;
    group->pulseToRad.PtoR[6] = 3;
    group->pulseToRad.PtoR[7] = 2;
}

void Ros_Testing_CtrlGroup_MakeFakeSiaRobot(CtrlGroup* group)
{
    bzero(group, sizeof(CtrlGroup));

    group->groupNo = 0;
    group->numAxes = 7;
    group->groupId = MP_R1_GID;

    group->axisType.type[0] = AXIS_ROTATION; //s
    group->axisType.type[1] = AXIS_ROTATION; //l
    group->axisType.type[2] = AXIS_ROTATION; //u
    group->axisType.type[3] = AXIS_ROTATION; //r
    group->axisType.type[4] = AXIS_ROTATION; //b
    group->axisType.type[5] = AXIS_ROTATION; //t
    group->axisType.type[6] = AXIS_ROTATION; //e
    group->axisType.type[7] = AXIS_INVALID;

    group->pulseToRad.PtoR[0] = 9;
    group->pulseToRad.PtoR[1] = 8;
    group->pulseToRad.PtoR[2] = 7;
    group->pulseToRad.PtoR[3] = 6;
    group->pulseToRad.PtoR[4] = 5;
    group->pulseToRad.PtoR[5] = 4;
    group->pulseToRad.PtoR[6] = 3;
    group->pulseToRad.PtoR[7] = 2;
}

BOOL Ros_Testing_CtrlGroup_PosConverters()
{
    CtrlGroup group;
    long motoPos[MAX_PULSE_AXES];
    double rosPos[MAX_PULSE_AXES];
    BOOL bOk, bAllTestsPassed;

    bAllTestsPassed = TRUE;

    //-------------------------------------------------------------------------
    // 6 DOF
    //-------------------------------------------------------------------------
    Ros_Testing_CtrlGroup_MakeFake6dofRobot(&group);

    for (int i = 0; i < MAX_PULSE_AXES; i += 1)
        motoPos[i] = (i * 1000) + 2000; //{2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000}

    Ros_CtrlGroup_ConvertToRosPos(&group, motoPos, rosPos);

    bOk = Ros_Testing_CompareDouble(rosPos[0], (2000.0/9.0));
    bOk &= Ros_Testing_CompareDouble(rosPos[1], (3000.0/8.0));
    bOk &= Ros_Testing_CompareDouble(rosPos[2], (4000.0/7.0));
    bOk &= Ros_Testing_CompareDouble(rosPos[3], (5000.0/6.0));
    bOk &= Ros_Testing_CompareDouble(rosPos[4], (6000.0/5.0));
    bOk &= Ros_Testing_CompareDouble(rosPos[5], (7000.0/4.0));

    bOk &= Ros_Testing_CompareDouble(rosPos[6], 0); //these two axes are not used. they should get zeroed out in Ros_CtrlGroup_ConvertToRosPos
    bOk &= Ros_Testing_CompareDouble(rosPos[7], 0);

    Ros_Debug_BroadcastMsg("Testing CtrlGroup ConvertToRosPos - 6 DOF style: %s", bOk ? "PASS" : "FAIL");
    if (!bOk)
    {
        Ros_Debug_BroadcastMsg("rosPos[0] = %.3f; rosPos[1] = %.3f; rosPos[2] = %.3f; rosPos[3] = %.3f; rosPos[4] = %.3f; rosPos[5] = %.3f; rosPos[6] = %.3f; rosPos[7] = %.3f",
            rosPos[0], rosPos[1], rosPos[2], rosPos[3], rosPos[4], rosPos[5], rosPos[6], rosPos[7]);

        bAllTestsPassed = FALSE;
    }

    rosPos[6] = rosPos[7] = 123.456; //these two axes are not used. they should get zeroed out in Ros_CtrlGroup_ConvertToMotoPos

    Ros_CtrlGroup_ConvertToMotoPos_FromSequentialOrdering(&group, rosPos, motoPos);

    bOk = Ros_Testing_CompareLong(motoPos[0], 2000);
    bOk &= Ros_Testing_CompareLong(motoPos[1], 3000);
    bOk &= Ros_Testing_CompareLong(motoPos[2], 4000);
    bOk &= Ros_Testing_CompareLong(motoPos[3], 5000);
    bOk &= Ros_Testing_CompareLong(motoPos[4], 6000);
    bOk &= Ros_Testing_CompareLong(motoPos[5], 7000);

    bOk &= Ros_Testing_CompareLong(motoPos[6], 0); //these two axes are not used. they should get zeroed out in Ros_CtrlGroup_ConvertToMotoPos
    bOk &= Ros_Testing_CompareLong(motoPos[7], 0);

    Ros_Debug_BroadcastMsg("Testing CtrlGroup ConvertToMotoPos - 6 DOF style: %s", bOk ? "PASS" : "FAIL");
    if (!bOk)
    {
        Ros_Debug_BroadcastMsg("motoPos[0] = %d; motoPos[1] = %d; motoPos[2] = %d; motoPos[3] = %d; motoPos[4] = %d; motoPos[5] = %d; motoPos[6] = %d; motoPos[7] = %d",
            motoPos[0], motoPos[1], motoPos[2], motoPos[3], motoPos[4], motoPos[5], motoPos[6], motoPos[7]);

        bAllTestsPassed = FALSE;
    }

    //-------------------------------------------------------------------------
    // DELTA
    //-------------------------------------------------------------------------
    Ros_Testing_CtrlGroup_MakeFakeDeltaRobot(&group);

    for (int i = 0; i < MAX_PULSE_AXES; i += 1)
        motoPos[i] = (i * 1000) + 2000; //{2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000}

    Ros_CtrlGroup_ConvertToRosPos(&group, motoPos, rosPos);

    bOk = Ros_Testing_CompareDouble(rosPos[0], (2000.0 / 9.0));
    bOk &= Ros_Testing_CompareDouble(rosPos[1], (3000.0 / 8.0));
    bOk &= Ros_Testing_CompareDouble(rosPos[2], (4000.0 / 7.0));
    bOk &= Ros_Testing_CompareDouble(rosPos[3], (7000.0 / 4.0));

    Ros_Debug_BroadcastMsg("Testing CtrlGroup ConvertToRosPos - Delta style: %s", bOk ? "PASS" : "FAIL");
    if (!bOk)
    {
        Ros_Debug_BroadcastMsg("rosPos[0] = %.3f; rosPos[1] = %.3f; rosPos[2] = %.3f; rosPos[3] = %.3f",
            rosPos[0], rosPos[1], rosPos[2], rosPos[3]);

        bAllTestsPassed = FALSE;
    }

    Ros_CtrlGroup_ConvertToMotoPos_FromSequentialOrdering(&group, rosPos, motoPos);

    bOk = Ros_Testing_CompareLong(motoPos[0], 2000);
    bOk &= Ros_Testing_CompareLong(motoPos[1], 3000);
    bOk &= Ros_Testing_CompareLong(motoPos[2], 4000);
    bOk &= Ros_Testing_CompareLong(motoPos[5], 7000);

    Ros_Debug_BroadcastMsg("Testing CtrlGroup ConvertToMotoPos - Delta style: %s", bOk ? "PASS" : "FAIL");
    if (!bOk)
    {
        Ros_Debug_BroadcastMsg("motoPos[0] = %d; motoPos[1] = %d; motoPos[2] = %d; motoPos[5] = %d",
            motoPos[0], motoPos[1], motoPos[2], motoPos[5]);

        bAllTestsPassed = FALSE;
    }

    //-------------------------------------------------------------------------
    // SIA
    //-------------------------------------------------------------------------
    Ros_Testing_CtrlGroup_MakeFakeSiaRobot(&group);

    for (int i = 0; i < MAX_PULSE_AXES; i += 1)
        motoPos[i] = (i * 1000) + 2000; //{2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000}

    Ros_CtrlGroup_ConvertToRosPos(&group, motoPos, rosPos);

    bOk = Ros_Testing_CompareDouble(rosPos[0], (2000.0 / 9.0));
    bOk &= Ros_Testing_CompareDouble(rosPos[1], (3000.0 / 8.0));
    bOk &= Ros_Testing_CompareDouble(rosPos[2], (8000.0 / 3.0));
    bOk &= Ros_Testing_CompareDouble(rosPos[3], (4000.0 / 7.0));
    bOk &= Ros_Testing_CompareDouble(rosPos[4], (5000.0 / 6.0));
    bOk &= Ros_Testing_CompareDouble(rosPos[5], (6000.0 / 5.0));
    bOk &= Ros_Testing_CompareDouble(rosPos[6], (7000.0 / 4.0));

    Ros_Debug_BroadcastMsg("Testing CtrlGroup ConvertToRosPos - SIA style: %s", bOk ? "PASS" : "FAIL");
    if (!bOk)
    {
        Ros_Debug_BroadcastMsg("rosPos[0] = %.3f; rosPos[1] = %.3f; rosPos[2] = %.3f; rosPos[3] = %.3f; rosPos[4] = %.3f; rosPos[5] = %.3f; rosPos[6] = %.3f",
            rosPos[0], rosPos[1], rosPos[2], rosPos[3], rosPos[4], rosPos[5], rosPos[6]);

        bAllTestsPassed = FALSE;
    }

    Ros_CtrlGroup_ConvertToMotoPos_FromSequentialOrdering(&group, rosPos, motoPos);

    bOk = Ros_Testing_CompareLong(motoPos[0], 2000);
    bOk &= Ros_Testing_CompareLong(motoPos[1], 3000);
    bOk &= Ros_Testing_CompareLong(motoPos[2], 4000);
    bOk &= Ros_Testing_CompareLong(motoPos[3], 5000);
    bOk &= Ros_Testing_CompareLong(motoPos[4], 6000);
    bOk &= Ros_Testing_CompareLong(motoPos[5], 7000);
    bOk &= Ros_Testing_CompareLong(motoPos[6], 8000);

    Ros_Debug_BroadcastMsg("Testing CtrlGroup ConvertToMotoPos - SIA style: %s", bOk ? "PASS" : "FAIL");
    if (!bOk)
    {
        Ros_Debug_BroadcastMsg("motoPos[0] = %d; motoPos[1] = %d; motoPos[2] = %d; motoPos[3] = %d; motoPos[4] = %d; motoPos[5] = %d; motoPos[6] = %d",
            motoPos[0], motoPos[1], motoPos[2], motoPos[3], motoPos[4], motoPos[5], motoPos[6]);

        bAllTestsPassed = FALSE;
    }

    return bAllTestsPassed;
}

BOOL Ros_Testing_CtrlGroup()
{
    BOOL bSuccess = TRUE;

    bSuccess &= Ros_Testing_CtrlGroup_PosConverters();

    return bSuccess;
}

#endif //MOTOROS2_TESTING_ENABLE
