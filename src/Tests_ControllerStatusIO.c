// Tests_ControllerStatusIO.c

// SPDX-FileCopyrightText: 2024, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2024, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifdef MOTOROS2_TESTING_ENABLE

#include "MotoROS.h"

void Ros_Testing_ControllerStatusIO_MakeFake6dofRobot(CtrlGroup* group, int groupNo, MP_GRP_ID_TYPE groupId)
{
    bzero(group, sizeof(CtrlGroup));

    group->groupNo = groupNo;
    group->groupId = groupId;
    group->numAxes = 6;

    group->baseTrackGroupId = (MP_GRP_ID_TYPE)-1;
    group->baseTrackGroupIndex = -1;

    group->axisType.type[0] = AXIS_ROTATION; //s
    group->axisType.type[1] = AXIS_ROTATION; //l
    group->axisType.type[2] = AXIS_ROTATION; //u
    group->axisType.type[3] = AXIS_ROTATION; //r
    group->axisType.type[4] = AXIS_ROTATION; //b
    group->axisType.type[5] = AXIS_ROTATION; //t
    group->axisType.type[6] = AXIS_INVALID;
    group->axisType.type[7] = AXIS_INVALID;
}

void Ros_Testing_ControllerStatusIO_MakeFakeBaseGroup(CtrlGroup* group, int groupNo, MP_GRP_ID_TYPE groupId)
{
    bzero(group, sizeof(CtrlGroup));

    group->groupNo = groupNo;
    group->groupId = groupId;
    group->numAxes = 1;

    group->baseTrackGroupId = (MP_GRP_ID_TYPE)-1;
    group->baseTrackGroupIndex = -1;

    group->axisType.type[0] = AXIS_LINEAR; //x
    group->axisType.type[1] = AXIS_INVALID;
    group->axisType.type[2] = AXIS_INVALID;
    group->axisType.type[3] = AXIS_INVALID;
    group->axisType.type[4] = AXIS_INVALID;
    group->axisType.type[5] = AXIS_INVALID;
    group->axisType.type[6] = AXIS_INVALID;
    group->axisType.type[7] = AXIS_INVALID;
}

void Ros_Testing_ControllerStatusIO_MakeFakeStationGroup(CtrlGroup* group, int groupNo, MP_GRP_ID_TYPE groupId)
{
    bzero(group, sizeof(CtrlGroup));

    group->groupNo = groupNo;
    group->groupId = groupId;
    group->numAxes = 1;

    group->baseTrackGroupId = (MP_GRP_ID_TYPE)-1;
    group->baseTrackGroupIndex = -1;

    group->axisType.type[0] = AXIS_ROTATION;
    group->axisType.type[1] = AXIS_INVALID;
    group->axisType.type[2] = AXIS_INVALID;
    group->axisType.type[3] = AXIS_INVALID;
    group->axisType.type[4] = AXIS_INVALID;
    group->axisType.type[5] = AXIS_INVALID;
    group->axisType.type[6] = AXIS_INVALID;
    group->axisType.type[7] = AXIS_INVALID;
}

void Ros_Testing_ControllerStatusIO_AssignRobotToBaseGroup(CtrlGroup* robot, CtrlGroup* base)
{
    robot->baseTrackGroupId = base->groupId;
    robot->baseTrackGroupIndex = base->groupNo;
}

BOOL Ros_Testing_ControllerStatusIO_ShouldWarnNoCalibDataLoaded_R1()
{
    BOOL bSuccess = TRUE;

    Controller controller;

    CtrlGroup* grp0 = (CtrlGroup*)mpMalloc(sizeof(CtrlGroup));
    controller.numGroup = 1;
    controller.ctrlGroups[0] = grp0;

    Ros_Testing_ControllerStatusIO_MakeFake6dofRobot(grp0, /*groupNo=*/ 0, /*groupId=*/ MP_R1_GID);

    //R1: NO warning, nothing to calibrate with a single group
    bSuccess &= FALSE == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ FALSE, /*bPublishTfEnabled=*/ FALSE);
    bSuccess &= FALSE == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ TRUE , /*bPublishTfEnabled=*/ FALSE);
    bSuccess &= FALSE == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ FALSE, /*bPublishTfEnabled=*/ TRUE);
    bSuccess &= FALSE == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ TRUE , /*bPublishTfEnabled=*/ TRUE);

    //report overall result
    Ros_Debug_BroadcastMsg("Testing Ros_Testing_ControllerStatusIO_ShouldWarnNoCalibDataLoaded_R1: %s", bSuccess ? "PASS" : "FAIL");

    mpFree(grp0);

    return bSuccess;
}

BOOL Ros_Testing_ControllerStatusIO_ShouldWarnNoCalibDataLoaded_R1B1()
{
    BOOL bSuccess = TRUE;

    Controller controller;
    CtrlGroup* grp0 = (CtrlGroup*)mpMalloc(sizeof(CtrlGroup));
    CtrlGroup* grp1 = (CtrlGroup*)mpMalloc(sizeof(CtrlGroup));

    controller.numGroup = 2;
    controller.ctrlGroups[0] = grp0;
    controller.ctrlGroups[1] = grp1;

    Ros_Testing_ControllerStatusIO_MakeFake6dofRobot(grp0, /*groupNo=*/ 0, /*groupId=*/ MP_R1_GID);
    Ros_Testing_ControllerStatusIO_MakeFakeBaseGroup(grp1, /*groupNo=*/ 1, /*groupId=*/ MP_B1_GID);
    Ros_Testing_ControllerStatusIO_AssignRobotToBaseGroup(/*robot=*/ grp0, /*base=*/ grp1);

    //R1+B1: NO warning, as base tracks don't get calibrated
    bSuccess &= FALSE == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ FALSE, /*bPublishTfEnabled=*/ FALSE);
    bSuccess &= FALSE == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ TRUE , /*bPublishTfEnabled=*/ FALSE);
    bSuccess &= FALSE == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ FALSE, /*bPublishTfEnabled=*/ TRUE);
    bSuccess &= FALSE == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ TRUE , /*bPublishTfEnabled=*/ TRUE);

    //report overall result
    Ros_Debug_BroadcastMsg("Testing Ros_Testing_ControllerStatusIO_ShouldWarnNoCalibDataLoaded_R1B1: %s", bSuccess ? "PASS" : "FAIL");

    mpFree(grp0);
    mpFree(grp1);

    return bSuccess;
}

BOOL Ros_Testing_ControllerStatusIO_ShouldWarnNoCalibDataLoaded_R1R2()
{
    BOOL bSuccess = TRUE;

    Controller controller;
    CtrlGroup* grp0 = (CtrlGroup*)mpMalloc(sizeof(CtrlGroup));
    CtrlGroup* grp1 = (CtrlGroup*)mpMalloc(sizeof(CtrlGroup));

    controller.numGroup = 2;
    controller.ctrlGroups[0] = grp0;
    controller.ctrlGroups[1] = grp1;

    Ros_Testing_ControllerStatusIO_MakeFake6dofRobot(grp0, /*groupNo=*/ 0, /*groupId=*/ MP_R1_GID);
    Ros_Testing_ControllerStatusIO_MakeFake6dofRobot(grp1, /*groupNo=*/ 1, /*groupId=*/ MP_R2_GID);

    //R1+R2: YES warning, but only if TF enabled AND calib failed to load
    bSuccess &= FALSE == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ FALSE, /*bPublishTfEnabled=*/ FALSE);
    bSuccess &= FALSE == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ TRUE , /*bPublishTfEnabled=*/ FALSE);
    bSuccess &= TRUE  == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ FALSE, /*bPublishTfEnabled=*/ TRUE);
    bSuccess &= FALSE == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ TRUE , /*bPublishTfEnabled=*/ TRUE);

    //report overall result
    Ros_Debug_BroadcastMsg("Testing Ros_Testing_ControllerStatusIO_ShouldWarnNoCalibDataLoaded_R1R2: %s", bSuccess ? "PASS" : "FAIL");

    mpFree(grp0);
    mpFree(grp1);

    return bSuccess;
}

BOOL Ros_Testing_ControllerStatusIO_ShouldWarnNoCalibDataLoaded_R1B1R2()
{
    BOOL bSuccess = TRUE;

    Controller controller;
    CtrlGroup* grp0 = (CtrlGroup*)mpMalloc(sizeof(CtrlGroup));
    CtrlGroup* grp1 = (CtrlGroup*)mpMalloc(sizeof(CtrlGroup));
    CtrlGroup* grp2 = (CtrlGroup*)mpMalloc(sizeof(CtrlGroup));

    controller.numGroup = 3;
    controller.ctrlGroups[0] = grp0;
    controller.ctrlGroups[1] = grp1;
    controller.ctrlGroups[2] = grp2;

    Ros_Testing_ControllerStatusIO_MakeFake6dofRobot(grp0, /*groupNo=*/ 0, /*groupId=*/ MP_R1_GID);
    Ros_Testing_ControllerStatusIO_MakeFakeBaseGroup(grp2, /*groupNo=*/ 2, /*groupId=*/ MP_B1_GID);
    Ros_Testing_ControllerStatusIO_AssignRobotToBaseGroup(/*robot=*/ grp0, /*base=*/ grp2);

    Ros_Testing_ControllerStatusIO_MakeFake6dofRobot(grp1, /*groupNo=*/ 1, /*groupId=*/ MP_R2_GID);

    //R1B1+R2: YES warning, but only if TF enabled AND calib failed to load
    bSuccess &= FALSE == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ FALSE, /*bPublishTfEnabled=*/ FALSE);
    bSuccess &= FALSE == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ TRUE , /*bPublishTfEnabled=*/ FALSE);
    bSuccess &= TRUE  == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ FALSE, /*bPublishTfEnabled=*/ TRUE);
    bSuccess &= FALSE == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ TRUE , /*bPublishTfEnabled=*/ TRUE);

    //report overall result
    Ros_Debug_BroadcastMsg("Testing Ros_Testing_ControllerStatusIO_ShouldWarnNoCalibDataLoaded_R1B1R2: %s", bSuccess ? "PASS" : "FAIL");

    mpFree(grp0);
    mpFree(grp1);
    mpFree(grp2);

    return bSuccess;
}

BOOL Ros_Testing_ControllerStatusIO_ShouldWarnNoCalibDataLoaded_R1S1()
{
    BOOL bSuccess = TRUE;

    Controller controller;
    CtrlGroup* grp0 = (CtrlGroup*)mpMalloc(sizeof(CtrlGroup));
    CtrlGroup* grp1 = (CtrlGroup*)mpMalloc(sizeof(CtrlGroup));

    controller.numGroup = 2;
    controller.ctrlGroups[0] = grp0;
    controller.ctrlGroups[1] = grp1;

    Ros_Testing_ControllerStatusIO_MakeFake6dofRobot(grp0, /*groupNo=*/ 0, /*groupId=*/ MP_R1_GID);
    Ros_Testing_ControllerStatusIO_MakeFakeStationGroup(grp1, /*groupNo=*/ 1, /*groupId=*/ MP_S1_GID);

    //R1S1: YES warning, but only if TF enabled AND calib failed to load
    bSuccess &= FALSE == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ FALSE, /*bPublishTfEnabled=*/ FALSE);
    bSuccess &= FALSE == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ TRUE , /*bPublishTfEnabled=*/ FALSE);
    bSuccess &= TRUE  == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ FALSE, /*bPublishTfEnabled=*/ TRUE);
    bSuccess &= FALSE == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ TRUE , /*bPublishTfEnabled=*/ TRUE);

    //report overall result
    Ros_Debug_BroadcastMsg("Testing Ros_Testing_ControllerStatusIO_ShouldWarnNoCalibDataLoaded_R1S1: %s", bSuccess ? "PASS" : "FAIL");

    mpFree(grp0);
    mpFree(grp1);

    return bSuccess;
}

BOOL Ros_Testing_ControllerStatusIO_ShouldWarnNoCalibDataLoaded_R1S1S2()
{
    BOOL bSuccess = TRUE;

    Controller controller;
    CtrlGroup* grp0 = (CtrlGroup*)mpMalloc(sizeof(CtrlGroup));
    CtrlGroup* grp1 = (CtrlGroup*)mpMalloc(sizeof(CtrlGroup));
    CtrlGroup* grp2 = (CtrlGroup*)mpMalloc(sizeof(CtrlGroup));

    controller.numGroup = 3;
    controller.ctrlGroups[0] = grp0;
    controller.ctrlGroups[1] = grp1;
    controller.ctrlGroups[2] = grp2;

    Ros_Testing_ControllerStatusIO_MakeFake6dofRobot(grp0, /*groupNo=*/ 0, /*groupId=*/ MP_R1_GID);
    Ros_Testing_ControllerStatusIO_MakeFakeStationGroup(grp1, /*groupNo=*/ 1, /*groupId=*/ MP_S1_GID);
    Ros_Testing_ControllerStatusIO_MakeFakeStationGroup(grp2, /*groupNo=*/ 2, /*groupId=*/ MP_S2_GID);

    //R1S1S2: YES warning, but only if TF enabled AND calib failed to load
    bSuccess &= FALSE == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ FALSE, /*bPublishTfEnabled=*/ FALSE);
    bSuccess &= FALSE == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ TRUE , /*bPublishTfEnabled=*/ FALSE);
    bSuccess &= TRUE  == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ FALSE, /*bPublishTfEnabled=*/ TRUE);
    bSuccess &= FALSE == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ TRUE , /*bPublishTfEnabled=*/ TRUE);

    //report overall result
    Ros_Debug_BroadcastMsg("Testing Ros_Testing_ControllerStatusIO_ShouldWarnNoCalibDataLoaded_R1S1S2: %s", bSuccess ? "PASS" : "FAIL");

    mpFree(grp0);
    mpFree(grp1);
    mpFree(grp2);

    return bSuccess;
}

BOOL Ros_Testing_ControllerStatusIO_ShouldWarnNoCalibDataLoaded_R1B1S1()
{
    BOOL bSuccess = TRUE;

    Controller controller;
    CtrlGroup* grp0 = (CtrlGroup*)mpMalloc(sizeof(CtrlGroup));
    CtrlGroup* grp1 = (CtrlGroup*)mpMalloc(sizeof(CtrlGroup));
    CtrlGroup* grp2 = (CtrlGroup*)mpMalloc(sizeof(CtrlGroup));

    controller.numGroup = 3;
    controller.ctrlGroups[0] = grp0;
    controller.ctrlGroups[1] = grp1;
    controller.ctrlGroups[2] = grp2;

    Ros_Testing_ControllerStatusIO_MakeFake6dofRobot(grp0, /*groupNo=*/ 0, /*groupId=*/ MP_R1_GID);
    Ros_Testing_ControllerStatusIO_MakeFakeBaseGroup(grp1, /*groupNo=*/ 1, /*groupId=*/ MP_B1_GID);
    Ros_Testing_ControllerStatusIO_AssignRobotToBaseGroup(/*robot=*/ grp0, /*base=*/ grp1);

    Ros_Testing_ControllerStatusIO_MakeFakeStationGroup(grp2, /*groupNo=*/ 2, /*groupId=*/ MP_S1_GID);

    //R1B1S1: YES warning, but only if TF enabled AND calib failed to load
    bSuccess &= FALSE == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ FALSE, /*bPublishTfEnabled=*/ FALSE);
    bSuccess &= FALSE == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ TRUE , /*bPublishTfEnabled=*/ FALSE);
    bSuccess &= TRUE  == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ FALSE, /*bPublishTfEnabled=*/ TRUE);
    bSuccess &= FALSE == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ TRUE , /*bPublishTfEnabled=*/ TRUE);

    //report overall result
    Ros_Debug_BroadcastMsg("Testing Ros_Testing_ControllerStatusIO_ShouldWarnNoCalibDataLoaded_R1B1S1: %s", bSuccess ? "PASS" : "FAIL");

    mpFree(grp0);
    mpFree(grp1);
    mpFree(grp2);

    return bSuccess;
}

BOOL Ros_Testing_ControllerStatusIO_ShouldWarnNoCalibDataLoaded_R1B1R2B2()
{
    BOOL bSuccess = TRUE;

    Controller controller;
    CtrlGroup* grp0 = (CtrlGroup*)mpMalloc(sizeof(CtrlGroup));
    CtrlGroup* grp1 = (CtrlGroup*)mpMalloc(sizeof(CtrlGroup));
    CtrlGroup* grp2 = (CtrlGroup*)mpMalloc(sizeof(CtrlGroup));
    CtrlGroup* grp3 = (CtrlGroup*)mpMalloc(sizeof(CtrlGroup));

    controller.numGroup = 4;
    controller.ctrlGroups[0] = grp0;
    controller.ctrlGroups[1] = grp1;
    controller.ctrlGroups[2] = grp2;
    controller.ctrlGroups[3] = grp3;

    Ros_Testing_ControllerStatusIO_MakeFake6dofRobot(grp0, /*groupNo=*/ 0, /*groupId=*/ MP_R1_GID);
    Ros_Testing_ControllerStatusIO_MakeFakeBaseGroup(grp1, /*groupNo=*/ 1, /*groupId=*/ MP_B1_GID);
    Ros_Testing_ControllerStatusIO_AssignRobotToBaseGroup(/*robot=*/ grp0, /*base=*/ grp1);

    Ros_Testing_ControllerStatusIO_MakeFake6dofRobot(grp2, /*groupNo=*/ 2, /*groupId=*/ MP_R2_GID);
    Ros_Testing_ControllerStatusIO_MakeFakeBaseGroup(grp3, /*groupNo=*/ 3, /*groupId=*/ MP_B2_GID);
    Ros_Testing_ControllerStatusIO_AssignRobotToBaseGroup(/*robot=*/ grp2, /*base=*/ grp3);

    //R1B1R2B2: YES warning, but only if TF enabled AND calib failed to load
    bSuccess &= FALSE == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ FALSE, /*bPublishTfEnabled=*/ FALSE);
    bSuccess &= FALSE == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ TRUE , /*bPublishTfEnabled=*/ FALSE);
    bSuccess &= TRUE  == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ FALSE, /*bPublishTfEnabled=*/ TRUE);
    bSuccess &= FALSE == Ros_Controller_ShouldWarnNoCalibDataLoaded(&controller, /*bCalibLoadedOk=*/ TRUE , /*bPublishTfEnabled=*/ TRUE);

    //report overall result
    Ros_Debug_BroadcastMsg("Testing Ros_Testing_ControllerStatusIO_ShouldWarnNoCalibDataLoaded_R1B1R2B2: %s", bSuccess ? "PASS" : "FAIL");

    mpFree(grp0);
    mpFree(grp1);
    mpFree(grp2);
    mpFree(grp3);

    return bSuccess;
}


BOOL Ros_Testing_ControllerStatusIO()
{
    BOOL bSuccess = TRUE;

    bSuccess &= Ros_Testing_ControllerStatusIO_ShouldWarnNoCalibDataLoaded_R1();
    bSuccess &= Ros_Testing_ControllerStatusIO_ShouldWarnNoCalibDataLoaded_R1B1();
    bSuccess &= Ros_Testing_ControllerStatusIO_ShouldWarnNoCalibDataLoaded_R1R2();
    bSuccess &= Ros_Testing_ControllerStatusIO_ShouldWarnNoCalibDataLoaded_R1B1R2();
    bSuccess &= Ros_Testing_ControllerStatusIO_ShouldWarnNoCalibDataLoaded_R1S1();
    bSuccess &= Ros_Testing_ControllerStatusIO_ShouldWarnNoCalibDataLoaded_R1S1S2();
    bSuccess &= Ros_Testing_ControllerStatusIO_ShouldWarnNoCalibDataLoaded_R1B1S1();
    bSuccess &= Ros_Testing_ControllerStatusIO_ShouldWarnNoCalibDataLoaded_R1B1R2B2();


    return bSuccess;
}

#endif //MOTOROS2_TESTING_ENABLE
