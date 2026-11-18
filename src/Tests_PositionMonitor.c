// Tests_PositionMonitor.c

// SPDX-FileCopyrightText: 2026, Jayden Vasquez
//
// SPDX-License-Identifier: Apache-2.0

#ifdef MOTOROS2_TESTING_ENABLE

#include "MotoROS.h"

static BOOL Ros_Testing_TcpFrameNameAfterToolChange(const char* initialName, int tool, const char* expectedName)
{
    rosidl_runtime_c__String name;
    rosidl_runtime_c__String__init(&name);
    rosidl_runtime_c__String__assign(&name, initialName);

    Ros_PositionMonitor_UpdateTcpFrameName(&name, tool);
    BOOL bSuccess = (strcmp(name.data, expectedName) == 0);

    if (!bSuccess)
        Ros_Debug_BroadcastMsg("%s: '%s' + tool %d: expected '%s', got '%s'", __func__, initialName, tool, expectedName, name.data);

    rosidl_runtime_c__String__fini(&name);
    return bSuccess;
}

BOOL Ros_Testing_Ros_PositionMonitor_UpdateTcpFrameName()
{
    BOOL bSuccess = TRUE;

    //tool unchanged: name unchanged
    bSuccess &= Ros_Testing_TcpFrameNameAfterToolChange("r1/tcp_0", 0, "r1/tcp_0");
    bSuccess &= Ros_Testing_TcpFrameNameAfterToolChange("r2/tcp_12", 12, "r2/tcp_12");

    //tool changed: name follows the active tool
    bSuccess &= Ros_Testing_TcpFrameNameAfterToolChange("r1/tcp_0", 1, "r1/tcp_1");
    bSuccess &= Ros_Testing_TcpFrameNameAfterToolChange("r1/tcp_0", 63, "r1/tcp_63");
    bSuccess &= Ros_Testing_TcpFrameNameAfterToolChange("r1/tcp_63", 5, "r1/tcp_5");
    bSuccess &= Ros_Testing_TcpFrameNameAfterToolChange("r2/tcp_3", 0, "r2/tcp_0");

    //frame prefix is kept, also when it contains underscores
    bSuccess &= Ros_Testing_TcpFrameNameAfterToolChange("my_cell/r1/tcp_0", 2, "my_cell/r1/tcp_2");

    Ros_Debug_BroadcastMsg("Testing Ros_PositionMonitor_UpdateTcpFrameName: %s", bSuccess ? "PASS" : "FAIL");
    return bSuccess;
}

BOOL Ros_Testing_PositionMonitor()
{
    BOOL bSuccess = TRUE;

    bSuccess &= Ros_Testing_Ros_PositionMonitor_UpdateTcpFrameName();

    return bSuccess;
}

#endif //MOTOROS2_TESTING_ENABLE
