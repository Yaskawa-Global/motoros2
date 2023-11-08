// ForceTorqueMonitor.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_FORCE_TORQUE_MONITOR_H
#define MOTOROS2_FORCE_TORQUE_MONITOR_H

typedef struct
{
    rcl_publisher_t forceTorqueTCP;
    rcl_publisher_t jointExternalTorque;
} ForceTorqueMonitor_Publishers;
extern ForceTorqueMonitor_Publishers g_publishers_ForceTorqueMonitor;

typedef struct
{
    geometry_msgs__msg__WrenchStamped* forceTorqueTCP;
    sensor_msgs__msg__JointState* jointExternalTorque;
} ForceTorqueMonitor_Messages;
extern ForceTorqueMonitor_Messages g_messages_ForceTorqueMonitor;

extern void Ros_ForceTorqueMonitor_Initialize();
extern void Ros_ForceTorqueMonitor_Cleanup();

extern void Ros_ForceTorqueMonitor_UpdateLocation();

#endif  // MOTOROS2_FORCE_TORQUE_MONITOR_H
