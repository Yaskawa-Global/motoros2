// ForceTorqueMonitor.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_FORCE_TORQUE_MONITOR_H
#define MOTOROS2_FORCE_TORQUE_MONITOR_H

//define register addresses 

#define TCP_FORCE_FX_REG_ID   1000320
#define TCP_FORCE_FY_REG_ID   1000321
#define TCP_FORCE_FZ_REG_ID   1000322
#define TCP_FORCE_MX_REG_ID   1000323
#define TCP_FORCE_MY_REG_ID   1000324
#define TCP_FORCE_MZ_REG_ID   1000325

#define EXTERNAL_TORQUE_JOINT_S_REG_ID   1000310
#define EXTERNAL_TORQUE_JOINT_L_REG_ID   1000311
#define EXTERNAL_TORQUE_JOINT_U_REG_ID   1000312
#define EXTERNAL_TORQUE_JOINT_R_REG_ID   1000313
#define EXTERNAL_TORQUE_JOINT_B_REG_ID   1000314
#define EXTERNAL_TORQUE_JOINT_T_REG_ID   1000315

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
