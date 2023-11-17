// ForceTorqueMonitor.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

ForceTorqueMonitor_Publishers g_publishers_ForceTorqueMonitor;
ForceTorqueMonitor_Messages g_messages_ForceTorqueMonitor;

static void Ros_ForceTorqueMonitor_Initialize_Publishers(rmw_qos_profile_t const* const qos_profile);

void Ros_ForceTorqueMonitor_Initialize()
{
    Ros_Debug_BroadcastMsg("Initializing ForceTorqueMonitor publishers");

    //==================================
    //create publishers for TCP force and external joint torque
    const rmw_qos_profile_t* qos_profile_js = Ros_ConfigFile_To_Rmw_Qos_Profile(g_nodeConfigSettings.qos_joint_states);
    Ros_ForceTorqueMonitor_Initialize_Publishers(qos_profile_js);

}

static void Ros_ForceTorqueMonitor_Initialize_Publishers(rmw_qos_profile_t const* const qos_profile)
{
    //create a publisher for TCP force torque
    rcl_ret_t ret = rclc_publisher_init(
        &g_publishers_ForceTorqueMonitor.forceTorqueTCP,
        &g_microRosNodeInfo.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, WrenchStamped),
        TOPIC_NAME_FORCE_TORQUE_TCP,
        qos_profile);
    motoRosAssert(ret == RCL_RET_OK, SUBCODE_FAIL_CREATE_PUBLISHER_JOINT_STATE_ALL);

    //create message for TCP force torque
    g_messages_ForceTorqueMonitor.forceTorqueTCP = geometry_msgs__msg__WrenchStamped__create();

    char formatBuffer[MAX_JOINT_NAME_LENGTH];
    snprintf(formatBuffer, MAX_TF_FRAME_NAME_LENGTH, "%sr%d/base", g_nodeConfigSettings.tf_frame_prefix, 1);
    rosidl_runtime_c__String__assign(&g_messages_ForceTorqueMonitor.forceTorqueTCP->header.frame_id, formatBuffer);

    //--------------

    //create a publisher for joint external torque
    ret = rclc_publisher_init(
        &g_publishers_ForceTorqueMonitor.jointExternalTorque,
        &g_microRosNodeInfo.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        TOPIC_NAME_JOINT_EXTERNAL_TORQUE,
        qos_profile);
    motoRosAssert(ret == RCL_RET_OK, SUBCODE_FAIL_CREATE_PUBLISHER_JOINT_STATE_ALL);

    //create message for joint external torque
    g_messages_ForceTorqueMonitor.jointExternalTorque = sensor_msgs__msg__JointState__create();
    rosidl_runtime_c__String__assign(&g_messages_ForceTorqueMonitor.jointExternalTorque->header.frame_id, "");
    rosidl_runtime_c__String__Sequence__init(&g_messages_ForceTorqueMonitor.jointExternalTorque->name, g_Ros_Controller.totalAxesCount); // Number of joints in message

    for (int jointIndex = 0; jointIndex < g_Ros_Controller.totalAxesCount; jointIndex += 1)
    {
        rosidl_runtime_c__String__assign(&g_messages_ForceTorqueMonitor.jointExternalTorque->name.data[jointIndex], g_nodeConfigSettings.joint_names[jointIndex]);
    }
    rosidl_runtime_c__float64__Sequence__init(&g_messages_ForceTorqueMonitor.jointExternalTorque->position, 0);
    rosidl_runtime_c__float64__Sequence__init(&g_messages_ForceTorqueMonitor.jointExternalTorque->velocity, 0);
    rosidl_runtime_c__float64__Sequence__init(&g_messages_ForceTorqueMonitor.jointExternalTorque->effort, g_Ros_Controller.totalAxesCount);
}

void Ros_ForceTorqueMonitor_Cleanup()
{
    rcl_ret_t ret;

    Ros_Debug_BroadcastMsg("Cleanup TCP force torque publisher");
    ret = rcl_publisher_fini(&g_publishers_ForceTorqueMonitor.forceTorqueTCP, &g_microRosNodeInfo.node);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up TCP force torque publisher: %d", ret);
    geometry_msgs__msg__WrenchStamped__destroy(g_messages_ForceTorqueMonitor.forceTorqueTCP);

    Ros_Debug_BroadcastMsg("Cleanup joint external torque publisher");
    ret = rcl_publisher_fini(&g_publishers_ForceTorqueMonitor.jointExternalTorque, &g_microRosNodeInfo.node);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up joint external torque publisher: %d", ret);
    sensor_msgs__msg__JointState__destroy(g_messages_ForceTorqueMonitor.jointExternalTorque);


}

void Ros_ForceTorqueMonitor_UpdateLocation()
{
    LONG status;
    rcl_ret_t ret;

    //Timestamp
    INT64 theTime = rmw_uros_epoch_nanos();

    Ros_Nanos_To_Time_Msg(theTime, &g_messages_ForceTorqueMonitor.forceTorqueTCP->header.stamp);
    Ros_Nanos_To_Time_Msg(theTime, &g_messages_ForceTorqueMonitor.jointExternalTorque->header.stamp);

    MP_IO_INFO registerInfo[6];
    USHORT registerValues[6];
    double forceTCPValues[6];
    double torqueJointValues[6];
    
    //assign TCP force data register adresses
    registerInfo[0].ulAddr = TCP_FORCE_FX_REG_ID;
    registerInfo[1].ulAddr = TCP_FORCE_FY_REG_ID;
    registerInfo[2].ulAddr = TCP_FORCE_FZ_REG_ID;
    registerInfo[3].ulAddr = TCP_FORCE_MX_REG_ID;
    registerInfo[4].ulAddr = TCP_FORCE_MY_REG_ID;
    registerInfo[5].ulAddr = TCP_FORCE_MZ_REG_ID;

    //Read TCP force data from registers
    status = mpReadIO(registerInfo, registerValues, 6);

    if (status != OK)
    {
        Ros_Debug_BroadcastMsg("Failed to get TCP force torque data: %u", status);
    }

    //convert register data to proper format and assign to corresponding wrench fields
    g_messages_ForceTorqueMonitor.forceTorqueTCP->wrench.force.x = (registerValues[0] - 10000.0) * 0.1;
    g_messages_ForceTorqueMonitor.forceTorqueTCP->wrench.force.y = (registerValues[1] - 10000.0) * 0.1;
    g_messages_ForceTorqueMonitor.forceTorqueTCP->wrench.force.z = (registerValues[2] - 10000.0) * 0.1;
    g_messages_ForceTorqueMonitor.forceTorqueTCP->wrench.torque.x = (registerValues[3] - 10000.0) * 0.1;
    g_messages_ForceTorqueMonitor.forceTorqueTCP->wrench.torque.y = (registerValues[4] - 10000.0) * 0.1;
    g_messages_ForceTorqueMonitor.forceTorqueTCP->wrench.torque.z = (registerValues[5] - 10000.0) * 0.1;

    //**********************************
    
    //assign joint external torque data register adresses
    registerInfo[0].ulAddr = EXTERNAL_TORQUE_JOINT_S_REG_ID;
    registerInfo[1].ulAddr = EXTERNAL_TORQUE_JOINT_L_REG_ID;
    registerInfo[2].ulAddr = EXTERNAL_TORQUE_JOINT_U_REG_ID;
    registerInfo[3].ulAddr = EXTERNAL_TORQUE_JOINT_R_REG_ID;
    registerInfo[4].ulAddr = EXTERNAL_TORQUE_JOINT_B_REG_ID;
    registerInfo[5].ulAddr = EXTERNAL_TORQUE_JOINT_T_REG_ID;

    //Read joint external torque data from registers
    status = mpReadIO(registerInfo, registerValues, 6);

    if (status != OK)
    {
        Ros_Debug_BroadcastMsg("Failed to get joint external torque: %u", status);
    }

    //convert result to double array
    for (int i = 0; i < 6; i += 1)
    {
        torqueJointValues[i] = registerValues[i];
        torqueJointValues[i] = (torqueJointValues[i] - 10000.0) * 0.1;
    }

    memcpy(g_messages_ForceTorqueMonitor.jointExternalTorque->effort.data, torqueJointValues, sizeof(double) * g_Ros_Controller.totalAxesCount);
    g_messages_ForceTorqueMonitor.jointExternalTorque->effort.size = g_Ros_Controller.totalAxesCount;

    //**********************************
    //Publish feedback topics

    ret = rcl_publish(&g_publishers_ForceTorqueMonitor.forceTorqueTCP, g_messages_ForceTorqueMonitor.forceTorqueTCP, NULL);
    RCL_UNUSED(ret);

    ret = rcl_publish(&g_publishers_ForceTorqueMonitor.jointExternalTorque, g_messages_ForceTorqueMonitor.jointExternalTorque, NULL);
    RCL_UNUSED(ret);
}
