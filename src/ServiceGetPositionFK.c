//ServiceGetPositionFK.c

// SPDX-FileCopyrightText: 2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

rcl_service_t g_serviceGetPositionFK;

ServiceGetPositionFK_Messages g_messages_GetPositionFK;

// shorten the typename a little, locally
typedef motoros2_interfaces__srv__GetPositionFK_Request GetPositionFK_Request;
typedef motoros2_interfaces__srv__GetPositionFK_Response GetPositionFK_Response;

void Ros_ServiceGetPositionFK_Initialize()
{
    MOTOROS2_MEM_TRACE_START(svc_get_position_fk_init);

    rcl_ret_t ret = rclc_service_init_default(&g_serviceGetPositionFK, &g_microRosNodeInfo.node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(motoros2_interfaces, srv, GetPositionFK),
        SERVICE_NAME_GET_POSITION_FK);
    motoRosAssert_withMsg(ret == RCL_RET_OK, SUBCODE_FAIL_INIT_SERVICE_GET_POSITION_FK,
        "Failed to init service (%d)", (int)ret);

    rosidl_runtime_c__String__init(&g_messages_GetPositionFK.response.message);
    geometry_msgs__msg__PoseStamped__init(&g_messages_GetPositionFK.response.result);

    MOTOROS2_MEM_TRACE_REPORT(svc_get_position_fk_init);
}

void Ros_ServiceGetPositionFK_Cleanup()
{
    MOTOROS2_MEM_TRACE_START(svc_get_position_fk_fini);

    rcl_ret_t ret;

    Ros_Debug_BroadcastMsg("Cleanup service " SERVICE_NAME_GET_POSITION_FK);
    ret = rcl_service_fini(&g_serviceGetPositionFK, &g_microRosNodeInfo.node);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg(
            "Failed cleaning up " SERVICE_NAME_GET_POSITION_FK " service: %d", ret);
    rosidl_runtime_c__String__fini(&g_messages_GetPositionFK.response.message);
    geometry_msgs__msg__PoseStamped__fini(&g_messages_GetPositionFK.response.result);

    MOTOROS2_MEM_TRACE_REPORT(svc_get_position_fk_fini);
}

//TODO(gavanderhoorn): refactor: create M+ Cart types <-> ROS types conversion lib
static void MpCoord_To_GeomMsgsPose(MP_COORD const* const mp_coord, geometry_msgs__msg__Pose* const geom_pose)
{
    geom_pose->position.x = NANOMETERS_TO_METERS(mp_coord->x);
    geom_pose->position.y = NANOMETERS_TO_METERS(mp_coord->y);
    geom_pose->position.z = NANOMETERS_TO_METERS(mp_coord->z);
    QuatConversion_MpCoordOrient_To_GeomMsgsQuaternion(
        mp_coord->rx, mp_coord->ry, mp_coord->rz,
        &geom_pose->orientation);
}

void Ros_ServiceGetPositionFK_Trigger(const void* request_msg, void* response_msg)
{
    GetPositionFK_Request* request = (GetPositionFK_Request*) request_msg;
    GetPositionFK_Response* response = (GetPositionFK_Response*) response_msg;

    Ros_Debug_BroadcastMsg("%s: entry", __func__);

    //init response message
    response->success = FALSE;
    response->result_code = 0;
    rosidl_runtime_c__String__assign(&response->message, "");
    rosidl_runtime_c__String__assign(
        &g_messages_GetPositionFK.response.result.header.frame_id, "");

    Ros_Debug_BroadcastMsg("%s: requested: grp no: %u, tool no: %u",
        __func__, request->group_no, request->tool_no);

    //make sure a valid group is specified (group_no is unsigned, so cannot be < 0)
    if (request->group_no >= g_Ros_Controller.numGroup)
    {
        //TODO(gavanderhoorn): shouldn't use "SelectionResultCodes" here
        response->result_code = motoros2_interfaces__msg__SelectionResultCodes__INVALID_CONTROL_GROUP;
        rosidl_runtime_c__String__assign(&response->message,
            motoros2_interfaces__msg__SelectionResultCodes__INVALID_CONTROL_GROUP_STR);
        goto DONE;
    }

    //make sure a valid tool is specified (tool_no is unsigned, so cannot be < 0)
    if (request->tool_no >= MAX_VALID_TOOL_INDEX)
    {
        //TODO(gavanderhoorn): shouldn't use "SelectionResultCodes" here
        response->result_code = motoros2_interfaces__msg__SelectionResultCodes__INVALID_SELECTION_INDEX;
        rosidl_runtime_c__String__assign(&response->message,
            motoros2_interfaces__msg__SelectionResultCodes__INVALID_SELECTION_INDEX_STR);
        goto DONE;
    }

    //TODO(gavanderhoorn): convert request->joint_angles to moto angles
    // Ros_CtrlGroup_ConvertRosUnitsToMotoUnits(..) always converts to pulses
    long motoAnglePos[MP_GRP_AXES_NUM] = { 0 };

    //call M+ API
    MP_COORD fk_result = { 0 };
    BITSTRING fig_ctrl = 0;
    //TODO(gavanderhoorn): verify this is always in robot base frame
    int res = mpConvAxesToCartPos(
        request->group_no,
        motoAnglePos,
        request->tool_no,
        &fig_ctrl, // unused
        &fk_result
    );

    Ros_Debug_BroadcastMsg("%s: mpConvAxesToCartPos(..): %d", __func__, res);

    //check
    if (res != OK)
    {
        rosidl_runtime_c__String__assign(&response->message, "mpConvAxesToCartPos(..) error");
        response->result_code = res;
        goto DONE;
    }

    //assign to result
    MpCoord_To_GeomMsgsPose(&fk_result, &response->result.pose);

    //construct frame_id and assign
    //TODO(gavanderhoorn): refactor, duplicated from PositionMonitor
    char formatBuffer[MAX_TF_FRAME_NAME_LENGTH] = { 0 };
    const char* frame_prefix = g_nodeConfigSettings.tf_frame_prefix;
    snprintf(formatBuffer, MAX_TF_FRAME_NAME_LENGTH, "%sr%d/base",
        frame_prefix, request->group_no + 1);
    rosidl_runtime_c__String__assign(
        &g_messages_GetPositionFK.response.result.header.frame_id,
        formatBuffer);

    //report to caller
    response->success = true;
    //TODO(gavanderhoorn): shouldn't use "SelectionResultCodes" here
    response->result_code = motoros2_interfaces__msg__SelectionResultCodes__OK;
    rosidl_runtime_c__String__assign(&response->message,
        motoros2_interfaces__msg__SelectionResultCodes__OK_STR);

DONE:
    Ros_Debug_BroadcastMsg("%s: exit: '%s' (%lu)", __func__, response->message.data,
        (unsigned long) response->result_code);
}
