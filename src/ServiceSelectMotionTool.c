//ServiceSelectMotionTool.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

rcl_service_t g_serviceSelectMotionTool;

ServiceSelectMotionTool_Messages g_messages_SelectMotionTool;

typedef motoros2_interfaces__srv__SelectMotionTool_Response SelectMotionToolResponse;
typedef motoros2_interfaces__srv__SelectMotionTool_Request SelectMotionToolRequest;


void Ros_ServiceSelectMotionTool_Initialize()
{
    MOTOROS2_MEM_TRACE_START(svc_select_tool_init);

    const rosidl_service_type_support_t* type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(motoros2_interfaces, srv, SelectMotionTool);

    rcl_ret_t ret = rclc_service_init_default(&g_serviceSelectMotionTool, &g_microRosNodeInfo.node, type_support, SERVICE_NAME_SELECT_MOTION_TOOL);
    motoRosAssert_withMsg(ret == RCL_RET_OK, SUBCODE_FAIL_INIT_SERVICE_SELECT_MOTION_TOOL, "Failed to init service (%d)", (int)ret);

    rosidl_runtime_c__String__init(&g_messages_SelectMotionTool.response.message);

    MOTOROS2_MEM_TRACE_REPORT(svc_select_tool_init);
}

void Ros_ServiceSelectMotionTool_Cleanup()
{
    rcl_ret_t ret;
    MOTOROS2_MEM_TRACE_START(svc_select_tool_fini);

    Ros_Debug_BroadcastMsg("Cleanup service select tool");
    ret = rcl_service_fini(&g_serviceSelectMotionTool, &g_microRosNodeInfo.node);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up select tool service: %d", ret);
    rosidl_runtime_c__String__fini(&g_messages_SelectMotionTool.response.message);

    MOTOROS2_MEM_TRACE_REPORT(svc_select_tool_fini);
}

void Ros_ServiceSelectMotionTool_Trigger(const void* request_msg, void* response_msg)
{
    Ros_Debug_BroadcastMsg("%s: entry", __func__);

    //NOTE: this implementation is different from the MotoROS1 implementation
    //      as it doesn't call mpSetToolNo(..).
    //      Rationale: mpSetToolNo(..) changes the tool used for teaching
    //      INFORM positions using the pendant. As there is no support in
    //      MotoROS2 for teaching INFORM positions, there is no need to
    //      change the 'teach tool'.
    //      Being able to change the tool used by the incremental-motion
    //      interface MotoROS2 uses (ie: mpExRcsIncrementMove et al.) *is*
    //      necessary, hence the implementation here in this service.

    SelectMotionToolResponse* response = (SelectMotionToolResponse*)response_msg;
    SelectMotionToolRequest* request = (SelectMotionToolRequest*)request_msg;

    //init response message
    response->success = false;

    Ros_Debug_BroadcastMsg("%s: requested: grp no: %u, tool: %u",
        __func__, request->group_number, request->tool_number);

    // we do not allow changing the tool unless in remote mode
    if (!Ros_Controller_IsRemote())
    {
        rosidl_runtime_c__String__assign(&response->message, motoros2_interfaces__msg__SelectionResultCodes__INVALID_CONTROLLER_STATE_STR);
        response->result_code.value = motoros2_interfaces__msg__SelectionResultCodes__INVALID_CONTROLLER_STATE;
        goto DONE;
    }

    //make sure a valid group is specified (group_number is unsigned, so cannot be < 0)
    if (request->group_number >= g_Ros_Controller.numGroup)
    {
        rosidl_runtime_c__String__assign(&response->message, motoros2_interfaces__msg__SelectionResultCodes__INVALID_CONTROL_GROUP_STR);
        response->result_code.value = motoros2_interfaces__msg__SelectionResultCodes__INVALID_CONTROL_GROUP;
        goto DONE;
    }

    //make sure a valid tool is specified (tool_number is unsigned, so cannot be < 0)
    if (request->tool_number >= MAX_VALID_TOOL_INDEX)
    {
        rosidl_runtime_c__String__assign(&response->message, motoros2_interfaces__msg__SelectionResultCodes__INVALID_SELECTION_INDEX_STR);
        response->result_code.value = motoros2_interfaces__msg__SelectionResultCodes__INVALID_SELECTION_INDEX;
        goto DONE;
    }

    //set tool that will be used by motion API (ie: passed by us to mpExRcsIncrementMove(..))
    //NOTE: this will change the 'motion tool' ONLY for those increments which
    //      haven't yet been added to the increment queue. See also the ROS 2
    //      'select_tool' service definition file in motoros2_interfaces.
    g_Ros_Controller.ctrlGroups[request->group_number]->tool = request->tool_number;

    //report to caller
    rosidl_runtime_c__String__assign(&response->message, motoros2_interfaces__msg__SelectionResultCodes__OK_STR);
    response->success = true;
    response->result_code.value = motoros2_interfaces__msg__SelectionResultCodes__OK;

DONE:
    Ros_Debug_BroadcastMsg("%s: exit: '%s' (%lu)", __func__, response->message.data,
        (unsigned long) response->result_code.value);
}
