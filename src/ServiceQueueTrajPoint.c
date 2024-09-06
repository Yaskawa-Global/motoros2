//ServiceQueueTrajPoint.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

rcl_service_t g_serviceQueueTrajPoint;

typedef motoros2_interfaces__srv__QueueTrajPoint_Request QueueTrajPointRequest;
typedef motoros2_interfaces__srv__QueueTrajPoint_Response QueueTrajPointResponse;

ServiceQueueTrajPoint_Messages g_messages_QueueTrajPoint;

void Ros_ServiceQueueTrajPoint_Initialize()
{
    MOTOROS2_MEM_TRACE_START(svc_queue_point_init);

    //--------------
    const rosidl_service_type_support_t* type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(motoros2_interfaces, srv, QueueTrajPoint);

    rcl_ret_t ret;
    ret = rclc_service_init_default(&g_serviceQueueTrajPoint, &g_microRosNodeInfo.node, type_support, SERVICE_NAME_QUEUE_TRAJ_POINT);
    motoRos_RCLAssertOK(ret, SUBCODE_FAIL_CREATE_SERVICE_QUEUE_POINT);

    //--------------
    //I'm intentionally allocating for all possible axes, rather than the actual number used.
    //If the user submits a point with too many axes, the message would get lost in the ether
    //if there's not enough memory to deserialize it. So, I would be unable to see the message
    //and provide an appropriate error code.
    int maxAxes = MAX_CONTROLLABLE_GROUPS * MP_GRP_AXES_NUM;

    //TODO: Update to use micro_ros_utilities for allocation
    g_messages_QueueTrajPoint.request = motoros2_interfaces__srv__QueueTrajPoint_Request__create();    
    rosidl_runtime_c__String__Sequence__init(&g_messages_QueueTrajPoint.request->joint_names, maxAxes);

    for (int i = 0; i < maxAxes; i += 1)
        rosidl_runtime_c__String__assign(&g_messages_QueueTrajPoint.request->joint_names.data[i], "012345678901234567890123456789012");

    rosidl_runtime_c__float64__Sequence__init(&g_messages_QueueTrajPoint.request->point.positions, maxAxes);
    rosidl_runtime_c__float64__Sequence__init(&g_messages_QueueTrajPoint.request->point.velocities, maxAxes);
    rosidl_runtime_c__float64__Sequence__init(&g_messages_QueueTrajPoint.request->point.accelerations, maxAxes);
    rosidl_runtime_c__float64__Sequence__init(&g_messages_QueueTrajPoint.request->point.effort, maxAxes);
    
    //--------------
    g_messages_QueueTrajPoint.response = motoros2_interfaces__srv__QueueTrajPoint_Response__create();
    rosidl_runtime_c__String__init(&g_messages_QueueTrajPoint.response->message);

    //--------------
    MOTOROS2_MEM_TRACE_REPORT(svc_queue_point_init);
}

void Ros_ServiceQueueTrajPoint_Cleanup()
{
    rcl_ret_t ret;
    MOTOROS2_MEM_TRACE_START(svc_queue_point_fini);

    Ros_Debug_BroadcastMsg("Cleanup service queue point");
    ret = rcl_service_fini(&g_serviceQueueTrajPoint, &g_microRosNodeInfo.node);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up " SERVICE_NAME_QUEUE_TRAJ_POINT " service: %d", ret);

    motoros2_interfaces__srv__QueueTrajPoint_Request__destroy(g_messages_QueueTrajPoint.request);

    motoros2_interfaces__srv__QueueTrajPoint_Response__destroy(g_messages_QueueTrajPoint.response);

    MOTOROS2_MEM_TRACE_REPORT(svc_queue_point_fini);
}

void Ros_ServiceQueueTrajPoint_Trigger(const void* request_msg, void* response_msg)
{
    QueueTrajPointRequest* request = (QueueTrajPointRequest*)request_msg;
    QueueTrajPointResponse* response = (QueueTrajPointResponse*)response_msg;

    Ros_Debug_BroadcastMsg("Streaming point received");

    if (!Ros_MotionControl_IsMotionMode_PointQueue())
    {
        response->result_code.value = motoros2_interfaces__msg__QueueResultEnum__WRONG_MODE;
        rosidl_runtime_c__String__assign(&response->message,
            motoros2_interfaces__msg__QueueResultEnum__WRONG_MODE_STR);

        Ros_Debug_BroadcastMsg("Point rejected due to wrong trajectory-mode");
    }
    else
    {
        response->result_code.value = Ros_MotionControl_ProcessQueuedTrajectoryPoint(request);

        switch (response->result_code.value)
        {
        case motoros2_interfaces__msg__QueueResultEnum__SUCCESS: 
            rosidl_runtime_c__String__assign(&response->message,
                motoros2_interfaces__msg__QueueResultEnum__SUCCESS_STR);
            break;

        case motoros2_interfaces__msg__QueueResultEnum__INIT_FAILURE:
            rosidl_runtime_c__String__assign(&response->message, 
                motoros2_interfaces__msg__QueueResultEnum__INIT_FAILURE_STR);
            break;

        case motoros2_interfaces__msg__QueueResultEnum__BUSY:
            rosidl_runtime_c__String__assign(&response->message, 
                motoros2_interfaces__msg__QueueResultEnum__BUSY_STR);
            break;

        case motoros2_interfaces__msg__QueueResultEnum__INVALID_JOINT_LIST:
            rosidl_runtime_c__String__assign(&response->message,
                motoros2_interfaces__msg__QueueResultEnum__INVALID_JOINT_LIST_STR);
            break;

        default:
            rosidl_runtime_c__String__assign(&response->message, 
                "Error occurred while adding the trajectory point to the queue.");
            break;
        }
        Ros_Debug_BroadcastMsg("Reply (%d): %s", response->result_code.value, response->message.data);
    }
}
