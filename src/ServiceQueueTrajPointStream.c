//ServiceQueueTrajPointStream.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

rcl_service_t g_serviceQueueTrajPointStream;

typedef motoros2_interfaces__srv__QueueTrajPointStream_Request QueueTrajPointStreamRequest;
typedef motoros2_interfaces__srv__QueueTrajPointStream_Response QueueTrajPointStreamResponse;

ServiceQueueTrajPointStream_Messages g_messages_QueueTrajPointStream;

static BOOL sequenceInitialized = FALSE;
static UINT16 lastAcceptedSequence = 0;
static UINT64 lastAcceptedPayloadHash = 0;

static UINT64 Ros_ServiceQueueTrajPointStream_HashBytes(UINT64 hash, const void* data, size_t length)
{
    const unsigned char* bytes = (const unsigned char*)data;
    for (size_t i = 0; i < length; i++)
    {
        hash ^= bytes[i];
        hash *= 1099511628211ULL;
    }
    return hash;
}

static UINT64 Ros_ServiceQueueTrajPointStream_PayloadHash(const QueueTrajPointStreamRequest* request)
{
    UINT64 hash = 1469598103934665603ULL;
    hash = Ros_ServiceQueueTrajPointStream_HashBytes(hash, &request->joint_names.size,
        sizeof(request->joint_names.size));
    for (size_t i = 0; i < request->joint_names.size; i++)
    {
        hash = Ros_ServiceQueueTrajPointStream_HashBytes(hash,
            request->joint_names.data[i].data, request->joint_names.data[i].size);
        const unsigned char separator = 0;
        hash = Ros_ServiceQueueTrajPointStream_HashBytes(hash, &separator, sizeof(separator));
    }

    const rosidl_runtime_c__double__Sequence* sequences[] = {
        &request->point.positions, &request->point.velocities,
        &request->point.accelerations, &request->point.effort
    };
    for (size_t i = 0; i < sizeof(sequences) / sizeof(sequences[0]); i++)
    {
        hash = Ros_ServiceQueueTrajPointStream_HashBytes(hash, &sequences[i]->size,
            sizeof(sequences[i]->size));
        hash = Ros_ServiceQueueTrajPointStream_HashBytes(hash, sequences[i]->data,
            sequences[i]->size * sizeof(sequences[i]->data[0]));
    }
    hash = Ros_ServiceQueueTrajPointStream_HashBytes(hash,
        &request->point.time_from_start.sec, sizeof(request->point.time_from_start.sec));
    hash = Ros_ServiceQueueTrajPointStream_HashBytes(hash,
        &request->point.time_from_start.nanosec, sizeof(request->point.time_from_start.nanosec));
    return hash;
}

void Ros_ServiceQueueTrajPointStream_ResetSequence()
{
    sequenceInitialized = FALSE;
    lastAcceptedSequence = 0;
    lastAcceptedPayloadHash = 0;
}

void Ros_ServiceQueueTrajPointStream_Initialize()
{
    MOTOROS2_MEM_TRACE_START(svc_queue_point_stream_init);
    Ros_ServiceQueueTrajPointStream_ResetSequence();

    //--------------
    const rosidl_service_type_support_t* type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(motoros2_interfaces, srv, QueueTrajPointStream);

    rcl_ret_t ret;
    ret = rclc_service_init_default(&g_serviceQueueTrajPointStream, &g_microRosNodeInfo.node, type_support, SERVICE_NAME_QUEUE_TRAJ_POINT_STREAM);
    motoRos_RCLAssertOK(ret, SUBCODE_FAIL_CREATE_SERVICE_QUEUE_POINT_STREAM);

    //--------------
    //I'm intentionally allocating for all possible axes, rather than the actual number used.
    //If the user submits a point with too many axes, the message would get lost in the ether
    //if there's not enough memory to deserialize it. So, I would be unable to see the message
    //and provide an appropriate error code.
    int maxAxes = MAX_CONTROLLABLE_GROUPS * MP_GRP_AXES_NUM;

    //TODO: Update to use micro_ros_utilities for allocation
    g_messages_QueueTrajPointStream.request = motoros2_interfaces__srv__QueueTrajPointStream_Request__create();
    rosidl_runtime_c__String__Sequence__init(&g_messages_QueueTrajPointStream.request->joint_names, maxAxes);

    for (int i = 0; i < maxAxes; i += 1)
        rosidl_runtime_c__String__assign(&g_messages_QueueTrajPointStream.request->joint_names.data[i], "012345678901234567890123456789012");

    rosidl_runtime_c__float64__Sequence__init(&g_messages_QueueTrajPointStream.request->point.positions, maxAxes);
    rosidl_runtime_c__float64__Sequence__init(&g_messages_QueueTrajPointStream.request->point.velocities, maxAxes);
    rosidl_runtime_c__float64__Sequence__init(&g_messages_QueueTrajPointStream.request->point.accelerations, maxAxes);
    rosidl_runtime_c__float64__Sequence__init(&g_messages_QueueTrajPointStream.request->point.effort, maxAxes);

    //--------------
    g_messages_QueueTrajPointStream.response = motoros2_interfaces__srv__QueueTrajPointStream_Response__create();

    //--------------
    MOTOROS2_MEM_TRACE_REPORT(svc_queue_point_stream_init);
}

void Ros_ServiceQueueTrajPointStream_Cleanup()
{
    rcl_ret_t ret;
    MOTOROS2_MEM_TRACE_START(svc_queue_point_stream_fini);

    Ros_ServiceQueueTrajPointStream_ResetSequence();
    Ros_Debug_BroadcastMsg("Cleanup service queue point stream");
    ret = rcl_service_fini(&g_serviceQueueTrajPointStream, &g_microRosNodeInfo.node);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up " SERVICE_NAME_QUEUE_TRAJ_POINT_STREAM " service: %d", ret);

    motoros2_interfaces__srv__QueueTrajPointStream_Request__destroy(g_messages_QueueTrajPointStream.request);

    motoros2_interfaces__srv__QueueTrajPointStream_Response__destroy(g_messages_QueueTrajPointStream.response);

    MOTOROS2_MEM_TRACE_REPORT(svc_queue_point_stream_fini);
}

void Ros_ServiceQueueTrajPointStream_Trigger(const void* request_msg, void* response_msg)
{
    QueueTrajPointStreamRequest* request = (QueueTrajPointStreamRequest*)request_msg;
    QueueTrajPointStreamResponse* response = (QueueTrajPointStreamResponse*)response_msg;

    Ros_Debug_BroadcastMsg("Streaming point received (sequence %u)", request->sequence);
    response->sequence = request->sequence;
    response->duplicate_acknowledged = FALSE;
    response->next_expected_sequence = sequenceInitialized
        ? (UINT16)(lastAcceptedSequence + 1) : request->sequence;

    if (!Ros_MotionControl_IsMotionMode_PointQueue())
    {
        response->result_code.value = motoros2_interfaces__msg__QueueResultEnum__WRONG_MODE;
        rosidl_runtime_c__String__assign(&response->message,
            motoros2_interfaces__msg__QueueResultEnum__WRONG_MODE_STR);
        response->queue_size = 0;
        response->underran_since_last_call = Ros_MotionControl_ReadAndClearPointQueueUnderran();

        Ros_Debug_BroadcastMsg("Point rejected due to wrong trajectory-mode");
        return;
    }

    UINT64 payloadHash = Ros_ServiceQueueTrajPointStream_PayloadHash(request);
    if (sequenceInitialized && request->sequence == lastAcceptedSequence)
    {
        if (payloadHash == lastAcceptedPayloadHash)
        {
            response->result_code.value = motoros2_interfaces__msg__QueueResultEnum__SUCCESS;
            response->queue_size = (UINT16)Ros_MotionControl_PointQueueCount(
                g_Ros_Controller.ctrlGroups[0]);
            response->underran_since_last_call = Ros_MotionControl_ReadAndClearPointQueueUnderran();
            response->next_expected_sequence = (UINT16)(lastAcceptedSequence + 1);
            response->duplicate_acknowledged = TRUE;
            rosidl_runtime_c__String__assign(&response->message,
                motoros2_interfaces__msg__QueueResultEnum__SUCCESS_STR);
            Ros_Debug_BroadcastMsg("Duplicate sequence %u acknowledged", request->sequence);
            return;
        }

        response->result_code.value = motoros2_interfaces__msg__QueueResultEnum__SEQUENCE_MISMATCH;
        response->queue_size = (UINT16)Ros_MotionControl_PointQueueCount(
            g_Ros_Controller.ctrlGroups[0]);
        response->underran_since_last_call = Ros_MotionControl_ReadAndClearPointQueueUnderran();
        rosidl_runtime_c__String__assign(&response->message,
            "Sequence matches last accepted point but payload differs");
        return;
    }

    if (sequenceInitialized && request->sequence != (UINT16)(lastAcceptedSequence + 1))
    {
        response->result_code.value = motoros2_interfaces__msg__QueueResultEnum__SEQUENCE_MISMATCH;
        response->queue_size = (UINT16)Ros_MotionControl_PointQueueCount(
            g_Ros_Controller.ctrlGroups[0]);
        response->underran_since_last_call = Ros_MotionControl_ReadAndClearPointQueueUnderran();
        rosidl_runtime_c__String__assign(&response->message,
            motoros2_interfaces__msg__QueueResultEnum__SEQUENCE_MISMATCH_STR);
        return;
    }

    // QueueTrajPointStream_Request and QueueTrajPoint_Request share identical
    // leading fields (joint_names, point) generated by the same rosidl generator.
    // Ros_MotionControl_EnqueueTrajectoryPoint reads only those two fields, so the
    // cast to QueueTrajPoint_Request* is layout-safe.
    UINT16 depth = 0;
    response->result_code.value =
        Ros_MotionControl_EnqueueTrajectoryPoint(
            (motoros2_interfaces__srv__QueueTrajPoint_Request*)request,
            POINT_QUEUE_ADMIT_FIFO, &depth);
    response->queue_size = depth;
    response->underran_since_last_call = Ros_MotionControl_ReadAndClearPointQueueUnderran();
    if (response->result_code.value == motoros2_interfaces__msg__QueueResultEnum__SUCCESS)
    {
        sequenceInitialized = TRUE;
        lastAcceptedSequence = request->sequence;
        lastAcceptedPayloadHash = payloadHash;
        response->next_expected_sequence = (UINT16)(lastAcceptedSequence + 1);
    }

    switch (response->result_code.value)
    {
    case motoros2_interfaces__msg__QueueResultEnum__SUCCESS:
        rosidl_runtime_c__String__assign(&response->message,
            motoros2_interfaces__msg__QueueResultEnum__SUCCESS_STR);
        break;

    case motoros2_interfaces__msg__QueueResultEnum__QUEUE_FULL:
        rosidl_runtime_c__String__assign(&response->message,
            motoros2_interfaces__msg__QueueResultEnum__QUEUE_FULL_STR);
        break;

    case motoros2_interfaces__msg__QueueResultEnum__INVALID_JOINT_LIST:
        rosidl_runtime_c__String__assign(&response->message,
            motoros2_interfaces__msg__QueueResultEnum__INVALID_JOINT_LIST_STR);
        break;

    case motoros2_interfaces__msg__QueueResultEnum__SEQUENCE_MISMATCH:
        rosidl_runtime_c__String__assign(&response->message,
            motoros2_interfaces__msg__QueueResultEnum__SEQUENCE_MISMATCH_STR);
        break;

    default:
    {
        //Otherwise it should be a forwarded Init_Trajectory_Status value.
        const char* initMsg = Ros_ErrorHandling_Init_Trajectory_Status_ToString((Init_Trajectory_Status)response->result_code.value);
        rosidl_runtime_c__String__assign(&response->message, initMsg);
        break;
    }
    }

    Ros_Debug_BroadcastMsg("Reply (%d): %s", response->result_code.value, response->message.data);
}
