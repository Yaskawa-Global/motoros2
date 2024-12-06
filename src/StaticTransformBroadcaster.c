// StaticTransformBroadcaster.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"
rcl_publisher_t g_publishers_transform_static;
tf2_msgs__msg__TFMessage* g_messages_transform_static;

void Ros_StaticTransformBroadcaster_Init()
{
    char formatBuffer[MAX_TF_FRAME_NAME_LENGTH];

    // default TF topic name
    bzero(formatBuffer, MAX_TF_FRAME_NAME_LENGTH);
    snprintf(formatBuffer, MAX_TF_FRAME_NAME_LENGTH, "%s", TOPIC_NAME_TF_STATIC);

    //check whether we should make the topic name absolute (so it can't/won't
    //be namespaced any further)
    if (g_nodeConfigSettings.namespace_tf == FALSE)
    {
        Ros_Debug_BroadcastMsg("StaticTransformBroadcaster: TF_STATIC topic absolute");
        snprintf(formatBuffer, MAX_TF_FRAME_NAME_LENGTH, "/%s", TOPIC_NAME_TF_STATIC);
    }

    Ros_Debug_BroadcastMsg("StaticTransformBroadcaster: publishing TF_STATIC to '%s'", formatBuffer);

    const rmw_qos_profile_t qos_profile_transform_static =
    {
        RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        1,
        RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
        RMW_QOS_DEADLINE_DEFAULT,
        RMW_QOS_LIFESPAN_DEFAULT,
        RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
        RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
        false
    };

    //-------------
    //create TF publisher (static)
    rcl_ret_t ret = rclc_publisher_init(
        &g_publishers_transform_static,
        &g_microRosNodeInfo.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
        formatBuffer,
        &qos_profile_transform_static);

    motoRosAssert(ret == RCL_RET_OK, SUBCODE_FAIL_CREATE_PUBLISHER_STATIC_TRANSFORM);

    //--------------
    //create message for static cartesian transform
    g_messages_transform_static = tf2_msgs__msg__TFMessage__create();

    motoRosAssert(geometry_msgs__msg__TransformStamped__Sequence__init(&g_messages_transform_static->transforms, STATIC_TRANSFORM_BROADCASTER_MAX_TF_COUNT),
        SUBCODE_FAIL_ALLOCATE_STATIC_TRANSFORM);
    g_messages_transform_static->transforms.size = 0;
}

bool Ros_StaticTransformBroadcaster_Send(geometry_msgs__msg__TransformStamped *msg, int count)
{
    rcl_ret_t ret;
    geometry_msgs__msg__TransformStamped__Sequence* transforms = &g_messages_transform_static->transforms;
    for (int i = 0; i < count; i++)
    {
        Ros_Debug_BroadcastMsg("Attempting to publish static transform, %s->%s...", msg->header.frame_id.data, msg->child_frame_id.data);
        bool match_found = false;
        for (int j = 0; j < transforms->size; j++)
        {
            if (rosidl_runtime_c__String__are_equal(&msg[i].child_frame_id, &transforms->data[j].child_frame_id))
            {
                match_found = true;
                Ros_Debug_BroadcastMsg("Transform already published. Updating data");
                geometry_msgs__msg__TransformStamped__copy(&msg[i], &transforms->data[j]);
                break;
            }
        }
        if (!match_found)
        {
            if (transforms->size < transforms->capacity)
            {
                Ros_Debug_BroadcastMsg("No match found. Adding transform to TF message.");
                geometry_msgs__msg__TransformStamped__copy(&msg[i], &transforms->data[transforms->size]);
                transforms->size++;
            }
            else
            {
                motoRosAssert_withMsg(FALSE, SUBCODE_FAIL_STATIC_TRANSFORM_MEM_LIMIT, "Memory limit reached");
            }
        }
    }
    ret = rcl_publish(&g_publishers_transform_static, g_messages_transform_static, NULL);
    if (ret == RCL_RET_OK) 
    {
        Ros_Debug_BroadcastMsg("Updated message successfully published");
        return true;
    }
    // publishing can fail, but we choose to ignore those errors in this implementation
    Ros_Debug_BroadcastMsg("Updated message failed to publish...");
    return false;
}

void Ros_StaticTransformBroadcaster_Cleanup()
{
    rcl_ret_t ret;
    Ros_Debug_BroadcastMsg("Cleanup TF STATIC publisher");
    ret = rcl_publisher_fini(&g_publishers_transform_static, &g_microRosNodeInfo.node);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up TF STATIC publisher: %d", ret);
    tf2_msgs__msg__TFMessage__destroy(g_messages_transform_static);
}

