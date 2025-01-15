// PositionMonitor.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

PositionMonitor_Publishers g_publishers_PositionMonitor;
PositionMonitor_Messages g_messages_PositionMonitor;

static void Ros_PositionMonitor_Initialize_GlobalJointStatePublisher(rmw_qos_profile_t const* const qos_profile);
static void Ros_PositionMonitor_Initialize_PerGroupJointStatePublisher(rmw_qos_profile_t const* const qos_profile, CtrlGroup* const ctrlGroup, int grpIndex);
static void Ros_PositionMonitor_Initialize_TfPublisher(rmw_qos_profile_t const* const qos_profile);

static int motoRos_PositionMonitor_totalRobots = 0;

void Ros_PositionMonitor_Initialize(TF_Static_Data* tf_static_data, rcl_publisher_t* publisher_transform_static, tf2_msgs__msg__TFMessage* msg_transform_static)
{
    MOTOROS2_MEM_TRACE_START(pos_mon_init);

    Ros_Debug_BroadcastMsg("Initializing PositionMonitor publishers");

    //==================================
    //create the global (ie: aggregrate) JointState publisher
    const rmw_qos_profile_t* qos_profile_js = Ros_ConfigFile_To_Rmw_Qos_Profile(g_nodeConfigSettings.qos_joint_states);
    Ros_PositionMonitor_Initialize_GlobalJointStatePublisher(qos_profile_js);

    //==================================
    //create a JointState publisher for each group
    motoRos_PositionMonitor_totalRobots = 0;
    for (int grpIndex = 0; grpIndex < g_Ros_Controller.numGroup; grpIndex++)
    {
        CtrlGroup* ctrlGroup = g_Ros_Controller.ctrlGroups[grpIndex];
        Ros_PositionMonitor_Initialize_PerGroupJointStatePublisher(qos_profile_js, ctrlGroup, grpIndex);
        if (Ros_CtrlGroup_IsRobot(ctrlGroup))
            motoRos_PositionMonitor_totalRobots += 1;
    }

    //==================================
    //Create publisher for cartesian transform
    //Rviz2 expects the QoS to be RELIABLE, but user could have configured something else
    const rmw_qos_profile_t* qos_profile_tf = Ros_ConfigFile_To_Rmw_Qos_Profile(g_nodeConfigSettings.qos_tf);
    Ros_PositionMonitor_Initialize_TfPublisher(qos_profile_tf);

    //==================================
    //Create publisher for static cartesian transform (e.g. tool0 to flange)

    Ros_StaticTransformBroadcaster_Init(publisher_transform_static, msg_transform_static);

    Ros_PositionMonitor_CalculateStaticTransforms(tf_static_data);

    MOTOROS2_MEM_TRACE_REPORT(pos_mon_init);
}

static void Ros_PositionMonitor_Initialize_GlobalJointStatePublisher(rmw_qos_profile_t const* const qos_profile)
{
    int iterator = 0;
    int grpIndex, jointIndex;

    //create a publisher for aggregate joint state of all groups
    rcl_ret_t ret = rclc_publisher_init(
        &g_publishers_PositionMonitor.jointStateAllGroups,
        &g_microRosNodeInfo.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        TOPIC_NAME_JOINT_STATES,
        qos_profile);
    motoRos_RCLAssertOK(ret, SUBCODE_FAIL_CREATE_PUBLISHER_JOINT_STATE_ALL);

    //create message for aggregate joint state of all groups
    g_messages_PositionMonitor.jointStateAllGroups = sensor_msgs__msg__JointState__create();
    rosidl_runtime_c__String__Sequence__init(&g_messages_PositionMonitor.jointStateAllGroups->name, g_Ros_Controller.totalAxesCount); // Number of joints in message
    iterator = 0;
    for (grpIndex = 0; grpIndex < g_Ros_Controller.numGroup; grpIndex += 1)
    {
        CtrlGroup* const ctrlGroup = g_Ros_Controller.ctrlGroups[grpIndex];

        for (jointIndex = 0; jointIndex < ctrlGroup->numAxes; jointIndex += 1)
        {
            //For seven-axis robots, we want the joints listed in sequential order. But, MotoPlus puts the E axis in the seventh slot.
            //Need to assign E to slot [2] and shift following axes up one.
            if ((ctrlGroup->numAxes == 7) && Ros_CtrlGroup_IsRobot(ctrlGroup))
            {
                if (jointIndex < 2) //copy name normally
                    rosidl_runtime_c__String__assign(&g_messages_PositionMonitor.jointStateAllGroups->name.data[iterator],
                        g_nodeConfigSettings.joint_names[(grpIndex * MP_GRP_AXES_NUM) + jointIndex]);

                else if (jointIndex == 2) //get name of 7th axis
                    rosidl_runtime_c__String__assign(&g_messages_PositionMonitor.jointStateAllGroups->name.data[iterator],
                        g_nodeConfigSettings.joint_names[(grpIndex * MP_GRP_AXES_NUM) + jointIndex + 4]);

                else //offset by one
                    rosidl_runtime_c__String__assign(&g_messages_PositionMonitor.jointStateAllGroups->name.data[iterator],
                        g_nodeConfigSettings.joint_names[(grpIndex * MP_GRP_AXES_NUM) + jointIndex - 1]);
            }
            else //no joint-swapping is needed
            {
                rosidl_runtime_c__String__assign(&g_messages_PositionMonitor.jointStateAllGroups->name.data[iterator],
                    g_nodeConfigSettings.joint_names[(grpIndex * MP_GRP_AXES_NUM) + jointIndex]);

            }

            iterator += 1;
        }
    }

    //init state arrays
    rosidl_runtime_c__float64__Sequence__init(&g_messages_PositionMonitor.jointStateAllGroups->position, g_Ros_Controller.totalAxesCount);
    rosidl_runtime_c__float64__Sequence__init(&g_messages_PositionMonitor.jointStateAllGroups->velocity, g_Ros_Controller.totalAxesCount);
    rosidl_runtime_c__float64__Sequence__init(&g_messages_PositionMonitor.jointStateAllGroups->effort, g_Ros_Controller.totalAxesCount);
}

static void Ros_PositionMonitor_Initialize_PerGroupJointStatePublisher(rmw_qos_profile_t const* const qos_profile, CtrlGroup* const ctrlGroup, int grpIndex)
{
    //create joint publisher
    char formatBuffer[MAX_JOINT_NAME_LENGTH];
    snprintf(formatBuffer, MAX_JOINT_NAME_LENGTH, "ctrl_groups/%s/%s", Ros_CtrlGroup_GRP_ID_String[ctrlGroup->groupId], TOPIC_NAME_JOINT_STATES);
    rcl_ret_t ret = rclc_publisher_init(
        &ctrlGroup->publisherJointState,
        &g_microRosNodeInfo.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        formatBuffer,
        qos_profile);
    motoRos_RCLAssertOK(ret, SUBCODE_FAIL_CREATE_PUBLISHER_JOINT_STATE);

    //create message for per-group joint states
    ctrlGroup->msgJointState = sensor_msgs__msg__JointState__create();
    rosidl_runtime_c__String__assign(&ctrlGroup->msgJointState->header.frame_id, "");
    rosidl_runtime_c__String__Sequence__init(&ctrlGroup->msgJointState->name, ctrlGroup->numAxes); // Number of joints in message
    for (int jointIndex = 0; jointIndex < ctrlGroup->numAxes; jointIndex += 1)
    {
        //For seven-axis robots, we want the joints listed in sequential order. But, MotoPlus puts the E axis in the seventh slot.
        //Need to assign E to slot [2] and shift following axes up one.
        if ((ctrlGroup->numAxes == 7) && Ros_CtrlGroup_IsRobot(ctrlGroup))
        {
            if (jointIndex < 2) //copy name normally
                rosidl_runtime_c__String__assign(&ctrlGroup->msgJointState->name.data[jointIndex],
                    g_nodeConfigSettings.joint_names[(grpIndex * MP_GRP_AXES_NUM) + jointIndex]);

            else if (jointIndex == 2) //get name of 7th axis
                rosidl_runtime_c__String__assign(&ctrlGroup->msgJointState->name.data[jointIndex],
                    g_nodeConfigSettings.joint_names[(grpIndex * MP_GRP_AXES_NUM) + jointIndex + 4]);

            else //offset by one
                rosidl_runtime_c__String__assign(&ctrlGroup->msgJointState->name.data[jointIndex],
                    g_nodeConfigSettings.joint_names[(grpIndex * MP_GRP_AXES_NUM) + jointIndex - 1]);
        }
        else //no joint-swapping is needed
        {
            rosidl_runtime_c__String__assign(&ctrlGroup->msgJointState->name.data[jointIndex],
                g_nodeConfigSettings.joint_names[(grpIndex * MP_GRP_AXES_NUM) + jointIndex]);

        }
    }

    //init state arrays
    rosidl_runtime_c__float64__Sequence__init(&ctrlGroup->msgJointState->position, ctrlGroup->numAxes);
    rosidl_runtime_c__float64__Sequence__init(&ctrlGroup->msgJointState->velocity, ctrlGroup->numAxes);
    rosidl_runtime_c__float64__Sequence__init(&ctrlGroup->msgJointState->effort, ctrlGroup->numAxes);
}

static void Ros_PositionMonitor_Initialize_TfPublisher(rmw_qos_profile_t const* const qos_profile)
{
    char formatBuffer[MAX_TF_FRAME_NAME_LENGTH];

    // default TF topic name
    bzero(formatBuffer, MAX_TF_FRAME_NAME_LENGTH);
    snprintf(formatBuffer, MAX_TF_FRAME_NAME_LENGTH, "%s", TOPIC_NAME_TF);

    //check whether we should make the topic name absolute (so it can't/won't
    //be namespaced any further)
    if (g_nodeConfigSettings.namespace_tf == FALSE)
    {
        Ros_Debug_BroadcastMsg("PositionMonitor: TF topic absolute");
        snprintf(formatBuffer, MAX_TF_FRAME_NAME_LENGTH, "/%s", TOPIC_NAME_TF);
    }

    Ros_Debug_BroadcastMsg("PositionMonitor: publishing TF to '%s'", formatBuffer);

    //-------------
    //create TF publisher (non-static)
    rcl_ret_t ret = rclc_publisher_init(
        &g_publishers_PositionMonitor.transform,
        &g_microRosNodeInfo.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
        formatBuffer,
        qos_profile);
    motoRos_RCLAssertOK(ret, SUBCODE_FAIL_CREATE_PUBLISHER_TRANSFORM);

    //--------------
    //create message for cartesian transform
    g_messages_PositionMonitor.transform = tf2_msgs__msg__TFMessage__create();

    motoRosAssert(geometry_msgs__msg__TransformStamped__Sequence__init(&g_messages_PositionMonitor.transform->transforms, motoRos_PositionMonitor_totalRobots * PUBLISHED_NUMBER_TRANSFORM_LINKS_PER_ROBOT),
                  SUBCODE_FAIL_ALLOCATE_TRANSFORM);

    bzero(formatBuffer, MAX_TF_FRAME_NAME_LENGTH);
    int robotIterator = 0;
    const char* frame_prefix = g_nodeConfigSettings.tf_frame_prefix;
    for (int i = 0; i < (motoRos_PositionMonitor_totalRobots * PUBLISHED_NUMBER_TRANSFORM_LINKS_PER_ROBOT); i += PUBLISHED_NUMBER_TRANSFORM_LINKS_PER_ROBOT)
    {
        snprintf(formatBuffer, MAX_TF_FRAME_NAME_LENGTH, "%sworld", frame_prefix);
        rosidl_runtime_c__String__assign(&g_messages_PositionMonitor.transform->transforms.data[i + publishIndex_tfLink_WorldToBase].header.frame_id, formatBuffer);

        snprintf(formatBuffer, MAX_TF_FRAME_NAME_LENGTH, "%sr%d/base", frame_prefix, robotIterator + 1);
        rosidl_runtime_c__String__assign(&g_messages_PositionMonitor.transform->transforms.data[i + publishIndex_tfLink_WorldToBase].child_frame_id, formatBuffer);
        rosidl_runtime_c__String__assign(&g_messages_PositionMonitor.transform->transforms.data[i + publishIndex_tfLink_BaseToFlange].header.frame_id, formatBuffer);

        snprintf(formatBuffer, MAX_TF_FRAME_NAME_LENGTH, "%sr%d/flange", frame_prefix, robotIterator + 1);
        rosidl_runtime_c__String__assign(&g_messages_PositionMonitor.transform->transforms.data[i + publishIndex_tfLink_BaseToFlange].child_frame_id, formatBuffer);
        rosidl_runtime_c__String__assign(&g_messages_PositionMonitor.transform->transforms.data[i + publishIndex_tfLink_FlangeToTcp].header.frame_id, formatBuffer);

        snprintf(formatBuffer, MAX_TF_FRAME_NAME_LENGTH, "%sr%d/tcp_%d", frame_prefix, robotIterator + 1, g_Ros_Controller.ctrlGroups[robotIterator]->tool);
        rosidl_runtime_c__String__assign(&g_messages_PositionMonitor.transform->transforms.data[i + publishIndex_tfLink_FlangeToTcp].child_frame_id, formatBuffer);

        robotIterator += 1;
    }
}

void Ros_PositionMonitor_CalculateStaticTransforms(TF_Static_Data *tf_static_data)
{
    MP_XYZ vectorOrg, vectorX, vectorY; //for mpMakeFrame
    vectorOrg.x = 0;    vectorOrg.y = 0;    vectorOrg.z = 0;
    vectorX.x = 0;      vectorX.y = 0;      vectorX.z = 1;
    vectorY.x = 0;      vectorY.y = -1;     vectorY.z = 0;
    mpMakeFrame(&vectorOrg, &vectorX, &vectorY, &tf_static_data->frameTool0ToFlange);
    mpInvFrame(&tf_static_data->frameTool0ToFlange, &tf_static_data->frameFlangeToTool0);
    mpFrameToZYXeuler(&tf_static_data->frameFlangeToTool0, &tf_static_data->coordFlangeToTool0);
}

bool Ros_PositionMonitor_Send_TF_Static(TF_Static_Data* tf_static_data, rcl_publisher_t* publisher_transform_static, tf2_msgs__msg__TFMessage* msg_transform_static)
{
    //Timestamp
    INT64 theTime = rmw_uros_epoch_nanos();
    geometry_msgs__msg__TransformStamped__Sequence messages;
    motoRosAssert(geometry_msgs__msg__TransformStamped__Sequence__init(&messages, motoRos_PositionMonitor_totalRobots),
            SUBCODE_FAIL_ALLOCATE_FLANGE_TOOL0);
    char formatBuffer[MAX_TF_FRAME_NAME_LENGTH];
    bzero(formatBuffer, MAX_TF_FRAME_NAME_LENGTH);
    const char* frame_prefix = g_nodeConfigSettings.tf_frame_prefix;
    for (int i = 0; i < motoRos_PositionMonitor_totalRobots; i++)
    {
        snprintf(formatBuffer, MAX_TF_FRAME_NAME_LENGTH, "%sr%d/flange", frame_prefix, i + 1);
        rosidl_runtime_c__String__assign(&messages.data[i].header.frame_id, formatBuffer);
        snprintf(formatBuffer, MAX_TF_FRAME_NAME_LENGTH, "%sr%d/tool0", frame_prefix, i + 1);
        rosidl_runtime_c__String__assign(&messages.data[i].child_frame_id, formatBuffer);
        Ros_MpCoord_To_GeomMsgsTransform(&tf_static_data->coordFlangeToTool0, &messages.data[i].transform);
        Ros_Nanos_To_Time_Msg(theTime, &messages.data[i].header.stamp);
    }
    messages.size = motoRos_PositionMonitor_totalRobots;
    bool ret = Ros_StaticTransformBroadcaster_Send(publisher_transform_static, msg_transform_static, messages.data, motoRos_PositionMonitor_totalRobots);
    geometry_msgs__msg__TransformStamped__Sequence__fini(&messages);
    return ret;
}

void Ros_PositionMonitor_Cleanup(rcl_publisher_t* publisher_transform_static, tf2_msgs__msg__TFMessage* msg_transform_static)
{
    MOTOROS2_MEM_TRACE_START(pos_mon_fini);

    rcl_ret_t ret;

    for (int groupNum = 0; groupNum < g_Ros_Controller.numGroup; groupNum += 1)
    {
        CtrlGroup* ctrlGroup = g_Ros_Controller.ctrlGroups[groupNum];

        if (ctrlGroup != NULL)
        {
            ret = rcl_publisher_fini(&ctrlGroup->publisherJointState, &g_microRosNodeInfo.node);
            if (ret != RCL_RET_OK)
                Ros_Debug_BroadcastMsg("Failed cleaning up jointstate publisher for group %d: %d",
                    ctrlGroup->groupNo, ret);
            sensor_msgs__msg__JointState__destroy(ctrlGroup->msgJointState);
        }
    }

    Ros_Debug_BroadcastMsg("Cleanup publisher joint state");
    ret = rcl_publisher_fini(&g_publishers_PositionMonitor.jointStateAllGroups, &g_microRosNodeInfo.node);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up global jointstate publisher: %d", ret);
    sensor_msgs__msg__JointState__destroy(g_messages_PositionMonitor.jointStateAllGroups);

    Ros_Debug_BroadcastMsg("Cleanup TF publisher");
    ret = rcl_publisher_fini(&g_publishers_PositionMonitor.transform, &g_microRosNodeInfo.node);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up TF publisher: %d", ret);
    tf2_msgs__msg__TFMessage__destroy(g_messages_PositionMonitor.transform);

    Ros_StaticTransformBroadcaster_Cleanup(publisher_transform_static, msg_transform_static);

    MOTOROS2_MEM_TRACE_REPORT(pos_mon_fini);
}

void Ros_PositionMonitor_CalculateTransforms(int groupIndex, long* pulsePos_moto, long* pulsePos_moto_track, TF_Static_Data *tf_static_data, INT64 timestamp)
{
    double track_pos_meters[MAX_PULSE_AXES];
    BITSTRING figure;
    MP_COORD cartesian_moto;
    char alarm_msg_buf[ERROR_MSG_MAX_SIZE] = { 0 };

    CtrlGroup* group = g_Ros_Controller.ctrlGroups[groupIndex];

    //some things (like external axes) cannot be converted to cartesian
    if (!Ros_CtrlGroup_IsRobot(group))
        return;

    //this CtrlGroup's pose can be converted, so update ROS transforms

    //first update stamp of this group's transforms
    for (int i = 0; i < PUBLISHED_NUMBER_TRANSFORM_LINKS_PER_ROBOT; i += 1)
    {
        Ros_Nanos_To_Time_Msg(timestamp, &g_messages_PositionMonitor.transform->transforms.data[(groupIndex * PUBLISHED_NUMBER_TRANSFORM_LINKS_PER_ROBOT) + i].header.stamp);
    }

    //=======================
    // Calculate World
    //=======================
    //---------------------------------------------------------
    //NOTE: the term 'base' is a ROS term. This is actually RF.
    //---------------------------------------------------------
    MP_COORD coordWorldToBase;

    memcpy(&coordWorldToBase, &group->robotCalibrationToBaseFrame, sizeof(MP_COORD));

    if (Ros_CtrlGroup_HasBaseTrack(group)) //add in the offset of the base track motion and mounting offset
    {
        MP_COORD coordTrackTravel;
        MP_FRAME frameTrackTravel, frameTrackToRobot, frameWorldToTrack;
        MP_FRAME frameWorldToTravel, frameWorldToRobot;

        //TODO: This isn't going to work for a motosweep

        CtrlGroup* baseTrackGroup = g_Ros_Controller.ctrlGroups[group->baseTrackGroupIndex];
        Ros_CtrlGroup_ConvertToRosPos(baseTrackGroup, pulsePos_moto_track, track_pos_meters);

        bzero(&coordTrackTravel, sizeof(MP_COORD));
        for (int i = 0; i < MAX_PULSE_AXES; i += 1)
        {
            //skip if not a configured axis
            if (Ros_CtrlGroup_IsInvalidAxis(baseTrackGroup, i))
                continue;

            switch (group->baseTrackInfo.motionType[i])
            {
            case MOTION_TYPE_X: coordTrackTravel.x = METERS_TO_MICROMETERS(track_pos_meters[i]); break;
            case MOTION_TYPE_Y: coordTrackTravel.y = METERS_TO_MICROMETERS(track_pos_meters[i]); break;
            case MOTION_TYPE_Z: coordTrackTravel.z = METERS_TO_MICROMETERS(track_pos_meters[i]); break;
            case MOTION_TYPE_RX: coordTrackTravel.rx = RAD_TO_DEG_0001(track_pos_meters[i]); break;
            case MOTION_TYPE_RY: coordTrackTravel.ry = RAD_TO_DEG_0001(track_pos_meters[i]); break;
            case MOTION_TYPE_RZ: coordTrackTravel.rz = RAD_TO_DEG_0001(track_pos_meters[i]); break;
            default:
                //should never happen, so complain and raise alarm
                snprintf(alarm_msg_buf, ERROR_MSG_MAX_SIZE, "Inv. motion type: %d (axis: %d)",
                    group->baseTrackInfo.motionType[i], i);
                Ros_Debug_BroadcastMsg("%s: %s", __func__, alarm_msg_buf);
                motoRosAssert_withMsg(false, SUBCODE_FAIL_INVALID_BASE_TRACK_MOTION_TYPE, alarm_msg_buf);
            }
        }
        mpZYXeulerToFrame(&coordWorldToBase, &frameWorldToTrack);
        mpZYXeulerToFrame(&coordTrackTravel, &frameTrackTravel);
        mpZYXeulerToFrame(&group->baseTrackInfo.offsetFromBaseToRobotOrigin, &frameTrackToRobot);

        mpMulFrame(&frameWorldToTrack, &frameTrackTravel, &frameWorldToTravel);
        mpMulFrame(&frameWorldToTravel, &frameTrackToRobot, &frameWorldToRobot);

        mpFrameToZYXeuler(&frameWorldToRobot, &coordWorldToBase);
    }

    geometry_msgs__msg__Transform* transform = &g_messages_PositionMonitor.transform->transforms.data[(groupIndex * PUBLISHED_NUMBER_TRANSFORM_LINKS_PER_ROBOT) + publishIndex_tfLink_WorldToBase].transform;
    Ros_MpCoord_To_GeomMsgsTransform(&coordWorldToBase, transform);

    //============================
    // Calculate Flange and Tool0
    //============================
    //
    // The "tool0" is actual motoman-flange. The ros "flange" is rotated (0, -90, 180).
    //
    // (The image requires [Memeful Comments] extension for Visual Studio)
    // <image url="$(ProjectDir)image_comments\flange_vs_tool0.png" />
    //
    //---------------------------------------------------------
    //NOTE: the term 'base' is a ROS term. This is actually RF.
    //---------------------------------------------------------
    // <image url="$(ProjectDir)image_comments\tf_diagram.png" />
    //
    MP_FRAME frameBaseToTcp, frameBaseToTool0, frameBaseToFlange, frameTool0ToTcp, frameTcpToTool0;
    MP_FRAME frameFlangeToTcp;
    MP_TOOL_RSP_DATA retToolData;
    MP_COORD coordToolData, coordBaseToFlange;

    long anglePos_moto[MAX_PULSE_AXES];
    mpConvPulseToAngle(groupIndex, pulsePos_moto, anglePos_moto);
    mpConvAxesToCartPos(groupIndex, anglePos_moto, group->tool, &figure, &cartesian_moto);

    //Get current position of TCP
    mpZYXeulerToFrame(&cartesian_moto, &frameBaseToTcp);

    //Get TCP definition
    mpGetToolData(group->tool, &retToolData);
    coordToolData.x = retToolData.x; coordToolData.y = retToolData.y; coordToolData.z = retToolData.z;
    coordToolData.rx = retToolData.rx; coordToolData.ry = retToolData.ry; coordToolData.rz = retToolData.rz;

    mpZYXeulerToFrame(&coordToolData, &frameTool0ToTcp);
    mpInvFrame(&frameTool0ToTcp, &frameTcpToTool0);


    mpMulFrame(&frameBaseToTcp, &frameTcpToTool0, &frameBaseToTool0);

    mpMulFrame(&frameBaseToTool0, &tf_static_data->frameTool0ToFlange, &frameBaseToFlange);
    mpFrameToZYXeuler(&frameBaseToFlange, &coordBaseToFlange);

    mpMulFrame(&tf_static_data->frameFlangeToTool0, &frameTool0ToTcp, &frameFlangeToTcp);
    mpFrameToZYXeuler(&frameFlangeToTcp, &coordToolData);

    //=======================

    transform = &g_messages_PositionMonitor.transform->transforms.data[(groupIndex * PUBLISHED_NUMBER_TRANSFORM_LINKS_PER_ROBOT) + publishIndex_tfLink_BaseToFlange].transform;
    Ros_MpCoord_To_GeomMsgsTransform(&coordBaseToFlange, transform);

    transform = &g_messages_PositionMonitor.transform->transforms.data[(groupIndex * PUBLISHED_NUMBER_TRANSFORM_LINKS_PER_ROBOT) + publishIndex_tfLink_FlangeToTcp].transform;
    Ros_MpCoord_To_GeomMsgsTransform(&coordToolData, transform);
}

void Ros_PositionMonitor_UpdateLocation(TF_Static_Data *tf_static_data)
{
    long pulsePos_moto[MAX_CONTROLLABLE_GROUPS][MAX_PULSE_AXES];
    long pulsePos_moto_track[MAX_CONTROLLABLE_GROUPS][MAX_PULSE_AXES];
    long pulseSpeed_moto[MAX_CONTROLLABLE_GROUPS][MAX_PULSE_AXES];
    double torque[MAX_CONTROLLABLE_GROUPS][MAX_PULSE_AXES];
    rcl_ret_t ret;

    //Timestamp
    INT64 theTime = rmw_uros_epoch_nanos();

    Ros_Nanos_To_Time_Msg(theTime, &g_messages_PositionMonitor.jointStateAllGroups->header.stamp);

    //Read feedback data for all groups prior to the conversion/formatting routines. This
    //ensures the timestamp is more accurate for all groups.
    for (int groupIndex = 0; groupIndex < g_Ros_Controller.numGroup; groupIndex += 1)
    {
        CtrlGroup* group = g_Ros_Controller.ctrlGroups[groupIndex];

        //----------------------------
        BOOL bRet = Ros_CtrlGroup_GetFBPulsePos(group, pulsePos_moto[groupIndex]);
        if (bRet != TRUE)
            continue;
        //if this is attached to a base track, get it's position too
        if (Ros_CtrlGroup_HasBaseTrack(group))
            Ros_CtrlGroup_GetFBPulsePos(g_Ros_Controller.ctrlGroups[group->baseTrackGroupIndex], pulsePos_moto_track[groupIndex]);

        //----------------------------
        bRet = Ros_CtrlGroup_GetFBServoSpeed(group, pulseSpeed_moto[groupIndex]);
        if (bRet != TRUE)
            continue;

        //----------------------------
        Ros_CtrlGroup_GetTorque(group, torque[groupIndex]);
    }

    //for each group
    int iteratorAllAxes = 0;
    for (int groupIndex = 0; groupIndex < g_Ros_Controller.numGroup; groupIndex += 1)
    {
        double radPos_ros[MAX_PULSE_AXES];
        double radSpeed_ros[MAX_PULSE_AXES];
        double torque_ros[MAX_PULSE_AXES];

        CtrlGroup* group = g_Ros_Controller.ctrlGroups[groupIndex];

        //----------------------------
        //TIME
        Ros_Nanos_To_Time_Msg(theTime, &group->msgJointState->header.stamp);

        //----------------------------
        //POSITION
        // joints
        Ros_CtrlGroup_ConvertToRosPos(group, pulsePos_moto[groupIndex], radPos_ros);
        memcpy(group->msgJointState->position.data, radPos_ros, sizeof(double) * group->numAxes);
        memcpy(&g_messages_PositionMonitor.jointStateAllGroups->position.data[iteratorAllAxes], radPos_ros, sizeof(double) * group->numAxes);

        // cartesian
        if (g_nodeConfigSettings.publish_tf)
            Ros_PositionMonitor_CalculateTransforms(groupIndex, pulsePos_moto[groupIndex], pulsePos_moto_track[groupIndex], tf_static_data, theTime);

        //----------------------------
        //VELOCITY
        Ros_CtrlGroup_ConvertToRosPos(group, pulseSpeed_moto[groupIndex], radSpeed_ros);
        memcpy(group->msgJointState->velocity.data, radSpeed_ros, sizeof(double) * group->numAxes);
        memcpy(&g_messages_PositionMonitor.jointStateAllGroups->velocity.data[iteratorAllAxes], radSpeed_ros, sizeof(double) * group->numAxes);

        //----------------------------
        //TORQUE
        Ros_CtrlGroup_ConvertToRosTorque(group, torque[groupIndex], torque_ros);
        memcpy(group->msgJointState->effort.data, torque_ros, sizeof(double) * group->numAxes);
        memcpy(&g_messages_PositionMonitor.jointStateAllGroups->effort.data[iteratorAllAxes], torque_ros, sizeof(double) * group->numAxes);

        //**********************************
        //Set data to be published - per group
        group->msgJointState->position.size =
            group->msgJointState->velocity.size =
            group->msgJointState->effort.size = group->numAxes;

        iteratorAllAxes += group->numAxes;
    }
    //**********************************
    //Set data to be published - all groups
    g_messages_PositionMonitor.jointStateAllGroups->position.size =
        g_messages_PositionMonitor.jointStateAllGroups->velocity.size =
        g_messages_PositionMonitor.jointStateAllGroups->effort.size = g_Ros_Controller.totalAxesCount;
    


    //**********************************
    //Publish feedback topics
    for (int groupIndex = 0; groupIndex < g_Ros_Controller.numGroup; groupIndex += 1)
    {
        ret = rcl_publish(&g_Ros_Controller.ctrlGroups[groupIndex]->publisherJointState, g_Ros_Controller.ctrlGroups[groupIndex]->msgJointState, NULL);
        // publishing can fail, but we choose to ignore those errors in this implementation
        RCL_UNUSED(ret);
    }

    ret = rcl_publish(&g_publishers_PositionMonitor.jointStateAllGroups, g_messages_PositionMonitor.jointStateAllGroups, NULL);
    RCL_UNUSED(ret);

    if (g_nodeConfigSettings.publish_tf)
    {
        ret = rcl_publish(&g_publishers_PositionMonitor.transform, g_messages_PositionMonitor.transform, NULL);
        RCL_UNUSED(ret);
    }
}
