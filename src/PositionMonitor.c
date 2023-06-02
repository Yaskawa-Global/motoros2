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
static void Ros_PositionMonitor_Initialize_TfPublisher(rmw_qos_profile_t const* const qos_profile, int totalRobots);


void Ros_PositionMonitor_Initialize()
{
    MOTOROS2_MEM_TRACE_START(pos_mon_init);

    Ros_Debug_BroadcastMsg("Initializing PositionMonitor publishers");

    //==================================
    //create the global (ie: aggregrate) JointState publisher
    const rmw_qos_profile_t* qos_profile_js = Ros_ConfigFile_To_Rmw_Qos_Profile(g_nodeConfigSettings.qos_joint_states);
    Ros_PositionMonitor_Initialize_GlobalJointStatePublisher(qos_profile_js);

    //==================================
    //create a JointState publisher for each group
    int totalRobots = 0;
    for (int grpIndex = 0; grpIndex < g_Ros_Controller.numGroup; grpIndex++)
    {
        CtrlGroup* ctrlGroup = g_Ros_Controller.ctrlGroups[grpIndex];
        Ros_PositionMonitor_Initialize_PerGroupJointStatePublisher(qos_profile_js, ctrlGroup, grpIndex);
        if (Ros_CtrlGroup_IsRobot(ctrlGroup))
            totalRobots += 1;
    }

    //==================================
    //Create publisher for cartesian transform
    //Rviz2 expects the QoS to be RELIABLE, but user could have configured something else
    const rmw_qos_profile_t* qos_profile_tf = Ros_ConfigFile_To_Rmw_Qos_Profile(g_nodeConfigSettings.qos_tf);
    Ros_PositionMonitor_Initialize_TfPublisher(qos_profile_tf, totalRobots);

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
    motoRosAssert(ret == RCL_RET_OK, SUBCODE_FAIL_CREATE_PUBLISHER_JOINT_STATE_ALL);

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
    motoRosAssert(ret == RCL_RET_OK, SUBCODE_FAIL_CREATE_PUBLISHER_JOINT_STATE);

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

static void Ros_PositionMonitor_Initialize_TfPublisher(rmw_qos_profile_t const* const qos_profile, int totalRobots)
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
    motoRosAssert(ret == RCL_RET_OK, SUBCODE_FAIL_CREATE_PUBLISHER_TRANSFORM);

    //--------------
    //create message for cartesian transform
    g_messages_PositionMonitor.transform = tf2_msgs__msg__TFMessage__create();

    motoRosAssert(geometry_msgs__msg__TransformStamped__Sequence__init(&g_messages_PositionMonitor.transform->transforms, totalRobots * NUMBER_TRANSFORM_LINKS_PER_ROBOT),
                  SUBCODE_FAIL_ALLOCATE_TRANSFORM);

    bzero(formatBuffer, MAX_TF_FRAME_NAME_LENGTH);
    int robotIterator = 0;
    const char* frame_prefix = g_nodeConfigSettings.tf_frame_prefix;
    for (int i = 0; i < (totalRobots * NUMBER_TRANSFORM_LINKS_PER_ROBOT); i += NUMBER_TRANSFORM_LINKS_PER_ROBOT)
    {
        snprintf(formatBuffer, MAX_TF_FRAME_NAME_LENGTH, "%sworld", frame_prefix);
        rosidl_runtime_c__String__assign(&g_messages_PositionMonitor.transform->transforms.data[i + tfLink_WorldToBase].header.frame_id, formatBuffer);

        snprintf(formatBuffer, MAX_TF_FRAME_NAME_LENGTH, "%sr%d/base", frame_prefix, robotIterator + 1);
        rosidl_runtime_c__String__assign(&g_messages_PositionMonitor.transform->transforms.data[i + tfLink_WorldToBase].child_frame_id, formatBuffer);
        rosidl_runtime_c__String__assign(&g_messages_PositionMonitor.transform->transforms.data[i + tfLink_BaseToFlange].header.frame_id, formatBuffer);

        snprintf(formatBuffer, MAX_TF_FRAME_NAME_LENGTH, "%sr%d/flange", frame_prefix, robotIterator + 1);
        rosidl_runtime_c__String__assign(&g_messages_PositionMonitor.transform->transforms.data[i + tfLink_BaseToFlange].child_frame_id, formatBuffer);
        rosidl_runtime_c__String__assign(&g_messages_PositionMonitor.transform->transforms.data[i + tfLink_FlangeToTool0].header.frame_id, formatBuffer);
        rosidl_runtime_c__String__assign(&g_messages_PositionMonitor.transform->transforms.data[i + tfLink_FlangeToTcp].header.frame_id, formatBuffer);

        snprintf(formatBuffer, MAX_TF_FRAME_NAME_LENGTH, "%sr%d/tool0", frame_prefix, robotIterator + 1);
        rosidl_runtime_c__String__assign(&g_messages_PositionMonitor.transform->transforms.data[i + tfLink_FlangeToTool0].child_frame_id, formatBuffer);

        snprintf(formatBuffer, MAX_TF_FRAME_NAME_LENGTH, "%sr%d/tcp_%d", frame_prefix, robotIterator + 1, g_Ros_Controller.ctrlGroups[robotIterator]->tool);
        rosidl_runtime_c__String__assign(&g_messages_PositionMonitor.transform->transforms.data[i + tfLink_FlangeToTcp].child_frame_id, formatBuffer);

        robotIterator += 1;
    }
}

void Ros_PositionMonitor_Cleanup()
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

    MOTOROS2_MEM_TRACE_REPORT(pos_mon_fini);
}

void Ros_PositionMonitor_CalculateTransforms(int groupIndex, long* pulsePos_moto, long* pulsePos_moto_track, INT64 timestamp)
{
    double track_pos_meters[MAX_PULSE_AXES];
    BITSTRING figure;
    MP_COORD cartesian_moto;

    CtrlGroup* group = g_Ros_Controller.ctrlGroups[groupIndex];

    //some things (like external axes) cannot be converted to cartesian
    if (!Ros_CtrlGroup_IsRobot(group))
        return;

    //this CtrlGroup's pose can be converted, so update ROS transforms

    //first update stamp of this group's transforms
    for (int i = 0; i < NUMBER_TRANSFORM_LINKS_PER_ROBOT; i += 1)
    {
        Ros_Nanos_To_Time_Msg(timestamp, &g_messages_PositionMonitor.transform->transforms.data[(groupIndex * NUMBER_TRANSFORM_LINKS_PER_ROBOT) + i].header.stamp);
    }

    //=======================
    // Calculate World
    //=======================
    //---------------------------------------------------------
    //NOTE: the term 'base' is a ROS term. This is actually RF.
    //---------------------------------------------------------
    MP_COORD coordWorldToBase;

    memcpy(&coordWorldToBase, &group->robotCalibrationToBaseFrame, sizeof(MP_COORD));

    if (group->baseTrackGroupIndex != -1) //add in the offset of the base track motion and mounting offset
    {
        MP_COORD coordTrackTravel;
        MP_FRAME frameTrackTravel, frameTrackToRobot, frameWorldToTrack;
        MP_FRAME frameWorldToTravel, frameWorldToRobot;

        //TODO: This isn't going to work for a motosweep

        Ros_CtrlGroup_ConvertToRosPos(g_Ros_Controller.ctrlGroups[group->baseTrackGroupIndex], pulsePos_moto_track, track_pos_meters);

        bzero(&coordTrackTravel, sizeof(MP_COORD));
        for (int i = 0; i < MAX_PULSE_AXES; i += 1)
        {
            switch (group->baseTrackInfo.motionType[i])
            {
            case MOTION_TYPE_X: coordTrackTravel.x = track_pos_meters[i]; break;
            case MOTION_TYPE_Y: coordTrackTravel.y = track_pos_meters[i]; break;
            case MOTION_TYPE_Z: coordTrackTravel.z = track_pos_meters[i]; break;
            case MOTION_TYPE_RX: coordTrackTravel.rx = track_pos_meters[i]; break;
            case MOTION_TYPE_RY: coordTrackTravel.ry = track_pos_meters[i]; break;
            case MOTION_TYPE_RZ: coordTrackTravel.rz = track_pos_meters[i]; break;
            }
        }
        mpZYXeulerToFrame(&coordWorldToBase, &frameWorldToTrack);
        mpZYXeulerToFrame(&coordTrackTravel, &frameTrackTravel);
        mpZYXeulerToFrame(&group->baseTrackInfo.offsetFromBaseToRobotOrigin, &frameTrackToRobot);

        mpMulFrame(&frameWorldToTrack, &frameTrackTravel, &frameWorldToTravel);
        mpMulFrame(&frameWorldToTravel, &frameTrackToRobot, &frameWorldToRobot);

        mpFrameToZYXeuler(&frameWorldToRobot, &coordWorldToBase);
    }

    geometry_msgs__msg__Transform* transform = &g_messages_PositionMonitor.transform->transforms.data[(groupIndex * NUMBER_TRANSFORM_LINKS_PER_ROBOT) + tfLink_WorldToBase].transform;
    transform->translation.x = NANOMETERS_TO_METERS(coordWorldToBase.x);
    transform->translation.y = NANOMETERS_TO_METERS(coordWorldToBase.y);
    transform->translation.z = NANOMETERS_TO_METERS(coordWorldToBase.z);
    QuatConversion_MpCoordOrient_To_GeomMsgsQuaternion(coordWorldToBase.rx, coordWorldToBase.ry, coordWorldToBase.rz, &transform->rotation);

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
    MP_FRAME frameTool0ToFlange, frameFlangeToTool0, frameFlangeToTcp;
    MP_TOOL_RSP_DATA retToolData;
    MP_COORD coordToolData, coordBaseToFlange, coordFlangeToTool0;
    MP_XYZ vectorOrg, vectorX, vectorY; //for mpMakeFrame

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

    //Make rotational frames
    vectorOrg.x = 0;    vectorOrg.y = 0;    vectorOrg.z = 0;
    vectorX.x = 0;      vectorX.y = 0;      vectorX.z = 1;
    vectorY.x = 0;      vectorY.y = -1;     vectorY.z = 0;
    mpMakeFrame(&vectorOrg, &vectorX, &vectorY, &frameTool0ToFlange);
    mpInvFrame(&frameTool0ToFlange, &frameFlangeToTool0);
    mpFrameToZYXeuler(&frameFlangeToTool0, &coordFlangeToTool0);

    mpMulFrame(&frameBaseToTool0, &frameTool0ToFlange, &frameBaseToFlange);
    mpFrameToZYXeuler(&frameBaseToFlange, &coordBaseToFlange);

    mpMulFrame(&frameFlangeToTool0, &frameTool0ToTcp, &frameFlangeToTcp);
    mpFrameToZYXeuler(&frameFlangeToTcp, &coordToolData);

    //=======================

    transform = &g_messages_PositionMonitor.transform->transforms.data[(groupIndex * NUMBER_TRANSFORM_LINKS_PER_ROBOT) + tfLink_BaseToFlange].transform;
    transform->translation.x = NANOMETERS_TO_METERS(coordBaseToFlange.x);
    transform->translation.y = NANOMETERS_TO_METERS(coordBaseToFlange.y);
    transform->translation.z = NANOMETERS_TO_METERS(coordBaseToFlange.z);
    QuatConversion_MpCoordOrient_To_GeomMsgsQuaternion(coordBaseToFlange.rx, coordBaseToFlange.ry, coordBaseToFlange.rz, &transform->rotation);

    transform = &g_messages_PositionMonitor.transform->transforms.data[(groupIndex * NUMBER_TRANSFORM_LINKS_PER_ROBOT) + tfLink_FlangeToTool0].transform;
    transform->translation.x = NANOMETERS_TO_METERS(coordFlangeToTool0.x);
    transform->translation.y = NANOMETERS_TO_METERS(coordFlangeToTool0.y);
    transform->translation.z = NANOMETERS_TO_METERS(coordFlangeToTool0.z);
    QuatConversion_MpCoordOrient_To_GeomMsgsQuaternion(coordFlangeToTool0.rx, coordFlangeToTool0.ry, coordFlangeToTool0.rz, &transform->rotation);

    transform = &g_messages_PositionMonitor.transform->transforms.data[(groupIndex * NUMBER_TRANSFORM_LINKS_PER_ROBOT) + tfLink_FlangeToTcp].transform;
    transform->translation.x = NANOMETERS_TO_METERS(coordToolData.x);
    transform->translation.y = NANOMETERS_TO_METERS(coordToolData.y);
    transform->translation.z = NANOMETERS_TO_METERS(coordToolData.z);
    QuatConversion_MpCoordOrient_To_GeomMsgsQuaternion(coordToolData.rx, coordToolData.ry, coordToolData.rz, &transform->rotation);
}

void Ros_PositionMonitor_UpdateLocation()
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
        if (group->baseTrackGroupIndex != -1)
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
            Ros_PositionMonitor_CalculateTransforms(groupIndex, pulsePos_moto[groupIndex], pulsePos_moto_track[groupIndex], theTime);

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
