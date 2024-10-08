// ActionServer_FJT.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

//====================================================================
//public data
rclc_action_server_t g_actionServerFollowJointTrajectory;
control_msgs__action__FollowJointTrajectory_SendGoal_Request g_actionServer_FJT_SendGoal_Request;
UINT32 g_actionServer_FJT_SendGoal_Request__sizeof;

//====================================================================
//private data
micro_ros_utilities_memory_conf_t feedback_msg_alloc_cfg = { 0 };

control_msgs__action__FollowJointTrajectory_FeedbackMessage feedback_FollowJointTrajectory;

//====================================================================
//Static Memory (used for FJT goal storage)

//TODO: shrink this once I have found a good buffer size
#define SIZEOF_BUFFER_FJT_GOAL (2000000)
UINT8 Ros_StaticAllocationBuffer_FJTgoal[SIZEOF_BUFFER_FJT_GOAL];


typedef enum
{
    GOAL_COMPLETE,
    GOAL_CANCEL,
    GOAL_ABORT_DUE_TO_ERROR
} GOAL_END_TYPE;

INT64 fjt_trajectory_start_time_ns;

rclc_action_goal_handle_t* fjt_active_goal_handle;
rclc_action_goal_handle_t* fjt_rejected_goal_handle;
control_msgs__action__FollowJointTrajectory_GetResult_Response fjt_result_response;
rcl_action_goal_state_t fjt_goal_state;

BOOL fjt_result_message_ready;

#define RESULT_REPONSE_ERROR_CODE(rosCode, motomanCode) ((rosCode * 100000) - motomanCode)

//====================================================================
//private declarations
void Ros_ActionServer_FJT_ResetProgressTracker();
void Ros_ActionServer_FJT_Goal_Complete(GOAL_END_TYPE goal_end_type);
void Ros_ActionServer_FJT_DeleteFeedbackMessage();

//===================================================================
void Ros_ActionServer_FJT_Initialize()
{
    MOTOROS2_MEM_TRACE_START(fjt_init);

    Ros_Debug_BroadcastMsg("Initializing ActionServer FollowJointTrajectory");

    fjt_active_goal_handle = NULL;
    fjt_rejected_goal_handle = NULL;
    fjt_result_message_ready = FALSE;

    //===============================================
    //allocation config for feedback messages
    feedback_msg_alloc_cfg.allocator = &g_motoros2_Allocator;
    feedback_msg_alloc_cfg.max_string_capacity = MAX_JOINT_NAME_LENGTH;
    feedback_msg_alloc_cfg.max_ros2_type_sequence_capacity = MAX_CONTROLLABLE_GROUPS * MP_GRP_AXES_NUM;
    feedback_msg_alloc_cfg.max_basic_type_sequence_capacity = MAX_CONTROLLABLE_GROUPS * MP_GRP_AXES_NUM;
    bzero(&feedback_FollowJointTrajectory, sizeof(feedback_FollowJointTrajectory));

    //===============================================
    //create action server
    const rosidl_action_type_support_t* action_type_support = ROSIDL_GET_ACTION_TYPE_SUPPORT(control_msgs, FollowJointTrajectory);
    rclc_action_server_init_default(&g_actionServerFollowJointTrajectory, &g_microRosNodeInfo.node, &g_microRosNodeInfo.support, action_type_support, ACTION_NAME_FOLLOW_JOINT_TRAJECTORY);

    //===============================================
    //configure how much memory to allocate for the FJT request message
    static micro_ros_utilities_memory_conf_t goal_svc_req_msg_alloc_cfg = { 0 };
    int maxAxes = MAX_CONTROLLABLE_GROUPS * MP_GRP_AXES_NUM;
    goal_svc_req_msg_alloc_cfg.max_string_capacity = MAX_JOINT_NAME_LENGTH;
    goal_svc_req_msg_alloc_cfg.max_ros2_type_sequence_capacity = maxAxes;
    goal_svc_req_msg_alloc_cfg.max_basic_type_sequence_capacity = maxAxes;

    micro_ros_utilities_memory_rule_t rules[] = {
        {"goal.trajectory.joint_names", maxAxes}, //number of joints
        {"goal.trajectory.joint_names.data", MAX_JOINT_NAME_LENGTH}, //string length for joint name
        {"goal.trajectory.points", MAX_NUMBER_OF_POINTS_PER_TRAJECTORY}, //number of points in trajectory
        {"goal.trajectory.points.positions", maxAxes}, //number of positions in a point (max axes)
        {"goal.trajectory.points.velocities", maxAxes}, //number of velocities in a point (max axes)
        {"goal.trajectory.points.accelerations", maxAxes}, //number of accelerations in a point (max axes)
        {"goal.trajectory.points.effort", maxAxes}, //number of effort in a point (max axes)

        //NOTE: Setting these to zero to 'disable' multi-dof trajectory
        {"goal.multi_dof_trajectory.joint_names", 0}, //each point will have cartesian position for each group
        {"goal.multi_dof_trajectory.joint_names.data", 0}, //string length for group name
        {"goal.multi_dof_trajectory.points", 0}, //number of points in trajectory
        {"goal.multi_dof_trajectory.points.transforms", 0}, //each point will have cartesian position for each group
        {"goal.multi_dof_trajectory.points.velocities", 0}, //each point will have cartesian position for each group
        {"goal.multi_dof_trajectory.points.accelerations", 0}, //each point will have cartesian position for each group

        {"goal.path_tolerance", MAX_NUMBER_OF_POINTS_PER_TRAJECTORY}, //number of points in trajectory
        {"goal.goal_tolerance", maxAxes}, //number of joints
    };

    goal_svc_req_msg_alloc_cfg.rules = rules;
    goal_svc_req_msg_alloc_cfg.n_rules = sizeof(rules) / sizeof(rules[0]);

    //----------------
    //Create goal-request message using STATIC buffer. My heap is very limited, so I'm cheating by using
    //static block of memory that is allocated in MemoryAllocation.c (Ros_StaticAllocationBuffer_FJTgoal)
    Ros_Debug_BroadcastMsg("Allocating FollowJointTrajectory goal request");
    Ros_Debug_BroadcastMsg("Maximum length of trajectories: %d points", MAX_NUMBER_OF_POINTS_PER_TRAJECTORY);

    g_actionServer_FJT_SendGoal_Request__sizeof = sizeof(g_actionServer_FJT_SendGoal_Request) +
        micro_ros_utilities_get_static_size(ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, action, FollowJointTrajectory_SendGoal_Request), goal_svc_req_msg_alloc_cfg);
    Ros_Debug_BroadcastMsg("g_actionServer_FJT_SendGoal_Request__sizeof = %d", g_actionServer_FJT_SendGoal_Request__sizeof);

    bzero(Ros_StaticAllocationBuffer_FJTgoal, sizeof(Ros_StaticAllocationBuffer_FJTgoal));
    micro_ros_utilities_create_static_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, action, FollowJointTrajectory_SendGoal_Request),
        &g_actionServer_FJT_SendGoal_Request,
        goal_svc_req_msg_alloc_cfg,
        Ros_StaticAllocationBuffer_FJTgoal,
        sizeof(Ros_StaticAllocationBuffer_FJTgoal));

    MOTOROS2_MEM_TRACE_REPORT(fjt_init);
}

void Ros_ActionServer_FJT_Cleanup()
{
    MOTOROS2_MEM_TRACE_START(fjt_fini);

    Ros_Debug_BroadcastMsg("Cleanup FollowJointTrajectory server");
    rclc_action_server_fini(&g_actionServerFollowJointTrajectory, &g_microRosNodeInfo.node);

    //Memory for actionServer_FJT_SendGoal_Request was not allocated off the heap. It was taken from a static buffer.
    //Clear the buffer and any pointers into it.
    bzero(&g_actionServer_FJT_SendGoal_Request, sizeof(g_actionServer_FJT_SendGoal_Request));
    bzero(Ros_StaticAllocationBuffer_FJTgoal, sizeof(Ros_StaticAllocationBuffer_FJTgoal));

    if (fjt_result_response.result.error_string.data != NULL)
        micro_ros_string_utilities_destroy(&fjt_result_response.result.error_string);

    Ros_ActionServer_FJT_DeleteFeedbackMessage();

    MOTOROS2_MEM_TRACE_REPORT(fjt_fini);
}

void Ros_ActionServer_FJT_DeleteFeedbackMessage()
{
    //See if the message has been allocated before attempting to dealloc.
    //TODO: verify this is absolutely necessary. micro_ros_utilities_destroy_message_memory(..)
    //      has been observed to fail destroying a message that hasn't been allocated, so
    //      adding this check
    if (feedback_FollowJointTrajectory.feedback.joint_names.capacity > 0)
    {
        micro_ros_utilities_destroy_message_memory(
            ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, action, FollowJointTrajectory_FeedbackMessage),
            &feedback_FollowJointTrajectory,
            feedback_msg_alloc_cfg);
    }
}

rcl_ret_t Ros_ActionServer_FJT_Goal_Received(rclc_action_goal_handle_t* goal_handle, void* context)
{
    (void)context;

    //-----------RECEIVE REQUEST
    control_msgs__action__FollowJointTrajectory_SendGoal_Request* pending_ros_goal_request =
        (control_msgs__action__FollowJointTrajectory_SendGoal_Request*)goal_handle->ros_goal_request;

    Ros_Debug_BroadcastMsg("Trajectory contains %d points", pending_ros_goal_request->goal.trajectory.points.size);


    bool bMotionModeOk = Ros_MotionControl_IsMotionMode_Trajectory();
    bool bSizeOk = (pending_ros_goal_request->goal.trajectory.points.size <= MAX_NUMBER_OF_POINTS_PER_TRAJECTORY);
    bool bMotionReady = Ros_Controller_IsMotionReady();

    if (bMotionModeOk && bSizeOk && !bMotionReady && Ros_Controller_IsEcoMode()) //energy saving function
    {
        Ros_Debug_BroadcastMsg("Energy saving function is active. Re-enabling the robot.");

        //invoke the start_traj_mode service
        motoros2_interfaces__srv__StartTrajMode_Response* responseMsg =
            motoros2_interfaces__srv__StartTrajMode_Response__create();

        Ros_ServiceStartTrajMode_Trigger(NULL, responseMsg);

        Ros_Debug_BroadcastMsg(responseMsg->message.data);

        if (responseMsg->result_code.value == MOTION_READY)
            bMotionReady = Ros_Controller_IsMotionReady();

        motoros2_interfaces__srv__StartTrajMode_Response__destroy(responseMsg);

        Ros_Debug_BroadcastMsg("Robot has been re-enabled.");
    }

    Init_Trajectory_Status trajStatus = INIT_TRAJ_OK;
    bool bInitOk = FALSE;
    if (bSizeOk && bMotionReady && bMotionModeOk)
    {
        trajStatus = Ros_MotionControl_InitTrajectory(pending_ros_goal_request);
        bInitOk = (trajStatus == INIT_TRAJ_OK);
    }

    //-----------RESPOND TO REQUEST
    if (bSizeOk && bMotionReady && bMotionModeOk && bInitOk)
    {
        // ---- Build feedback message
        micro_ros_utilities_create_message_memory(
            ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, action, FollowJointTrajectory_FeedbackMessage),
            &feedback_FollowJointTrajectory,
            feedback_msg_alloc_cfg);

        fjt_active_goal_handle = goal_handle;

        Ros_Debug_BroadcastMsg("feedback_msg_alloc_cfg size = %d bytes", sizeof(feedback_FollowJointTrajectory) + micro_ros_utilities_get_dynamic_size(ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, action, FollowJointTrajectory_FeedbackMessage), feedback_msg_alloc_cfg));

        //populate joint names into the feedback message (copy from the /joint_states message)
        int numJoints = g_messages_PositionMonitor.jointStateAllGroups->name.size;
        feedback_FollowJointTrajectory.feedback.joint_names.size = numJoints;
        for (int i = 0; i < numJoints; i += 1)
        {
            rosidl_runtime_c__String__assign(&feedback_FollowJointTrajectory.feedback.joint_names.data[i],
                                             g_messages_PositionMonitor.jointStateAllGroups->name.data[i].data);
        }

        Ros_ActionServer_FJT_ResetProgressTracker();

        fjt_trajectory_start_time_ns = rmw_uros_epoch_nanos();
    }
    else
    {
        //In order to provide feedback to the user about "why" it was rejected,
        //we must first accept the goal and then abort it w/o executing it.
        //https://github.com/ros2/rclc/issues/271

        if (fjt_rejected_goal_handle) //result string already pending
            micro_ros_string_utilities_destroy(&fjt_result_response.result.error_string);

        fjt_rejected_goal_handle = goal_handle;

        int motomanErrorCode = 0;
        if (!bSizeOk)
        {
            motomanErrorCode = INIT_TRAJ_TOO_BIG;
            rosidl_runtime_c__String__assign(&fjt_result_response.result.error_string,
                "Trajectory contains too many points (Not enough memory).");
        }
        else if (!bMotionReady)
        {
            motomanErrorCode = Ros_Controller_GetNotReadySubcode(false);
            rosidl_runtime_c__String__assign(&fjt_result_response.result.error_string,
                Ros_ErrorHandling_MotionNotReadyCode_ToString((MotionNotReadyCode)motomanErrorCode));
        }
        else if (!bMotionModeOk)
        {
            motomanErrorCode = INIT_TRAJ_WRONG_MODE;
            rosidl_runtime_c__String__assign(&fjt_result_response.result.error_string, 
                "Must call " SERVICE_NAME_START_TRAJ_MODE " service.");
        }
        else if (!bInitOk)
        {
            motomanErrorCode = trajStatus;
            switch (motomanErrorCode)
            {
            case INIT_TRAJ_TOO_SMALL:
                rosidl_runtime_c__String__assign(&fjt_result_response.result.error_string,
                    "Trajectory must contain at least two points.");
                break;
            case INIT_TRAJ_INVALID_STARTING_POS:
                rosidl_runtime_c__String__assign(&fjt_result_response.result.error_string,
                    "The first point must match the robot's current position.");
                break;
            case INIT_TRAJ_INVALID_VELOCITY:
                rosidl_runtime_c__String__assign(&fjt_result_response.result.error_string,
                    "The commanded velocity is too high.");
                break;
            case INIT_TRAJ_ALREADY_IN_MOTION:
                rosidl_runtime_c__String__assign(&fjt_result_response.result.error_string,
                    "Already running a trajectory.");
                break;
            case INIT_TRAJ_INVALID_JOINTNAME:
                rosidl_runtime_c__String__assign(&fjt_result_response.result.error_string,
                    "Invalid joint name specified. Check motoros2_config.yaml.");
                break;
            case INIT_TRAJ_INCOMPLETE_JOINTLIST:
                rosidl_runtime_c__String__assign(&fjt_result_response.result.error_string,
                    "Trajectory must contain data for all joints.");
                break;
            case INIT_TRAJ_INVALID_TIME:
                rosidl_runtime_c__String__assign(&fjt_result_response.result.error_string,
                    "Invalid time in trajectory.");
                break;
            case INIT_TRAJ_BACKWARD_TIME:
                rosidl_runtime_c__String__assign(&fjt_result_response.result.error_string,
                    "Trajectory message contains waypoints that are not strictly increasing in time.");
                break;
            case INIT_TRAJ_WRONG_NUMBER_OF_POSITIONS:
                rosidl_runtime_c__String__assign(&fjt_result_response.result.error_string,
                    "Trajectory did not contain position data for all axes.");
                break;
            case INIT_TRAJ_WRONG_NUMBER_OF_VELOCITIES:
                rosidl_runtime_c__String__assign(&fjt_result_response.result.error_string,
                    "Trajectory did not contain velocity data for all axes.");
                break;
            case INIT_TRAJ_INVALID_ENDING_VELOCITY:
                rosidl_runtime_c__String__assign(&fjt_result_response.result.error_string,
                    "The final point in the trajectory must have zero velocity.");
                break;
            case INIT_TRAJ_INVALID_ENDING_ACCELERATION:
                rosidl_runtime_c__String__assign(&fjt_result_response.result.error_string,
                    "The final point in the trajectory must have zero acceleration.");
                break;
            case INIT_TRAJ_DUPLICATE_JOINT_NAME:
                rosidl_runtime_c__String__assign(&fjt_result_response.result.error_string,
                    "The trajectory contains duplicate joint names.");
                break;
            default:
                rosidl_runtime_c__String__assign(&fjt_result_response.result.error_string,
                    "Trajectory initialization failed. Generic failure.");
            }
        }

        fjt_result_response.result.error_code = RESULT_REPONSE_ERROR_CODE(control_msgs__action__FollowJointTrajectory_Result__INVALID_GOAL, motomanErrorCode);
        fjt_result_response.status = GOAL_STATE_ABORTED;

        fjt_result_message_ready = TRUE;

        Ros_Debug_BroadcastMsg("FollowJointTrajectory - Goal request rejected");
        Ros_Debug_BroadcastMsg("The trajectory will be accepted and then immediately aborted");
        Ros_Debug_BroadcastMsg(fjt_result_response.result.error_string.data);
    }

    return RCL_RET_ACTION_GOAL_ACCEPTED;
}

void Ros_ActionServer_FJT_ResetProgressTracker()
{
    control_msgs__action__FollowJointTrajectory_Feedback* feedback = &feedback_FollowJointTrajectory.feedback;


    //===========================================
    //Reset the progress tracker to the current position.
    //The PositionMonitor functions are already polling the information we need and
    //storing it in g_messages_PositionMonitor.
    //
    feedback->actual.time_from_start.sec =
        feedback->actual.time_from_start.nanosec = 0;
    //===========================================
    //actual position
    memcpy(feedback->actual.positions.data,
        g_messages_PositionMonitor.jointStateAllGroups->position.data,
        sizeof(double) * g_messages_PositionMonitor.jointStateAllGroups->position.size);

    feedback->actual.positions.size = g_messages_PositionMonitor.jointStateAllGroups->position.size;

    //--------------------
    //actual velocity
    memcpy(feedback->actual.velocities.data,
        g_messages_PositionMonitor.jointStateAllGroups->velocity.data,
        sizeof(double) * g_messages_PositionMonitor.jointStateAllGroups->velocity.size);

    feedback->actual.velocities.size = g_messages_PositionMonitor.jointStateAllGroups->velocity.size;

    //--------------------
    //actual effort
    memcpy(feedback->actual.effort.data,
        g_messages_PositionMonitor.jointStateAllGroups->effort.data,
        sizeof(double) * g_messages_PositionMonitor.jointStateAllGroups->effort.size);

    feedback->actual.effort.size = g_messages_PositionMonitor.jointStateAllGroups->effort.size;

    //===========================================
    //desired position
    memcpy(feedback->desired.positions.data,
        g_messages_PositionMonitor.jointStateAllGroups->position.data,
        sizeof(double) * g_messages_PositionMonitor.jointStateAllGroups->position.size);

    feedback->desired.positions.size = g_messages_PositionMonitor.jointStateAllGroups->position.size;

    //--------------------
    //desired velocity
    bzero(feedback->desired.velocities.data,
        sizeof(double) * g_messages_PositionMonitor.jointStateAllGroups->velocity.size);

    //--------------------
    //desired effort
    bzero(feedback->desired.effort.data,
        sizeof(double) * g_messages_PositionMonitor.jointStateAllGroups->effort.size);

    //===========================================
    //error in position (zero)
    bzero(feedback->error.positions.data,
        sizeof(double) * g_messages_PositionMonitor.jointStateAllGroups->position.size);

    feedback->error.positions.size = g_messages_PositionMonitor.jointStateAllGroups->position.size;

    //TODO: do multidof too
}

//Called from TrajectoryMotionControl::Ros_MotionControl_IncMoveLoopStart
void Ros_ActionServer_FJT_UpdateProgressTracker(MP_EXPOS_DATA* incrementData)
{
    if (fjt_active_goal_handle == NULL)
        return;

    int iteratorAllAxes = 0;
    for (int groupIndex = 0; groupIndex < g_Ros_Controller.numGroup; groupIndex += 1)
    {
        double radRosOrder[MP_GRP_AXES_NUM];
        CtrlGroup* ctrlGroup = g_Ros_Controller.ctrlGroups[groupIndex];
        Ros_CtrlGroup_ConvertToRosPos(ctrlGroup, incrementData->grp_pos_info[groupIndex].pos, radRosOrder);

        for (int i = 0; i < ctrlGroup->numAxes; i += 1, iteratorAllAxes += 1)
            feedback_FollowJointTrajectory.feedback.desired.positions.data[iteratorAllAxes] = feedback_FollowJointTrajectory.feedback.actual.positions.data[iteratorAllAxes] + radRosOrder[i];
    }
}

//Called from Communication Executor
void Ros_ActionServer_FJT_ProcessFeedback()
{
    if (fjt_active_goal_handle != NULL && !fjt_result_message_ready)
    {
        // ---- Publish feedback

        //-----------------------------------------------------------------------------
        //Populate feedback_FollowJointTrajectory;
        feedback_FollowJointTrajectory.goal_id = fjt_active_goal_handle->goal_id;

        //Use timestamp from when this positional data was captured
        feedback_FollowJointTrajectory.feedback.header.stamp = g_messages_PositionMonitor.transform->transforms.data[0].header.stamp;

        //The PositionMonitor functions are already polling the information we need and
        //storing it in g_messages_PositionMonitor.
        memcpy(feedback_FollowJointTrajectory.feedback.actual.positions.data,  //POSITION
            g_messages_PositionMonitor.jointStateAllGroups->position.data,
            sizeof(double) * g_messages_PositionMonitor.jointStateAllGroups->position.size);

        memcpy(feedback_FollowJointTrajectory.feedback.actual.velocities.data, //VELOCITY
            g_messages_PositionMonitor.jointStateAllGroups->velocity.data,
            sizeof(double) * g_messages_PositionMonitor.jointStateAllGroups->velocity.size);

        memcpy(feedback_FollowJointTrajectory.feedback.actual.effort.data,     //EFFORT
            g_messages_PositionMonitor.jointStateAllGroups->effort.data,
            sizeof(double) * g_messages_PositionMonitor.jointStateAllGroups->effort.size);

        for (int i = 0; i < (MAX_CONTROLLABLE_GROUPS * MP_GRP_AXES_NUM); i += 1)
        {
            //.desired was set in ActionServer_FJT_UpdateProgressTracker (called from increment-move loop)
            feedback_FollowJointTrajectory.feedback.error.positions.data[i] =
                feedback_FollowJointTrajectory.feedback.desired.positions.data[i] -
                feedback_FollowJointTrajectory.feedback.actual.positions.data[i];
        }

        //-----------------------------------------------------------------------------
        rclc_action_publish_feedback(fjt_active_goal_handle, &feedback_FollowJointTrajectory);

        //-----------------------------------------------------------------------------
        //Check to see if the trajectory is complete
        if (Ros_Controller_IsAnyFaultActive()) //See if an alarm occurred during execution of the trajectory
        {
            Ros_Debug_BroadcastMsg("Alarm/error occurred while executing trajectory. Aborting.");

            Ros_MotionControl_StopMotion(/*bKeepJobRunning = */ FALSE);

            Ros_ActionServer_FJT_Goal_Complete(GOAL_ABORT_DUE_TO_ERROR);
        }
        else if ((!Ros_MotionControl_HasDataToProcess()) && !Ros_Controller_IsInMotion())
        {
            Ros_Debug_BroadcastMsg("Trajectory complete");

            Ros_ActionServer_FJT_Goal_Complete(GOAL_COMPLETE);
        }
        else if ((!Ros_MotionControl_HasDataToProcess()) && Ros_Controller_IsInMotion())
        {
            Ros_Debug_BroadcastMsg("Robot still in motion");
        }
    }
}

static STATUS Ros_ActionServer_FJT_Parse_GoalPosTolerances(
    control_msgs__msg__JointTolerance__Sequence const* const goal_joint_tolerances /* in */,
    rosidl_runtime_c__String__Sequence const* const joint_names /* in */,
    double* posTolerances /* out */, size_t posTolerances_len /* in */)
{
    if (goal_joint_tolerances == NULL || joint_names == NULL || posTolerances == NULL)
    {
        Ros_Debug_BroadcastMsg("%s: one or more inputs NULL, abort", __func__);
        return -1;
    }

    //always configure defaults
    for (int i = 0; i < posTolerances_len; ++i)
        posTolerances[i] = DEFAULT_FJT_GOAL_POSITION_TOLERANCE;

    //if caller hasn't passed any JointTolerances, set all entries to default
    if (goal_joint_tolerances->size == 0)
    {
        Ros_Debug_BroadcastMsg("%s: no joint tolerances specified, returning %d defaults",
            __func__, posTolerances_len);
        return OK;
    }

    //it's OK if we are passed more JointTolerance instances than we have joints,
    //as technically a client could send duplicate tolerance specs (ie: multiple
    //JointTolerance instances referring to the same joint).
    //Last-setting wins, so we can just parse all of them.

    //it's NOT ok to have less elements in the output array than the number of
    //known joint names: maximum nr of tolerances we can store is equal to the
    //number of joints this controller is configured with, so we must have at
    //least that number of elements in the output array
    if (posTolerances_len < joint_names->size)
    {
        Ros_Debug_BroadcastMsg("%s: pos tolerance output array too small: %d < %d (nr of joints)",
            __func__, posTolerances_len, joint_names->size);
        return -2;
    }

    //this:
    // - loops over all JointTolerance instances
    // - checks which joint they reference (based on the joint name)
    // - looks up joint index of that joint in the joint_names array
    // - stores the position tolerance value in the output array at the correct index for that joint
    for (size_t jtol_idx = 0; jtol_idx < goal_joint_tolerances->size; jtol_idx +=1)
    {
        rosidl_runtime_c__String selected_tolerance_name = goal_joint_tolerances->data[jtol_idx].name;
        Ros_Debug_BroadcastMsg("%s: parsing JointTolerance for '%s': pos: %f", __func__,
            selected_tolerance_name.data, goal_joint_tolerances->data[jtol_idx].position);

        size_t ptol_idx = 0;
        for (; ptol_idx < joint_names->size; ptol_idx += 1)
        {
            rosidl_runtime_c__String selected_position_name = joint_names->data[ptol_idx];
            if (rosidl_runtime_c__String__are_equal(&selected_position_name, &selected_tolerance_name))
            {
                Ros_Debug_BroadcastMsg("%s: mapping '%s' (at %d) to internal index %d",
                    __func__, selected_position_name.data, jtol_idx, ptol_idx);
                posTolerances[ptol_idx] = goal_joint_tolerances->data[jtol_idx].position;
                break;
            }
        }

        //couldn't find joint
        if (ptol_idx == joint_names->size)
        {
            //TODO(gavanderhoorn): make this a fatal error?
            Ros_Debug_BroadcastMsg("%s: WARNING: couldn't find '%s' in internal joint names, ignoring",
                __func__, selected_tolerance_name.data);
        }
    }

    return OK;
}

/**
 * Note: this assumes neither 'traj_point_names' nor 'internal_jnames' contain
 * duplicate joint names. This function does not check whether this is true.
 */
static STATUS Ros_ActionServer_FJT_Reorder_TrajPt_To_Internal_Order(
    trajectory_msgs__msg__JointTrajectoryPoint const* const traj_point /* in */,
    rosidl_runtime_c__String__Sequence const* const traj_point_jnames /* in */,
    rosidl_runtime_c__String__Sequence const* const internal_jnames /* in */,
    double* trajPtValues /* in/out */, size_t trajPtValues_len /* in */)
{
    if (traj_point == NULL || traj_point_jnames == NULL || internal_jnames == NULL || trajPtValues == NULL)
    {
        Ros_Debug_BroadcastMsg("%s: one or more inputs NULL, abort", __func__);
        return -1;
    }

    if (traj_point_jnames->size != internal_jnames->size)
    {
        Ros_Debug_BroadcastMsg("%s: partial traj pt not supported (%d names, need: %d)",
            __func__, traj_point_jnames->size, internal_jnames->size);
        return -2;
    }

    if (traj_point->positions.size != internal_jnames->size)
    {
        Ros_Debug_BroadcastMsg("%s: partial traj pt not supported (%d pos, need: %d)",
            __func__, traj_point->positions.size, internal_jnames->size);
        return -3;
    }

    if (trajPtValues_len < traj_point->positions.size)
    {
        Ros_Debug_BroadcastMsg("%s: output array too small: %d < %d (nr of joints)",
            __func__, trajPtValues_len, traj_point->positions.size);
        return -4;
    }

    //this:
    // - loops over all joint names in the trajectory point
    // - retrieves the position data for each joint in the traj point
    // - determines at which index the joint name occurs in the internal joint name ordering
    // - stores the retrieved position data at the identified internal index
    for (size_t traj_jname_idx = 0; traj_jname_idx < traj_point_jnames->size; traj_jname_idx += 1)
    {
        rosidl_runtime_c__String traj_jname = traj_point_jnames->data[traj_jname_idx];
        for (size_t internal_jname_idx = 0; internal_jname_idx < internal_jnames->size; internal_jname_idx += 1)
        {
            rosidl_runtime_c__String internal_jname = internal_jnames->data[internal_jname_idx];
            if (rosidl_runtime_c__String__are_equal(&internal_jname, &traj_jname))
            {
                trajPtValues[internal_jname_idx] = traj_point->positions.data[traj_jname_idx];
                Ros_Debug_BroadcastMsg("%s: mapping ('%s'; %f) at %d to internal index %d",
                    __func__, traj_jname.data, trajPtValues[internal_jname_idx], traj_jname_idx, internal_jname_idx);
                break;
            }
        }
    }

    return OK;
}

void Ros_ActionServer_FJT_Goal_Complete(GOAL_END_TYPE goal_end_type)
{
    //**********************************************************************
    if (goal_end_type == GOAL_COMPLETE)
    {
        double diff;
        INT64 timeTolerance;
        INT64 trajectory_end_time_ns = rmw_uros_epoch_nanos();

        //-----------------------------------------------------------------------
        //check to see if each axis is in the desired location
        BOOL positionOk = TRUE;
#define MR2_JTA_MAX_NUM_AXES (MAX_CONTROLLABLE_GROUPS * MP_GRP_AXES_NUM)
        int numAxesToCheck = feedback_FollowJointTrajectory.feedback.joint_names.size;

        //NOTE: we allocate for the maximum nr of axes, but only check the nr of configured
        //axes later (which is in almost all cases significantly smaller)
        double violators[MR2_JTA_MAX_NUM_AXES];
        double posTolerance[MR2_JTA_MAX_NUM_AXES];
        bzero(violators, sizeof(violators));
        bzero(posTolerance, sizeof(posTolerance));

        //'parse' the JointTolerance elements from the goal. Map their 'name:tolerance'
        //to the 'joint_index:tolerance' we need
        STATUS statusParseGoalTolerance = Ros_ActionServer_FJT_Parse_GoalPosTolerances(
            &g_actionServer_FJT_SendGoal_Request.goal.goal_tolerance,
            &feedback_FollowJointTrajectory.feedback.joint_names,
            posTolerance, numAxesToCheck);

        BOOL bToleranceParseOk = TRUE;
        if (statusParseGoalTolerance != OK)
        {
            Ros_Debug_BroadcastMsg("%s: parsing 'goal_tolerance' field failed: %d", __func__, statusParseGoalTolerance);
            //TODO(gavanderhoorn): implement tolerance parsing in Ros_ActionServer_FJT_Goal_Received(..) instead
            bToleranceParseOk = FALSE;
            Ros_Debug_BroadcastMsg("%s: skipping goal tolerance check", __func__);
            goto goal_complete_skip_tolerance_comparison;
        }

        //retrieve the last traj pt from the goal traj and re-order position values such
        //that they correspond to the internal MotoROS2 ordering (as used in
        //feedback_FollowJointTrajectory.feedback)
        double lastTrajPtPositions[MR2_JTA_MAX_NUM_AXES];
        bzero(lastTrajPtPositions, sizeof(lastTrajPtPositions));
        size_t finalTrajPtIdx = g_actionServer_FJT_SendGoal_Request.goal.trajectory.points.size - 1;

        STATUS statusGoalToleranceReorder = Ros_ActionServer_FJT_Reorder_TrajPt_To_Internal_Order(
            &g_actionServer_FJT_SendGoal_Request.goal.trajectory.points.data[finalTrajPtIdx],
            &g_actionServer_FJT_SendGoal_Request.goal.trajectory.joint_names,
            &feedback_FollowJointTrajectory.feedback.joint_names,
            lastTrajPtPositions,
            numAxesToCheck);

        if (statusGoalToleranceReorder != OK)
        {
            Ros_Debug_BroadcastMsg("%s: reordering final traj pt failed: %d", __func__, statusGoalToleranceReorder);
            //TODO(gavanderhoorn): check re-ordering in Ros_ActionServer_FJT_Goal_Received(..) instead
            bToleranceParseOk = FALSE;
            Ros_Debug_BroadcastMsg("%s: skipping goal tolerance check", __func__);
            goto goal_complete_skip_tolerance_comparison;
        }


        //compare current state with target positions from last traj point
        for (int axis = 0; axis < numAxesToCheck; axis += 1)
        {
            double current_jstate = feedback_FollowJointTrajectory.feedback.actual.positions.data[axis];
            diff = fabs(lastTrajPtPositions[axis] - current_jstate);
            Ros_Debug_BroadcastMsg("target pos (%12.8f) - actual pos (%12.8f) = (%12.8f)",
                lastTrajPtPositions[axis], current_jstate, diff);

            double chosen_posTolerance = posTolerance[axis];
            if (chosen_posTolerance == 0.0) //user did NOT provide a tolerance
                chosen_posTolerance = DEFAULT_FJT_GOAL_POSITION_TOLERANCE;

            // A negative tolerance means that tolerance doesn't matter for this axis
            if ((chosen_posTolerance > 0.0) && (diff > chosen_posTolerance))
            {
                positionOk = FALSE;
                //record the deviation for use in the feedback message below
                violators[axis] = diff;
            }
        }

goal_complete_skip_tolerance_comparison: ;
        //-----------------------------------------------------------------------
        //check execution time
        control_msgs__action__FollowJointTrajectory_SendGoal_Request*  ros_goal_request = (control_msgs__action__FollowJointTrajectory_SendGoal_Request*)fjt_active_goal_handle->ros_goal_request;
        trajectory_msgs__msg__JointTrajectoryPoint__Sequence* points = &ros_goal_request->goal.trajectory.points;
        INT64 desiredTime = Ros_Duration_Msg_To_Nanos(&points->data[points->size - 1].time_from_start); //the desired time of the last point in the trajectory

        INT64 totalTime = (trajectory_end_time_ns - fjt_trajectory_start_time_ns);

        diff = abs(desiredTime - totalTime);
        timeTolerance = Ros_Duration_Msg_To_Nanos(&g_actionServer_FJT_SendGoal_Request.goal.goal_time_tolerance);
        if (timeTolerance == 0) //user did NOT provide a tolerance
        {
            timeTolerance = DEFAULT_FJT_GOAL_TIME_TOLERANCE;
            Ros_Debug_BroadcastMsg("FJT using DEFAULT goal time tolerance: %lld ns", timeTolerance);
        }
        else
        {
            Ros_Debug_BroadcastMsg("FJT using goal time tolerance: %lld ns", timeTolerance);
        }

        BOOL timeOk = (diff <= timeTolerance);

        //-----------------------------------------------------------------------
        if (bToleranceParseOk && positionOk && timeOk)
        {
            Ros_Debug_BroadcastMsg("FJT action successful");
            fjt_result_response.status = GOAL_STATE_SUCCEEDED;
            fjt_goal_state = GOAL_STATE_SUCCEEDED;

            rosidl_runtime_c__String__assign(&fjt_result_response.result.error_string, "Trajectory completed successfully!");

            fjt_result_response.result.error_code = control_msgs__action__FollowJointTrajectory_Result__SUCCESSFUL;
        }
        else
        {
            Ros_Debug_BroadcastMsg("FJT action failed");
            fjt_result_response.status = GOAL_STATE_ABORTED;
            fjt_goal_state = GOAL_STATE_ABORTED;

            char msgBuffer[500] = { 0 };

            if (!bToleranceParseOk)
            {
                sprintf(msgBuffer, "Parsing goal_tolerance failed (%d; %d). Failing trajectory execution.", statusParseGoalTolerance, statusGoalToleranceReorder);
                rosidl_runtime_c__String__assign(&fjt_result_response.result.error_string, msgBuffer);
                fjt_result_response.result.error_code = RESULT_REPONSE_ERROR_CODE(control_msgs__action__FollowJointTrajectory_Result__GOAL_TOLERANCE_VIOLATED, FAIL_TRAJ_TOLERANCE_PARSE);
            }
            else if (!positionOk)
            {
                sprintf(msgBuffer, "Final position was outside tolerance. Check robot safety-limits that could be inhibiting motion.");
                for (int axis = 0; axis < numAxesToCheck; axis += 1)
                {
                    //append info on which joints were outside tolerance
                    if (violators[axis])
                    {
                        char formatBuffer[64] = { 0 };
                        snprintf(formatBuffer, 64, " [%s: %d deviation]",
                            feedback_FollowJointTrajectory.feedback.joint_names.data[axis].data,
                            violators[axis]);
                        strcat(msgBuffer, formatBuffer);
                    }
                }
                rosidl_runtime_c__String__assign(&fjt_result_response.result.error_string, msgBuffer);
                fjt_result_response.result.error_code = RESULT_REPONSE_ERROR_CODE(control_msgs__action__FollowJointTrajectory_Result__GOAL_TOLERANCE_VIOLATED, FAIL_TRAJ_POSITION);
            }
            else if (!timeOk)
            {
                builtin_interfaces__msg__Duration durationDesired = points->data[points->size - 1].time_from_start;
                builtin_interfaces__msg__Duration durationActual;
                Ros_Nanos_To_Duration_Msg(totalTime, &durationActual);

                sprintf(msgBuffer,
                    "Execution time was outside tolerance. Target time was %d.%u seconds. Actual time was %d.%u seconds.",
                    durationDesired.sec, durationDesired.nanosec,
                    durationActual.sec, durationActual.nanosec);
                rosidl_runtime_c__String__assign(&fjt_result_response.result.error_string, msgBuffer);
                fjt_result_response.result.error_code = RESULT_REPONSE_ERROR_CODE(control_msgs__action__FollowJointTrajectory_Result__GOAL_TOLERANCE_VIOLATED, FAIL_TRAJ_TIME);
            }
        }

        Ros_Debug_BroadcastMsg(fjt_result_response.result.error_string.data);

        Ros_ActionServer_FJT_DeleteFeedbackMessage();
    }

    //**********************************************************************
    else if (goal_end_type == GOAL_CANCEL)
    {
        fjt_result_response.status = GOAL_STATE_CANCELED;
        fjt_goal_state = GOAL_STATE_CANCELED;

        rosidl_runtime_c__String__assign(&fjt_result_response.result.error_string, "Goal was cancelled by the user.");

        fjt_result_response.result.error_code = RESULT_REPONSE_ERROR_CODE(control_msgs__action__FollowJointTrajectory_Result__GOAL_TOLERANCE_VIOLATED, FAIL_TRAJ_CANCEL);

        Ros_ActionServer_FJT_DeleteFeedbackMessage();
    }

    //**********************************************************************
    else if (goal_end_type == GOAL_ABORT_DUE_TO_ERROR)
    {
        fjt_result_response.status = GOAL_STATE_ABORTED;
        fjt_goal_state = GOAL_STATE_ABORTED;

        rosidl_runtime_c__String__assign(&fjt_result_response.result.error_string,
          "Goal was aborted due to an alarm or error (check RobotStatus messages).");

        fjt_result_response.result.error_code = RESULT_REPONSE_ERROR_CODE(control_msgs__action__FollowJointTrajectory_Result__INVALID_GOAL, FAIL_TRAJ_ALARM);

        Ros_ActionServer_FJT_DeleteFeedbackMessage();
    }

    fjt_result_message_ready = TRUE;

    //----------------------------------------------------
    Ros_Debug_BroadcastMsg("FJT action complete");
}

bool Ros_ActionServer_FJT_Goal_Cancel(rclc_action_goal_handle_t* goal_handle, void* context)
{
    (void)context;
    (void)goal_handle;

    Ros_Debug_BroadcastMsg("Goal Canceled");

    Ros_MotionControl_StopMotion(/*bKeepJobRunning = */ TRUE);

    Ros_ActionServer_FJT_Goal_Complete(GOAL_CANCEL);

    return true;
}

void Ros_ActionServer_FJT_ProcessResult()
{
    rcl_ret_t rc;

    if (fjt_active_goal_handle && fjt_result_message_ready)
    {
        rc = rclc_action_send_result(fjt_active_goal_handle, fjt_goal_state, &fjt_result_response);
        if (rc == RCL_RET_OK)
        {
            micro_ros_string_utilities_destroy(&fjt_result_response.result.error_string);
            fjt_active_goal_handle = NULL;
            fjt_result_message_ready = FALSE;
        }

    }
    else if (fjt_rejected_goal_handle && fjt_result_message_ready)
    {
        rc = rclc_action_send_result(fjt_rejected_goal_handle, GOAL_STATE_ABORTED, &fjt_result_response);
        if (rc == RCL_RET_OK)
        {
            micro_ros_string_utilities_destroy(&fjt_result_response.result.error_string);
            fjt_rejected_goal_handle = NULL;
            fjt_result_message_ready = FALSE;
        }
    }
}


//included here as this tests 'static' functions
#define MOTOROS2_INCLUDE_TESTS_FJT_C
#include "Tests_ActionServer_FJT.c"
#undef MOTOROS2_INCLUDE_TESTS_FJT_C
