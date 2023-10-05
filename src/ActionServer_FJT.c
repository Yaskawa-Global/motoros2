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

        //Give time for the controller to recognize that the INFORM cursor is sitting on
        //a WAIT instructions. Without this delay, the first call to mpExRcsIncrementMove
        //will fail with a (-1).
        Ros_Sleep(100);

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
            motomanErrorCode = Ros_Controller_GetNotReadySubcode();
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
        int maxAxes = MAX_CONTROLLABLE_GROUPS * MP_GRP_AXES_NUM;
        double violators[MAX_CONTROLLABLE_GROUPS * MP_GRP_AXES_NUM];

        bzero(violators, sizeof(violators));
        for (int axis = 0; axis < maxAxes; axis += 1)
        {
            diff = fabs(feedback_FollowJointTrajectory.feedback.desired.positions.data[axis] - feedback_FollowJointTrajectory.feedback.actual.positions.data[axis]);

            double posTolerance = g_actionServer_FJT_SendGoal_Request.goal.goal_tolerance.data[axis].position; //user-provided a goal_tolerance
            if (posTolerance == 0.0) //user did NOT provide a tolerance
                posTolerance = DEFAULT_FJT_GOAL_POSITION_TOLERANCE;

            if (diff > posTolerance)
            {
                positionOk = FALSE;
                //record the deviation for use in the feedback message below
                violators[axis] = diff;
            }
        }

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
        if (positionOk && timeOk)
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

            if (!positionOk)
            {
                sprintf(msgBuffer, "Final position was outside tolerance. Check robot safety-limits that could be inhibiting motion.");
                for (int axis = 0; axis < maxAxes; axis += 1)
                {
                    //append info on which joints were outside tolerance
                    if (violators[axis])
                    {
                        char formatBuffer[64] = { 0 };
                        snprintf(formatBuffer, 64, " [%s: %.4f deviation]",
                            feedback_FollowJointTrajectory.feedback.joint_names.data[axis],
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
