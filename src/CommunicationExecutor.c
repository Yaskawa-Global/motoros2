//Communication.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

Communication_NodeInfo g_microRosNodeInfo;
BOOL g_Ros_Communication_AgentIsConnected;
rmw_init_options_t* rmw_connectionoptions;

void Ros_Communication_ConnectToAgent()
{
    rcl_ret_t ret = RCL_RET_ERROR;
    g_Ros_Communication_AgentIsConnected = FALSE;

    // log hard-coded RMW settings
    Ros_Debug_BroadcastMsg("RMW settings 1: "
        "%d; %d; %d; %d; %d; %d",
        RMW_UXRCE_ENTITY_CREATION_TIMEOUT,
        RMW_UXRCE_ENTITY_DESTROY_TIMEOUT,
        RMW_UXRCE_PUBLISH_RELIABLE_TIMEOUT,
        RMW_UXRCE_STREAM_HISTORY_INPUT,
        RMW_UXRCE_STREAM_HISTORY_OUTPUT,
        RMW_UXRCE_MAX_TRANSPORT_MTU
    );
    Ros_Debug_BroadcastMsg("RMW settings 2: "
        "%d; %d; %d; %d; %d; %d; %d",
        RMW_UXRCE_MAX_SESSIONS,
        RMW_UXRCE_MAX_NODES,
        RMW_UXRCE_MAX_PUBLISHERS,
        RMW_UXRCE_MAX_SUBSCRIPTIONS,
        RMW_UXRCE_MAX_SERVICES,
        RMW_UXRCE_MAX_CLIENTS,
        RMW_UXRCE_MAX_TOPICS
    );
    Ros_Debug_BroadcastMsg("RMW settings 3: "
        "%d; %d; %d; %d",
        RMW_UXRCE_NODE_NAME_MAX_LENGTH,
        RMW_UXRCE_TOPIC_NAME_MAX_LENGTH,
        RMW_UXRCE_TYPE_NAME_MAX_LENGTH,
        RMW_UXRCE_ENTITY_NAMING_BUFFER_LENGTH
    );

    //For good measure
    bzero(&g_microRosNodeInfo, sizeof(g_microRosNodeInfo));

    g_microRosNodeInfo.initOptions = rcl_get_zero_initialized_init_options();

    ret = rcl_init_options_init(&g_microRosNodeInfo.initOptions, g_motoros2_Allocator);
    motoRosAssert(ret == RCL_RET_OK, SUBCODE_FAIL_OPTIONS_INIT);

    Ros_Debug_BroadcastMsg("Using ROS domain ID: %d", g_nodeConfigSettings.ros_domain_id);
    ret = rcl_init_options_set_domain_id(&g_microRosNodeInfo.initOptions, g_nodeConfigSettings.ros_domain_id);
    motoRosAssert(ret == RCL_RET_OK, SUBCODE_FAIL_OPTIONS_INIT_DOMAIN_ID);

    rmw_connectionoptions = rcl_init_options_get_rmw_init_options(&g_microRosNodeInfo.initOptions);
    motoRosAssert(rmw_connectionoptions != NULL, SUBCODE_FAIL_RMW_OPTIONS_INIT);

    //Construct the RMW Micro-XRCE-DDS client key to use
    //Use MAC last 4 bytes as client key (1 byte from the OUI, 3 bytes from NIC)
    UCHAR macId[6];
    motoRosAssert_withMsg(
        Ros_GetMacAddress(macId) == OK,
        SUBCODE_FAIL_MP_NICDATA,
        "Must enable ETHERNET function");
    uint32_t client_key = *((uint32_t*) &macId[2]);
    //Swap to make NIC bytes LSB
    client_key = mpNtohl(client_key);
    Ros_Debug_BroadcastMsg("Using client key: 0x%08X", client_key);
    rmw_uros_options_set_client_key(client_key, rmw_connectionoptions);

    Ros_Debug_BroadcastMsg("Attempting to connect to Micro-ROS PC Agent (at udp://%s:%s)",
        g_nodeConfigSettings.agent_ip_address, g_nodeConfigSettings.agent_port_number);
    //Also print to console, for easier debugging (but only if not logging to stdout already)
    if (!g_nodeConfigSettings.log_to_stdout)
    {
        Ros_Debug_LogToConsole("Attempting to connect to Micro-ROS PC Agent (at udp://%s:%s)",
            g_nodeConfigSettings.agent_ip_address, g_nodeConfigSettings.agent_port_number);
    }

    //Wait for agent to become available
    ret = RCL_RET_ERROR;
    int retry_count = 0;
    do
    {
        Ros_Sleep(1000);

        //Broadcast msg roughly every 20 seconds (as a poor-mans indication of liveness)
        //Note: 20 seconds, as we sleep 1 sec *and* rmw_uros_ping_agent_options(..) uses
        //      a 1 second timeout below
        retry_count += 1;
        if (retry_count == 10)
        {
            Ros_Debug_BroadcastMsg("Attempting to connect to Micro-ROS PC Agent (at udp://%s:%s)",
                g_nodeConfigSettings.agent_ip_address, g_nodeConfigSettings.agent_port_number);
            retry_count = 0;
        }

        //If the user has explicitly provided an address/port for the agent, then use it.
        rmw_uros_options_set_udp_address((const char*)g_nodeConfigSettings.agent_ip_address, (const char*)g_nodeConfigSettings.agent_port_number, rmw_connectionoptions);

        ret = rmw_uros_ping_agent_options(1000, 1, rmw_connectionoptions);

    } while (ret != RCL_RET_OK);

    g_Ros_Communication_AgentIsConnected = TRUE;
    Ros_Debug_BroadcastMsg("Found Micro-ROS PC Agent");
}

void Ros_Communication_Initialize()
{
    //==================================
    //create node
    MOTOROS2_MEM_TRACE_START(comm_exec_init);

    rcl_ret_t ret = rclc_support_init_with_options(&g_microRosNodeInfo.support,
        0, NULL, &g_microRosNodeInfo.initOptions, &g_motoros2_Allocator);
    Ros_Debug_BroadcastMsg("rclc_support_init_with_options = %d", (int)ret);
    motoRosAssert(ret == RCL_RET_OK, SUBCODE_FAIL_SUPPORT_INIT);

    //construct a faux command line to pass to rcl_parse_arguments(..)
    char* faux_argv[MAX_REMAP_RULE_NUM];
    size_t faux_argc = 0;

    //create copy of remap rule cfg item as Ros_ConstructFauxArgv(..) will
    //clobber it
    char remap_rules_str_[MAX_REMAP_RULE_LEN];
    strncpy(remap_rules_str_, g_nodeConfigSettings.remap_rules, MAX_REMAP_RULE_LEN);
    Ros_Debug_BroadcastMsg("remap_rules str: '%s'", remap_rules_str_);
    Ros_Debug_BroadcastMsg("len(remap_rules str): %d", strlen(remap_rules_str_));

    bzero(faux_argv, MAX_REMAP_RULE_NUM);
    faux_argc = Ros_ConstructFauxArgv(
        remap_rules_str_, faux_argv, MAX_REMAP_RULE_NUM);

    rcl_node_options_t node_options = rcl_node_get_default_options();
    if (faux_argc > 0)
    {
        //parse the just constructed argv
        ret = rcl_parse_arguments(
            faux_argc, (char const* const*)faux_argv, g_motoros2_Allocator,
            &node_options.arguments);
        Ros_Debug_BroadcastMsg("rcl_parse_arguments = %d", (int)ret);
        if (ret != RCL_RET_OK)
        {
            mpSetAlarm(ALARM_CONFIGURATION_FAIL, "Invalid remap rule format", SUBCODE_CONFIGURATION_FAIL_NODE_INIT_ARG_PARSE);
            node_options = rcl_node_get_default_options();
        }
    }

    //init node with the remap rules and other options from the config file
    ret = rclc_node_init_with_options(
        &g_microRosNodeInfo.node, g_nodeConfigSettings.node_name,
        g_nodeConfigSettings.node_namespace, &g_microRosNodeInfo.support,
        &node_options);
    Ros_Debug_BroadcastMsg("rclc_node_init_with_options = %d", (int)ret);
    motoRosAssert(ret == RCL_RET_OK, SUBCODE_FAIL_NODE_INIT);

    //we're done with it
    ret = rcl_node_options_fini(&node_options); RCL_UNUSED(ret);
    Ros_CleanupFauxArgv(faux_argv, faux_argc);

    MOTOROS2_MEM_TRACE_REPORT(comm_exec_init);
}

void Ros_Communication_Cleanup()
{
    MOTOROS2_MEM_TRACE_START(comm_exec_fini);
    rcl_ret_t ret;

    Ros_Debug_BroadcastMsg("Cleanup node");
    ret = rcl_node_fini(&g_microRosNodeInfo.node);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up node: %d", ret);

    Ros_Debug_BroadcastMsg("Cleanup init options");
    ret = rcl_init_options_fini(&g_microRosNodeInfo.initOptions);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up init options: %d", ret);

    Ros_Debug_BroadcastMsg("Cleanup support");
    ret = rclc_support_fini(&g_microRosNodeInfo.support);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up support: %d", ret);

    MOTOROS2_MEM_TRACE_REPORT(comm_exec_fini);
}

void Ros_Communication_PingAgentConnection(rcl_timer_t* timer, int64_t last_call_time)
{
    //Ensure the Agent is still connected.
    const int timeout_ms = 1000;
    const uint8_t attempts = 3;
    rcl_ret_t ret = rmw_uros_ping_agent_options(timeout_ms, attempts, rmw_connectionoptions);
    g_Ros_Communication_AgentIsConnected = (ret == RCL_RET_OK);

    if (g_Ros_Communication_AgentIsConnected && g_nodeConfigSettings.sync_timeclock_with_agent)
        rmw_uros_sync_session(100);
}

void Ros_Communication_PublishActionFeedback(rcl_timer_t* timer, int64_t last_call_time)
{
    Ros_ActionServer_FJT_ProcessFeedback();
    Ros_ActionServer_FJT_ProcessResult();
}

void Ros_Communication_RunIoExecutor(rclc_executor_t* executor, SEM_ID semIoExecutorStatus)
{
    mpSemTake(semIoExecutorStatus, NO_WAIT);

    while (g_Ros_Communication_AgentIsConnected)
    {
        Ros_Sleep(g_nodeConfigSettings.executor_sleep_period);

        // timeout specified in nanoseconds
        rclc_executor_spin_some(executor, RCL_MS_TO_NS(1));
    }
    Ros_Debug_BroadcastMsg("Terminating I/O Executor Task");

    //notify parent task that this has finished
    mpSemGive(semIoExecutorStatus);

    mpDeleteSelf;
}

void Ros_Communication_StartExecutors(SEM_ID semCommunicationExecutorStatus)
{
    rcl_ret_t rc;

    rcl_timer_t timerPingAgent = rcl_get_zero_initialized_timer();
    rcl_timer_t timerPublishActionFeedback = rcl_get_zero_initialized_timer();

    mpSemTake(semCommunicationExecutorStatus, NO_WAIT);

    //---------------------------------
    //Create timers
    rc = rclc_timer_init_default(&timerPingAgent, &g_microRosNodeInfo.support, RCL_MS_TO_NS(PERIOD_COMMUNICATION_PING_AGENT_MS), Ros_Communication_PingAgentConnection);
    motoRosAssert_withMsg(rc == RCL_RET_OK, SUBCODE_FAIL_TIMER_INIT_PING, "Failed creating rclc timer (%d)", (int)rc);

    rc = rclc_timer_init_default(&timerPublishActionFeedback, &g_microRosNodeInfo.support, RCL_MS_TO_NS(g_nodeConfigSettings.action_feedback_publisher_period), Ros_Communication_PublishActionFeedback);
    motoRosAssert_withMsg(rc == RCL_RET_OK, SUBCODE_FAIL_TIMER_INIT_ACTION_FB, "Failed creating rclc timer (%d)", (int)rc);

    //---------------------------------
    //Create executors
    rclc_executor_t executor_motion_control;
    executor_motion_control = rclc_executor_get_zero_initialized_executor();

    rc = rclc_executor_init(&executor_motion_control, &g_microRosNodeInfo.support.context, QUANTITY_OF_HANDLES_FOR_MOTION_EXECUTOR, &g_motoros2_Allocator);
    motoRosAssert_withMsg(rc == RCL_RET_OK, SUBCODE_FAIL_CREATE_MOTION_EXECUTOR, "Failed creating motion control executor (%d)", (int)rc);

    rclc_executor_t executor_io_control;
    executor_io_control = rclc_executor_get_zero_initialized_executor();

    rc = rclc_executor_init(&executor_io_control, &g_microRosNodeInfo.support.context, QUANTITY_OF_HANDLES_FOR_IO_EXECUTOR, &g_motoros2_Allocator);
    motoRosAssert_withMsg(rc == RCL_RET_OK, SUBCODE_FAIL_CREATE_IO_EXECUTOR, "Failed creating I/O control executor (%d)", (int)rc);

    //==========================================================
    //Add entities to motion executor
    //
    // WARNING: Be sure to update QUANTITY_OF_HANDLES_FOR_MOTION_EXECUTOR
    //
    rc = rclc_executor_add_timer(&executor_motion_control, &timerPingAgent);
    motoRosAssert_withMsg(rc == RCL_RET_OK, SUBCODE_FAIL_TIMER_ADD_PING, "Failed adding timer (%d)", (int)rc);

    rc = rclc_executor_add_timer(&executor_motion_control, &timerPublishActionFeedback);
    motoRosAssert_withMsg(rc == RCL_RET_OK, SUBCODE_FAIL_TIMER_ADD_ACTION_FB, "Failed adding timer (%d)", (int)rc);

    rc = rclc_executor_add_action_server(&executor_motion_control,
        &g_actionServerFollowJointTrajectory,
        1,
        &g_actionServer_FJT_SendGoal_Request,
        g_actionServer_FJT_SendGoal_Request__sizeof,
        Ros_ActionServer_FJT_Goal_Received,
        Ros_ActionServer_FJT_Goal_Cancel,
        &g_actionServerFollowJointTrajectory);
    motoRosAssert_withMsg(rc == RCL_RET_OK, SUBCODE_FAIL_ADD_FJT_SERVER, "Failed adding FJT server (%d)", (int)rc);

    rc = rclc_executor_add_service(
        &executor_motion_control, &g_serviceStopTrajMode, &g_messages_StopTrajMode.request,
        &g_messages_StopTrajMode.response, Ros_ServiceStopTrajMode_Trigger);
    motoRosAssert_withMsg(rc == RCL_RET_OK, SUBCODE_FAIL_ADD_SERVICE_STOP_TRAJ, "Failed adding service (%d)", (int)rc);

    rc = rclc_executor_add_service(
        &executor_motion_control, &g_serviceResetError, &g_messages_ResetError.request,
        &g_messages_ResetError.response, Ros_ServiceResetError_Trigger);
    motoRosAssert_withMsg(rc == RCL_RET_OK, SUBCODE_FAIL_ADD_SERVICE_RESET_ERROR, "Failed adding service (%d)", (int)rc);

    rc = rclc_executor_add_service(
        &executor_motion_control, &g_serviceStartTrajMode, &g_messages_StartTrajMode.request,
        &g_messages_StartTrajMode.response, Ros_ServiceStartTrajMode_Trigger);
    motoRosAssert_withMsg(rc == RCL_RET_OK, SUBCODE_FAIL_ADD_SERVICE_START_TRAJ_MODE, "Failed adding service (%d)", (int)rc);

    rc = rclc_executor_add_service(
        &executor_motion_control, &g_serviceStartPointQueueMode, &g_messages_StartPointQueueMode.request,
        &g_messages_StartPointQueueMode.response, Ros_ServiceStartPointQueueMode_Trigger);
    motoRosAssert_withMsg(rc == RCL_RET_OK, SUBCODE_FAIL_ADD_SERVICE_START_QUEUE_MODE, "Failed adding service (%d)", (int)rc);

    rc = rclc_executor_add_service(
        &executor_motion_control, &g_serviceQueueTrajPoint, g_messages_QueueTrajPoint.request,
        g_messages_QueueTrajPoint.response, Ros_ServiceQueueTrajPoint_Trigger);
    motoRosAssert_withMsg(rc == RCL_RET_OK, SUBCODE_FAIL_ADD_SERVICE_QUEUE_POINT, "Failed adding service (%d)", (int)rc);

    rc = rclc_executor_add_service(
        &executor_motion_control, &g_serviceSelectMotionTool, &g_messages_SelectMotionTool.request,
        &g_messages_SelectMotionTool.response, Ros_ServiceSelectMotionTool_Trigger);
    motoRosAssert_withMsg(rc == RCL_RET_OK, SUBCODE_FAIL_ADD_SERVICE_SELECT_MOTION_TOOL, "Failed adding service (%d)", (int)rc);

    //==========================================================
    //Add entities to I/O executor
    //
    // WARNING: Be sure to update QUANTITY_OF_HANDLES_FOR_IO_EXECUTOR
    //
    // 
    rc = rclc_executor_add_service(
        &executor_io_control, &g_serviceReadSingleIO, &g_messages_ReadWriteIO.req_single_io_read,
        &g_messages_ReadWriteIO.resp_single_io_read, Ros_ServiceReadSingleIO_Trigger);
    motoRosAssert_withMsg(rc == RCL_RET_OK, SUBCODE_FAIL_ADD_SERVICE_READ_SINGLE_IO, "Failed adding service (%d)", (int)rc);

    rc = rclc_executor_add_service(
        &executor_io_control, &g_serviceReadGroupIO, &g_messages_ReadWriteIO.req_group_io_read,
        &g_messages_ReadWriteIO.resp_group_io_read, Ros_ServiceReadGroupIO_Trigger);
    motoRosAssert_withMsg(rc == RCL_RET_OK, SUBCODE_FAIL_ADD_SERVICE_READ_GROUP_IO, "Failed adding service (%d)", (int)rc);

    rc = rclc_executor_add_service(
        &executor_io_control, &g_serviceWriteSingleIO, &g_messages_ReadWriteIO.req_single_io_write,
        &g_messages_ReadWriteIO.resp_single_io_write, Ros_ServiceWriteSingleIO_Trigger);
    motoRosAssert_withMsg(rc == RCL_RET_OK, SUBCODE_FAIL_ADD_SERVICE_WRITE_SINGLE_IO, "Failed adding service (%d)", (int)rc);

    rc = rclc_executor_add_service(
        &executor_io_control, &g_serviceWriteGroupIO, &g_messages_ReadWriteIO.req_group_io_write,
        &g_messages_ReadWriteIO.resp_group_io_write, Ros_ServiceWriteGroupIO_Trigger);
    motoRosAssert_withMsg(rc == RCL_RET_OK, SUBCODE_FAIL_ADD_SERVICE_WRITE_GROUP_IO, "Failed adding service (%d)", (int)rc);

    rc = rclc_executor_add_service(
        &executor_io_control, &g_serviceReadMRegister, &g_messages_ReadWriteIO.req_mreg_read,
        &g_messages_ReadWriteIO.resp_mreg_read, Ros_ServiceReadMRegister_Trigger);
    motoRosAssert_withMsg(rc == RCL_RET_OK, SUBCODE_FAIL_ADD_SERVICE_READ_M_REG, "Failed adding service (%d)", (int)rc);

    rc = rclc_executor_add_service(
        &executor_io_control, &g_serviceWriteMRegister, &g_messages_ReadWriteIO.req_mreg_write,
        &g_messages_ReadWriteIO.resp_mreg_write, Ros_ServiceWriteMRegister_Trigger);
    motoRosAssert_withMsg(rc == RCL_RET_OK, SUBCODE_FAIL_ADD_SERVICE_WRITE_M_REG, "Failed adding service (%d)", (int)rc);

    //===========================================================

    // Optional prepare for avoiding allocations during spin
    rclc_executor_prepare(&executor_motion_control);
    rclc_executor_prepare(&executor_io_control);

    //===========================================================
    //===========================================================
    //===========================================================
    
    // Start executor that runs the I/O executor
    // (This task deletes itself when the agent disconnects.)
    SEM_ID semIoExecutorStatus = mpSemBCreate(SEM_Q_FIFO, SEM_FULL);
    mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE,
        (FUNCPTR)Ros_Communication_RunIoExecutor,
        (int)&executor_io_control, (int)semIoExecutorStatus, 0, 0, 0, 0, 0, 0, 0, 0);

    while (g_Ros_Communication_AgentIsConnected)
    {
        Ros_Sleep(g_nodeConfigSettings.executor_sleep_period);

        // timeout specified in nanoseconds
        rclc_executor_spin_some(&executor_motion_control, RCL_MS_TO_NS(1));

        Ros_MotionControl_ValidateMotionModeIsOk();
    }
    
    //wait for Ros_Communication_RunIoExecutor task to finish before cleaning shared resources
    mpSemTake(semIoExecutorStatus, WAIT_FOREVER);
    mpSemDelete(semIoExecutorStatus);

    //===========================================================
    //===========================================================
    //===========================================================

    //stop robot motion if it's currently executing a trajectory and MotoROS2
    //has been configured to stop it
    if (Ros_Controller_IsInMotion())
    {
        if (Ros_MotionControl_IsRosControllingMotion() && g_nodeConfigSettings.stop_motion_on_disconnect)
        {
            Ros_Debug_BroadcastMsg("Agent disappeared. Stopping active ROS-controlled motion.");
            Ros_MotionControl_StopMotion(/*bKeepJobRunning = */ TRUE);
        }
        else
        {
            Ros_Debug_BroadcastMsg("Agent disappeared. No active ROS-controlled "
                "motion or not configured to stop it.");
            Ros_Debug_BroadcastMsg("Waiting for motion to stop before releasing memory");
            while (Ros_Controller_IsInMotion()) //wait for motion to complete before terminating tasks
            {
                Ros_Sleep(1000);
            }
        }
    }

    Ros_Debug_BroadcastMsg("Cleanup motion control executor");
    rclc_executor_fini(&executor_motion_control);

    Ros_Debug_BroadcastMsg("Cleanup I/O control executor");
    rclc_executor_fini(&executor_io_control);

    Ros_Debug_BroadcastMsg("Cleanup timer for action feedback");
    rc = rcl_timer_fini(&timerPublishActionFeedback);
    if (rc != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up action feedback timer: %d", rc);

    Ros_Debug_BroadcastMsg("Cleanup timer for ping");
    rc = rcl_timer_fini(&timerPingAgent);
    if (rc != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up ping timer: %d", rc);

    //notify main task that this has finished
    mpSemGive(semCommunicationExecutorStatus);

    mpDeleteSelf;
}
