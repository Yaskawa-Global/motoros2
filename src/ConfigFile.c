// ConfigFile.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

/*
------------------------------------
Example parsing-event sequence
https://github.com/meffie/libyaml-examples (scan.c)
------------------------------------
stream-start-event (1)
 -document-start-event (3)
 - -mapping-start-event (9)
 - - -scalar-event (6) = {value="automatic_agent_discovery", length=25}
 - - -scalar-event (6) = {value="false", length=5}
 - - -scalar-event (6) = {value="agent_ip_address", length=16}
 - - -scalar-event (6) = {value="192.168.1.50", length=12}
 - - -scalar-event (6) = {value="agent_port_number", length=17}
 - - -scalar-event (6) = {value="8888", length=4}
 - - -scalar-event (6) = {value="sync_timeclock_with_agent", length=25}
 - - -scalar-event (6) = {value="true", length=4}
 - - -scalar-event (6) = {value="publish_tf", length=10}
 - - -scalar-event (6) = {value="true", length=4}
 - - -scalar-event (6) = {value="joint_names", length=11}
 - - -sequence-start-event (7)
 - - - -sequence-start-event (7)
 - - - - -scalar-event (6) = {value="group_1/joint_1", length=15}
 - - - - -scalar-event (6) = {value="group_1/joint_2", length=15}
 - - - - -scalar-event (6) = {value="group_1/joint_3", length=15}
 - - - - -scalar-event (6) = {value="group_1/joint_4", length=15}
 - - - - -scalar-event (6) = {value="group_1/joint_5", length=15}
 - - - - -scalar-event (6) = {value="group_1/joint_6", length=15}
 - - - -sequence-end-event (8)
 - - - -sequence-start-event (7)
 - - - - -scalar-event (6) = {value="group_2/joint_1", length=15}
 - - - - -scalar-event (6) = {value="group_2/joint_2", length=15}
 - - - - -scalar-event (6) = {value="group_2/joint_3", length=15}
 - - - - -scalar-event (6) = {value="group_2/joint_4", length=15}
 - - - - -scalar-event (6) = {value="group_2/joint_5", length=15}
 - - - - -scalar-event (6) = {value="group_2/joint_6", length=15}
 - - - -sequence-end-event (8)
 - - -sequence-end-event (8)
 - - -scalar-event (6) = {value="logging", length=7}
 - - -mapping-start-event (9)
 - - - -scalar-event (6) = {value="verbosity", length=9}
 - - - -scalar-event (6) = {value="1", length=1}
 - - - -scalar-event (6) = {value="log_to_stdout", length=13}
 - - - -scalar-event (6) = {value="false", length=5}
 - - -mapping-end-event (10)
 - - -scalar-event (6) = {value="clock_periods", length=11}
 - - -mapping-start-event (9)
 - - - -scalar-event (6) = {value="executor_sleep_period", length=19}
 - - - -scalar-event (6) = {value="10", length=2}
 - - - -scalar-event (6) = {value="action_feedback_publisher_period", length=32}
 - - - -scalar-event (6) = {value="20", length=2}
 - - - -scalar-event (6) = {value="controller_status_monitor_period", length=33}
 - - - -scalar-event (6) = {value="10", length=2}
 - - -mapping-end-event (10)
 - - -scalar-event (6) = {value="publisher_qos", length=13}
 - - -mapping-start-event (9)
 - - - -scalar-event (6) = {value="robot_status", length=12}
 - - - -scalar-event (6) = {value="best_effort", length=11}
 - - - -scalar-event (6) = {value="joint_states", length=12}
 - - - -scalar-event (6) = {value="best_effort", length=11}
 - - - -scalar-event (6) = {value="tf", length=2}
 - - - -scalar-event (6) = {value="reliable", length=8}
 - - -mapping-end-event (10)
 - -mapping-end-event (10)
 -document-end-event (4)
stream-end-event (2)
*/

#include "MotoROS.h"

Ros_Configuration_Settings g_nodeConfigSettings;

char* joint_names_iterator;

typedef enum
{
    Value_String,
    Value_Int,
    Value_Bool,
    Value_JointNameArray,
    Value_Qos
} Value_Type;

typedef struct
{
    const char yamlKey[MAX_YAML_STRING_LEN];
    void* valueToSet;
    Value_Type typeOfValue;
} Configuration_Item;

Configuration_Item Ros_ConfigFile_Items[] =
{
    { "ros_domain_id", &g_nodeConfigSettings.ros_domain_id, Value_Int },
    { "node_name", g_nodeConfigSettings.node_name, Value_String },
    { "node_namespace", g_nodeConfigSettings.node_namespace, Value_String },
    { "remap_rules", g_nodeConfigSettings.remap_rules, Value_String },
    { "agent_ip_address", g_nodeConfigSettings.agent_ip_address, Value_String },
    { "agent_port_number", g_nodeConfigSettings.agent_port_number, Value_String },
    { "sync_timeclock_with_agent", &g_nodeConfigSettings.sync_timeclock_with_agent, Value_Bool },
    { "namespace_tf", &g_nodeConfigSettings.namespace_tf, Value_Bool },
    { "publish_tf", &g_nodeConfigSettings.publish_tf, Value_Bool },
    { "joint_names", &joint_names_iterator, Value_JointNameArray },
    { "log_to_stdout", &g_nodeConfigSettings.log_to_stdout, Value_Bool },
    { "executor_sleep_period", &g_nodeConfigSettings.executor_sleep_period, Value_Int },
    { "action_feedback_publisher_period", &g_nodeConfigSettings.action_feedback_publisher_period, Value_Int },
    { "controller_status_monitor_period", &g_nodeConfigSettings.controller_status_monitor_period, Value_Int },
    { "robot_status", &g_nodeConfigSettings.qos_robot_status, Value_Qos },
    { "joint_states", &g_nodeConfigSettings.qos_joint_states, Value_Qos },
    { "tf", &g_nodeConfigSettings.qos_tf, Value_Qos },
    { "tf_frame_prefix", &g_nodeConfigSettings.tf_frame_prefix, Value_String },
    { "stop_motion_on_disconnect", &g_nodeConfigSettings.stop_motion_on_disconnect, Value_Bool },
    { "inform_job_name", &g_nodeConfigSettings.inform_job_name, Value_String },
    { "allow_custom_inform_job", &g_nodeConfigSettings.allow_custom_inform_job, Value_Bool },
};

void Ros_ConfigFile_SetAllDefaultValues()
{
    //=========
    //ros_domain_id
    //TODO(gavanderhoorn): make this an unsigned int
    g_nodeConfigSettings.ros_domain_id = DEFAULT_ROS_DOMAIN_ID;

    //=========
    //node_name
    UCHAR macId[6];
    motoRosAssert_withMsg(
        Ros_GetMacAddress(macId) == OK,
        SUBCODE_FAIL_MP_NICDATA,
        "Must enable ETHERNET function");
    //last three bytes of MAC ID are a unique identifier
    sprintf(g_nodeConfigSettings.node_name, "%s_%2x_%2x_%2x", DEFAULT_NODE_NAME, macId[3], macId[4], macId[5]);

    //=========
    //node_namespace
    sprintf(g_nodeConfigSettings.node_namespace, "%s", DEFAULT_NODE_NAMSPACE);

    //=========
    //remap_rules (a single space-separated string for now)
    sprintf(g_nodeConfigSettings.remap_rules, "%s", DEFAULT_REMAP_RULES);

    //=========
    //agent_ip_address and agent_port_number
    //No default value. Must be specified by user.
    bzero(g_nodeConfigSettings.agent_ip_address, MAX_YAML_STRING_LEN);
    bzero(g_nodeConfigSettings.agent_port_number, MAX_YAML_STRING_LEN);

    //=========
    //sync_timeclock_with_agent
    g_nodeConfigSettings.sync_timeclock_with_agent = DEFAULT_SYNCTIME;

    //=========
    //publish_tf
    g_nodeConfigSettings.publish_tf = DEFAULT_PUBLISH_TF;

    //=========
    //namespace_tf
    g_nodeConfigSettings.namespace_tf = DEFAULT_NAMESPACE_TF;

    //=========
    //joint_names
    //If the joint_name values are not set in the configuration file, then they will be
    //set to their default value in Ros_Controller_Initialize().
    bzero(g_nodeConfigSettings.joint_names, sizeof(g_nodeConfigSettings.joint_names));
    joint_names_iterator = g_nodeConfigSettings.joint_names[0];

    //=========
    //log_to_stdout
    g_nodeConfigSettings.log_to_stdout = DEFAULT_LOG_TO_STDOUT;

    //=========
    //executor_sleep_period
    g_nodeConfigSettings.executor_sleep_period = DEFAULT_EXECUTOR_SLEEP_PERIOD;

    //=========
    //feedback_publisher_period
    g_nodeConfigSettings.action_feedback_publisher_period = DEFAULT_FEEDBACK_PUBLISH_PERIOD;

    //=========
    //controller_status_monitor_period
    g_nodeConfigSettings.controller_status_monitor_period = DEFAULT_CONTROLLER_IO_PERIOD;

    //=========
    //qos_robot_status
    g_nodeConfigSettings.qos_robot_status = DEFAULT_QOS_ROBOT_STATUS;

    //=========
    //qos_joint_states
    g_nodeConfigSettings.qos_joint_states = DEFAULT_QOS_JOINT_STATES;

    //=========
    //qos_tf
    g_nodeConfigSettings.qos_tf = DEFAULT_QOS_TF;

    //=========
    //tf_frame_prefix
    sprintf(g_nodeConfigSettings.tf_frame_prefix, "%s", DEFAULT_TF_FRAME_PREFIX);

    //=========
    //stop_motion_on_disconnect
    g_nodeConfigSettings.stop_motion_on_disconnect = DEFAULT_STOP_MOTION_ON_DISCON;

    //=========
    //inform_job_name
    snprintf(g_nodeConfigSettings.inform_job_name, MAX_JOB_NAME_LEN, "%s", DEFAULT_INFORM_JOB_NAME);

    //allow_custom_inform
    g_nodeConfigSettings.allow_custom_inform_job = DEFAULT_ALLOW_CUSTOM_INFORM;
}

void Ros_ConfigFile_CheckYamlEvent(yaml_event_t* event)
{
    static Configuration_Item* activeItem = NULL;
    static int nestedListCounter = 0;
    static int jointIteratorCountdownPerGroup = MP_GRP_AXES_NUM;

    char* t[] = { "y", "Y", "yes", "Yes", "YES", "true", "True", "TRUE", "on", "On", "ON", "1", NULL };
    char* f[] = { "n", "N", "no", "No", "NO", "false", "False", "FALSE", "off", "Off", "OFF", "0", NULL };
    BOOL bBoolValueFound;

    if (event->type == YAML_SCALAR_EVENT)
    {
        if (activeItem) //this is the value for a particular yamlKey
        {
            if (event->data.scalar.length > 0)
            {
                switch (activeItem->typeOfValue)
                {
                case Value_String:
                    strcpy((char*)activeItem->valueToSet, (char*)event->data.scalar.value);
                    Ros_Debug_BroadcastMsg("Config: %s = %s", (char*)activeItem->yamlKey, (char*)activeItem->valueToSet);
                    break;

                case Value_Int:
                    *(int*)activeItem->valueToSet = atoi((char*)event->data.scalar.value);
                    Ros_Debug_BroadcastMsg("Config: %s = %d", (char*)activeItem->yamlKey, *(int*)activeItem->valueToSet);
                    break;

                case Value_Bool:
                    bBoolValueFound = FALSE;
                    for (char** p = t; *p; p++)
                    {
                        if (strcmp((char*)event->data.scalar.value, *p) == 0)
                        {
                            *(BOOL*)activeItem->valueToSet = TRUE;
                            bBoolValueFound = TRUE;
                            break;
                        }
                    }
                    for (char** p = f; *p; p++)
                    {
                        if (strcmp((char*)event->data.scalar.value, *p) == 0)
                        {
                            *(BOOL*)activeItem->valueToSet = FALSE;
                            bBoolValueFound = TRUE;
                            break;
                        }
                    }

                    if (!bBoolValueFound)
                    {
                        mpSetAlarm(ALARM_CONFIGURATION_FAIL, "Invalid BOOL in motoros2_config", SUBCODE_CONFIGURATION_INVALID_BOOLEAN_VALUE);

                        Ros_Debug_BroadcastMsg("'%s' is not a valid boolean specifier", (char*)event->data.scalar.value);
                        Ros_Debug_BroadcastMsg("Config: %s left at default value", (char*)activeItem->yamlKey);
                    }
                    else
                        Ros_Debug_BroadcastMsg("Config: %s = %d", (char*)activeItem->yamlKey, *(BOOL*)activeItem->valueToSet);

                    break;

                case Value_JointNameArray:
                    strncpy(*(char**)activeItem->valueToSet, (char*)event->data.scalar.value, MAX_YAML_STRING_LEN);
                    Ros_Debug_BroadcastMsg("Config: %s = %s", (char*)activeItem->yamlKey, *(char**)activeItem->valueToSet);
                    //Ros_Debug_BroadcastMsg(" - joint name saved at 0x%X (iter = 0x%X)", (int)*(char**)activeItem->valueToSet, (int)joint_names_iterator);
                    joint_names_iterator += MAX_JOINT_NAME_LENGTH;
                    jointIteratorCountdownPerGroup -= 1;
                    break;

                case Value_Qos:
                    if (strcmp((char*)event->data.scalar.value, "sensor_data") == 0)
                        *(Ros_QoS_Profile_Setting*)activeItem->valueToSet = ROS_QOS_PROFILE_SENSOR_DATA;
                    else if (strcmp((char*)event->data.scalar.value, "default") == 0)
                        *(Ros_QoS_Profile_Setting*)activeItem->valueToSet = ROS_QOS_PROFILE_DEFAULT;
                    else
                    {
                        mpSetAlarm(ALARM_CONFIGURATION_FAIL, "Invalid QOS in motoros2_config", SUBCODE_CONFIGURATION_INVALID_QOS_VALUE);

                        *(Ros_QoS_Profile_Setting*)activeItem->valueToSet = ROS_QOS_PROFILE_DEFAULT;
                        Ros_Debug_BroadcastMsg(
                            "Falling back to '%s' profile for '%s': unrecognised profile: '%s'",
                            "default",
                            (char*)activeItem->yamlKey,
                            (char*)event->data.scalar.value);
                    }

                    Ros_Debug_BroadcastMsg("Config: %s = %s", (char*)activeItem->yamlKey, (char*)event->data.scalar.value);
                    break;
                }
            }

            if (activeItem->typeOfValue != Value_JointNameArray)
                activeItem = NULL;
        }
        else //look through list of Ros_ConfigFile_Items and find the matching yamlKey string
        {
            int numberOfItems = sizeof(Ros_ConfigFile_Items) / sizeof(Configuration_Item);
            for (int i = 0; i < numberOfItems; i += 1)
            {
                if (event->data.scalar.length > 0)
                {
                    if (strcmp(Ros_ConfigFile_Items[i].yamlKey, (char*)event->data.scalar.value) == 0)
                    {
                        activeItem = &Ros_ConfigFile_Items[i];
                    }
                }
            }
        }
    }
    else if (event->type == YAML_SEQUENCE_START_EVENT && activeItem->typeOfValue == Value_JointNameArray)
    {
        nestedListCounter += 1;
        jointIteratorCountdownPerGroup = MP_GRP_AXES_NUM;
    }
    else if (event->type == YAML_SEQUENCE_END_EVENT && activeItem->typeOfValue == Value_JointNameArray)
    {
        nestedListCounter -= 1;
        joint_names_iterator += (jointIteratorCountdownPerGroup * MAX_JOINT_NAME_LENGTH); //skip to next group in array

        if (nestedListCounter == 0) //all sub-lists in [joint_names] have been processed
        {
            activeItem = NULL;

            Ros_Debug_BroadcastMsg("List of configured joint names:");
            for (int i = 0; i < MAX_CONTROLLABLE_GROUPS; i += 1)
            {
                Ros_Debug_BroadcastMsg("---");
                for (int j = 0; j < MP_GRP_AXES_NUM; j += 1)
                {
                    if (strlen(g_nodeConfigSettings.joint_names[(i * MP_GRP_AXES_NUM) + j]) > 0)
                        Ros_Debug_BroadcastMsg(g_nodeConfigSettings.joint_names[(i * MP_GRP_AXES_NUM) + j]);
                    else
                        Ros_Debug_BroadcastMsg("x");
                }
            }
            Ros_Debug_BroadcastMsg("---");
        }
    }
}

void Ros_ConfigFile_CheckUsbForNewConfigFile()
{
    int ret;
    int fdUsb;

    const int MAX_PATH_LEN = 128;
    char usbFilePath[MAX_PATH_LEN];
    char sramFilePath[MAX_PATH_LEN];

    char usbFilePathRename[MAX_PATH_LEN];
    MP_CALENDAR_RSP_DATA calendar;

    ret = snprintf(usbFilePath, MAX_PATH_LEN, "%s\\%s", MP_USB0_DEV_DOS, CONFIG_FILE_NAME);
    if (ret < 0 || ret >= MAX_PATH_LEN)
    {
        //No point in alarming. There's nothing the user can do about it. But, they can still load
        //the file through the pendant.
        Ros_Debug_BroadcastMsg("ERROR: Path to config file on USB exceeds maximum. Config file must be loaded through the pendant menu.");
        return;
    }

    mpGetCalendar(&calendar);
    ret = snprintf(usbFilePathRename, MAX_PATH_LEN, FORMAT_CONFIG_FILE_BACKUP, MP_USB0_DEV_DOS, CONFIG_FILE_NAME, 
        calendar.usYear, calendar.usMonth, calendar.usDay, calendar.usHour, calendar.usMin, calendar.usSec);
    if (ret < 0 || ret >= MAX_PATH_LEN)
    {
        //No point in alarming. There's nothing the user can do about it. It'll just be truncated on the rename.
        Ros_Debug_BroadcastMsg("WARNING: Config file will be renamed with a timestamp. The timestamp will be truncated due to the length.");
    }

    ret = snprintf(sramFilePath, MAX_PATH_LEN, "%s\\%s", MP_SRAM_DEV_DOS, CONFIG_FILE_NAME);
    if (ret < 0 || ret >= MAX_PATH_LEN)
    {
        //No point in alarming. There's nothing the user can do about it. But, they can still load
        //the file through the pendant.
        Ros_Debug_BroadcastMsg("ERROR: Path to config file on SRAM exceeds maximum. Config file must be loaded through the pendant menu.");
        return;
    }

    fdUsb = mpOpen(usbFilePath, O_RDONLY, 0);

    if (fdUsb >= 0) //file exists on USB drive
    {
        mpRemove(sramFilePath); //don't care if it fails (file might not exist)

        int fdSram = mpCreate(sramFilePath, O_WRONLY);
        if (fdSram < 0)
        {
            mpSetAlarm(ALARM_CONFIGURATION_FAIL, "Set S2C1102=2; Init SRAMDRV.DAT", SUBCODE_CONFIGURATION_SRAM_ACCESS_FAILURE);
            mpSetAlarm(ALARM_CONFIGURATION_FAIL, "Fail to copy config file", SUBCODE_CONFIGURATION_SRAM_ACCESS_FAILURE);
            mpClose(fdUsb);
            return;
        }

        int bytesRead;

        do //copy file contents to SRAM
        {
            const int BUFFER_SIZE = 1024;
            char buffer[BUFFER_SIZE];
            bytesRead = mpRead(fdUsb, buffer, BUFFER_SIZE);
            if (bytesRead > 0)
            {
                ret = mpWrite(fdSram, buffer, bytesRead);
                if (ret != bytesRead)
                {
                    mpSetAlarm(ALARM_CONFIGURATION_FAIL, "Initialize SRAMDRV.DAT", SUBCODE_CONFIGURATION_SRAM_WRITE_FAILURE);
                    mpSetAlarm(ALARM_CONFIGURATION_FAIL, "Fail to copy config file", SUBCODE_CONFIGURATION_SRAM_WRITE_FAILURE);
                    break;
                }
            }
        } while (bytesRead > 0);

        mpClose(fdUsb);
        mpClose(fdSram);

        Ros_Debug_BroadcastMsg("Renaming configuration file to mark it as processed");
        mpRename(usbFilePath, usbFilePathRename);

        Ros_Debug_BroadcastMsg("Configuration file updated from CN102 USB drive.");
    }
    else
        Ros_Debug_BroadcastMsg("No new configuration file found on CN102 USB drive.");
}

void Ros_ConfigFile_ValidateCriticalSettings()
{
    //==============================================================================
    //Any invalid settings checked in this function will result in a fatal assertion.
    //These are essential for MotoROS2 operation.
    //==============================================================================

    //-----------------------------------------
    //Verify domain ID is legal
    //
    //See also https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html
    //
    //NOTE: the ROS 2 documentation explains different OS have different
    //      sets of legal values. We'll assume the same as the
    //      documentation, and use the most restricted set (that of Linux).
    //
    //NOTE 2: this does not affect the micro-ROS side (ie: MotoROS2).
    //        This setting is used by the Agent to join the correct
    //        DDS domain.
    motoRosAssert_withMsg(g_nodeConfigSettings.ros_domain_id >= MIN_ROS_DOMAIN_ID_LINUX && g_nodeConfigSettings.ros_domain_id <= MAX_ROS_DOMAIN_ID_LINUX,
        SUBCODE_CONFIGURATION_INVALID_DOMAIN_ID,
        "Domain ID (%d) invalid",
        g_nodeConfigSettings.ros_domain_id);

    //-----------------------------------------
    //Verify we have an Agent address
    motoRosAssert_withMsg(strlen(g_nodeConfigSettings.agent_ip_address) != 0 && strlen(g_nodeConfigSettings.agent_port_number) != 0,
        SUBCODE_CONFIGURATION_MISSING_AGENT_IP,
        "Missing Agent IP/Port");

    //-----------------------------------------
    //Verify agent is on my subnet (or a gateway is specified)
    BOOL bAgentOnMySubnet = FALSE;
    ULONG ip_be1, subnetmask_be1, gateway_be1;
    ULONG ip_be2, subnetmask_be2, gateway_be2;
    UINT8 macId[6];
    ULONG agent_ip_be;

    agent_ip_be = mpInetAddr(g_nodeConfigSettings.agent_ip_address);

    //check first lan port
    if (Ros_mpNICData(ROS_USER_LAN1, &ip_be1, &subnetmask_be1, macId, &gateway_be1) == OK)
        bAgentOnMySubnet = ((ip_be1 & subnetmask_be1) == (agent_ip_be & subnetmask_be1));

    //check second lan port
    if (!bAgentOnMySubnet && (Ros_mpNICData(ROS_USER_LAN2, &ip_be2, &subnetmask_be2, macId, &gateway_be2) == OK))
        bAgentOnMySubnet = ((ip_be2 & subnetmask_be2) == (agent_ip_be & subnetmask_be2));

    if (!bAgentOnMySubnet) //check if gateway is configured
        bAgentOnMySubnet = (gateway_be1 != 0) || (gateway_be2 != 0);

    motoRosAssert_withMsg(bAgentOnMySubnet,
        SUBCODE_CONFIGURATION_INVALID_AGENT_SUBNET,
        "Agent IP on wrong subnet");

    //-----------------------------------------
    //Verify we have node name
    motoRosAssert_withMsg(Ros_strnlen(g_nodeConfigSettings.node_name, MAX_YAML_STRING_LEN) != 0,
        SUBCODE_CONFIGURATION_INVALID_NODE_NAME,
        "Must specify node name");

    //-----------------------------------------
    //Verify we have an INFORM job name
    motoRosAssert_withMsg(Ros_strnlen(g_nodeConfigSettings.inform_job_name, MAX_YAML_STRING_LEN) != 0,
        SUBCODE_CONFIGURATION_INVALID_JOB_NAME,
        "Must specify INFORM job name");
}

void Ros_ConfigFile_ValidateNonCriticalSettings()
{
    //==============================================================================
    //Any settings checked in this function that are found to be invalid will be set
    //to their DEFAULT value.
    //==============================================================================

    //NOTE: QoS and BOOL settings were validated in Ros_ConfigFile_CheckYamlEvent

    //-----------------------------------------------------------------------------
    if (g_nodeConfigSettings.executor_sleep_period < MIN_EXECUTOR_SLEEP_PERIOD ||
        g_nodeConfigSettings.executor_sleep_period > MAX_EXECUTOR_SLEEP_PERIOD)
    {
        Ros_Debug_BroadcastMsg("executor_sleep_period value %d is invalid; reverting to default of %d", 
            g_nodeConfigSettings.executor_sleep_period, DEFAULT_EXECUTOR_SLEEP_PERIOD);

        mpSetAlarm(ALARM_CONFIGURATION_FAIL, "Invalid executor_sleep_period", SUBCODE_CONFIGURATION_INVALID_EXECUTOR_PERIOD);

        g_nodeConfigSettings.executor_sleep_period = DEFAULT_EXECUTOR_SLEEP_PERIOD;
    }

    //-----------------------------------------------------------------------------
    if (g_nodeConfigSettings.action_feedback_publisher_period < MIN_FEEDBACK_PUBLISH_PERIOD ||
        g_nodeConfigSettings.action_feedback_publisher_period > MAX_FEEDBACK_PUBLISH_PERIOD)
    {
        Ros_Debug_BroadcastMsg("action_feedback_publisher_period value %d is invalid; reverting to default of %d",
            g_nodeConfigSettings.action_feedback_publisher_period, DEFAULT_FEEDBACK_PUBLISH_PERIOD);

        mpSetAlarm(ALARM_CONFIGURATION_FAIL, "Invalid fb_publisher_period", SUBCODE_CONFIGURATION_INVALID_FEEDBACK_PERIOD);

        g_nodeConfigSettings.action_feedback_publisher_period = DEFAULT_FEEDBACK_PUBLISH_PERIOD;
    }

    //-----------------------------------------------------------------------------
    if (g_nodeConfigSettings.controller_status_monitor_period < MIN_CONTROLLER_IO_PERIOD ||
        g_nodeConfigSettings.controller_status_monitor_period > MAX_CONTROLLER_IO_PERIOD)
    {
        Ros_Debug_BroadcastMsg("controller_status_monitor_period value %d is invalid; reverting to default of %d",
            g_nodeConfigSettings.controller_status_monitor_period, DEFAULT_CONTROLLER_IO_PERIOD);

        mpSetAlarm(ALARM_CONFIGURATION_FAIL, "Invalid status_monitor_period", SUBCODE_CONFIGURATION_INVALID_IO_PERIOD);

        g_nodeConfigSettings.controller_status_monitor_period = DEFAULT_CONTROLLER_IO_PERIOD;
    }
}

void Ros_ConfigFile_Parse()
{
    BOOL bAlarmOnce = TRUE;
    BOOL bOkToInit = TRUE;

    Ros_ConfigFile_SetAllDefaultValues();

    Ros_ConfigFile_CheckUsbForNewConfigFile();

    do
    {
        if (!bOkToInit)
            Ros_Sleep(3000);

        bOkToInit = TRUE;

        //-----------------------------------------
        //Parse file
        yaml_parser_t parser;
        yaml_event_t event;
        char sramFilePath[128];
        int fd;
        struct stat fileStat;

        sprintf(sramFilePath, "%s\\%s", MP_SRAM_DEV_DOS, CONFIG_FILE_NAME);

        Ros_Debug_BroadcastMsg("Checking configuration file: %s", CONFIG_FILE_NAME);

        fd = mpOpen(sramFilePath, O_RDONLY, 0);

        if (fd >= 0)
        {
            mpFstat(fd, &fileStat);

            const size_t st_size_sz = fileStat.st_size + 1;
            char* string = (char*)mpMalloc(st_size_sz);
            bzero(string, st_size_sz);

            mpRead(fd, string, fileStat.st_size);

            mpClose(fd);

            yaml_parser_initialize(&parser);
            yaml_parser_set_input_string(&parser, (UCHAR*)string, fileStat.st_size);

            yaml_event_type_t event_type;
            do
            {
                if (!yaml_parser_parse(&parser, &event))
                {
                    Ros_Debug_BroadcastMsg("Failed to parse: %s", parser.problem);
                    yaml_parser_delete(&parser);
                    bOkToInit = FALSE;
                    continue;
                }
                Ros_ConfigFile_CheckYamlEvent(&event);
                event_type = event.type;
                yaml_event_delete(&event);
            } while (event_type != YAML_STREAM_END_EVENT);

            yaml_parser_delete(&parser);

            mpFree(string);
        }
        else
        {
            if (bAlarmOnce)
            {
                mpSetAlarm(ALARM_CONFIGURATION_FAIL, "Missing " APPLICATION_NAME " cfg file", SUBCODE_CONFIGURATION_MISSINGFILE);
                bAlarmOnce = FALSE;
            }
            bOkToInit = FALSE;
            continue;
        }

    } while (!bOkToInit);

    Ros_ConfigFile_ValidateCriticalSettings();
    Ros_ConfigFile_ValidateNonCriticalSettings();
}

rmw_qos_profile_t const* const Ros_ConfigFile_To_Rmw_Qos_Profile(Ros_QoS_Profile_Setting val)
{
    if (val == ROS_QOS_PROFILE_SENSOR_DATA)
        return &rmw_qos_profile_sensor_data;
    if (val == ROS_QOS_PROFILE_PARAMETERS)
        return &rmw_qos_profile_parameters;
    if (val == ROS_QOS_PROFILE_DEFAULT)
        return &rmw_qos_profile_default;
    if (val == ROS_QOS_PROFILE_SERVICES)
        return &rmw_qos_profile_services_default;
    if (val == ROS_QOS_PROFILE_PARAMETER_EVENTS)
        return &rmw_qos_profile_parameter_events;
    if (val == ROS_QOS_PROFILE_SYSTEM_DEFAULT)
        return &rmw_qos_profile_system_default;
    return &rmw_qos_profile_unknown;
}
