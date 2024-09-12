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
    Value_Qos,
    Value_UserLanPort,
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
    { "userlan_monitor_enabled", &g_nodeConfigSettings.userlan_monitor_enabled, Value_Bool },
    { "userlan_monitor_port", &g_nodeConfigSettings.userlan_monitor_port, Value_UserLanPort },
    { "ignore_missing_calib_data", &g_nodeConfigSettings.ignore_missing_calib_data, Value_Bool },
    { "userlan_debug_broadcast_enabled", &g_nodeConfigSettings.userlan_debug_broadcast_enabled, Value_Bool },
    { "userlan_debug_broadcast_port", &g_nodeConfigSettings.userlan_debug_broadcast_port, Value_UserLanPort },
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

    //TODO(gavanderhoorn): get MAC only from interface which is used for
    //ROS traffic (https://github.com/Yaskawa-Global/motoros2/issues/57)
    STATUS status = Ros_GetMacAddress(ROS_USER_LAN1, macId);
    if (status != OK)
    {
        Ros_Debug_BroadcastMsg("%s: Ros_GetMacAddress: iface: %d; error: %d",
            __func__, ROS_USER_LAN1, status);
        motoRosAssert_withMsg(FALSE, SUBCODE_CONFIGURATION_FAIL_MP_NICDATA0,
            "Must enable ETHERNET function");
    }

#if defined (YRC1000) || defined (YRC1000u)
    //Try second interface if first one didn't succeed
    if (status != OK && (status = Ros_GetMacAddress(ROS_USER_LAN2, macId)) != OK)
    {
        Ros_Debug_BroadcastMsg("%s: Ros_GetMacAddress: iface: %d; error: %d",
            __func__, ROS_USER_LAN2, status);
        motoRosAssert_withMsg(FALSE, SUBCODE_CONFIGURATION_FAIL_MP_NICDATA1,
            "Must enable ETHERNET function");
    }
#endif

    //last three bytes of MAC ID are a unique identifier
    sprintf(g_nodeConfigSettings.node_name, "%s_%02x_%02x_%02x", DEFAULT_NODE_NAME, macId[3], macId[4], macId[5]);

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

    //userlan monitoring
    g_nodeConfigSettings.userlan_monitor_enabled = DEFAULT_ULAN_MON_ENABLED;
    g_nodeConfigSettings.userlan_monitor_port = DEFAULT_ULAN_MON_LINK;

    //ignore_missing_calib_data
    g_nodeConfigSettings.ignore_missing_calib_data = DEFAULT_IGNORE_MISSING_CALIB;

    //userlan debug broadcast
    g_nodeConfigSettings.userlan_debug_broadcast_enabled = DEFAULT_ULAN_DEBUG_BROADCAST_ENABLED;
    g_nodeConfigSettings.userlan_debug_broadcast_port = DEFAULT_ULAN_DEBUG_BROADCAST_PORT;
}

void Ros_ConfigFile_CheckYamlEvent(yaml_event_t* event)
{
    static Configuration_Item* activeItem = NULL;
    static int nestedListCounter = 0;
    static int jointIteratorCountdownPerGroup = MP_GRP_AXES_NUM;

    char* t[] = { "y", "Y", "yes", "Yes", "YES", "true", "True", "TRUE", "on", "On", "ON", "1", NULL };
    char* f[] = { "n", "N", "no", "No", "NO", "false", "False", "FALSE", "off", "Off", "OFF", "0", NULL };

    if (event->type == YAML_SCALAR_EVENT)
    {
        if (activeItem) //this is the value for a particular yamlKey
        {
            if (event->data.scalar.length > 0)
            {
                BOOL bBoolValueFound = FALSE;

                switch (activeItem->typeOfValue)
                {
                case Value_String:
                    strcpy((char*)activeItem->valueToSet, (char*)event->data.scalar.value);
                    break;

                case Value_Int:
                    *(int*)activeItem->valueToSet = atoi((char*)event->data.scalar.value);
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
                    }
                    break;

                case Value_JointNameArray:
                    strncpy(*(char**)activeItem->valueToSet, (char*)event->data.scalar.value, MAX_YAML_STRING_LEN);
                    //Ros_Debug_BroadcastMsg(" - joint name saved at 0x%X (iter = 0x%X)", (int)*(char**)activeItem->valueToSet, (int)joint_names_iterator);
                    joint_names_iterator += MAX_JOINT_NAME_LENGTH;
                    jointIteratorCountdownPerGroup -= 1;
                    break;

                case Value_Qos:
                    if (strcmp((char*)event->data.scalar.value, ROS_QOS_PROFILE_SENSOR_DATA_NAME) == 0)
                        *(Ros_QoS_Profile_Setting*)activeItem->valueToSet = ROS_QOS_PROFILE_SENSOR_DATA;
                    else if (strcmp((char*)event->data.scalar.value, ROS_QOS_PROFILE_DEFAULT_NAME) == 0)
                        *(Ros_QoS_Profile_Setting*)activeItem->valueToSet = ROS_QOS_PROFILE_DEFAULT;
                    else
                    {
                        mpSetAlarm(ALARM_CONFIGURATION_FAIL, "Invalid QOS in motoros2_config", SUBCODE_CONFIGURATION_INVALID_QOS_VALUE);

                        *(Ros_QoS_Profile_Setting*)activeItem->valueToSet = ROS_QOS_PROFILE_DEFAULT;
                        Ros_Debug_BroadcastMsg(
                            "Falling back to '%s' profile for '%s': unrecognised profile: '%s'",
                            ROS_QOS_PROFILE_DEFAULT_NAME,
                            (char*)activeItem->yamlKey,
                            (char*)event->data.scalar.value);
                    }
                    break;

                case Value_UserLanPort:
#if defined (FS100) || defined (DX200)
                    // single port, override whatever was configured
                    * (Ros_UserLan_Port_Setting*)activeItem->valueToSet = CFG_ROS_USER_LAN1;
                    Ros_Debug_BroadcastMsg("DX200 or FS100: override to 'USER_LAN1'");

#elif defined (YRC1000) || defined (YRC1000u)
                    if (strncmp((char*)event->data.scalar.value, "USER_LAN1", 9) == 0)
                        *(Ros_UserLan_Port_Setting*)activeItem->valueToSet = CFG_ROS_USER_LAN1;
                    else if (strncmp((char*)event->data.scalar.value, "USER_LAN2", 9) == 0)
                        *(Ros_UserLan_Port_Setting*)activeItem->valueToSet = CFG_ROS_USER_LAN2;
                    else
                    {
                        //Note: ideally, we'd disable user lan monitoring or user lan debug
                        //broadcast here. However, we can't guarantee the 'userlan_monitor_enabled' 
                        // or the 'userlan_debug_broadcast_enabled' setting won't be parsed after
                        //this one. If it were to be parsed after the corresponding port setting, we'd
                        //be disabling it here, only to have it re-enabled later.
                        //Set the config value to the 'disabled' sentinel value and let the
                        //validation code below handle the fallout.
                        Ros_Debug_BroadcastMsg(
                            "Unrecognised value for '%s': '%s'. '%s' will be disabled",
                            (char*)activeItem->yamlKey,
                            (char*)event->data.scalar.value,
                            (char*)activeItem->yamlKey);
                        *(Ros_UserLan_Port_Setting*)activeItem->valueToSet = CFG_ROS_USER_LAN_DISABLED;
                    }
#else
#error Unsupported platform
#endif
                    //Note: this logs whatever was in the .yaml, NOT the verified/parsed value above
                    Ros_Debug_BroadcastMsg("Config: %s = %s", (char*)activeItem->yamlKey,
                        (char*)event->data.scalar.value);
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

STATUS Ros_ConfigFile_HostOnNetworkInterface(char* const host, USHORT if_no, BOOL* const reachable)
{
    ULONG nic_ip, nic_subnetmask, nic_gateway;
    ULONG host_ip;
    UINT8 dont_care[6];
    STATUS status = OK;
    BOOL host_on_network = FALSE;

    Ros_Debug_BroadcastMsg("%s: checking '%s' on iface %d", __func__, host, if_no);

    host_ip = mpInetAddr(host);
    status = Ros_mpNICData(if_no, &nic_ip, &nic_subnetmask, dont_care, &nic_gateway);
    if (status != OK)
        return status;

    host_on_network = ((nic_ip & nic_subnetmask) == (host_ip & nic_subnetmask));

    //check to see if gateway could be used, and assume the Agent is reachable through it
    if (!host_on_network)
    {
        Ros_Debug_BroadcastMsg("%s: not on network, gateway: 0x%08X", __func__, nic_gateway);
        host_on_network = (nic_gateway != 0)
            && ((nic_gateway & nic_subnetmask) == (host_ip & nic_subnetmask));
    }

    Ros_Debug_BroadcastMsg("%s: exit: on network: %s", __func__,
        host_on_network ? "true" : "false");

    *reachable = host_on_network;
    return OK;
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

    //check first lan port
    STATUS status = Ros_ConfigFile_HostOnNetworkInterface(
        g_nodeConfigSettings.agent_ip_address, ROS_USER_LAN1, &bAgentOnMySubnet);
    motoRosAssert_withMsg(status == OK, SUBCODE_CONFIGURATION_AGENT_ON_NET_CHECK,
        "Host on NIC check 1");

#if defined (YRC1000) || defined (YRC1000u)
    if (!bAgentOnMySubnet)
    {
        //check second lan port
        status = Ros_ConfigFile_HostOnNetworkInterface(
            g_nodeConfigSettings.agent_ip_address, ROS_USER_LAN2, &bAgentOnMySubnet);
        motoRosAssert_withMsg(status == OK, SUBCODE_CONFIGURATION_AGENT_ON_NET_CHECK,
            "Host on NIC check 2");
    }
#endif

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

    //-----------------------------------------------------------------------------
    if (g_nodeConfigSettings.userlan_monitor_enabled)
    {
        Ros_Debug_BroadcastMsg("UserLan monitor enabled, checking port setting ..");

        //try to auto-detect if the port was not configured
        if (g_nodeConfigSettings.userlan_monitor_port == CFG_ROS_USER_LAN_AUTO)
        {
            BOOL bAgentOnInterface = FALSE;
            STATUS status = Ros_ConfigFile_HostOnNetworkInterface(
                g_nodeConfigSettings.agent_ip_address, ROS_USER_LAN1, &bAgentOnInterface);
            motoRosAssert_withMsg(status == OK, SUBCODE_CONFIGURATION_AGENT_ON_NET_CHECK,
                "Host on NIC check 1 auto-detect");

            if (bAgentOnInterface)
            {
                g_nodeConfigSettings.userlan_monitor_port = CFG_ROS_USER_LAN1;
                Ros_Debug_BroadcastMsg("UserLan monitor auto-detect port: %d",
                    g_nodeConfigSettings.userlan_monitor_port);
            }
        }

#if defined (YRC1000) || defined (YRC1000u)
        //on these controllers we can try the second interface, if we haven't
        //already determined we should monitor the first
        if (g_nodeConfigSettings.userlan_monitor_port == CFG_ROS_USER_LAN_AUTO)
        {
            BOOL bAgentOnInterface = FALSE;
            STATUS status = Ros_ConfigFile_HostOnNetworkInterface(
                g_nodeConfigSettings.agent_ip_address, ROS_USER_LAN2, &bAgentOnInterface);
            motoRosAssert_withMsg(status == OK, SUBCODE_CONFIGURATION_AGENT_ON_NET_CHECK,
                "Host on NIC check 2 auto-detect");

            if (bAgentOnInterface)
            {
                g_nodeConfigSettings.userlan_monitor_port = CFG_ROS_USER_LAN2;
                Ros_Debug_BroadcastMsg("UserLan monitor auto-detect port: %d",
                    g_nodeConfigSettings.userlan_monitor_port);
            }
        }
#endif

        //if we still haven't determined which port to monitor, we'll raise an
        //alarm and disable monitoring. There is no appropriate default value
        //here, and user intervention is required.
        if (g_nodeConfigSettings.userlan_monitor_port == CFG_ROS_USER_LAN_AUTO)
        {
            mpSetAlarm(ALARM_CONFIGURATION_FAIL, "UserLan port detect failed",
                SUBCODE_CONFIGURATION_USERLAN_MONITOR_AUTO_DETECT_FAILED);
            g_nodeConfigSettings.userlan_monitor_enabled = FALSE;
            Ros_Debug_BroadcastMsg(
                "UserLan port auto-detection failed, disabling monitor");
        }

        //otherwise, either auto-detect worked, or a fixed value was configured.
        //In both cases, verify it's an acceptable value.
        else
        {
#if defined (YRC1000) || defined (YRC1000u)
            if (g_nodeConfigSettings.userlan_monitor_port != CFG_ROS_USER_LAN1 &&
                g_nodeConfigSettings.userlan_monitor_port != CFG_ROS_USER_LAN2)

#elif defined (FS100) || defined (DX200)
            if (g_nodeConfigSettings.userlan_monitor_port != CFG_ROS_USER_LAN1)
#endif
            {
                mpSetAlarm(ALARM_CONFIGURATION_FAIL, "Bad UserLan monitor port in cfg",
                    SUBCODE_CONFIGURATION_INVALID_USERLAN_MONITOR_PORT);
                g_nodeConfigSettings.userlan_monitor_enabled = FALSE;
                Ros_Debug_BroadcastMsg(
                    "userlan_monitor_port value %d is invalid, disabling monitor",
                    g_nodeConfigSettings.userlan_monitor_port);
            }
        }
    }

    if (g_nodeConfigSettings.userlan_debug_broadcast_enabled)
    {
        Ros_Debug_BroadcastMsg("UserLan debug broadcast enabled, checking port setting...");

#if defined (YRC1000) || defined (YRC1000u)
        if (g_nodeConfigSettings.userlan_debug_broadcast_port != CFG_ROS_USER_LAN1 &&
            g_nodeConfigSettings.userlan_debug_broadcast_port != CFG_ROS_USER_LAN2 && 
            g_nodeConfigSettings.userlan_debug_broadcast_port != CFG_ROS_USER_LAN_ALL)
#elif defined (FS100) || defined (DX200)
        if (g_nodeConfigSettings.userlan_debug_broadcast_port != CFG_ROS_USER_LAN1)
#endif
        {
            mpSetAlarm(ALARM_CONFIGURATION_FAIL, "Bad UserLan debug port in cfg",
                SUBCODE_CONFIGURATION_INVALID_USERLAN_DEBUG_BROADCAST_PORT);
            Ros_Debug_BroadcastMsg(
                "userlan_debug_broadcast_port value %d is invalid, disabling debug broadcast",
                g_nodeConfigSettings.userlan_debug_broadcast_port);
            g_nodeConfigSettings.userlan_debug_broadcast_enabled = FALSE;
        }
    }


}

const char* const Ros_ConfigFile_Rmw_Qos_ProfileSetting_ToString(Ros_QoS_Profile_Setting val)
{
    if (val == ROS_QOS_PROFILE_SENSOR_DATA)
        return ROS_QOS_PROFILE_SENSOR_DATA_NAME;
    if (val == ROS_QOS_PROFILE_PARAMETERS)
        return ROS_QOS_PROFILE_PARAMETERS_NAME;
    if (val == ROS_QOS_PROFILE_DEFAULT)
        return ROS_QOS_PROFILE_DEFAULT_NAME;
    if (val == ROS_QOS_PROFILE_SERVICES)
        return ROS_QOS_PROFILE_SERVICES_NAME;
    if (val == ROS_QOS_PROFILE_PARAMETER_EVENTS)
        return ROS_QOS_PROFILE_PARAMETER_EVENTS_NAME;
    if (val == ROS_QOS_PROFILE_SYSTEM_DEFAULT)
        return ROS_QOS_PROFILE_SYSTEM_DEFAULT_NAME;
    return ROS_QOS_PROFILE_UNKNOWN_NAME;
}

void Ros_ConfigFile_PrintActiveConfiguration(Ros_Configuration_Settings const* const config)
{
    Ros_Debug_BroadcastMsg("Config: ros_domain_id = %d", config->ros_domain_id);
    Ros_Debug_BroadcastMsg("Config: node_name = '%s'", config->node_name);
    Ros_Debug_BroadcastMsg("Config: node_namespace = '%s'", config->node_namespace);
    Ros_Debug_BroadcastMsg("Config: remap_rules = '%s'", config->remap_rules);
    Ros_Debug_BroadcastMsg("Config: agent_ip_address = '%s'", config->agent_ip_address);
    Ros_Debug_BroadcastMsg("Config: agent_port_number = '%s'", config->agent_port_number);
    Ros_Debug_BroadcastMsg("Config: sync_timeclock_with_agent = %d", config->sync_timeclock_with_agent);
    Ros_Debug_BroadcastMsg("Config: namespace_tf = %d", config->namespace_tf);
    Ros_Debug_BroadcastMsg("Config: publish_tf = %d", config->publish_tf);
    Ros_Debug_BroadcastMsg("List of configured joint names:");

    for (int i = 0; i < MAX_CONTROLLABLE_GROUPS; i += 1)
    {
        Ros_Debug_BroadcastMsg("---");
        for (int j = 0; j < MP_GRP_AXES_NUM; j += 1)
        {
            if (strlen(config->joint_names[(i * MP_GRP_AXES_NUM) + j]) > 0)
                Ros_Debug_BroadcastMsg("'%s'", config->joint_names[(i * MP_GRP_AXES_NUM) + j]);
            else
                Ros_Debug_BroadcastMsg("x");
        }
    }
    Ros_Debug_BroadcastMsg("---");

    Ros_Debug_BroadcastMsg("Config: logging.log_to_stdout = %d", config->log_to_stdout);
    Ros_Debug_BroadcastMsg("Config: update_periods.executor_sleep_period = %d", config->executor_sleep_period);
    Ros_Debug_BroadcastMsg("Config: update_periods.action_feedback_publisher_period = %d", config->action_feedback_publisher_period);
    Ros_Debug_BroadcastMsg("Config: update_periods.controller_status_monitor_period = %d", config->controller_status_monitor_period);
    Ros_Debug_BroadcastMsg("Config: publisher_qos.robot_status = '%s'", Ros_ConfigFile_Rmw_Qos_ProfileSetting_ToString(config->qos_robot_status));
    Ros_Debug_BroadcastMsg("Config: publisher_qos.joint_states = '%s'", Ros_ConfigFile_Rmw_Qos_ProfileSetting_ToString(config->qos_joint_states));
    Ros_Debug_BroadcastMsg("Config: publisher_qos.tf = '%s'", Ros_ConfigFile_Rmw_Qos_ProfileSetting_ToString(config->qos_tf));
    Ros_Debug_BroadcastMsg("Config: tf_frame_prefix = '%s'", config->tf_frame_prefix);
    Ros_Debug_BroadcastMsg("Config: stop_motion_on_disconnect = %d", config->stop_motion_on_disconnect);
    Ros_Debug_BroadcastMsg("Config: inform_job_name = '%s'", config->inform_job_name);
    Ros_Debug_BroadcastMsg("Config: allow_custom_inform_job = %d", config->allow_custom_inform_job);
    Ros_Debug_BroadcastMsg("Config: userlan_monitor_enabled = %d", config->userlan_monitor_enabled);
    Ros_Debug_BroadcastMsg("Config: userlan_monitor_port = %d", config->userlan_monitor_port);
    Ros_Debug_BroadcastMsg("Config: ignore_missing_calib_data = %d", config->ignore_missing_calib_data);
}

void Ros_ConfigFile_Parse()
{
    BOOL bAlarmOnce = TRUE;
    BOOL bOkToInit = TRUE;

    Ros_ConfigFile_SetAllDefaultValues();

    do
    {
#if defined (FS100) || defined (YRC1000) || defined (YRC1000u)
        //config file always resides on USB for DX200, so only check
        //on YRC1000 and micro
        Ros_ConfigFile_CheckUsbForNewConfigFile();
#endif

        if (!bOkToInit)
            Ros_Sleep(3000);

        bOkToInit = TRUE;

        //-----------------------------------------
        //Parse file
        yaml_parser_t parser;
        yaml_event_t event;
        const int CHAR_BUFFER_SIZE = 128;
        char configFilePath[CHAR_BUFFER_SIZE];
        char storageDrive[CHAR_BUFFER_SIZE];
        int fd;
        struct stat fileStat;

#if defined (FS100) || defined (DX200)
        snprintf(storageDrive, CHAR_BUFFER_SIZE, "%s", MP_USB0_DEV_DOS);
#elif defined (YRC1000) || defined (YRC1000u)
        snprintf(storageDrive, CHAR_BUFFER_SIZE, "%s", MP_SRAM_DEV_DOS);
#else
#error Ros_ConfigFile_Parse: unsupported platform
#endif

        snprintf(configFilePath, CHAR_BUFFER_SIZE, "%s\\%s", storageDrive, CONFIG_FILE_NAME);

        Ros_Debug_BroadcastMsg("Checking configuration file: %s", CONFIG_FILE_NAME);

        fd = mpOpen(configFilePath, O_RDONLY, 0);

#if defined (DX200)
        if (fd < 0)
        {
            //try again using second USB port
            snprintf(configFilePath, CHAR_BUFFER_SIZE, "%s\\%s", MP_USB1_DEV_DOS, CONFIG_FILE_NAME);
            fd = mpOpen(configFilePath, O_RDONLY, 0);
        }
#endif

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
#if defined(YRC1000) || defined(YRC1000u)
    if(g_nodeConfigSettings.userlan_debug_broadcast_enabled &&
        (g_nodeConfigSettings.userlan_debug_broadcast_port == CFG_ROS_USER_LAN1 ||
        g_nodeConfigSettings.userlan_debug_broadcast_port == CFG_ROS_USER_LAN2))
        Ros_Debug_SetFromConfig();
#endif
    Ros_ConfigFile_PrintActiveConfiguration(&g_nodeConfigSettings);
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
