// ConfigFile.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_CONFIG_FILE_H
#define MOTOROS2_CONFIG_FILE_H

//For ROS_USER_LAN1 and ROS_USER_LAN2
#include "MotoROS_PlatformLib.h"


#define CONFIG_FILE_NAME                "motoros2_config.yaml"
#define FORMAT_CONFIG_FILE_BACKUP       "%s\\%s.%04d%02d%02d_%02d%02d%02d" //Example: "MPUSB0\motoros2_config.yaml.202207029_081011"

#define MAX_YAML_STRING_LEN             128

#define DEFAULT_ROS_DOMAIN_ID           0
#define MIN_ROS_DOMAIN_ID_LINUX         0
#define MAX_ROS_DOMAIN_ID_LINUX         101

#define DEFAULT_NODE_NAME               "motoman" //will be suffixed with MAC ID

#define DEFAULT_NODE_NAMSPACE           ""

#define DEFAULT_SYNCTIME                TRUE

#define DEFAULT_PUBLISH_TF              TRUE

#define DEFAULT_NAMESPACE_TF            TRUE

// NOTE: We do not prefix joints by the "motoman Grp ID" here, but use the generic
// group & joint names instead to avoid the OEM-specific names.
#define DEFAULT_JOINT_NAME_FMT          "group_%d/joint_%d"

#define DEFAULT_LOG_TO_STDOUT           FALSE

#define DEFAULT_EXECUTOR_SLEEP_PERIOD   10 //ms
#define MIN_EXECUTOR_SLEEP_PERIOD       1
#define MAX_EXECUTOR_SLEEP_PERIOD       100

#define DEFAULT_FEEDBACK_PUBLISH_PERIOD 20 //ms
#define MIN_FEEDBACK_PUBLISH_PERIOD     1
#define MAX_FEEDBACK_PUBLISH_PERIOD     100

#define DEFAULT_CONTROLLER_IO_PERIOD    10 //ms
#define MIN_CONTROLLER_IO_PERIOD        1
#define MAX_CONTROLLER_IO_PERIOD        100

#define DEFAULT_QOS_ROBOT_STATUS        ROS_QOS_PROFILE_SENSOR_DATA

#define DEFAULT_QOS_JOINT_STATES        ROS_QOS_PROFILE_SENSOR_DATA

#define DEFAULT_QOS_TF                  ROS_QOS_PROFILE_DEFAULT

#define DEFAULT_TF_FRAME_PREFIX         ""

#define DEFAULT_STOP_MOTION_ON_DISCON   TRUE

#define DEFAULT_ALLOW_CUSTOM_INFORM     FALSE

#define DEFAULT_INFORM_JOB_NAME         "INIT_ROS"

// based on rmw/qos_profiles.h
typedef enum
{
    ROS_QOS_PROFILE_SENSOR_DATA = 1,
    ROS_QOS_PROFILE_PARAMETERS = 2,
    ROS_QOS_PROFILE_DEFAULT = 3,
    ROS_QOS_PROFILE_SERVICES = 4,
    ROS_QOS_PROFILE_PARAMETER_EVENTS = 5,
    ROS_QOS_PROFILE_SYSTEM_DEFAULT = 6,
} Ros_QoS_Profile_Setting;

#define ROS_QOS_PROFILE_SENSOR_DATA_NAME "sensor_data"
#define ROS_QOS_PROFILE_PARAMETERS_NAME "parameters"
#define ROS_QOS_PROFILE_DEFAULT_NAME "default"
#define ROS_QOS_PROFILE_SERVICES_NAME "services"
#define ROS_QOS_PROFILE_PARAMETER_EVENTS_NAME "parameter_events"
#define ROS_QOS_PROFILE_SYSTEM_DEFAULT_NAME "system_default"
#define ROS_QOS_PROFILE_UNKNOWN_NAME "unknown"

#define DEFAULT_REMAP_RULES             ""
#define MAX_REMAP_RULE_NUM              16
#define MAX_REMAP_RULE_LEN              256

typedef enum
{
    CFG_ROS_USER_LAN_DISABLED = -3,  //sentinel
    CFG_ROS_USER_LAN_ALL = -2,  //sentinel
    CFG_ROS_USER_LAN_AUTO = -1,  //sentinel
    CFG_ROS_USER_LAN1 = ROS_USER_LAN1,
    CFG_ROS_USER_LAN2 = ROS_USER_LAN2,
} Ros_UserLan_Port_Setting;

#define DEFAULT_ULAN_MON_ENABLED                TRUE
#define DEFAULT_ULAN_MON_LINK                   CFG_ROS_USER_LAN_AUTO

#define DEFAULT_IGNORE_MISSING_CALIB            FALSE

#define DEFAULT_ULAN_DEBUG_BROADCAST_ENABLED     TRUE
#define DEFAULT_ULAN_DEBUG_BROADCAST_PORT        CFG_ROS_USER_LAN_ALL
typedef struct
{
    //TODO(gavanderhoorn): add support for unsigned types
    int ros_domain_id;

    char node_name[MAX_YAML_STRING_LEN];
    char node_namespace[MAX_YAML_STRING_LEN];

    char remap_rules[MAX_REMAP_RULE_LEN];

    char agent_ip_address[MAX_YAML_STRING_LEN];
    char agent_port_number[MAX_YAML_STRING_LEN];

    BOOL sync_timeclock_with_agent;

    BOOL publish_tf;
    BOOL namespace_tf;

    char joint_names[MAX_CONTROLLABLE_GROUPS * MP_GRP_AXES_NUM][MAX_JOINT_NAME_LENGTH];

    BOOL log_to_stdout;

    int executor_sleep_period;
    int action_feedback_publisher_period;
    int controller_status_monitor_period;

    Ros_QoS_Profile_Setting qos_robot_status;
    Ros_QoS_Profile_Setting qos_joint_states;
    Ros_QoS_Profile_Setting qos_tf;

    char tf_frame_prefix[MAX_YAML_STRING_LEN];

    BOOL stop_motion_on_disconnect;

    char inform_job_name[MAX_JOB_NAME_LEN];

    BOOL allow_custom_inform_job;

    BOOL userlan_monitor_enabled;
    Ros_UserLan_Port_Setting userlan_monitor_port;

    BOOL ignore_missing_calib_data;

    BOOL userlan_debug_broadcast_enabled;
    Ros_UserLan_Port_Setting userlan_debug_broadcast_port;
} Ros_Configuration_Settings;

extern Ros_Configuration_Settings g_nodeConfigSettings;

extern void Ros_ConfigFile_Parse();

extern rmw_qos_profile_t const* const Ros_ConfigFile_To_Rmw_Qos_Profile(Ros_QoS_Profile_Setting val);

#endif  // MOTOROS2_CONFIG_FILE_H
