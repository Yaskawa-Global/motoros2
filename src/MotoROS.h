//MotoROS.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_MOTOROS_H
#define MOTOROS2_MOTOROS_H

#define APPLICATION_NAME            "MotoROS2"
#define APPLICATION_VERSION         "0.1.4pre"

#include "motoPlus.h"

#include "motoplus_libmicroros_config.h"

//============================================
// Micro-ROS headers
//============================================
#include <rcl/rcl.h>
#include <rcl/types.h>
#include <rcl_action/rcl_action.h>
#include <rcl/error_handling.h>
#include <rcl/publisher.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>

#include <rcutils/allocator.h>
#include <rcutils/split.h>

#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include <rmw/rmw.h>
#include <rmw/types.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/init_options.h>
#include <rmw_microros/timing.h>

#include <libyaml_vendor/yaml.h>

//============================================
// Data types for communication
//============================================
#include <std_srvs/srv/trigger.h>
#include <sensor_msgs/msg/joint_state.h>
#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/quaternion.h>
#include <tf2_msgs/msg/tf_message.h>
#include <industrial_msgs/msg/robot_status.h>
#include <trajectory_msgs/msg/joint_trajectory.h>
#include <trajectory_msgs/msg/joint_trajectory_point.h>
#include <control_msgs/action/follow_joint_trajectory.h>
#include <motoros2_interfaces/srv/read_single_io.h>
#include <motoros2_interfaces/srv/read_group_io.h>
#include <motoros2_interfaces/srv/write_single_io.h>
#include <motoros2_interfaces/srv/write_group_io.h>
#include <motoros2_interfaces/srv/read_m_register.h>
#include <motoros2_interfaces/srv/write_m_register.h>
#include <motoros2_interfaces/msg/io_result_codes.h>
#include <motoros2_interfaces/srv/reset_error.h>
#include <motoros2_interfaces/srv/start_traj_mode.h>
#include <motoros2_interfaces/srv/start_point_queue_mode.h>
#include <motoros2_interfaces/srv/queue_traj_point.h>
#include <motoros2_interfaces/srv/select_motion_tool.h>
#include <motoros2_interfaces/srv/list_inform_jobs.h>
#include <motoros2_interfaces/msg/inform_job_crud_result_codes.h>

//============================================
// MotoROS
//============================================
#include "MotoPlusExterns.h"
#include "Debug.h"
#include "FileUtilityFunctions.h"
#include "CommunicationExecutor.h"
#include "Quaternion_Conversion.h"
#include "ErrorHandling.h"
#include "MemoryAllocation.h"
#include "MemoryTracing.h"
#include "CmosParameterExtraction.h"
#include "ActionServer_FJT.h"
#include "CtrlGroup.h"
#include "ControllerStatusIO.h"
#include "PositionMonitor.h"
#include "ServiceQueueTrajPoint.h"
#include "ServiceReadWriteIO.h"
#include "ServiceResetError.h"
#include "ServiceStartTrajMode.h"
#include "ServiceStartPointQueueMode.h"
#include "ServiceStopTrajMode.h"
#include "ServiceSelectMotionTool.h"
#include "ServiceInformJobList.h"
#include "MotionControl.h"
#include "ConfigFile.h"
#include "RosApiNameConstants.h"
#include "TimeConversionUtils.h"
#include "Tests_CtrlGroup.h"
#include "Tests_TestUtils.h"
#include "Tests_RosMotoPlusConversionUtils.h"
#include "Tests_ControllerStatusIO.h"
#include "Tests_ActionServer_FJT.h"
#include "FauxCommandLineArgs.h"
#include "InformCheckerAndGenerator.h"
#include "MathConstants.h"
#include "MotoROS_PlatformLib.h"
#include "Ros_mpGetRobotCalibrationData.h"
#include "RosMotoPlusConversionUtils.h"

extern void Ros_Sleep(float milliseconds);

#endif  // MOTOROS2_MOTOROS_H
