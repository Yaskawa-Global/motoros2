<!--
SPDX-FileCopyrightText: 2023, Yaskawa America, Inc.
SPDX-FileCopyrightText: 2023, Delft University of Technology

SPDX-License-Identifier: CC-BY-SA-4.0
-->

# ROS API

This section provides a brief overview of the ROS API (ie: topics, services and actions) of MotoROS2.

Note: only the non-namespaced and non-prefixed names are shown here.

## Action servers

### follow_joint_trajectory

Type: [control_msgs/action/FollowJointTrajectory](https://github.com/ros-controls/control_msgs/blob/a555c37f1a3536bb452ea555c58fdd9344d87614/control_msgs/action/FollowJointTrajectory.action)

Execute the trajectory submitted as part of the goal, under the conditions specified by the goal (only the `goal_time_tolerance` and `goal_tolerance` fields are supported by MotoROS2 in the current implementation).

MotoROS2 attempts to execute the motion encoded by the [JointTrajectory](https://github.com/ros2/common_interfaces/blob/37ebe90cbfa91bcdaf69d6ed39c08859c4c3bcd4/trajectory_msgs/msg/JointTrajectory.msg) as faithfully as possible.
Due to requirements on the dynamics, accelerations specified are recalculated by MotoROS2 based on segment duration and velocities in each individual `JointTrajectoryPoint`.

Note: MotoROS2 has extended the possible set of values returned in the `error_code` field of the final action result.
Returned error values are always of the form `-ECCCCC`, where `E` is [the ROS defined error code](https://github.com/ros-controls/control_msgs/blob/a555c37f1a3536bb452ea555c58fdd9344d87614/control_msgs/action/FollowJointTrajectory.action#L35-L39) and `CCCCC` is [a MotoROS2 error code](https://github.com/yaskawa-global/motoros2_interfaces/blob/d6805d32714df4430f7db3d8ddc736c340ddeba8/msg/MotionReadyEnum.msg).

Example: `error_code: -100101` decodes into `-1` and `101`, which would indicate the goal is invalid because there is an active alarm.

## Actions called

None.

## Subscribed topics

None.

## Published topics

### joint_states

Type: [sensor_msgs/msg/JointState](https://github.com/ros2/common_interfaces/blob/37ebe90cbfa91bcdaf69d6ed39c08859c4c3bcd4/sensor_msgs/msg/JointState.msg)

Joint state for all joints in all groups of the connected robot:

- joint position (rad or metre)
- joint velocity (rad/sec or metres/sec)
- joint effort: torque for revolute joints (Nm), force for prismatic joints (N)

### ctrl_groups/.../joint_states

Joint states for the joints in a specific motion group (fi: `r1`, `b1`, `s1`, etc).
One topic per configured motion group.
`JointState` messages published on these topics will only contain information on the joints in the motion group.

This topic carries the same message type as the global `joint_states` topic.

### robot_status

Type: [industrial_msgs/msg/RobotStatus](https://github.com/ros-industrial/industrial_core/blob/d547cdcfdaf3bc0d46325215b8219b0a190c8e6c/industrial_msgs/msg/RobotStatus.msg)

Aggregate controller/robot status (ie: drives enabled, motion possible, active error, etc).

Use this topic (in addition to the `result_code`s) to determine whether there are any error conditions preventing `start_traj_mode` from activating the servos and subsequently enabling trajectory mode.

### tf

Type: [tf2_msgs/msg/TFMessage](https://github.com/ros2/geometry2/blob/51a7f24191198eb9fc8124d36aba5bb2f7ad84f3/tf2_msgs/msg/TFMessage.msg)

Standard ROS topic onto which TF transforms are broadcast.

Note: this topic is only namespaced if a namespace is configured *and* `namespace_tf` is set to `true` in the configuration file.

## Services

### read_group_io

Type: [motoros2_interfaces/srv/ReadGroupIO](https://github.com/yaskawa-global/motoros2_interfaces/blob/d6805d32714df4430f7db3d8ddc736c340ddeba8/srv/ReadGroupIO.srv)

Retrieve the state/value of the addressed Group IO element.

Please refer to the documentation embedded in the service definition for more information about addressing and general service behaviour.

### read_mregister

Type: [motoros2_interfaces/srv/ReadMRegister](https://github.com/yaskawa-global/motoros2_interfaces/blob/d6805d32714df4430f7db3d8ddc736c340ddeba8/srv/ReadMRegister.srv)

Retrieve the value of the addressed M register.

Please refer to the documentation embedded in the service definition for more information about addressing and general service behaviour.

### read_single_io

Type: [motoros2_interfaces/srv/ReadSingleIO](https://github.com/yaskawa-global/motoros2_interfaces/blob/d6805d32714df4430f7db3d8ddc736c340ddeba8/srv/ReadSingleIO.srv)

Retrieve the state/value of the addressed IO element.

Please refer to the documentation embedded in the service definition for more information about addressing and general service behaviour.

### reset_error

Type: [motoros2_interfaces/srv/ResetError](https://github.com/yaskawa-global/motoros2_interfaces/blob/d6805d32714df4430f7db3d8ddc736c340ddeba8/srv/ResetError.srv)

Attempt to reset controller errors and alarms.

Inspect the `result_code` field to determine the result of the invocation, and check the relevant fields of the `RobotStatus` messages to determine overall controller status.

Note: errors and alarms which require physical operator intervention (e-stops, etc) can not be reset by this service.

### start_traj_mode

Type: [motoros2_interfaces/srv/StartTrajMode](https://github.com/yaskawa-global/motoros2_interfaces/blob/d6805d32714df4430f7db3d8ddc736c340ddeba8/srv/StartTrajMode.srv)

Attempts to enable servo drives, activate trajectory mode, and set the job-cycle mode to allow execution of INIT_ROS.
This allows the action server (`follow_joint_trajectory`, see below) to execute incoming `FollowJointTrajectory` action goals.

Note: this service may fail if controller state prevents it from transitioning to trajectory mode.
Inspect the `result_code` to determine the cause.
Check the relevant fields of the `RobotStatus` messages to determine overall controller status.

The `reset_error` service can be used to attempt to reset errors and alarms.

### start_point_queue_mode

Type: [motoros2_interfaces/srv/StartPointQueueMode](https://github.com/yaskawa-global/motoros2_interfaces/blob/d6805d32714df4430f7db3d8ddc736c340ddeba8/srv/StartPointQueueMode.srv)

Attempts to enable servo drives, activate the point-queue motion mode, and set the job-cycle mode to allow execution of INIT_ROS.
This allows the `queue_traj_point` service (see below) to execute incoming `QueueTrajPoint` requests.

Note: this service may fail if controller state prevents it from transitioning to trajectory mode.
Inspect the `result_code` to determine the cause.
Check the relevant fields of the `RobotStatus` messages to determine overall controller status.

The `reset_error` service can be used to attempt to reset errors and alarms

### stop_traj_mode

Type: [std_srvs/srv/Trigger](https://github.com/ros2/common_interfaces/blob/37ebe90cbfa91bcdaf69d6ed39c08859c4c3bcd4/std_srvs/srv/Trigger.srv)

Attempt to deactivate motion mode.
The servo drives will remain enabled, if they were enabled before this service was called.

This service will fail if called while motion is being executed.

To stop a currently executing FollowJointTrajectory motion, cancel the active goal (either using the action client which submitted it, or after inspecting the list of active goals of the action server and submitting a cancel request for a specific goal id).

### queue_traj_point

Type: [motoros2_interfaces/srv/QueueTrajPoint](https://github.com/yaskawa-global/motoros2_interfaces/blob/d6805d32714df4430f7db3d8ddc736c340ddeba8/srv/QueueTrajPoint.srv)

Submit a `JointTrajectoryPoint` to be queued for continuous motion.

The `start_point_queue_mode` service must have been called prior to attempting to use this service.

If this service fails, inspect the `QueueResultEnum` field in the reply to determine the cause.
The most common type of failure is `BUSY`.
This is caused when the system is still processing a previously queued point.

### write_group_io

Type: [motoros2_interfaces/srv/WriteGroupIO](https://github.com/yaskawa-global/motoros2_interfaces/blob/d6805d32714df4430f7db3d8ddc736c340ddeba8/srv/WriteGroupIO.srv)

Write a value to the addressed Group IO element.

Please refer to the documentation embedded in the service definition for more information about legal values, addressing and general service behaviour.

### write_mregister

Type: [motoros2_interfaces/srv/WriteMRegister](https://github.com/yaskawa-global/motoros2_interfaces/blob/d6805d32714df4430f7db3d8ddc736c340ddeba8/srv/WriteMRegister.srv)

Write a value to the addressed M register.

Please refer to the documentation embedded in the service definition for more information about legal values, addressing and general service behaviour.

### write_single_io

Type: [motoros2_interfaces/srv/WriteSingleIO](https://github.com/yaskawa-global/motoros2_interfaces/blob/d6805d32714df4430f7db3d8ddc736c340ddeba8/srv/WriteSingleIO.srv)

Write a value to the addressed IO element.

Please refer to the documentation embedded in the service definition for more information about legal values, addressing and general service behaviour.

## Services called

None.

## Parameters

None.

Note: this documents *ROS parameters*, which are currently not supported.

## Required TF transforms

None.

## Provided TF transforms

### world → base

Transform from the origin of the Yaskawa *BF* to the origin of the current Yaskawa *RF*.

Note: the child frame `base` follows ROS-Industrial conventions, it is not the Yaskawa BF.

Note 2: this transform will not be correct for multi-robot setups (for example, an `R1+R2` configuration) until the robot group(s) have been calibrated.
See also [Incorrect transform tree origin with multi-robot setups](../README.md#incorrect-transform-tree-origin-with-multi-robot-setups).

### base → flange

Transform from Yaskawa *RF* to ROS-Industrial `flange` frame.

This frame's origin coincides with the Yaskawa flange, but is rotated such that it always follows REP-103 (ie: relative to the link, X+ is forward, Y+ left, Z+ up, instead of Z+ forward).

Attaching EEF models to this frame now becomes straightforward, as the frame orientation is always the same across different robots (with different zero poses).

### flange → tool0

Transform from the ROS-Industrial `flange` frame to the ROS-Industrial `tool0` frame (an "all zeros toolframe").

This frame coincides with the location and orientation of an *unconfigured* tool file on the Yaskawa controller, and will never change (ie: it's a static transform), not even if tool files are configured.

Note: this is not the same as Yaskawa's *Tool No. 0* (ie: tool file 0).
Only if that tool file is still unconfigured (ie: all zeros) would `tool0` and tool file 0 coincide.

### flange → tcp_N

Transform from ROS-Industrial `flange` to the currently active Yaskawa TCP.
