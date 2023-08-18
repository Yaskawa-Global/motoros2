<!--
SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
SPDX-FileCopyrightText: 2022-2023, Delft University of Technology

SPDX-License-Identifier: CC-BY-SA-4.0
-->

# Changelog

## Forthcoming

First release with official support for DX200 controllers.

New functionality:

- Added link-state monitor: LAN cable disconnects now cause immediate MotoROS2 shutdown ([#27](https://github.com/Yaskawa-Global/motoros2/pull/27))
- Added full DX200 support ([#49](https://github.com/Yaskawa-Global/motoros2/pull/49))
- Debug messages are now stamped with the time they are logged at ([#61](https://github.com/Yaskawa-Global/motoros2/pull/61))
- Debug log now includes full configuration, including defaults, instead of only parsed keys ([#81](https://github.com/Yaskawa-Global/motoros2/pull/81))
- `PANELBOX.LOG` and the M+ application list show the supported ROS 2 version ([#126](https://github.com/Yaskawa-Global/motoros2/pull/126))

Changes:

- Improved error messages returned by trajectory processing code (`FollowJointTrajectory` action server) ([#63](https://github.com/Yaskawa-Global/motoros2/pull/63))
- Added missing alarm description for `8011[55]` ([#70](https://github.com/Yaskawa-Global/motoros2/pull/70))
- `MotoROS_PlatformLib` updated to `0.2.11` ([#90](https://github.com/Yaskawa-Global/motoros2/pull/90))
- An active error no longer causes excessive debug logger traffic ([#105](https://github.com/Yaskawa-Global/motoros2/pull/105))
- `/start_traj_mode`: now returns improved error messages in case `INIT_ROS` could not be started ([#106](https://github.com/Yaskawa-Global/motoros2/pull/106))
- `/start_traj_mode`: don't attempt to enable servos and/or start `INIT_ROS` if there are active errors and/or alarms ([#115](https://github.com/Yaskawa-Global/motoros2/pull/115))
- Clarified installation procedure and troubleshooting with active FSU ([#130](https://github.com/Yaskawa-Global/motoros2/pull/130))

## 0.1.0 (2023-05-23)

First public release.

**Breaking change for beta1 participants**:

Users who previously participated in the closed `beta1` test must remove the `motoros2_interfaces` and `industrial_msgs` packages from their Colcon workspaces (from the `src`, `build` and `install` spaces), and follow the installation instructions in the [Build and installation](https://github.com/yaskawa-global/motoros2_client_interface_dependencies#build-and-installation) section in the `README` of the new `yaskawa-global/motoros2_client_interface_dependencies` package.
Breaking changes have been introduced in this version of MotoROS2, and failure to update and rebuild the workspace will lead to the inability to receive messages from MotoROS2 and/or crashes of client applications.

The old `motoros2_interfaces-beta1` repository will also be *archived* and will not be updated any more.

Changes:

- Significantly reduced shutdown time: MotoROS2 now takes approximately 4 seconds to shutdown after it detects the micro-ROS Agent has disconnected (after the default timeout, which is set to 10 seconds)
- Improved publisher performance (`RobotStatus`, `JointState`, TF)
- Migrated motion-related services to dedicated executor
- MotoROS2 now publishes all active alarms & errors on `robot_status`, as opposed to only the one raised most recently
- `reset_error` now also resets MotoROS2 internal errors
- Extended `FollowJointTrajectory` goal validation. MotoROS2 now requires:
  - positive and non-zero durations for all segments
  - monotonically increasing timestamps on all trajectory points
  - `positions` and `velocities` for all joints
  - zero velocity and acceleration for the last trajectory point
- Extended config file loader to verify names are present for all joints in a group when providing custom joint names
- Fixed incorrect `result_code` returned by `start_traj_mode`/`start_point_queue_mode`: services now return `MotionReadyEnum::READY` (ie: `1`) instead of `0` on success
- Updated Visual Studio solution to support Foxy builds
- Updated Visual Studio solution to support Galactic builds
- Migrated platform-specific functionality to a common library
- Migrated custom string handling code to corresponding `rcutils` functionality
- Switched to using M+ `libmicroros` configuration header
- Corrected capitalization of "micro-ROS" in the application info struct

Notices:

- Switched MotoROS2 to the `Apache-2.0` open-source license (from `BSD-3-Clause`)

## 0.0.15 (2022-11-30)

New functionality:

- Added a point queuing service.
  - The `queue_traj_point` service allows an indefinite number of points to be continuously queued for execution
  - A sample node is available to convert standard `FollowJointTrajectory` commands into the point queuing format
  - Note: must call `start_point_queue_mode` service before submitting points to the queue
- Ported the `select_tool` service from MotoROS1
  - Note: this service was renamed to `select_motion_tool` to better reflect its purpose and intended use

Changes:

- Agent disconnects will no longer cause MotoROS2 to stop INFORM (ie: non-ROS) controlled motion
- Only re-activate trajectory mode when in eco-mode if trajectory mode was already active
- Fix automatic INFORM job generation for controllers with multiple motion groups
- Base the micro-ROS client key on the MAC address of the controller's NIC instead of a random value
- Correct TF `base` frame location calculation for multi-robot configurations
- MotoROS2 now validates `time_from_start` on all points in a `JointTrajectory`
- Post fatal alarms on failed micro-ROS API calls
- Correct control flow to no longer starve other tasks of CPU during processing of (long) trajectories on controllers with multiple motion groups

## 0.0.14 (2022-09-09)

**BREAKING CHANGE:**

The names for the `publisher_qos` settings in `motoros2_config.yaml` have changed.

`best_effort` has changed to `sensor_data`

`reliable` has changed to `default`

If you are using QoS settings that are different than the default, then you MUST update your motoros2_config.yaml file with the new names.

Changes:

- Add support for remapping of ROS API names.
- Automatically generate the required INFORM job.
- Automatically validate the content of the INFORM job. (A custom job may be used if configured in the yaml.)
- Prevent MotoROS2 from starting multiple times.
- Include yamllint config in MotoROS2 beta distributions.
- Detect if too many groups are connected to the robot controller.

## 0.0.13 (2022-08-02)

Changes:

- Support configurable ROS domain ID
- Treat misconfigured Agent IP and port and ROS domain ID as fatal errors
- Correctly map Motoman<->ROS joint order for 7 axis robots
- Correct processing of trajectory goals with joints in arbitrary orders
- Properly abort a running trajectory if there is a robot alarm/error
- After configuration file update (via `CN102`): rename instead of remove the file
- Address cppcheck warnings and errors
- Include assertion sub code in fatal error debug log
- Fix torque/force reporting for station and base axes
- Include internal MotoROS2 errors in `RobotStatus::in_error`

## 0.0.12 (2022-06-27)

First `beta1` release.

Changes:

- Change application name to MotoROS2
- Utilize `motoros2_interfaces`
- Rename service `robot_enable` -> `start_traj_mode`
- Rename service `robot_disable` -> `stop_traj_mode`
- Change the alarm code numbers to avoid conflicts
- Add `reset_error` service
- Remove parameter server (unused)
- Fix deallocation of FJT feedback message (lock up when agent disconnects)

## 0.0.11 (2022-05-26)

Changes:

- Automatic `robot_enable` service if Energy Saving Function engages.
- Stop robot motion if connection to Agent is lost. (configurable in yaml)
- Custom joint names may be specified in yaml config file.
- FJT Action goals may have joints listed in any order.
- The yaml config file may be overwritten w/o initializing `SRAMDRV.DAT`
- Increase maximum trajectory length to 200 points.

## 0.0.10 (2022-05-19)

Changes:

- Fix array oob issue
- Add `robot_disable` service
- Add to configuration file:
  - Optional tf prefix
  - Optional namespace
  - Decide whether `tf` topic should be namespaced or absolute

## 0.0.9 (2022-05-17)

Changes:

- Add yaml configuration file
- Fix behavior when cancelling an active trajectory (smoother stop)
- Add tool0 to `/tf`
- Improve memory cleanup after agent disconnects from robot

## 0.0.8 (2022-05-06)

Changes:

- Fix undefined behavior when a point in the trajectory used a time from start that was not a whole second.

## 0.0.7 (2022-05-05)

Changes:

- This fixes a leak that will break the internal motion queue if the Agent disconnects/reconnects multiple times.

## 0.0.6 (2022-05-04)

Changes:

- Fix issue where 'large' trajectories are discarded.
- Fix parsing of joint names on incoming trajectory.
- Fix validation of time tolerance upon trajectory completion.
- If any tolerance causes a trajectory to 'fail', then include specific details about the failure in the return message.

## 0.0.5 (2022-04-20)

Changes:

- Provide reason code and message when rejecting a trajectory.
- Add timestamps to published topics.
- Change joint names to be 1-based.
- Fix initialization/startup error if pendant boots with active alarm.
- Fix multi-group trajectory processing.
- Prefix tcp TF frames with the group number.
- Improve detection of motion_ready when power-saving mode is engaged.
- Synchronize timestamps with the PC Agent.

## 0.0.4 (2022-04-06)

Changes:

- Fix detection of trajectory completion.
- Fix memory leak.
- Add some additional debug output messages.

## 0.0.3 (2022-03-17)

Changes:

- Services to control robot I/O.
- Qualify whether trajectory was successful or not.
- Allow agent on PC to disconnect and reconnect multiple times.

## 0.0.2 (2022-03-11)

Changes:

- `robot_enable` service implemented
