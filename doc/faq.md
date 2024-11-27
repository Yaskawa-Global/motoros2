<!--
SPDX-FileCopyrightText: 2023, Yaskawa America, Inc.
SPDX-FileCopyrightText: 2023, Delft University of Technology

SPDX-License-Identifier: CC-BY-SA-4.0
-->

# Frequently Asked Questions

## Does MotoROS2 require a MotoPlus SDK license?

*Deployment* of MotoROS2 on a supported Yaskawa Motoman robot controller does **not** require a MotoPlus SDK license.

MotoROS2 *development* (ie: building MotoROS2 from source) does require a MotoPlus SDK license.

Development of a ROS 2 application that interfaces with MotoROS2 does **not** require a MotoPlus SDK license.

## Can MotoROS2 be installed on DX100 or FS100 controllers?

MotoROS2 only supports DX200, YRC1000 and YRC1000micro controllers.

## Does MotoROS2 support all Yaskawa Motoman robots?

MotoROS2 is manipulator agnostic, and as such is expected to be compatible with all Yaskawa Motoman robots which can be used with the controller series supported by MotoROS2 (ie: DX200, YRC1000 and YRC1000micro).

## Can MotoROS2 be used with SDA robots?

The current version of MotoROS2 is only compatible with DX200, YRC1000 and YRC1000micro controllers.
These controllers are not used with SDA robots (ie: dual-arm robots), and as such, MotoROS2 does not support SDA robots.

Note: the limitation is not MotoROS2, but the controller.
Users of SDA robots paired with a DX200 or YRC1000 (or newer) wishing to use MotoROS2 are encouraged to contact the developers.

## Can MotoROS2 be used with Scara robots?

The current version of MotoROS2 is compatible with DX200, YRC1000 and YRC1000micro controllers.
Provided the robot is used with a DX200 or YRC1000 generation controller, it should be supported by MotoROS2.

## Can MotoROS2 be used with Delta robots?

The current version of MotoROS2 is only compatible with DX200, YRC1000 and YRC1000micro controllers.
These controllers are not used with Delta robots, and as such, MotoROS2 does not support Delta robots.

Note: the limitation is not MotoROS2, but the controller.
Users of Delta robots paired with a DX200 or YRC1000 (or newer) wishing to use MotoROS2 are encouraged to contact the developers.

## Can MotoROS2 be used with palletizing robots?

The current version of MotoROS2 is compatible with DX200, YRC1000 and YRC1000micro controllers.
Provided the palletizing robot is used with a DX200 or YRC1000 generation controller, it should be supported by MotoROS2.

## Can MotoROS2 be used with welding robots?

The current version of MotoROS2 is compatible with DX200, YRC1000 and YRC1000micro controllers.
Provided the robot is used with a DX200 or YRC1000 generation controller, the *motion* of the arm can be controlled by MotoROS2.

Note: MotoROS2 does not have any process-controls.
It cannot be used to control the welding power source.
Nor is it compatible with UWI or Weldcom2.

To control the power source, the ROS 2 application must interface to the welder directly.

## Can multiple MotoROS2 instances run on the same network?

MotoROS2 can be deployed to as many Yaskawa Motoman robot controllers as required.
By default, each instance will use the same names for topics, services and actions.
This can be resolved by *namespacing* (see [Can MotoROS2 run in a namespace?](#can-motoros2-run-in-a-namespace)), which causes each controller to use unique names for the resources it makes available (ie: topics, services and actions).

## Can multiple MotoROS2 instances run on the same controller?

Only a single instance of MotoROS2 can be run on a controller.

## Does MotoROS2 support multiple motion groups?

MotoROS2 supports Yaskawa Motoman controllers with multiple motion groups.
The maximum number of groups supported is `8`.

MotoROS2 will include all joints from all groups in `JointState` messages, and all joints from all groups are expected to be present in `FollowJointTrajectory` action goals (see also [No support for partial goals](../README.md#no-support-for-partial-goals)).

## Can MotoROS2 be used with ROS 1?

MotoROS2 is not directly compatible with ROS 1, as it uses ROS 2 technology (ie: the RCL and RCLC layers) and ROS 2 communication infrastructure (micro-ROS `rmw_microxrcedds`), which are not compatible with ROS 1.

Bridging solutions between ROS 1 and ROS 2 could be considered (such as the `ros1_bridge` package or eProsima's *Integration Service*), but these have not been tested, nor does Yaskawa Motoman provide support for such configurations.

## How do I build MotoROS2 in my Colcon workspace?

As MotoROS2 is not a ROS 2 package, but a MotoPlus application, it cannot be built in a Colcon workspace.
Users are also not required to build MotoROS2 in order to use it.
Instead, the [installation procedure](../README.md#installation) relies on pre-built binaries made available by Yaskawa Motoman (on the [Releases page](../README.md#downloading-the-files)).
The companion package `motoros2_interfaces` can be built in a Colcon workspace though, and should be treated as any other ROS 2 package providing interface definition files (ie: message, service and action definitions).

## How do I start MotoROS2 as part of a launch file?

MotoROS2 is always active as long as the controller is on-line, and as such does not need to be started as part of a launch file.

**Note**: the [micro-ROS Agent](../README.md#the-micro-ros-agent) is *not* automatically started, and must be active for MotoROS2 to be able to interact with regular ROS 2 applications.

Starting the Agent as part of a launch file would be possible, as it's a ROS 2 package.
The package name would be `micro_ros_agent`, the name of the node would also be `micro_ros_agent`.
Make sure to provide the required/expected arguments (refer to [micro-ROS Agent](../README.md#the-micro-ros-agent)), or reuse the `.launch.py` provided by the `micro_ros_agent` package itself.

However, as there is no required startup order between the Agent and MotoROS2, the developers would recommend to treat the Agent as a system service, and manage it independently from the ROS 2 application.
Using the Agent's [Docker image](../README.md#using-docker-linux-only) makes this quite easy on Linux.

## Does MotoROS2 support ROS parameters?

MotoROS2 does not currently use ROS parameters, although it does include an initial implementation of a parameter server.

This is due to a limitation in RCL on micro-ROS, which does not support string parameters.

## Is MotoROS2 a life-cycle node?

The current implementation does not yet use life-cycle nodes.
This may change in the future, as we are interested in the advantages they bring for life-cycle coordination.

## Can MotoROS2 run in a namespace?

Yes, namespacing all topics, services and actions MotoROS2 exposes is supported.

Edit the `motoros2_config.yaml` file and set the `node_namespace` item to the desired namespace.
Follow [Updating the configuration](../README.md#updating-the-configuration) to propagate this change to MotoROS2.

## Is MotoROS2 compatible with MoveIt?

As MotoROS2 publishes `JointState` messages and accepts `FollowJointTrajectory` action goals, MoveIt can be configured to interface with MotoROS2.

Refer to [Usage - With MoveIt](../README.md#with-moveit) for information on how to update the configuration of a MoveIt configuration package to work with MotoROS2.

## How can individual motion groups be controlled?

The current implementation of MotoROS2 does not support addressing individual motion groups on controllers with multiple motion groups.

Refer to [No support for asynchronous motion](../README.md#no-support-for-asynchronous-motion) for some more information.

## Can names of joints be changed?

The names used by MotoROS2 for joints (in `JointState` messages for instance) by default follow the naming scheme `joint_M` for single-group and `group_N/joint_M` for multi-group (with `N` and `M` ∈ ℕ⁺: 1, 2, 3, 4, ...).

Custom joint names can be configured through the configuration file, specifically the `joint_names` key.
This key is a list-of-lists, with each top-level list corresponding to a group, and each entry inside each list corresponding to the joint in that group.

Order of the groups follows Motoman order: first *robots*, then *base axes* and finally *station axes*.
For a controller with three groups, of which groups 1 and 2 are robots, and 3 contains a station axis, the two robots (groups 1 and 2) are listed first in `joint_names`, followed by the group with the station axis (group 3):

```yaml
joint_names:
- [robot_1_axis_1, robot_1_axis_2, ...]
- [robot_2_axis_1, ...]
- [station_axis_1]
```

For a controller with 3 groups: a robot (`R1`), a track (`B1`) and a single axis positioner/turn-table (`S1`):

```yaml
joint_names:
- [joint_s, joint_l, joint_u, joint_r, joint_b, joint_t]
- [track_s]
- [turn_table_s]
```

Note the use of the brand-specific joint names as used by `motoman_driver` in ROS 1 (this naming convention is no longer needed, nor recommended).
This would allow re-using the `xacro:macro`s as provided by the ROS 1 Motoman robot support packages for the robot model.

Order of joints also follows Motoman order, so for a single group system, but with 7 joints:

```yaml
joint_names:
- [r1/joint_s, r1/joint_l, r1/joint_u, r1/joint_r, r1/joint_b, r1/joint_t, r1/joint_e]
```

Note the `E` joint at the end of the list, despite being physically located after the `L` joint, and the use of the group IDs as prefixes.

After changing the configuration, the [changes will need to be propagated to the Yaskawa controller](../README.md#updating-the-configuration).

## How to avoid TF frame name clashes

MotoROS2 supports a `tf_frame_prefix` parameter, similar to the same parameter supported by the ROS 2 `robot_state_publisher` node.
This parameter can be used to prevent clashes with other nodes publishing transforms MotoROS2 also publishes (as by default, MotoROS2 will publish on the global `/tf` topic, similar to other ROS 2 nodes).

As an example, `tf_frame_prefix` could be set to `robot1/`, to prefix all frame names coming from a robot mounted "on the left" (in a dual-arm setup for instance). Resulting frame names would include `robot1/r1/base`, `robot1/r1/flange`, `robot1/r1/tool0` and `robot1/r1/tcp_0`.

After changing the configuration, the [changes will need to be propagated to the Yaskawa controller](../README.md#updating-the-configuration).

## Is MotoROS2 compatible with ros2_control?

MotoROS2 is currently not directly compatible with `ros2_control` (as in: none of the controllers `ros2_control` provides can directly be used with MotoROS2, nor can `ros2_control` controllers be loaded *into* MotoROS2).
Future development may introduce a mode which could be made compatible with `ros2_control`, but this is currently not on the [roadmap](../README.md#provisional-roadmap).

## Does MotoROS2 support SROS2?

MotoROS2 currently does not support any of the (security) features provided by SROS2.
This is due to the used RMW (ie: `rmw_microxrcedds`) not supporting those features.

## Is the micro-ROS Agent always needed?

As MotoROS2 is using micro-ROS as its RMW (technically: `rmw_microxrcedds`), the micro-ROS Agent is always needed for it to be able to communicate with nodes using other RMWs.

## Can non-ROS applications communicate with MotoROS2?

This has not been tested, but technically should be possible, provided they use DDS.
Plain DDS applications can be written to be able to communicate with ROS 2 nodes, which requires using a compatible DDS RMW, using ROS 2 IDL files, adhering to ROS 2 naming schemes and ROS 2 name mangling.
The ROS 2 community has examples for how to do this with regular ROS 2 nodes, and it is expected MotoROS2 would behave similarly.

## Can non-DDS applications communicate with MotoROS2?

There is no support for interaction with MotoROS2 from non-DDS applications at this time.
