<!--
SPDX-FileCopyrightText: 2023, Yaskawa America, Inc.
SPDX-FileCopyrightText: 2023, Delft University of Technology

SPDX-License-Identifier: CC-BY-SA-4.0
-->

# Troubleshooting

## Debug log client

This repository contains a debugging log client script in the `tools` folder.
It listens for debug output from MotoROS2, displays it on the console and logs it to a file in a session log (`*.txt`).

The log client is written in Python, and can be run on Windows and Linux.
To start it, either run it manually (in a `cmd` or Bash session) using `python3 /path/to/debug_listener.py`, or on Windows by double-clicking the `debug_listener.cmd` file.

Note: you *must* have a recent version of Python 3 installed on the machine you intend to run the script on (the authors have tested the script with Python `3.8`, but newer versions are also expected to be compatible).
On Windows, `python.exe` *must* be on the `%PATH%` (ie: you should be able to start `python` in a `cmd` shell from any location, without having to specify the path).

By default, logs will be written to the current working directory (`CWD`).
On Windows the `.cmd` file changes the `CWD` to the location of the script before starting it, so all logs will end up in the `tools` folder.

When you encounter an issue using MotoROS2, please start the debug client script and keep it running in the background while you reproduce the issue.
Attach the log it produces and a copy of the `PANELBOX.LOG` from the robot's teach pendant to any support tickets you open on the [Issue tracker](https://github.com/yaskawa-global/motoros2/issues).

## MotoROS2 error and alarm codes

### ERROR 3200: NOP or END instruction not found

The robot controller requires all files use Windows-style end-of-line terminator.
This is `<carriage return><line feed>`.
Many common text editors in the Linux environment will automatically standardize all EOL terminators to be Unix-style; this removes the `<carriage return>`, resulting in the error message.

*Solution:*
Open this job file in a text editor that allows you to use Windows-style EOL (`<0xD><0xA>` or `<CR><LF>`) or, given that the job is relatively small, manually recreate the job using the programming pendant.
Another alternative is to copy the `.jbi` and `.dat` files under Windows.

### ERROR 4120: Concurrent I/O memory is full

This error can occur when attempting to load the `motoros2_config.yaml` configuration file.
It will occur if the SRAM drive was not initialized in the [installation procedure](../README.md#installation).

Please note that this error can also occur if the SRAM storage is full.
This can occur if another application, such as Simple Connect, is using the storage.
The files for the other application must be removed to make room for MotoROS2.

*How to initialize SRAM drive:*

 1. turn on the robot controller while holding the `{Main Menu}` key on the keypad to enter *Maintenance* mode.
 1. upgrade to *MANAGEMENT* security level by touching `[System Info]`→`[Security]` (default password is all `9`'s)
 1. touch `[File]`→`[Initialize]` and select `USER DEFINED FILES`
 1. select `SRAM RAM DRIVE` and initialize it

### Alarm: 1020[5]

*Example:*

```text
ALARM 1020
 MOTOPLUS APPLICATION LOAD ERROR
[5]
```

*Solution:*
This alarm occurs in conjunction with `1050` and `4207`.
It is caused due to missing symbols when building from source.

Make sure the MotoROS2 binary is linked against all the required libraries.
The provided Visual Studio solution should be set up correctly.
Pay special attention to any errors or warnings displayed by Visual Studio as part of the build process.

If the error persists, you may need to upgrade the robot controller system software.
For YRC1000, the controller must have `YAS2.80.00-00` or higher.
For YRC1000micro, the controller must have `YBS2.45.00-00` or higher.
Please contact Yaskawa technical support for assistance with upgrading the controller.

### Alarm: 1020[6]

```text
ALARM 1020
 MOTOPLUS APPLICATION LOAD ERROR
[6]
```

*Solution:*
This alarm occurs in conjunction with `1050` and `4207`.
There are two possible causes for this alarm.

 1. Corruption of the MotoROS2 binary
 1. Using MotoROS2 binary built for the wrong controller model

To remedy 1: download a fresh copy of the correct MotoROS2 binary and verify integrity of the file.

*Note*: make sure to check the binary copied to the removable medium (ie: usb stick or SD card), as corruption of files during transfer to such a medium is also possible.

To remedy 2: make sure that you are downloading the binary for your controller model.
For example, the YRC1000 binary will not run on an DX200.

### Alarm: 1050[1]

*Example:*

```text
ALARM 1050
 SET-UP PROCESS ERROR(SYSCON)
[1]
```

*Solution:*
If this alarm occurs in conjunction with `1020` (subcode `5` or `6`) and `4207` (subcode `1101`), please refer to [Alarm: 1020[5 - 6]](#alarm-10205).

### Alarm: 4207[1101]

*Example:*

```text
ALARM 4207
 SYSTEM ERROR(MOTION)
[1101]
```

*Solution:*
If this alarm occurs in conjunction with `1020` (subcode `5` or `6`) and `1050` (subcode `1`), please refer to [Alarm: 1020[5 - 6]](#alarm-10205).

### Alarm: 4430[6]

*Example:*

```text
ALARM 4430
 CPU COMMUNICATION ERROR
[6]
```

*Solution:*
This alarm occurs if you run the DX100 version of the `INIT_ROS` Inform job on a non-DX100 controller.
Please delete the `INIT_ROS` job from your pendant and replace it with the version for all other controllers.

### Alarm: 4997[4]

*Example:*

```text
ALARM 4997
 M-SAF DATA CRC UNMATCH
[4]
ALARM 8001
 Speed FB enabled, reboot now.
[10]
```

*Solution:*
This alarm occurs if the FSU is enabled when installing MotoROS2.
The MotoROS2 driver attempts to enable the *Speed Feedback* parameters, but is unable to change the required parameters due to FSU settings.
You must temporarily disable the CRC check for the *Speed Feedback* update to complete.

First, remove MotoROS2 from the controller:

- Boot the controller into *Maintenance* mode by powering on the controller while holding `{Main Menu}` on the keypad.
- Touch `[System Info]`→`[Security]` and upgrade to *MANAGEMENT* security level.
- Touch `[MotoPlus Apl]`→`[Delete]`.
- Select the MotoROS2 `.out` file and press `{Enter}` to confirm removal of the MotoROS2 driver.
- Turn off the robot controller.

Disable the CRC check:

- Turn on the robot controller and let it boot into *Normal Operation* mode (do not hold any keys).
- Switch the pendant to TEACH mode.
- From the main menu, touch `[System]`→`[Security]` and upgrade to *SAFETY* security level (the default password is all `5`s).
- Touch `[Setup]`→`[Function Enable]`.
- Navigate to *SAVE DATA CRC CHECK FUNC (FSU)*.
- Set this feature to *INVALID*.
- Turn off the robot controller.

Reinstall MotoROS2:

- Boot the controller into *Maintenance* mode by powering on the controller while holding `{Main Menu}` on the keypad.
- Reinstall the MotoROS2 `.out` file following the [regular installation procedure](https://github.com/Yaskawa-Global/motoros2#installation).
- Reboot the robot controller and let it boot into *Normal Operation* mode (do not hold any keys).
- After rebooting, you should see the alarm: `Alarm 8001 [10]` ("Speed FB enabled, reboot now").
- *Speed Feedback* has now permanently been enabled.

Re-enable the CRC check:

- Reboot once more and follow the steps in the *Disable the CRC check* section above to re-enable the *SAVE DATA CRC CHECK FUNC (FSU)* (be sure to set it to *VALID*).

No more alarms should be raised and MotoROS2 should now be installed.

If the alarm is raised again, save a copy of the `ALL.PRM` from the robot's teach pendant.
Open a new issue on the [Issue tracker](https://github.com/yaskawa-global/motoros2/issues), describe the problem and attach the `ALL.PRM` to the issue (alternatively: send it to the MotoROS2 developers in a private email).
Include a verbatim copy of the alarm text as seen on the teach pendant (alarm number and `[subcode]`).

### Alarm: 8001[10]

*Example:*

```text
ALARM 8001
 Speed FB enabled, reboot now
[10]
```

*Solution:*
This alarm is a one-time notice that feedback speed data has been automatically enabled.
This information will automatically be included with the feedback position data that is published as `JointState` messages.
When the alarm occurs, reboot the robot controller.
You should not see this alarm again.

### Alarm: 8003[100 - 111]

*Example:*

```text
ALARM 8003
 MotoROS2: Controller cfg invalid
[100]
```

*Solution:*
Your robot controller requires internal configuration changes to support the MotoROS2 driver.

For DX200: ensure the controller is updated to at least `DN2.44.00-00`.

For YRC1000 and YRC1000micro: ensure the controller is updated to at least `YAS2.80.00-00` (for YRC1000) and `YBS2.45.00-00` (for YRC1000micro).
If the system software version is below this, please contact Yaskawa Motoman for assistance with upgrading the controller.

Then boot the controller into *Maintenance* mode by holding `{Main Menu}` on the keypad.
Touch `[System Info]`→`[Security]` and upgrade to *MANAGEMENT* security level.
Touch `[System Info]`→`[Setup]` then select `OPTION FUNCTION`.
Cursor down to `MOTOMAN DRIVER` and set this to `USED`.
Now reboot the robot controller.

### Alarm: 8003[1]

*Example:*

```text
ALARM 8003
 MotoROS2 Cfg: Set RS000=2
[1]
```

*Solution:*
An internal parameter is not set properly in the robot controller.
Touch `[System Info]`→`[Security]` and upgrade to *MANAGEMENT* security level.
Then touch `[Parameter]`→`[RS]` and set the value of `RS000 = 2`.
Then reboot your robot controller.

### Alarm: 8003[2]

*Example:*

```text
ALARM 8003
 MotoROS2 Cfg: Set S2C541=0
[2]
```

*Solution:*
An internal parameter is not set properly in the robot controller.
Touch `[System Info]`→`[Security]` and upgrade to *MANAGEMENT* security level.
Then touch `[Parameter]`→`[S2C]` and set the value of `S2C541 = 0`.
Then reboot your robot controller.

### Alarm: 8003[3]

*Example:*

```text
ALARM 8003
 MotoROS2 Cfg: Set S2C542=0
[3]
```

*Solution:*
An internal parameter is not set properly in the robot controller.
Touch `[System Info]`→`[Security]` and upgrade to *MANAGEMENT* security level.
Then touch `[Parameter]`→`[S2C]` and set the value of `S2C542 = 0`.
Then reboot your robot controller.

### Alarm: 8003[4]

*Example:*

```text
ALARM 8003
 MotoROS2 Cfg: Set S2C1100=1
[4]
```

*Solution:*
An internal parameter is not set properly in the robot controller.
Touch `[System Info]`→`[Security]` and upgrade to *MANAGEMENT* security level.
Then touch `[Parameter]`→`[S2C]` and set the value of `S2C1100 = 1`.
Then reboot your robot controller.

### Alarm: 8003[5]

*Example:*

```text
ALARM 8003
 MotoROS2 Cfg: Set S2C1103=2
[5]
```

*Solution:*
An internal parameter is not set properly in the robot controller.
Touch `[System Info]`→`[Security]` and upgrade to *MANAGEMENT* security level.
Then touch `[Parameter]`→`[S2C]` and set the value of `S2C1103 = 2`.
Then reboot your robot controller.

### Alarm: 8003[6]

*Example:*

```text
ALARM 8003
 MotoROS2 Cfg: Set S2C1117=1
[6]
```

*Solution:*
An internal parameter is not set properly in the robot controller.
Touch `[System Info]`→`[Security]` and upgrade to *MANAGEMENT* security level.
Then touch `[Parameter]`→`[S2C]` and set the value of `S2C1117 = 1`.
Then reboot your robot controller.

### Alarm: 8003[7]

*Example:*

```text
ALARM 8003
 MotoROS2 Cfg: Set S2C1119=0 or 2
[7]
```

*Solution:*
An internal parameter is not set properly in the robot controller.
Touch `[System Info]`→`[Security]` and upgrade to *MANAGEMENT* security level.
Then touch `[Parameter]`→`[S2C]` and set the value of `S2C1119`.
A value of `2` will enable the telnet option to see any output messages from the MotoROS2 driver.
A value of `0` will disable the telnet option.
Reboot your robot controller after changing this parameter.

### Alarm: 8003[8]

*Example:*

```text
ALARM 8003
 MotoROS2 not compatible with PFL
[8]
```

or:

```text
ALARM 8003
 MotoROS2 not compatible with HC10
[8]
```

*Solution:*
Old versions of the MotoROS2 driver are not compatible with the human-collaborative (HC) robots.
You must update to v1.9.0 or newer.
Reboot the robot controller while holding `{Main Menu}` on the keypad to enter *Maintenance* mode.
Touch `[System Info]`→`[Security]` and upgrade to *MANAGEMENT* security level.
Then touch `[MotoPlus Apl]`→`[Delete]`.
Select the `MotoROS2.out` file and press `{Enter}` to confirm removal of the MotoROS2 driver.
Now follow the installation tutorial to install the latest version.

Additionally, the robot controller must meet a minimum version of system software.
For YRC1000, the controller must have `YAS2.80.00-00` or higher.
For YRC1000micro, the controller must have `YBS2.45.00-00` or higher.
Please contact Yaskawa technical support for assistance in upgrading the controller software.

### Alarm: 8003[9]

*Example:*

```text
ALARM 8003
 MotoROS2 not compatible with HC10
[9]
```

*Solution:*
Old versions of the MotoROS2 driver are not compatible with the human-collaborative (HC) robots.
You must update to v1.9.0 or newer.
Reboot the robot controller while holding `{Main Menu}` on the keypad to enter *Maintenance* mode.
Touch `[System Info]`→`[Security]` and upgrade to *MANAGEMENT* security level.
Then touch `[MotoPlus Apl]`→`[Delete]`.
Select the `MotoROS2.out` file and press `{Enter}` to confirm removal of the MotoROS2 driver.
Now follow the installation tutorial to install the latest version.

Additionally, the robot controller must meet a minimum version of system software.
For YRC1000, the controller must have `YAS2.80.00-00` or higher.
For YRC1000micro, the controller must have `YBS2.45.00-00` or higher.
Please contact Yaskawa technical support for assistance in upgrading the controller software.

### Alarm: 8003[11]

*Example:*

```text
ALARM 8003
 HC/Convey trking not compatible
[11]
```

*Solution:*
The Yaskawa Conveyor Tracking function is not compatible with HC robots.
Please contact Yaskawa technical support to have Conveyor Tracking function disabled.

### Alarm: 8010[2]

*Example:*

```text
ALARM 8010
 FAILED TO CREATE TASK
[2]
```

*Solution:*

This alarm is caused by having another MotoPlus application installed on the system which is using task-priority level `MP_PRI_IP_CLK_TAKE`.
There can only be one `MP_PRI_IP_CLK_TAKE` task installed on the system, and it is required for `MotoROS2`.

Either remove the additional MotoPlus application(s) installed on the controller, or modify the source code of the additional application so that it doesn’t use `MP_PRI_IP_CLK_TAKE`.

### Alarm: 8010[xx]

*Example:*

```text
ALARM 8010
 FAILED TO CREATE TASK
[0]
```

*Solution:*

1. Check the [setup instructions](../README.md#installation) to ensure that `MotoPlus FUNC.` and `MOTOMAN DRIVER` have been enabled.
2. Verify there are no other MotoPlus applications (`.out` file) installed on the robot controller.
3. Contact Yaskawa support to request that they enable additional MotoPlus tasks.

### Alarm: 8011[xx]

*Example:*

```text
ALARM 8011
 MotoROS2: Fatal Error
[0]
```

*Solution:*
Check the [setup instructions](../README.md#installation) to ensure that the robot controller is configured properly.

If the behavior persists, save a copy of the output of the [debug-listener script](#debug-log-client) and the `PANELBOX.LOG` from the robot's teach pendant.
Open a new issue on the [Issue tracker](https://github.com/yaskawa-global/motoros2/issues), describe the problem and attach `PANELBOX.LOG` and the debug log to the issue.
Include a verbatim copy of the alarm text as seen on the teach pendant (alarm number and `[subcode]`).

### Alarm: 8011[15]

*Example:*

```text
ALARM 8011
 Domain ID (x) invalid
[15]
```

*Solution:*
The `ros_domain_id` key must be configured in the `motoros2_config.yaml` configuration file.
The value must be between `0` and `101`.

After correcting the configuration, the [changes will need to be propagated to the Yaskawa controller](../README.md#updating-the-configuration).

### Alarm: 8011[16]

*Example:*

```text
ALARM 8011
 Missing Agent IP/Port
[16]
```

*Solution:*
The `agent_ip_address` and `agent_port_number` keys must be configured in the `motoros2_config.yaml` configuration file.
This must be the IP address of the PC that runs the micro-ROS Agent application.
The port must match the number that was used when [starting the agent](../README.md#the-micro-ros-agent) (default is `8888`).

After correcting the configuration, the [changes will need to be propagated to the Yaskawa controller](../README.md#updating-the-configuration).

### Alarm: 8011[17]

*Example:*

```text
ALARM 8011
 Agent IP on wrong subnet
[17]
```

*Solution:*
The `agent_ip_address` key in the `motoros2_config.yaml` configuration file is an address that is not reachable by the robot controller.

Options:

1. Modify the `agent_ip_address` key and specify an IP address that is on the robot's subnet.
   Now follow the instructions [to propagate the changes to the Yaskawa controller](../README.md#updating-the-configuration).
2. Modify the robot controller's IP so it is on the Agent's subnet.
3. Modify the robot controller's network settings to add a gateway which can reach the Agent's IP address.

Refer to the relevant Yaskawa Motoman documentation for information on how to change the controller's network configuration.

### Alarm: 8011[19]

*Example:*

```text
ALARM 8011
 Must enable ETHERNET function
[19]
```

*Solution:*
The ETHERNET function must be enabled for one (or both) LAN interface in the robot controller.
Please contact your local Yaskawa representative to request this function.

### Alarm: 8011[20]

*Example:*

```text
ALARM 8011
 MotoROS2 - Multiple Instances
[20]
```

*Solution:*
Multiple instances of the MotoROS2 MotoPlus application have been detected.
Only one instance may run on a single robot controller.

1. turn on the robot controller while holding the `{Main Menu}` key on the keypad to enter *Maintenance* mode.
1. touch `[MotoPlus APL]`→`[Delete]` and remove any extra `.out` files that have been installed.
1. check the `CN102` USB port on the CPU board inside the controller cabinet. If there are any `.out` files on the root of this drive, delete them.

### Alarm: 8011[21]

*Example:*

```text
ALARM 8011
 Must specify node name
[21]
```

*Solution:*
The `node_name` key must be configured in the `motoros2_config.yaml` configuration file.
The name must not be blank.

After correcting the configuration, the [changes will need to be propagated to the Yaskawa controller](../README.md#updating-the-configuration).

### Alarm: 8011[22]

*Example:*

```text
ALARM 8011
 Must specify INFORM job name
[22]
```

*Solution:*
The `inform_job_name` key must be configured in the `motoros2_config.yaml` configuration file.
The name must not be blank.

After correcting the configuration, the [changes will need to be propagated to the Yaskawa controller](../README.md#updating-the-configuration).

### Alarm: 8011[23 - 54]

*Example:*

```text
ALARM 8011
 MotoROS2: Fatal Error
 Failed adding ...
[xx]
```

or:

```text
ALARM 8011
 MotoROS2: Fatal Error
 Failed to init ...
[xx]
```

or:

```text
ALARM 8011
 MotoROS2: Fatal Error
 Failed creating ...
[xx]
```

Where `[xx]` is a subcode in the ranges `[23 - 54]` or `[56 - 58]`.

*Solution:*
These alarms are often caused by version incompatibilities between ROS 2 (on the client PC), micro-ROS (as part of MotoROS2) and/or the micro-ROS Agent.

Ensure only compatible versions are used.

As an example: the *Humble* version of MotoROS2 should only be used with ROS 2 *Humble* on the client PC and with the *Humble* version of the micro-ROS Agent.
Please also verify the client PC uses a version of ROS 2 that is [supported by MotoROS2](https://github.com/Yaskawa-Global/motoros2#general-requirements).

If the behavior persists, save a copy of the output of the [debug-listener script](#debug-log-client) and the `PANELBOX.LOG` from the robot's teach pendant.
Open a new issue on the [Issue tracker](https://github.com/yaskawa-global/motoros2/issues).
Describe the problem and include the following items:

- `PANELBOX.LOG`
- output from [debug-listener script](#debug-log-client) (complete and unedited)
- version of MotoROS2
- version of micro-ROS Agent
- version of ROS 2 on client PC
- copy of `motoros2_config.yaml` copied from the robot controller
- verbatim copy of the alarm text as seen on the teach pendant (alarm number and `[subcode]`).

### Alarm: 8011[55]

*Example:*

```text
ALARM 8011
 Empty custom joint name
[55]
ALARM 8013
 group: G, axis: A
[12]
```

Where `G` could be `r1`, `r2`, etc, or `b1`, `b2`, etc or `s1`, `s2`, etc and `A` indicates the specific axis for which the name is empty.

*Solution:*
Verify the `joint_names` dictionary in the `motoros2_config.yaml` configuration file contains names for all joints across all motion groups.

Joint names must not be blank and there must be an entry for every joint in a motion group.

*Note*: MotoROS2 does not support configuring custom joint names for a subset of joints and/or motion groups.
If only a subset of joints should be configured with a custom name, specify the default name for all other joints.

Refer to [FAQ: Can names of joints be changed?](faq.md#can-names-of-joints-be-changed) for more information about configuring custom joint names.

After correcting the configuration, the [changes will need to be propagated to the Yaskawa controller](../README.md#updating-the-configuration).

### Alarm: 8011[56 - 58]

Please refer to [Alarm: 8011[23 - 54]](#alarm-801123---54).

### Alarm: 8011[59]

*Example:*

```text
ALARM 8011
 Host on NIC check x
[59]
```

Where `x` is either `1` or `2`.

*Solution:*
This problem is often caused by the `agent_ip_address` key in the `motoros2_config.yaml` configuration file set to an address that is not reachable by the robot controller.

Options:

1. Modify the `agent_ip_address` key and specify an IP address that is on the robot's subnet.
   Now follow the instructions [to propagate the changes to the Yaskawa controller](../README.md#updating-the-configuration).
2. Modify the robot controller's IP so it is on the Agent's subnet.
3. Modify the robot controller's network settings to add a gateway which can reach the Agent's IP address.

Refer to the relevant Yaskawa Motoman documentation for information on how to change the controller's network configuration.

### Alarm: 8011[60 - 62]

*Example:*

```text
ALARM 8011
 Must enable ETHERNET function
[60 - 62]
```

*Solution:*
The ETHERNET function must be enabled for one (or both) LAN interface in the robot controller.
Please contact your local Yaskawa representative to request this function.

### Alarm: 8011[63]

*Example:*

```text
ALARM 8011
 Inv. motion type: N (axis: A)
[63]
```

Where `N` is an integer indicating the configured motion type and `A` indicates the specific axis for which the motion type is invalid.

*Solution:*
MotoROS2 encountered an illegal value when evaluating base track coordinates for TF broadcast.

This alarm should not be raised during normal usage of MotoROS2.

Save a copy of the output of the [debug-listener script](#debug-log-client) and the `PANELBOX.LOG` from the robot's teach pendant.
Open a new issue on the [Issue tracker](https://github.com/yaskawa-global/motoros2/issues), describe the problem and attach `PANELBOX.LOG` and the debug log to the issue.
Include a verbatim copy of the alarm text as seen on the teach pendant (alarm number and `[subcode]`).

### Alarm: 8011[64]

*Example:*

```text
ALARM 8011
 Must enable ETHERNET function
[64]
```

*Solution:*
The ETHERNET function must be enabled for one (or both) LAN interface in the robot controller.
Please contact your local Yaskawa representative to request this function.

### Alarm: 8011[65]

*Example:*

```text
ALARM 8011
 Enable LAN port 1 for debug
[65]
```

*Solution:*
The ETHERNET function must be enabled for the LAN interface that was specified in the config file.
Either change the interface specified in the config file to a LAN interface that is enabled, or enable the corresponding LAN interface on the controller.
Please contact your local Yaskawa representative to request the ETHERNET function if it is not enabled.

### Alarm: 8012[xx]

*Example:*

```text
ALARM 8012
 OUT OF MEMORY
[0]
```

*Solution:*
Verify there are no other MotoPlus applications (`.out` file) installed on the robot controller.

If the behavior persists, save a copy of the output of the [debug-listener script](#debug-log-client) and the `PANELBOX.LOG` from the robot's teach pendant.
Open a new issue on the [Issue tracker](https://github.com/yaskawa-global/motoros2/issues), describe the problem and attach `PANELBOX.LOG` and the debug log to the issue.
Include a verbatim copy of the alarm text as seen on the teach pendant (alarm number and `[subcode]`).

### Alarm: 8013[0]

*Example:*

```text
ALARM 8013
 Missing MotoROS2 cfg file
[0]
```

*Solution:*
Follow the [setup instructions](../README.md#installation) to load the `motoros2_config.yaml` configuration file.
Double check the setting of the `S2C` parameters.
Additionally, on YRC-generation controllers, be sure to follow the steps for initializing SRAM.

### Alarm: 8013[1]

*Example:*

```text
ALARM 8013
 Fail to copy config file
[1]
ALARM 8013
 Set S2C1102=2; Init SRAMDRV.DAT
[1]
```

*Solution:*
Set robot parameter `S2C1102 = 2` and follow the [setup instructions](../README.md#installation) to initialize `SRAMDRV.DAT`.

### Alarm: 8013[2]

*Example:*

```text
ALARM 8013
 Fail to copy config file
[2]
ALARM 8013
 Initialize SRAMDRV.DAT
[2]
```

*Solution:*
Follow the [setup instructions](../README.md#installation) to initialize `SRAMDRV.DAT`.

### Alarm: 8013[3]

*Example:*

```text
ALARM 8013
 Invalid BOOL in motoros2_config
[3]
```

*Solution:*
A key in the `motoros2_config.yaml` configuration file is set to an invalid value.
Boolean values should be set to either `true` or `false`.

After correcting the configuration, the [changes will need to be propagated to the Yaskawa controller](../README.md#updating-the-configuration).

### Alarm: 8013[4]

*Example:*

```text
ALARM 8013
 Invalid QOS in motoros2_config
[4]
```

*Solution:*
A key in the `motoros2_config.yaml` configuration file is set to an invalid value.
QoS values should be set to either `sensor_data` or `default`.

After correcting the configuration, the [changes will need to be propagated to the Yaskawa controller](../README.md#updating-the-configuration).

### Alarm: 8013[5]

*Example:*

```text
ALARM 8013
 Invalid executor_sleep_period
[5]
```

*Solution:*
The `executor_sleep_period` key in the `motoros2_config.yaml` configuration file is set to an invalid value.
This must be set to an integer value between `1` and `100` milliseconds.

After correcting the configuration, the [changes will need to be propagated to the Yaskawa controller](../README.md#updating-the-configuration).

### Alarm: 8013[6]

*Example:*

```text
ALARM 8013
 Invalid fb_publisher_period
[6]
```

*Solution:*
The `feedback_publisher_period` key in the `motoros2_config.yaml` configuration file is set to an invalid value.
This must be set to an integer value between `1` and `100` milliseconds.

After correcting the configuration, the [changes will need to be propagated to the Yaskawa controller](../README.md#updating-the-configuration).

### Alarm: 8013[7]

*Example:*

```text
ALARM 8013
 Invalid status_monitor_period
[7]
```

*Solution:*
The `controller_status_monitor_period` key in the `motoros2_config.yaml` configuration file is set to an invalid value.
This must be set to an integer value between `1` and `100` milliseconds.

After correcting the configuration, the [changes will need to be propagated to the Yaskawa controller](../README.md#updating-the-configuration).

### Alarm: 8013[8 - 9]

*Example:*

```text
ALARM 8013
 Too many remap rules
[8]
```

*Solution:*
The `motoros2_config.yaml` configuration file contains too many remap rules.
A maximum of 16 rules may be specified.

After correcting the configuration, the [changes will need to be propagated to the Yaskawa controller](../README.md#updating-the-configuration).

### Alarm: 8013[10 - 11]

*Example:*

```text
ALARM 8013
 Invalid remap rule format
[10]
```

*Solution:*
The `motoros2_config.yaml` configuration file contains a remap rule that has been specified in the wrong format.

Example format: `remap_rules: "joint_states:=my_joint_states read_single_io:=io/read_single"`

After correcting the configuration, the [changes will need to be propagated to the Yaskawa controller](../README.md#updating-the-configuration).

### Alarm: 8013[12]

*Example:*

```text
ALARM 8011
 Empty custom joint name
[55]
ALARM 8013
 group: G, axis: A
[12]
```

*Solution:*
This alarm occurs in conjunction with `8011` (subcode `55`), please refer to [Alarm: 8011[55]](#alarm-801155).

### Alarm: 8013[13]

*Example:*

```text
ALARM 8013
 Invalid UserLan port in cfg
[13]
```

*Solution:*
The `userlan_monitor_port` key in the `motoros2_config.yaml` configuration file is set to an invalid value.
LAN port monitoring will be disabled for this session.

On YRC1000 and YRC1000u, this must be set to either `USER_LAN1` or `USER_LAN2`.

No other values are supported.

Example: `userlan_monitor_port: USER_LAN1`.

After correcting the configuration, the [changes will need to be propagated to the Yaskawa controller](../README.md#updating-the-configuration).

### Alarm: 8013[14]

*Example:*

```text
ALARM 8013
 UserLan port detect failed
[14]
```

*Solution:*
Because `userlan_monitor_enabled` was set to `true` but no value was supplied for `userlan_monitor_port`, MotoROS2 attempted to auto-detect the LAN port to monitor.
This auto-detection failed, and as a result MotoROS2 has disabled LAN port monitoring for this session.

To rule out a transient failure, reboot the controller.

If the alarm is raised again, and if auto-detection is not needed or desired, make sure `userlan_monitor_port` is not commented out (ie: does not have a `#` at the start of the line) and set it to an appropriate value.

On YRC1000 and YRC1000u, set it to either `USER_LAN1` or `USER_LAN2`, depending on which LAN port is used to connect the controller to the PC running the micro-ROS Agent application.

If auto-detection is to be used, verify `agent_ip_address` is set to an IP that can be reached by MotoROS2 over the LAN port which is connected to the PC running the micro-ROS Agent application (either directly, or via a default gateway configured on the controller).

After correcting the configuration, the [changes will need to be propagated to the Yaskawa controller](../README.md#updating-the-configuration).

If the behavior persists, save a copy of the output of the [debug-listener script](#debug-log-client) and the `PANELBOX.LOG` from the robot's teach pendant.
Open a new issue on the [Issue tracker](https://github.com/yaskawa-global/motoros2/issues), describe the problem and attach `PANELBOX.LOG` and the debug log to the issue.
Include a verbatim copy of the alarm text as seen on the teach pendant (alarm number and `[subcode]`).

### Alarm: 8013[15]

*Example:*

```text
ALARM 8013
 LAN monitor fail
[15]
```

*Solution:*
MotoROS2 tried to monitor the LAN port configured in `userlan_monitor_port`, but was unable to retrieve link status and as a result has disabled LAN port monitoring for this session.

To rule out a transient failure, reboot the controller.

If the alarm is raised again, make sure `userlan_monitor_port` is set to the correct value (ie: the LAN port used to connect the controller to the PC running the micro-ROS Agent application) and is not commented out (ie: does not have a `#` at the start of the line).

If the configuration file has to be updated, the [changes will need to be propagated to the Yaskawa controller](../README.md#updating-the-configuration).

If the behavior persists, save a copy of the output of the [debug-listener script](#debug-log-client) and the `PANELBOX.LOG` from the robot's teach pendant.
Open a new issue on the [Issue tracker](https://github.com/yaskawa-global/motoros2/issues), describe the problem and attach `PANELBOX.LOG` and the debug log to the issue.
Include a verbatim copy of the alarm text as seen on the teach pendant (alarm number and `[subcode]`).

### Alarm: 8013[16]

*Example:*

```text
ALARM 8013
 No calibration: invalid TF
[16]
```

*Solution:*
MotoROS2 was unable to load any kinematic calibration data during initialisation.
This calibration data is used to update the origins of TF frames broadcast by MotoROS2 if TF broadcasts are enabled.

Without (valid) calibration data the origins of distinct TF trees might overlap (see [Incorrect transform tree origin with multi-robot setups](../README.md#incorrect-transform-tree-origin-with-multi-robot-setups)), creating potentially dangerous situations when that TF data is consumed by applications which for example use it for collision avoidance motion planning.

This alarm is only raised if all of the following conditions are true:

1. MotoROS2 is configured to broadcast TF (`publish_tf` is `true`)
1. the controller is configured with multiple motion groups (ie: multiple robots)
1. none of the motion groups have been calibrated against each other

The alarm can be prevented by performing (robot) group calibration or by disabling TF broadcasts (set `publish_tf` to `false`).

In case TF broadcasting for uncalibrated multi-group systems is still desired, the alarm can be disabled by setting the `ignore_missing_calib_data` item in the MotoROS2 configuration to `true` (default is: `false`).
When disabling the alarm, make absolutely sure consuming applications are capable of disambiguating potentially overlapping TF trees.

In case of any updates to the configuration file, [changes will need to be propagated to the Yaskawa controller](../README.md#updating-the-configuration).

In case the alarm is still raised after calibration was performed, TF broadcasting was disabled and/or the alarm was disabled, save a copy of the output of the [debug-listener script](#debug-log-client) and the `PANELBOX.LOG` and `RBCALIB.DAT` files from the robot's teach pendant.
Open a new issue on the [Issue tracker](https://github.com/yaskawa-global/motoros2/issues), describe the problem and attach `PANELBOX.LOG`, `RBCALIB.DAT` and the debug log to the issue.
Include a verbatim copy of the alarm text as seen on the teach pendant (alarm number and `[subcode]`).

### Alarm: 8013[17]

*Example:*

```text
ALARM 8013
 Bad UserLan debug port in cfg
[17]
```

*Solution:*
The `userlan_debug_broadcast_port` key in the `motoros2_config.yaml` configuration file is set to an invalid value.
Debug broadcasting will be disabled for this session.

On YRC1000 and YRC1000u, this must be set to either `USER_LAN1` or `USER_LAN2`.

No other values are supported.

Example: `userlan_debug_broadcast_port: USER_LAN1`.

After correcting the configuration, the [changes will need to be propagated to the Yaskawa controller](../README.md#updating-the-configuration).

### Alarm: 8014[0]

*Example:*

```text
ALARM 8014
 MotoROS2 failed to validate job
[0]
```

*Solution:*
Follow the [setup instructions](../README.md#installation) to ensure that all robot parameters are set correctly.
In particular, be sure to check `S2C1102 = 2`.

### Alarm: 8014[1]

*Example:*

```text
ALARM 8014
 Invalid MotoROS2 job detected
[1]
```

*Solution:*
An invalid MotoROS2 INFORM job was detected (the job contains INFORM statements MotoROS2 did not expect).

If MotoROS2 should use the custom job, instead of a default, auto-generated one, please update the `allow_custom_inform_job` field in the yaml configuration file.
After correcting the configuration, the [changes will need to be propagated to the Yaskawa controller](../README.md#updating-the-configuration).

If there is no need to use a customised job, delete the existing job and reboot to allow the default job to be generated.

If the alarm is posted again after restarting the controller, make sure to allow sufficient time for the controller to properly delete the job (flushing changes to the file system may take some time).
Allow for at least 20 seconds between deleting the job and restarting the controller.

### Alarm: 8014[2]

*Example:*

```text
ALARM 8014
 MotoROS2 failed to generate job
[2]
```

*Solution:*
There was a failure when generating the default INFORM job.
Please obtain the standard job from the Github repository.

### Alarm: 8014[3]

*Example:*

```text
ALARM 8014
 MotoROS2 failed to load job
[3]
```

*Solution:*
There was a failure when generating the default INFORM job.
Please obtain the standard job from the Github repository and load it using the teach pendant.

### Alarm: 8015[0]

*Example:*

```text
ALARM 8015
 Failed to parse RBCALIB.DAT
[0]
```

*Solution:*
Check the following robot parameters on the teach pendant and make sure they are set to the values shown here:

- S2C1103 = 2
- S2C1117 = 1

If that does not resolve the issue, please contact Yaskawa technical support for assistance.
Include a copy of the `ALL.PRM` and `CMOS.BIN` from your robot controller.

### Alarm: 8015[1 - 4]

*Example:*

```text
ALARM 8015
 Failed to parse RBCALIB.DAT
[1]
```

*Solution:*
Open a new ticket on the MotoROS2 [Issue tracker](https://github.com/yaskawa-global/motoros2/issues).
Please include a copy of the `RBCALIB.DAT` from your robot controller along with the output from the [Debug log client](#debug-log-client).

### Alarm: 8016[0]

*Example:*

```text
ALARM 8016
 Set job-cycle to AUTO
[0]
```

*Solution:*
The job cycle is currently set to `STEP` and MotoROS2 was unable to automatically change it to `AUTO`.
This will prevent the `INIT_ROS` from operating continuously and will prevent the software from accepting any incoming trajectories.

 1. upgrade to *MANAGEMENT* security level by touching `[System Info]`→`[Security]` (default password is all `9`'s)
 1. touch `[Job]`→`[Cycle]`
 1. change `WORK SELECT` to `AUTO`
 1. touch `[Setup]`→`[Operate Cond.]`
 1. change `CYCLE SWITCH IN REMOTE MODE` to `AUTO`

If the problem persists, verify that the `CIOPRG.LST` ladder program is not writing to `#40050 - #40052`.
Please contact Yaskawa Technical Support for assistance if you are not familiar with the Concurrent I/O Ladder Program.
