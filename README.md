<!--
SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
SPDX-FileCopyrightText: 2022-2023, Delft University of Technology

SPDX-License-Identifier: CC-BY-SA-4.0
-->

<h1><img src="doc/img/logo.png" alt="MotoROS2 logo" style="width: 70%" /></h1>

![YRC1000: supported](https://img.shields.io/badge/YRC1000-supported-blue.svg)
![YRC1000micro: supported](https://img.shields.io/badge/YRC1000micro-supported-blue.svg)
![DX200: supported](https://img.shields.io/badge/DX200-supported-blue.svg)
![FS100: not supported](https://img.shields.io/badge/FS100-not%20supported-inactive.svg)
![DX100: not supported](https://img.shields.io/badge/DX100-not%20supported-inactive.svg)

[![license - Apache-2.0](https://img.shields.io/:license-Apache--2.0-yellowgreen.svg "MotoROS2 itself is licensed under the Apache-2.0 license. Please see the LICENSES directory for additional license files")](https://opensource.org/licenses/Apache-2.0)
[![support level: consortium / vendor](https://img.shields.io/badge/support%20level-consortium%20/%20vendor-brightgreen.svg)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

![Version: 0.1.3](https://img.shields.io/badge/version-0.1.3-informational.svg)

## Overview

This repository hosts the source code of the MotoROS2 project.
Together with the [motoros2_interfaces](https://github.com/yaskawa-global/motoros2_interfaces) repository, MotoROS2 is all that's needed to allow a Yaskawa Motoman industrial robot controller to be controlled by a ROS 2 application.

The following sections document how to download, install, configure, use and troubleshoot MotoROS2.

## Quickstart

- check [prerequisites](#general-requirements)
- download the [latest release](#download) and extract onto USB stick
- edit the [configuration file](#configuration), at least Agent IP and port
- insert USB stick into pendant
- *Maintenance* mode: load the MotoPlus application [onto the controller](#installation):
  - DX200: `mr2_dx2_*.out`
  - YRC1000: `mr2_yrc1_*.out`
  - YRC1000micro: `mr2_yrc1m_*.out`
- *Online* mode:
  - DX200: insert USB stick with `motoros2_config.yaml` into `CN106` in the controller cabinet
  - YC1000[micro]: load `motoros2_config.yaml` via `USER DEFINED FILES`
- start an instance of the [micro-ROS Agent](#the-micro-ros-agent) on a PC
- reboot controller
- verify [MotoROS2 is running](#verifying-successful-installation)
- read about [known issues and limitations](#known-issues--limitations)
- [troubleshoot](#troubleshooting) problems

## TOC

- [Quickstart](#quickstart)
- [General Requirements](#general-requirements)
  - [Checking the system software version](#checking-the-system-software-version)
- [Option Function compatibility](#option-function-compatibility)
- [Download](#download)
  - [Downloading the files](#downloading-the-files)
  - [Extracting the files](#extracting-the-files)
  - [Verifying integrity of the download](#verifying-integrity-of-the-download)
- [Configuration](#configuration)
  - [Verifying YAML correctness](#verifying-yaml-correctness)
- [Example INFORM jobs](#example-inform-jobs)
- [Installation](#installation)
  - [Checking MotoPlus configuration](#checking-motoplus-configuration)
  - [DX200, YRC1000 and YRC1000micro](#dx200-yrc1000-and-yrc1000micro)
- [Building from source](doc/build_from_source.md)
- [Updating the configuration](#updating-the-configuration)
- [The micro-ROS Agent](#the-micro-ros-agent)
  - [Using Docker (Linux Only)](#using-docker-linux-only)
  - [Using the ROS 2 package](#using-the-ros-2-package)
- [Verifying successful installation](#verifying-successful-installation)
- [Usage](#usage)
  - [Basic usage with ROS](#basic-usage-with-ros)
  - [Commanding motion](#commanding-motion)
  - [With MoveIt](#with-moveit)
- [ROS API](doc/ros_api.md)
- [Default QoS settings](#default-qos-settings)
  - [Publisher QoS](#publisher-qos)
  - [Service server QoS](#service-server-qos)
  - [Action server QoS](#action-server-qos)
- [Known issues & limitations](#known-issues--limitations)
  - [Functional Safety Unit interaction](#functional-safety-unit-interaction)
  - [Only FastDDS is supported](#only-fastdds-is-supported)
  - [Maximum length of trajectories](#maximum-length-of-trajectories)
  - [No support for asynchronous motion](#no-support-for-asynchronous-motion)
  - [No support for partial goals](#no-support-for-partial-goals)
  - [Upper limit to publishing frequency](#upper-limit-to-publishing-frequency)
  - [Incorrect transform tree origin with multi-robot setups](#incorrect-transform-tree-origin-with-multi-robot-setups)
  - [Memory leak](#memory-leak)
- [Provisional roadmap](#provisional-roadmap)
- [Frequently Asked Questions](doc/faq.md)
- [Troubleshooting](doc/troubleshooting.md)

## General Requirements

The following general requirements must be met in order to be able to use MotoROS2 with a Yaskawa Motoman robot controller:

- controller series: DX200, YRC1000, or YRC1000micro
- minimum versions of system software:
  - `DN2.44.00-00` for DX200
  - `YAS2.80.00-00` for YRC1000
  - `YBS2.45.00-00` for YRC1000micro
- the controller must have a correctly configured network connection:
  - DX200 and YRC1000micro: `LAN`
  - YRC1000: either `LAN2` or `LAN3`
- ROS 2 version: Foxy, Galactic, Humble, or Jazzy.
  MotoROS2 does not support ROS 2 Iron Irwini nor Rolling Ridley.
- Docker or a from-source build of the micro-ROS Agent
- FastDDS as RMW (even when using ROS 2 Galactic)

### Checking the system software version

To check the version of the system software:

 1. touch `{MAIN MENU}` on the pendant keypad
 1. touch `[System Info]`→`[Version]`

Look for the version number starting with `DN`, `YAS` or `YBS`.

## Option Function compatibility

The current version of MotoROS2 (`0.1.3`) is ***not*** compatible with the following Option Function(s) and/or other MotoPlus application(s):

- Simple Connect

  *Simple Connect* is an application installed by Yaskawa America to manage peripheral equipment in a robotic work cell.
  It is primarily installed on robots that were purchased with an arc-welding configuration.
  This application is not compatible with MotoROS2 at this time and must be removed to run MotoROS2.

- Absolute Accuracy Compensation

  This feature improves accuracy of supported robots when performing Cartesian motions from INFORM jobs.
  MotoROS2 currently does not support Cartesian motion and therefore can't benefit from *Absolute Accuracy Compensation*.
  Please refer to [Yaskawa-Global/motoros2#206](https://github.com/Yaskawa-Global/motoros2/issues/206) for an investigation into the compatibility of MotoROS2 with controllers with *Absolute Accuracy Compensation* enabled.

Please post questions about compatibility of MotoROS2 with specific controller features, Option Functions and/or MotoPlus applications in the [Discussion](https://github.com/Yaskawa-Global/motoros2/discussions/categories/q-a) forum.
Include information on the specific controller and robot model, as well as system software version and information on the MotoPlus application and/or controller feature in question.

## Download

### Downloading the files

All MotoROS2 releases are available from the [Releases](https://github.com/yaskawa-global/motoros2/releases) page.

Each release is a `.zip`, which contains the main executable (`.out`), a template configuration file (`.yaml`) and the INFORM jobs (several `.jbi`s).

Save the ZIP file to a convenient location.

### Extracting the files

Extract the `.zip` file somewhere to a temporary location.

Note that after installation, neither the `.zip` itself, nor any of the files in it are needed any more, as the driver node itself runs *on* the Yaskawa Motoman controller.

### Verifying integrity of the download

Check the integrity of the downloaded binary to avoid loading corrupted or incomplete binaries onto the controller.

The example command below uses `md5sum` on Linux, but any utility which can calculate MD5 hashes could be used, on any OS.

To calculate the MD5 hash on Debian/Ubuntu for the main MotoROS2 binary, run the following commands (on other OS, use the appropriate commands or tools instead):

```shell
$ cd /path/to/where/the/binary/was/saved
$ md5sum -b mr2_yrc1_h.out
e2d088b765a0bfed501aa213a1be1de0  mr2_yrc1_h.out
```

Compare the output of `md5sum` when run against the binary downloaded in the previous section ([Downloading the files](#downloading-the-files)) with the values listed in the following table.
The values must match *exactly*.

|**Controller** |**ROS 2 Version** | **File**          |**Version** | **MD5 hash**                       |
|:--------------|:-----------------|:------------------|:-----------|:-----------------------------------|
| DX200         | Foxy             | `mr2_dx2_f.out`   | `0.1.3`    | `a9a9e10403f726062c25d97654fad316` |
| DX200         | Galactic         | `mr2_dx2_g.out`   | `0.1.3`    | `e8db7512215da240b28b985f2f2af98b` |
| DX200         | Humble           | `mr2_dx2_h.out`   | `0.1.3`    | `611bda537655cf8a60d85600da6043f4` |
| YRC1000       | Foxy             | `mr2_yrc1_f.out`  | `0.1.3`    | `84bfb44e2043372127d9dfc1157a79b5` |
| YRC1000       | Galactic         | `mr2_yrc1_g.out`  | `0.1.3`    | `866e090b6c724429ce03117712c951f4` |
| YRC1000       | Humble           | `mr2_yrc1_h.out`  | `0.1.3`    | `e2d088b765a0bfed501aa213a1be1de0` |
| YRC1000micro  | Foxy             | `mr2_yrc1m_f.out` | `0.1.3`    | `027e77b427a212aa63e5d7962d48ad92` |
| YRC1000micro  | Galactic         | `mr2_yrc1m_g.out` | `0.1.3`    | `042d753a7729784fec8c5c23bef3e685` |
| YRC1000micro  | Humble           | `mr2_yrc1m_h.out` | `0.1.3`    | `c0e61adbf5bf6fd6a734211f15bb0f0a` |

If the hash matches, proceed with the next section, [Configuration](#configuration).

If the hash does not match, download the correct version from the [Releases](https://github.com/yaskawa-global/motoros2/releases) page again (to exclude a failed download was the cause), extract it and run `md5sum` on the binary again.
If the hash still doesn't match, report the issue on the [issue tracker](https://github.com/yaskawa-global/motoros2/issues).
Be sure to describe which version of MotoROS2 was downloaded, from where, how it was extracted, and a verbatim copy-paste of the hashing tool command executed and the output received.

**Note**: please verify you ran `md5sum` against the `.out` file, not the `.zip` nor any other file included in the release.

## Configuration

All MotoROS2 distribution `.zip` files come with a template `.yaml` file, which *must* be updated before deployment to a specific Yaskawa Motoman robot controller.

For more information on all configuration settings, please refer to the `motoros2_config.yaml` file in the release `.zip`.

All fields have defaults, except the IP address and UDP port number at which MotoROS2 expects the micro-ROS Agent to be reachable.
These fields must be set to correct values by users, as otherwise MotoROS2 will not be able to communicate with the ROS 2 node graph.

Edit the template `.yaml` and change the `agent_ip_address` and `agent_port_number` to point to the host which will run the micro-ROS Agent application.
Review the rest of the configuration file and change values as necessary for your specific deployment.

### Verifying YAML correctness

We recommend checking YAML syntax of the configuration file before copying it to the controller.

**Note**: we recommend using an off-line tool, to prevent information in the `.yaml` file from being submitted to an on-line service.
Users are of course free to use tools they prefer, but we'll only document using `yamllint` (ie: an off-line tool) in this section.

`yamllint` is written in Python, so works on Windows, Linux and OSX.
Many package managers (such as `apt`, on Ubuntu and Debian) include it in their default distributions, but the versions available are generally too old, so the developers use a Python virtual environment:

```shell
python3 -m venv $HOME/venv_yamllint
source $HOME/venv_yamllint/bin/activate
(venv_yamllint) pip3 install yamllint
```

MotoROS2 comes with a configuration file for [yamllint](https://yamllint.readthedocs.io/en/stable) which facilitates checking `motoros2_config.yaml` with the settings the developers use.

To verify the file has been correctly edited:

```shell
(venv_yamllint) cd /path/to/motoros2/config
(venv_yamllint) yamllint -s . && echo "all ok"
```

For a correctly formatted file, this should print `all ok`.

If `yamllint` prints warnings or errors, correct the offending line(s) and rerun `yamllint`.

## Example INFORM jobs

All MotoROS2 release `.zip`s contain a copy of the default INFORM jobs (`.jbi` and associated `.dat`).

The following variants are shipped with MotoROS2 in the `robot_jobs` sub directory:

| **Directory**               | **Description**      | **Supported controller(s)**  |
|:----------------------------|:---------------------|:-----------------------------|
| `single_arm`                | Single robot group   | DX200, YRC1000, YRC1000micro |
| `single_arm_with_ext_axis`  | Robot + station axis | DX200, YRC1000, YRC1000micro |
| `single_arm_with_base_axis` | Robot + base axis    | DX200, YRC1000, YRC1000micro |
| `two_arms`                  | Two robot groups     | DX200, YRC1000, YRC1000micro |
| `sda_dual_arm`              | SDA robots *only*    | -                            |

**These jobs are not required.**
The INFORM job will be automatically generated at startup.
These are only provided for convenience to show the required commands if a custom job is required.

If needed, open a new issue on the [Issue tracker](https://github.com/yaskawa-global/motoros2/issues) to ask for support with creating a custom job.

## Installation

### Checking MotoPlus configuration

Use the following steps to verify MotoPlus has been correctly configured for MotoROS2, and the necessary settings are active:

 1. boot the controller while holding `{MAIN MENU}` on the pendant keypad to enter *Maintenance* mode
 1. upgrade to *MANAGEMENT* security level by touching `[System Info]`→`[Security]` (default password is all `9`'s)
 1. touch `[System Info]`→`[Setup]` and select `OPTION FUNCTION`
 1. move to `MotoPlus FUNC.`, make sure it is set to `USED`. If it isn't, set it to `USED`
 1. move cursor down to `MOTOMAN DRIVER` and make sure it is set to `USED`. If it isn't, set it to `USED`

### DX200, YRC1000, and YRC1000micro

Place the `.out` (main binary), `.yaml` (configuration), and `.dat` (I/O names) files on an external storage device: Compact Flash (CF), Secure Digital (SD), and USB sticks can be used depending on the controller model.
Insert the storage device into the robot's programming pendant.

If the controller is configured with the Functional Safety Unit (FSU), then `SAVE DATA CRC CHECK FUNC (FSU)` must be temporarily disabled during the installation procedure.

Turn on the robot controller to enter *Normal Operation* mode.

In *Normal Operation* mode:

 1. upgrade to *SAFETY* security level by touching `[System Info]`→`[Security]` (default password is all `5`'s)
 1. touch `[Setup]`→`[Function Enable]`
 1. navigate to `SAVE DATA CRC CHECK FUNC (FSU)`
 1. set this feature to `INVALID`
 1. turn off the robot controller

Turn on the robot controller while holding the `{Main Menu}` key on the keypad to enter *Maintenance* mode.
You may release the key when you see the Yaskawa logo appear on the screen.

In *Maintenance* mode:

 1. upgrade to *MANAGEMENT* security level by touching `[System Info]`→`[Security]` (default password is all `9`'s)
 1. touch `[MotoPlus APL]`→`[Device]` to select CF, SD, or USB memory type
 1. touch `[MotoPlus APL]`→`[Load (User App)]` to select and load the `mr2_*_*.out` file
 1. touch `[MotoPlus APL]`→`[File List]` and verify that MotoROS2 was properly installed and no other MotoPlus applications are currently loaded on the controller
 1. (YRC1000[micro] only): touch `[File]`→`[Initialize]` and select `USER DEFINED FILES`
 1. (YRC1000[micro] only): select `SRAM RAM DRIVE` and initialize it
 1. rotate the pendant key-switch (upper left of pendant) fully counter-clockwise into `TEACH` mode
 1. reboot the robot controller into regular mode

In *Normal Operation* mode:

 1. you will get alarm `8013 [0] Missing MotoROS2 cfg file`.
 touch `[RESET]` to clear the alarm
 1. upgrade to *MANAGEMENT* security level by touching `[System Info]`→`[Security]` (default password is all `9`'s)
 1. touch `[PARAMETER]`→`[S2C]` and set the following parameters:
     1. `S2C541 = 0`
     1. `S2C542 = 0`
     1. `S2C1102 = 2`
     1. `S2C1104 = 2`
     1. `S2C1117 = 1` (DX200 only)
     1. `S2C1250 = 1`
     1. `S2C1402 = 3`

 If a custom INFORM job will be used:

 1. touch `[EX MEMORY]`→`[Load]`
 1. cursor to `JOB` and press `[SELECT]`
 1. cursor to your job file and press `[SELECT]` then `[ENTER]`

#### YRC1000 and YRC1000 micro

 1. touch `[EX MEMORY]`→`[Load]`
 1. cursor to `USER DEFINED FILES` and press `[SELECT]`
 1. cursor to `motoros2_config.yaml` and press `[SELECT]` then `[ENTER]`

#### DX200

 1. power down the DX200 controller
 1. copy the `motoros2_config.yaml` file to a USB storage drive
 1. insert the USB drive into the `CN106` USB port inside the controller cabinet
 1. leave the drive in place and close the controller cabinet
 1. power up the DX200 controller

Note: on DX200, the USB stick used to store `motoros2_config.yaml` can't be removed, it *must* remain inserted into the USB port labelled `CN106`, or at least as long as MotoROS2 is installed on the controller.
Without the USB stick in `CN106`, MotoROS2 would not be able to load its configuration and alarms will be raised on each controller (re)boot.

### All supported controllers

Within 30 seconds of loading the configuration file, you should get alarm `8001[10] Speed FB enabled, reboot now`.
Reboot again and there should be no alarms.

If you receive any errors or alarms after rebooting, please refer to the [Troubleshooting](#troubleshooting) section for information on how to remedy the issue.

If `SAVE DATA CRC CHECK FUNC (FSU)` was disabled at the start of this procedure, then it can now be reenabled.

 1. upgrade to *SAFETY* security level by touching `[System Info]`→`[Security]` (default password is all `5`'s)
 1. touch `[Setup]`→`[Function Enable]`
 1. navigate to `SAVE DATA CRC CHECK FUNC (FSU)`
 1. set this feature to `VALID`

## Building from source

Please refer to [doc/Building from source](doc/build_from_source.md).

## Updating the configuration

It may be necessary to update MotoROS2 configuration during or after initial deployment.

### YRC1000 and YRC1000micro

#### Controller software YAS4.70 or YBS3.02 or later

In *Normal Operation* mode:

 1. touch `[EX MEMORY]`→`[Device]`
 1. cursor to `JOB & U.D. FILE LOAD OVERWRITE` and verify it is set to `VALID`.
 Change it to `VALID` if it isn't.
 1. touch `[EX MEMORY]`→`[Load]`
 1. cursor to `USER DEFINED FILES` and press `[SELECT]`
 1. cursor to `motoros2_config.yaml` and press `[SELECT]` then `[ENTER]`
 1. touch `[YES]` to overwrite
 1. reboot the robot controller

#### Older controller software

Due to the way the controller treats files, `motoros2_config.yaml` cannot be directly overwritten using the `[EX MEMORY]` menu.
Instead, MotoROS2 has a built-in mechanism which updates the controller's copy of the `.yaml` file with a new version placed on a USB stick.

To update the configuration file on the controller:

 1. place an updated version of the `motoros2_config.yaml` file in the root directory of a USB stick
 1. power down the robot controller and open the cabinet
 1. locate the USB port labelled `CN102` on the robot's CPU board and insert the USB stick into it
 1. power on the controller and wait for it to fully boot (ie: you see the regular UI on the teach pendant)
 1. verify MotoROS2 has started
 1. remove the USB stick from the controller

MotoROS2 should automatically parse the new file and update its internal configuration.
In case of errors in the configuration file, or incompatible settings, alarms and/or errors will be posted to the teach pendant.

After the new file is processed, it will be renamed on the USB stick.
The filename will be appended with the current date/time of the robot controller.
This is to show that the file was processed and maintain some historical record.

NOTE: the USB port on the teach pendant can not be used to update the configuration.
*Only* the USB port labelled `CN102` is checked by MotoROS2.

CF and SD cards are not supported when updating the configuration.

NOTE: When submitting support requests, please always get a 'current' copy of the configuration file from the teach pendant.
You should not rely on the historical configuration files and assume that is currently loaded into the controller.

To extract a copy of your current configuration from the teach pendant:

1. if necessary: restart the controller in *Normal* mode
1. touch `[EX MEMORY]`→`[SAVE]`
1. cursor to `USER DEFINED FILES` and press `[SELECT]`
1. cursor to `motoros2_config.yaml` and press `[SELECT]` then `[ENTER]`

### DX200 only

 1. power down the robot controller and open the cabinet
 1. locate the USB port labelled `CN106` on the robot's CPU board and remove the USB stick
 1. overwrite the `motoros2_config.yaml` file in the root directory of a USB stick with an updated version
 1. replace the USB stick into `CN106` and close the controller cabinet
 1. power on the controller and wait for it to fully boot (ie: you see the regular UI on the teach pendant)
 1. verify MotoROS2 has started

## The micro-ROS Agent

The micro-ROS Agent acts as the transparent bridge between MotoROS2 and ROS 2.
As micro-ROS applications can not directly communicate with ROS 2 RMWs, the Agent must always be running for MotoROS2 to function correctly.

There are two main ways to run the Agent: using a Docker image or by building it in a Colcon workspace.

Using the Docker image is recommended, as it's less complicated than building with Colcon.

**Note**: either the Docker image *or* a from-source build is needed.
Choose one or the other.

### Using Docker (Linux Only)

The command shown here starts the `jazzy` version of the `micro-ros-agent` Docker image.
However, always make sure to use a version of the Agent image which corresponds to the version of ROS 2 that is being used.

With ROS 2 Humble, use `microros/micro-ros-agent:humble`.
With ROS 2 Jazzy, use `microros/micro-ros-agent:jazzy`.

To start the Agent (on a machine with Docker already installed and setup to allow non-root access):

```shell
docker run \
  -it \
  --rm \
  --net=host \
  --user=$(id -u):$(id -g) \
  microros/micro-ros-agent:jazzy \
    udp4 \
    --port 8888
```

**Note**: be sure to update the `--port` parameter to use the same value as was chosen for the `agent_port_number` configuration item in the `motoros2_config.yaml`.

### Using the ROS 2 package

The micro-ROS Agent can also be built as a ROS 2 package in a Colcon workspace.
This procedure is rather involved, so only do this if the pre-configured Docker image can not be used.

The following sections show how to build the Jazzy version of the Agent in a dedicated workspace (adapt the paths used below if a different workspace should be used instead).

Note: always make sure to use a version of the Agent which corresponds to the version of ROS 2 that is being used.
For ROS 2 Foxy, checkout the `foxy` branch.
For ROS 2 Humble, checkout the `humble` branch.
For ROS 2 Jazzy, checkout the `jazzy` branch.

#### Linux (Debian/Ubuntu)

This requires a working ROS 2 development environment (compiler, CMake, Git, Colcon, `rosdep`, etc).

In a terminal:

```shell
source /opt/ros/jazzy/setup.bash
sudo apt update
rosdep update --rosdistro=$ROS_DISTRO
mkdir -p $HOME/micro_ros_agent_ws/src
git clone \
  -b jazzy \
  https://github.com/micro-ROS/micro_ros_setup.git \
  $HOME/micro_ros_agent_ws/src/micro_ros_setup
rosdep install \
  --from-paths $HOME/micro_ros_agent_ws/src \
  --ignore-src \
  -y
cd $HOME/micro_ros_agent_ws
colcon build
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```

The Agent application will only need to be built *once*, unless it needs to be updated.

Finally, to run the Agent:

```shell
source /opt/ros/humble/setup.bash
source $HOME/micro_ros_agent_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

**Note**: be sure to update the `--port` parameter to use the same value as was chosen for the `agent_port_number` configuration item in the `motoros2_config.yaml`.

#### Windows

This requires a working ROS 2 on Windows development environment (Visual Studio, CMake, Git, Colcon, etc).

Open a Visual Studio *Developer Command Prompt*.

Or open a regular command prompt and `call "C:\Program Files\Microsoft Visual Studio\2022\Enterprise\VC\Auxiliary\Build\vcvars64.bat"`.
If you are using a different version of Visual Studio, update the path to match.

Now execute the following commands:

```batch
call "C:\path\to\ros2-jazzy\local_setup.bat"
mkdir "%USERPROFILE%\micro_ros_agent_ws\src"
git clone ^
  -b jazzy ^
  https://github.com/micro-ROS/micro_ros_msgs.git ^
  "%USERPROFILE%\micro_ros_agent_ws\src\micro_ros_msgs"
git clone ^
  -b jazzy ^
  https://github.com/micro-ROS/micro-ROS-Agent.git ^
  "%USERPROFILE%\micro_ros_agent_ws\src\micro-ROS-Agent"
cd "%USERPROFILE%\micro_ros_agent_ws"
colcon build ^
  --merge-install ^
  --packages-up-to micro_ros_agent ^
  --cmake-args ^
  "-DUAGENT_BUILD_EXECUTABLE=OFF" ^
  "-DUAGENT_P2P_PROFILE=OFF" ^
  "--no-warn-unused-cli"
```

The Agent application will only need to be built *once*, unless it needs to be updated.

Finally, to run the Agent:

```batch
cd "%USERPROFILE%\micro_ros_agent_ws
call install\setup.bat
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

**Note**: be sure to update the `--port` parameter to use the same value as was chosen for the `agent_port_number` configuration item in the `motoros2_config.yaml`.

## Verifying successful installation

After the final reboot of the controller, and after [starting the micro-ROS Agent](#the-micro-ros-agent), the Agent should show MotoROS2 registering its publishers, services and action servers.

Note: if you are using ROS 2 Galactic, please first read [Only FastDDS is supported](#only-fastdds-is-supported).

On a PC with a supported ROS 2 installation (ie: Foxy, Galactic (with FastDDS), Humble, or Jazzy):

1. open a new terminal
1. `source` the ROS 2 installation

Now run `ros2 node list`.

Provided you have no other ROS 2 nodes running, you should see a single `motoman_ab_cd_ef` node listed, with `ab_cd_ef` being the last three bytes of the MAC address of the controller.
If you set `node_name` to something else in [the configuration](#updating-the-configuration), you should of course see the expected node name listed instead.

## Usage

### Basic usage with ROS

Note: if you are using ROS 2 Galactic, please first read [Only FastDDS is supported](#only-fastdds-is-supported).

As MotoROS2 uses micro-ROS, you must always make sure to first [start the micro-ROS Agent](#the-micro-ros-agent).
After registration with the Agent, MotoROS2 behaves like any other ROS 2 node.

For initial discovery of the ROS API, we recommend using the [ros2 node](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html), [ros2 topic](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html), [ros2 service](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html) and [ros2 action](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html) commandlets.
Refer also to the [ROS API](#ros-api) section for more information on the various topics, services and actions MotoROS2 supports.

In order to be able to interact with MotoROS2 (either using the command line tools or from ROS 2 nodes), the message, service and action definitions must first be made available.

MotoROS2 uses interface definitions from:

- `control_msgs`
- `industrial_msgs`
- `motoros2_interfaces`
- `sensor_msgs`
- `std_srvs`
- `tf2_msgs`

Except `industrial_msgs` and `motoros2_interfaces`, these are all standard ROS 2 packages.

The [motoros2_client_interface_dependencies](https://github.com/yaskawa-global/motoros2_client_interface_dependencies) package has been created to facilitate installation of all required interface packages.
Please refer to the readme of `motoros2_client_interface_dependencies` for information on how to download, configure, build and install the necessary packages in a Colcon workspace.

After installing `motoros2_client_interface_dependencies`, you should be able to `ros2 topic echo` all topics MotoROS2 publishes to and `ros2 service call` all services MotoROS2 serves.

Even though the action server can also be interacted with from the command line using `ros2 action send_goal`, we do not recommend this.
Instead, write a `FollowJointTrajectory` action *client* script or use a motion planning environment which is capable of interfacing with `FollowJointTrajectory` action *servers*, such as MoveIt.

### Commanding motion

The ROS API of MotoROS2 for commanding motion is similar to that of `motoman_driver` (with MotoROS1), and client applications are recommended to implement a similar flow of control to keep track of the state of the robot before, during and after trajectory and motion execution.

The following provides a high-level overview of the behaviour a client application should implement to successfully interact with the [FollowJointTrajectory](doc/ros_api.md#follow_joint_trajectory) action server offered by MotoROS2.
While not all steps are absolutely necessary, checking for errors and monitoring execution progress facilitates achieving robust execution and minimises unexpected behaviour in both client and server.

To submit a trajectory goal for execution by an idle MotoROS2 (so no active ROS nor INFORM motion):

1. create a `FollowJointTrajectory` action client, pointing it to the [follow_joint_trajectory](doc/ros_api.md#follow_joint_trajectory) action server of MotoROS2
1. retrieve a message from the [robot_status](doc/ros_api.md#robot_status) topic to verify the robot is idle (inspect the `RobotStatus::in_motion` field) and store the current `JointState` by retrieving a message from the [joint_states](doc/ros_api.md#joint_states) topic
1. generate (using a motion planner for instance) or construct a `trajectory_msgs/JointTrajectory` message, making sure to use the state stored in step 2 as the start state (ie: the first `JointTrajectoryPoint`).
Using correct start states for trajectories is absolutely required, and MotoROS2 will reject any trajectory which does not start at the current state
1. retrieve a message from the [robot_status](doc/ros_api.md#robot_status) topic and inspect the `in_error` and `error_code` fields
1. if there are no active errors, go to step 6.
If there are active errors, remedy the cause, call the [reset_error](doc/ros_api.md#reset_error) service and go to step 4
1. call the [start_traj_mode](doc/ros_api.md#start_traj_mode) service, to put MotoROS2 into its trajectory execution mode.
Inspect the `result_code` of the service response.
If MotoROS2 reported success, proceed to step 7; if instead, it reported an error, remedy the cause and go to step 4
1. construct a `FollowJointTrajectory` goal and store the trajectory created in step 3 in it
1. submit the goal to the server, using the client created in step 1
1. wait for MotoROS2 to accept the goal.
After acceptance, keep monitoring goal state while MotoROS2 executes it (note: this is default behaviour of the ROS 2 action client. It will call registered callbacks and forward state received from MotoROS2 to user code automatically)
1. if the goal is `aborted` or `rejected`, inspect the `error_code` field of the returned action result.
Otherwise wait for MotoROS2 to report completion of the goal.
As a final check, inspect the `error_code` field of the result to ascertain execution was successful (refer to [follow_joint_trajectory](doc/ros_api.md#follow_joint_trajectory) for information on how to decode the field's value)
1. if there are more trajectories to execute, return to step 2.
Otherwise call the [stop_traj_mode](doc/ros_api.md#stop_traj_mode) service to exit MotoROS2's trajectory execution mode

Interaction with the *point streaming* interface (MotoROS2 `0.0.15` and newer) would be similar, although no `FollowJointTrajectory` action client would be created, no goals would be submitted and monitoring robot status would be done purely by subscribing to the [robot_status](doc/ros_api.md#robot_status) topic (instead of relying on an action client to report trajectory execution status).

### With MoveIt

Make sure to first have read [Basic usage with ROS](#basic-usage-with-ros) and [Commanding motion](#commanding-motion).

MotoROS2 exposes the required `JointState` and `FollowJointTrajectory` interfaces for MoveIt to be able to control motion.

For this to work, a MoveIt configuration package needs to be updated to send `FollowJointTrajectory` action goals to the correct action server, which requires changes to MoveIt's *controller configuration*.
This part of the configuration can typically be found in a file called `moveit_controllers.yaml`, which is normally located in the `config` sub directory of a MoveIt configuration package.

The MoveIt defaults for subscribing to `JointState` messages should not need any changes (unless MotoROS2 has been configured with a non-default setting for the namespace (see below)).

The following snippet shows an example of how to update the `controller_names` and `action_ns` keys for a robot with a single group, 6 joints, with their default joint names and no namespacing configured on MotoROS2's side:

```yaml
...

controller_names:
  - follow_joint_trajectory

follow_joint_trajectory:
  action_ns: ""
  type: FollowJointTrajectory
  default: true
  joints:
   - joint_1
   - joint_2
   - joint_3
   - joint_4
   - joint_5
   - joint_6

...
```

Here, `follow_joint_trajectory` is the name of the MotoROS2 action server, but used as the name of the MoveIt controller.
Note also the empty `action_ns` key.

Note: depending on whether MotoROS2 is configured to *namespace* its action server (see [Can MotoROS2 run in a namespace?](doc/faq.md#can-motoros2-run-in-a-namespace)), the `action_ns` and name of the controller will need to be updated.

In case [joint names have been changed from their defaults](doc/faq.md#can-names-of-joints-be-changed), the names specified in the MoveIt configuration will also need to be updated.

## ROS API

Please refer to [doc/ROS-API](doc/ros_api.md).

## Default QoS settings

### Publisher QoS

The default QoS profiles used for the topics MotoROS2 publishes to are listed in this table:

| Topic          | Profile       | Comment                   |
|----------------|---------------|---------------------------|
| `joint_states` | *Sensor data* | *Best-effort* reliability |
| `robot_status` | *Sensor data* | Same                      |
| `tf`           | *Default*     | *Reliable* reliability    |

Please refer to [About Quality of Service settings: QoS profiles](https://docs.ros.org/en/jazzy/Concepts/About-Quality-of-Service-Settings.html#qos-profiles) in the general ROS 2 documentation for more information about these default profiles.

These values are based on tests with other ROS 2 components and applications and analyses of implementations of subscribers by the authors of MotoROS2 (examples include RViz2 and MoveIt2).
But these profiles can not be compatible with all possible combinations of subscribers, and thus have been made configurable.

If needed, update [the configuration](#configuration) by changing the `publisher_qos` settings: `joint_states`, `robot_status` and/or `tf` items.
Follow [Updating the configuration](#updating-the-configuration) to propagate this change to MotoROS2.

Note: a default QoS profile for topics like `joint_states` is a topic for discussion in the ROS 2 community.
Relevant link: [ros-planning/moveit2#1190](https://github.com/ros-planning/moveit2/issues/1190).

### Service server QoS

MotoROS2 uses the default ROS 2 / micro-ROS profile for all its service servers.
This is called *Services* in the ROS 2 documentation on QoS profiles.

QoS for services is currently not configurable.

### Action server QoS

MotoROS2 uses the default ROS 2 / micro-ROS profiles for all its action servers.
As actions use both topics as well as services, both the *Services* and the *Default* profile are used.

QoS for action servers is currently not configurable.

The action clients shipped by ROS 2 as part of the `rclcpp` and `rclpy` client libraries are by default compatible with MotoROS2.

## Known issues & limitations

MotoROS2 is still under development.
As such, there are currently a number of known issues and limitations users should be aware of and should take into account.

This section documents these and provides work-arounds where possible.

### Functional Safety Unit interaction

**Description**: the Functional Safety Unit (FSU) can be configured to limit the motion of the robot to avoid injury of surrounding personnel.
This can include reducing the speed of the arm, restricting the space where the robot may operate, or stopping all motion while external sensors are triggered.

In version `0.1.1` and older of MotoROS2, any limitation imposed by the FSU will result in an undefined and incomplete trajectory execution.
There is no explicit indicator back to the client PC for notifying that there is any deviation.
The client must monitor the feedback `/joint_states` topic to determine if the robot is executing the trajectory as planned.

Versions `0.1.2` and newer have improved support for the FSU when used with MotoROS2 controlled motion.
If an FSU limitation is imposed, MotoROS2 will try to execute the complete trajectory as specified in the goal, but depending on the exact FSU configuration, may do so at a reduced speed.
Motion planners (such as MoveIt) must therefore be configured to allow for such slowdowns to prevent them from prematurely aborting trajectories.

Unfortunately, MotoROS2 does not currently publish speed reduction ratios imposed by an FSU, nor does it report FSU activation status.
We are aware of this limitation and may address it in a future release.

### Only FastDDS is supported

**Description**: the current implementation of MotoROS2 can only communicate with ROS 2 applications which use eProsima FastDDS as their RMW layer.
None of the other RMWs are supported, including Cyclone DDS, which is the default RMW of ROS 2 Galactic.

Symptoms of this incompatibility are seemingly functional ROS 2 network connections, where topics are succesfully published and subscribed to, but service invocations and action goal submissions appear to *hang*.

**Note**: ROS 2 Foxy, ROS 2 Humble, ROS 2 Jazzy, and ROS 2 Rolling all use FastDDS by default.
If you haven't changed your default RMW, you should not need to change anything for MotoROS2.

**Work-around**: unfortunately, this limitation is caused by a middleware-layer incompatibility with respect to how service requests are (de)serialised by the respective RMWs ([ros2/rmw_cyclonedds#184](https://github.com/ros2/rmw_cyclonedds/issues/184)), and without significant changes to the way MotoROS2 operates, has no known work-around.

Users with ROS 2 Galactic installed could install the `ros-galactic-rmw-fastrtps-cpp` package and run `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` before using any ROS 2 functionality on their host systems.
Note that the `RMW_IMPLEMENTATION` environment variable must be `export`ed to all shells which are used to run ROS 2 applications.

Please note that the `ros-galactic-rmw-fastrtps-cpp` package must be install *prior* to building the client application workspace.
If the workspace has already been built, it must be rebuilt after installing the package.

Refer to [Working with multiple ROS 2 middleware implementations](https://docs.ros.org/en/jazzy/How-To-Guides/Working-with-multiple-RMW-implementations.html) in the ROS 2 documentation for more information about configuring different RMWs with ROS 2.

### Maximum length of trajectories

**Description**: due to controller resource constraints and implementation details of micro-ROS, MotoROS2 imposes an upper limit on the number of `JointTrajectoryPoint`s in a `JointTrajectory`s submitted as part of `control_msgs/FollowJointTrajectory` action goals.

This maximum number of points in a single trajectory is currently **`200`**.

This number was derived from testing on a two-robot system (12 axes total).
On larger systems with more control groups, it is possible 200 points may exceed the memory threshold for transmission.

Unfortunately, due to a known issue with micro-ROS ([micro-ROS/micro-ROS-Agent#143](https://github.com/micro-ROS/micro-ROS-Agent/issues/143)), MotoROS2 currently cannot check whether incoming trajectories are too long, nor can MotoROS2 notify the action client in those cases.

Please make sure to check trajectory length *before* submitting goals, as client applications are currently responsible for making sure trajectories do not go over this limit.

**Note**: this is strictly a limit on the *number of trajectory points*, not on the total time duration of a trajectory.

**Work-around**: client applications could split long trajectories into smaller sections, each no longer than the maximum of `200` trajectory points.
While motion continuity will not be maintained between trajectories, this approach would allow for longer (as in: longer in time) motions to be commanded by a ROS 2 client.
Whether this would be an acceptable work-around depends on whether the application and the motions it uses support natural stopping points or dwell times.

If the memory threshold is exceeded due to a large trajectory, then the robot will not be notified of the commanded trajectory.
Clients should utilize an appropriate timeout to detect whether the robot responds to a commanded trajectory.

### No support for asynchronous motion

**Description**: MotoROS2 currently only supports synchronous motion across all motion groups configured on the Yaskawa controller.
Controllers with multiple motion groups are supported, but motion groups cannot execute motions independently from each other.

**Work-around**: none at this time.
However, goals which move only a subset of groups (and/or joints) can be created: retrieve the current state of all groups and update target poses for the groups (and/or joints) which should move only.
By keeping the state of all other groups (and/or joints) static, those groups (and/or joints) will not move.

This is not true asynchronous motion, but could allow for some use-cases to still be implemented with the current versions of MotoROS2.

### No support for partial goals

**Description**: goals submitted to MotoROS2 *must* always include information for all groups and all joints configured on the controller, even if those groups or joints are not supposed to move.
So called *partial goals* are currently not supported and will be rejected by MotoROS2.

**Work-around**: please refer to the work-around described in [No support for asynchronous motion](#no-support-for-asynchronous-motion).

### Upper limit to publishing frequency

**Description**: the frequency for publishing topics is configurable in the `motoros2_config.yaml` configuration file.
Initial testing has revealed that the MotoROS2 is limited to about 100 Hz.

**Work-around**: none at this time.
The issue is being investigated.

### Incorrect transform tree origin with multi-robot setups

**Description**: when configured to broadcast TF frames for controllers with multiple robots, the origins of the transform trees for the different robot groups will be coincident, unless the group combination has been calibrated.
For example, an `R1+R2` configuration, without calibration, will have the origins of the transform trees for `R1` and `R2` in the same place, making their TF trees overlap.

**Solution**: perform the calibration for the robot group(s).
Refer to the relevant Yaskawa Motoman documentation for more information on how to perform a robot-to-robot calibration.

After robot-calibration, the transform between the shared `world` frame and each robot's `base` frame will be known, and MotoROS2 will include it in the transforms it broadcasts.

### Memory leak

**Description**: there is a small memory leak which occurs each time the micro-ROS Agent disconnects from the controller.

**Work-around**: none at this time.
The issue is being investigated.

### Some group combinations won't publish data

**Description**: it has been observed that an R1+R2+S1 system will not publish data `/joint_states` or `/tf`.
All other topics and services work as expected.

**Work-around**: such a multi-group system would need to be broken up into a independent systems.
(E.g. `R1+R2+S1` would be broken up into `R1+S1` and another `R1`).

The cause of this behavior is unknown.
The issue is being investigated.

## Provisional roadmap

This section gives a brief overview of features, enhancements and other tasks that are currently on the MotoROS2 roadmap.
Some of these may depend on external developments (ie: in ROS 2 or micro-ROS), or on Yaskawa internal development priorities.
As such, no statements are made about priorities or development schedule for any of these items.

The following items are on the MotoROS2 roadmap, and are listed here in no particular order:

- support FS100 controllers
- read/write of controller variables
- CRUD of INFORM job files (ie: create, retrieve, update, delete)
- starting/stopping INFORM jobs (other than `INIT_ROS`)
- native (ie: Agent-less) communication
- support asynchronous motion / partial goals
- complete ROS parameter server support (there is currently no support for `string`s in RCL)
- real-time position streaming interface (skipping MotoROS2's internal motion queue)
- Cartesian motion interfaces
- velocity control (based on `mpExRcsIncrementMove(..)`)
- integration with ROS logging (`rosout`)
- publishing static transforms to `tf_static`
- integrate a UI into the teach pendant / Smart Pendant
- add support for on-line trajectory replacement to the FJT action server (similar to the ROS1 [joint_trajectory_controller](http://wiki.ros.org/joint_trajectory_controller/UnderstandingTrajectoryReplacement))
- integration of process control for peripherals attached to robot (welding, cutting, painting, etc)

## Frequently Asked Questions

Please refer to [doc/FAQ](doc/faq.md).

## Troubleshooting

Please refer to [doc/Troubleshooting](doc/troubleshooting.md).
