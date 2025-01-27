<!--
SPDX-FileCopyrightText: 2023, Yaskawa America, Inc.
SPDX-FileCopyrightText: 2023, Delft University of Technology

SPDX-License-Identifier: CC-BY-SA-4.0
-->

# Network Configuration

You will need to use a LAN cable to allow the robot controller to communicate with the PC running the [micro-ROS Agent](../README.md#understanding-the-micro-ros-agent).

- On DX200 and YRC1000micro, this will connect to the `LAN` port.
- On the YRC1000, you may choose to connect this to either the `LAN2` port or the `LAN3` port

The simplest configuration will have the robot simply plugged directly in to the PC.
More complicated configurations are common, but is recommended to have a direct connection until initial setup is complete to reduce points of failure.

## Configuring Controller

### YRC1000 and YRC1000micro

1. boot the controller while holding `{MAIN MENU}` on the pendant keypad to enter *Maintenance* mode
1. upgrade to *MANAGEMENT* security level by touching `[System Info]`→`[Security]` (default password is all `9`'s)
1. touch `[System Info]`→`[Setup]` and select `OPTION FUNCTION`
1. move to `LAN INTERFACE SETTING`
1. make sure `IP ADDRESS SETTING(LAN[X])` for the port you are using is set to `MANUAL SETTING`.
If it is not, make sure that `DNS SETTING` and `SNTP SETTING` are not set to `DHCP SETTING` for the NIC you are using (change them if you need to), and then set `IP ADDRESS SETTING` to `MANUAL SETTING`
1. hit `{ENTER}` on the pendant keypad and touch `[OK]`

### DX200

1. boot the controller while holding `{MAIN MENU}` on the pendant keypad to enter *Maintenance* mode
1. upgrade to *MANAGEMENT* security level by touching `[System Info]`→`[Security]` (default password is all `9`'s)
1. touch `[System Info]`→`[Setup]` and select `OPTION FUNCTION`
1. move to `NETWORK SETTING` and then to `HOST SETUP`
1. make sure that `IP ADDRESS SETTING` is set to `MANUAL SETTING`.
If it is not, make sure that `DNS SETTING` and `SNTP SETTING` are not set to `DHCP SETTING` (change them if you need to), and then set `IP ADDRESS SETTING` to `MANUAL SETTING`
1. hit `{ENTER}` on the pendant keypad and touch `[OK]`

## Get controller and PC on same subnet

If the controller and PC are not on the same subnet, choose one of the following options:

- Modify the `agent_ip_address` key in the `motoros2_config.yaml` file and specify an IP address that is on the robot's subnet.
See [here](../README.md#configuration-file) for information about the config file.
Then [propagate the changes to the Yaskawa controller](../README.md#updating-the-configuration).
You will need to ensure that the PC running the micro-ROS agent application uses this static IP address on the network port connected to the robot controller.
-Modify the robot controller's IP and subnet mask so it is on the subnet of the PC running the micro-ROS agent.
- Modify the robot controller's network settings to add a gateway which can reach the IP address of the subnet of the PC running the micro-ROS agent.

## Network issues

If you are unable to connect the PC running the micro-ROS agent application and the controller, the network configuration may be the issue.
Network problems can sometimes be the root of seemingly unrelated problems, such as rcl/rclc errors.

To troubleshoot network issues, first ensure that the robot controller's IP address is `MANUAL SETTING`.
It is important that it is not using DHCP.

If that is not the problem, check that the `agent_ip_address` in the yaml configuration matches the IP address of the PC.
Also make sure that the [robot controller IP address is on the same subnet as the PC](#get-controller-and-pc-on-same-subnet).

Make sure that there are no IP address on the network conflicts with either the client PC or the robot controller.

After taking these steps, test if the client PC and the controller are connected by opening a terminal on the PC and attempting to ping the robot controller's IP.
An example using the default controller IP for LAN2 is shown below.

```shell
ping 192.168.1.31
```

If you receive a near-immediate response upon attempting to ping the controller, then the potential network issues mentioned above are resolved.
Try to connect again via the micro-ROS agent. If the micro-ROS agent still cannot connect to the controller, there may be a firewall configuration issue or a hardware issue.

If you do not receive a near-immediate after attempting to ping the controller, there is still some issue.
You may have made a mistake previously, or there could be a firewall issue, or there could be a network hardware issue.

To rule out a hardware issue, try plugging the client PC directly into the controller, bypassing any additional hardware points of failure (e.g. switches).
If that doesn't work, try using a new cable, or verify that your cable works elsewhere.

If you determine that it is not a hardware issue, check if it is a firewall issue.

### Firewall issues

Firewall rules on the PC running the micro-ROS agent application may prevent it from connecting to the robot controller.
When using MotoROS2, the controller and the PC must be able to send UDP packets back and forth.

Assuming the PC is running Ubuntu, you can check the status of the firewall using the default firewall configuration tool `ufw` with the command below.

```shell
sudo ufw status
```

If the status is inactive, you are either not using a firewall or the default firewall is disabled.
If the status shows as active, you should use the following command to see what firewall rules have been added that may prevent communication.

```shell
sudo ufw show added
```

As an example of a rule that may interfere with MotoROS2 communication, `ufw deny from any proto udp` as a high priority rule would make it impossible to connect.

There are many ways to fix the issue if it is a firewall.
You could create firewall rules that specifically permit the connection with higher priority than the rules that deny communication.
For example, if the rules from the following set of commands are given high priority, they will allow for UDP connection for host PC IP address `192.168.1.15` and controller IP address `192.168.1.31` on port `8888` for ROS2 communication and on port `21789` for debug communication.

Note that this an extremely narrow set of rules that would permit communication.
Much more concise rules could be applied.

```shell
sudo ufw allow out from 192.168.1.15 to 192.168.1.31 proto udp
sudo ufw allow from 192.168.1.31 to 192.168.1.15 port 8888 proto udp
sudo ufw allow from 192.168.1.31 to 192.168.1.15 port 21789 proto udp
sudo ufw allow from 192.168.1.31 to 192.168.1.255 port 21789 proto udp
```

You will have to ensure that firewall rules allowing for UDP communication have higher priority than those which block communication.

You could also choose to remove firewall rules that are blocking communication, rather than adding rules that explicitly allow communication.

If you wish to disable your firewall to test if it is the source of your problem, you may do so, but be aware of the risks associated with that.

It is possible that firewall rules are not visible via `ufw`, but that they are still in place.
That goes beyond the scope of this troubleshooting guide, but be aware of the possibility.

If you have gone through all steps described here and you are still having issues, save a copy of the output of the [debug-listener script](troubleshooting.md#debug-log-client) and the `PANELBOX.LOG` from the robot's teach pendant.
Open a new issue on the [Issue tracker](https://github.com/yaskawa-global/motoros2/issues), describe the problem and attach `PANELBOX.LOG` and the debug log to the issue.
