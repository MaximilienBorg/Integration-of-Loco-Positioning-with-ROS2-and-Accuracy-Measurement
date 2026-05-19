# Integration of Loco Positioning with ROS 2 and Accuracy Measurement

This repository contains the ROS 2 (Humble) architecture and data analysis scripts for an autonomous multi-robot tracking system. It enables a **Crazyflie 2.1 nano-UAV** to dynamically track a **TurtleBot 4 UGV** by fusing absolute Ultra-Wideband (UWB) positioning with decentralized middleware.

Developed as part of the robotics curriculum at the **Faculté Polytechnique de Mons (UMONS)**.

## System Setup
To bypass WSL 2 USB passthrough constraints, the deployment utilizes a hybrid bridging technique:
* **WSL 2 (Ubuntu 22.04):** Hosts the ROS 2 graph and custom tracking algorithms.
* **Windows Host:** Runs the native Bitcraze `cfclient` and interfaces with the Crazyradio PA.
* **Network:** DDS UDP multicast bridged between the Linux subsystem and Windows.

