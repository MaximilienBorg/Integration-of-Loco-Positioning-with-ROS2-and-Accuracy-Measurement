# Integration of Loco Positioning with ROS 2 and Accuracy Measurement

This repository contains the ROS 2 (Humble) architecture and data analysis scripts for an autonomous multi-robot tracking system. It enables a Crazyflie 2.1 nano-UAV to dynamically track a TurtleBot 4 UGV by fusing absolute Ultra-Wideband (UWB) positioning with decentralized middleware.

Developed as part of the robotics curriculum at the Faculté Polytechnique de Mons (UMONS).

## System Setup

To bypass WSL 2 USB passthrough constraints, the deployment utilizes a hybrid bridging technique:

* **WSL 2 (Ubuntu 22.04):** Hosts the ROS 2 graph and custom tracking algorithms.
* **Windows Host:** Runs the native Bitcraze `cfclient` and interfaces with the Crazyradio PA.
* **Network:** DDS UDP multicast bridged between the Linux subsystem and Windows.

## Localization Accuracy Measurement

This part quantifies the accuracy of the Loco Positioning System (LPS, TDoA3 mode) against a Qualisys optical motion-capture system used as millimeter-scale ground truth, establishing the positioning uncertainty the tracking controller has to absorb.

* **Setup:** Crazyflie 2.1 with the Loco Positioning Deck, six UWB anchors deployed in a staggered low/high arrangement over a ~4 m × 3.7 m × 2 m volume, with Qualisys as ground truth.
* **Acquisition:** LPS positions logged on the host PC as `flight_log_*.csv`; Qualisys marker positions exported from QTM as a TSV file.
* **Analysis (`trajectory_comparison.py`):** loads both files, filters LPS outliers, synchronizes them by file timestamp, resamples onto a common grid, aligns the frames by a constrained yaw rotation, and reports the RMSE, per-axis errors and error distributions.
