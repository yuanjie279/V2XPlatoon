# Echelon-based Collaborative Resource Allocation (ECRA) Protocol Simulator 
---------------------------------
December 25th, 2024 — Research Implementation  

This repository contains the implementation of the Echelon-based Collaborative Resource Allocation (ECRA) protocol for vehicular platoon communications, along with the Semi-Persistent Scheduling (SPS) baseline for performance comparison.  
The ECRA protocol introduces intelligent error detection mechanisms, including classification of Resource Conflict Error (RCE), Channel Interference Error (CIE), and Signal Attenuation Error (SAE) using fuzzy logic approaches.  
We implemented role-based differentiated processing for Leader (L), Key Follower (KF), and Standard Follower (SF) vehicles, incorporating adaptive power control and dynamic waiting window adjustments.  
The simulator operates with real-world vehicle trajectory data and supports Nakagami fading channel modeling for realistic performance evaluation.  
---------------------------------

This simulator is designed for Enhanced Cooperative Resource Allocation in C-V2X Mode 4 vehicular platoon communications.  
The default parameter setting assumes a 100 ms transmission interval (10 Hz beacon rate).  
The simulation supports vehicle role classification, intelligent error type detection, dynamic power adjustment, and adaptive resource reselection mechanisms.  

## Implementation Structure

- **ECRA.py** — Main ECRA protocol implementation with fuzzy logic error classification  
- **SPS.py** — Semi-Persistent Scheduling baseline protocol for comparison  
- **utils.py** — Utility functions for SINR calculation, distance measurement, and resource selection  

## Core Features

The ECRA protocol implements the following key mechanisms:
- Intelligent Error Detection: Fuzzy logic-based classification of RCE, CIE, and SAE errors  
- Role-Based Processing: Differentiated strategies for Leader, Key Follower, and Standard Follower vehicles  
- Dynamic Power Control: Adaptive transmit power adjustment for Signal Attenuation Error mitigation  
- Resource Quality Evaluation: SINR and RSSI-based resource assessment with role-specific weighting  
- Waiting Window Optimization: Channel Congestion and Resource Interference (CCRI) based window adjustment  

## Acknowledgement

The SUMO-based simulation environment setup in this repository is partially inspired by the implementation from  
[Simulators-for-SPS](https://github.com/xinuvic/Simulators-for-SPS).  
We gratefully acknowledge their contribution to the community.

## Notes

This repository provides the open-source implementation of the ECRA protocol simulator.  
It contains the core mechanisms and functionalities for research and community use.  
Certain extended modules and experimental features used in internal studies are not included in this release.  
Nevertheless, this open-source version is fully functional for simulation, evaluation, and further development.  
