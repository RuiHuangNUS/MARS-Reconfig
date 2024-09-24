# MARS-Reconfig (coming soon)
The **MARS-Reconfig** is a 
|                     A video  of this project             |
:----------------------------------------------------------------------------------------------------------------------------------:
[![NetFlix on UWP](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/movie_cover.png?raw=true)](https://youtu.be/SB0hwK33088 "NetFlix on UWP")
https://youtu.be/SB0hwK33088
|                     A diagram of a XX             |
<div align="center">
  <img src="https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/Fig1.png?raw=true" alt="diagram" width="400"/>
</div>

MARS is tasked to track a spiral trajectory with two faulty units. The faulty propellers are marked red. (a) MARS crashed after the complete failure of two units (all rotors are broken). (b) MARS can track trajectories after self-reconfiguration. 

Please find out more details in our paper: "Robust Self-Reconfiguration for Fault-Tolerant Control of Modular Aerial Robot Systems" 
## Table of contents
1. [Project Overview](#project-Overview)
2. [Dependency Packages](#Dependency-Packages)
3. [How to Use](#How-to-Use)
      1. [A: Distributed Learning of Adaptive Weightings](#A-Distributed-Learning-of-Adaptive-Weightings)
      2. [B: Distributed Learning of Adaptive References](#B-Distributed-Learning-of-Adaptive-References)
4. [Contact Us](#Contact-Us)

## Why Self-Reconfiguration?

  (a)  Before Re-configuration    |   (b) After Re-configuration (Ours) 
:---------------------------------------------------------------:|:--------------------------------------------------------------:
![cl_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/3x2_fault_track.gif) | ![ol_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/3x2_self_reconfiguration_track.gif)

## How to Self-Reconfigure?
1. Quantifying the Optimal Configuration

We Calculate the optimal configuration with maximum remaining control authority using controllability margin (CM)
  (a) Calculate the Optimal Reconfiguration in a 3×2 assembly     |   (b) Calculate the Optimal Reconfiguration in a 3×3 assembly 
:---------------------------------------------------------------:|:--------------------------------------------------------------:
![cl_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/robot_configuration_3x2.gif) | ![ol_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/robot_configuration_3x3.gif)

2. Ensures the safe transfer of all units
We designed the Minimum Controllable Subassembly to enable the transfer of faulty units.
<div align="center">
  <img src="https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/self_reconfiguration_flow.png?raw=true" alt="diagram" width="400"/>
</div>
3. Examples

 (a) 3×2 assembly: full disassembly  |   (b) 3×2 assembly: partial disassembly 
:---------------------------------------------------------------:|:--------------------------------------------------------------:
![cl_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/full_self_reconfiguration_3x2.gif) | ![ol_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/partial_self_reconfiguration_3x2.gif)
 (c) 3×3 assembly: full disassembly  |   (d) 3×3 assembly: partial disassembly 
![cl_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/full_self_reconfiguration_3x3.gif) | ![ol_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/partial_self_reconfiguration_3x3.gif)

## Comparison with the baseline method[1]

 (a) 3×2 assembly: full disassembly  |   (b) 3×2 assembly: partial disassembly 
:---------------------------------------------------------------:|:--------------------------------------------------------------:
![cl_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/3x2_comparison.gif) | ![ol_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/3x3_comparison_origin.gif)
 (c) 3×3 assembly: full disassembly  |   (d) 3×3 assembly: partial disassembly 
![cl_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/3x3_full.gif) | ![ol_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/3x3_partial.gif)

## Trajectory tracking
<div align="center">
  <img src="https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/3x2_full_track.gif?raw=true" alt="diagram" width="400"/>
</div>

## 1. Project Overview
The project consists of two folders, which correspond to the **Algorithm** and **Simulation** in the paper that show the following two advantages of our method.
1. We propose an efficient controllability analysis for MARS using a quasi-static model under control constraints.
2. We propose a novel controllability margin (CM)--based method for calculating optimal self-reconfiguration sequences.


## 2. Dependency Packages
Please make sure that the following packages have already been installed before running the source code.
* CoppeliaSim: version 4.6.0 Info: https://www.coppeliarobotics.com/
* imageio: version X.x.x Info: https://scikit-learn.org/stable/whats_new/v1.0.html

## 3. How to Use
First and foremost, the implementation for Auto-Multilift is straightforward to setup. The source code has been comprehensively annotated to facilitate ease of use. To reproduce the simulation results presented in the paper, simply follow the steps outlined below, sequentially, after downloading and decompressing all the necessary folders.
All the control methods of different configurations are based on previous works [[1]](#1).

### A: Algorithm 1 Find Optimal Reconfiguration

1. Open the Python file '**Algorithm1_Find_Optimal_Reconfiguration.py**' in the folder '**Algorithm**'
2. Before running, please do the following settings:
   * Set the number of quadrotors on line 42 (i.e., the fifth number in the 'uav_para' array).
   * Set the load mass value on line 43 (i.e., the first number in the 'load_para' array).
   * Set the MPC horizon on line 52 (the default value is 10).
   * Set the higher-level loss horizon on line 53 (the default value is 20).

### B: Algorithm 3 Plan Disassembly and Assembly Sequence

1. Open the Python file '**XXX.py**' in the folder '**Algorithm**'
2. Before running, please do the following settings:
   * Set the number of quadrotors on line 40 to the same as that used in Section A (i.e., learning of adaptive weightings). 
   * Set the load mass value on line 43 (i.e., the first number in the 'load_para' array).
   * Set the MPC horizon on line 52 (the default value is 10).
   * Set the higher-level loss horizon on line 53 (the default value is 20).

### C: Simulation 3: Full disassembly in a 3×3 assembly
1. Simulation 1: Full disassembly in a 3×2 assembly
2. Simulation 2: Partial disassembly in a 3×2 assembly
3. Simulation 3: Full disassembly in a 3×3 assembly
4. Simulation 4: Partial disassembly in a 3×3 assembly
1.Full disassembly in a 3×3 assembly: Open the file '**test.ttt**' in the folder '**Simulation**'

## 4. Contact Us
If you encounter a bug in your implementation of the code, please do not hesitate to inform me.
* Name: Mr. Rui Huang
* Email: ruihuang@nus.edu.sg

## References
<a id="1">[1]</a> HUANG, R., SHENG, H., Qian, C. H. E. N., Ziting, R. A. N., Zhen, X. U. E., Jiacheng, L. I., & Tong, L. I. U. (2024). Adaptive configuration control of combined UAVs based on leader-wingman mode. Chinese Journal of Aeronautics.

