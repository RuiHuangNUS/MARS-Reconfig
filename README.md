# MARS-Reconfig

## Project Overview
The project **MARS-Reconfig** consists of two folders, which correspond to the **Algorithm** and **Simulation** in the paper that show the following two advantages of our method.
1. We propose an efficient controllability analysis for MARS using a quasi-static model under control constraints.
2. We propose a novel controllability margin (CM)--based method for calculating optimal self-reconfiguration sequences.

Please find out more details in our paper: "Robust Self-Reconfiguration for Fault-Tolerant Control of Modular Aerial Robot Systems" 


|                     A video  of this project             |
:----------------------------------------------------------------------------------------------------------------------------------:
[![NetFlix on UWP](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/movie_cover.png?raw=true)](https://youtu.be/SB0hwK33088 "NetFlix on UWP")
https://youtu.be/SB0hwK33088
|                     A diagram of the self-reconfiguration             |
<div align="center">
  <img src="https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/Fig1.png?raw=true" alt="diagram" width="400"/>
</div>
MARS is tasked to track a spiral trajectory with two faulty units. The faulty propellers are marked red. (a) MARS crashed after the complete failure of two units (all rotors are broken). (b) MARS can track trajectories after self-reconfiguration. 

<!-- ## Table of contents
1. [Project Overview](#project-Overview)
2. [Dependency Packages](#Dependency-Packages)
3. [How to Use](#How-to-Use)
      1. [A: Distributed Learning of Adaptive Weightings](#A-Distributed-Learning-of-Adaptive-Weightings)
      2. [B: Distributed Learning of Adaptive References](#B-Distributed-Learning-of-Adaptive-References)
4. [Contact Us](#Contact-Us) -->

## 1 Why Self-Reconfiguration?

  (a)  Before Re-configuration    |   (b) After Re-configuration (Ours) 
:---------------------------------------------------------------:|:--------------------------------------------------------------:
![cl_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/3x2_fault_track.gif) | ![ol_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/3x2_self_reconfiguration_track.gif)

Advantages:
1. More control authority – Improved robustness against unit faulty
2. Improved trajectory tracking performance

## 2 How to Self-Reconfigure?
### 2.1 Quantifying the optimal configuration

We Calculate the optimal configuration with maximum remaining control authority using controllability margin (CM)
  (a) Calculate the Optimal Reconfiguration in a 3×2 assembly     |   (b) Calculate the Optimal Reconfiguration in a 3×3 assembly 
:---------------------------------------------------------------:|:--------------------------------------------------------------:
![cl_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/robot_configuration_3x2.gif) | ![ol_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/robot_configuration_3x3.gif)

Advantages:
1. No need for optimization with an objective function (Less time consumption)
2. The optimal configuration ensures controllability and is theoretically guaranteed

### 2.2 Ensures the safe transfer of all units
We designed the Minimum Controllable Subassembly to enable the transfer of faulty units.
<div align="center">
  <img src="https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/self_reconfiguration_flow.png?raw=true" alt="diagram" width="400"/>
</div>

Advantages:
The minimum controllable subassembly ensures the safety of faulty units

### 2.3 Examples

 (a) 3×2 assembly: full disassembly  |   (b) 3×2 assembly: partial disassembly 
:---------------------------------------------------------------:|:--------------------------------------------------------------:
![cl_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/full_self_reconfiguration_3x2.gif) | ![ol_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/partial_self_reconfiguration_3x2.gif)
 (c) 3×3 assembly: full disassembly  |   (d) 3×3 assembly: partial disassembly 
![cl_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/full_self_reconfiguration_3x3.gif) | ![ol_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/partial_self_reconfiguration_3x3.gif)

Advantages:
Each step of disassembly and assembly is ensured to be theoretically optimal

## 3 Comparison with the baseline method [[2]](#2)

 (a) 3×3 assembly: full disassembly (comparison) |   (b) 3×3 assembly: partial disassembly (comparison origin)
:---------------------------------------------------------------:|:--------------------------------------------------------------:
![cl_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/3x2_comparison.gif) | ![ol_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/3x3_comparison_origin.gif)
 (c) 3×3 assembly: full disassembly (ours) |   (d) 3×3 assembly: partial disassembly (ours)
![cl_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/3x3_full.gif) | ![ol_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/3x3_partial.gif)

Advantages:
1. Higher controllability margins (Control robustness)
2. With improvements of up to 264.50% (blue) and 138.63% (green) compared to [[2]](#2).
3. Fewer disassembly and assembly times (save energy and time)
4. Reducing the number of steps by 81.81% and 45.45%, respectively.
5. Less oscillation in the reassembled drone after docking and separation.

## 4 Trajectory tracking
<div align="center">
  <img src="https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/3x2_full_track.gif?raw=true" alt="diagram" width="500"/>
</div>

Advantages:
1. No oscillation during trajectory switching after self-reconfiguration
2. Improved trajectory tracking after self-reconfiguration

## 5 How to Use (coming soon)
First and foremost, the implementation for MARS-Reconfig is straightforward to setup. The source code has been comprehensively annotated to facilitate ease of use. To reproduce the simulation results presented in the paper, simply follow the steps outlined below, sequentially, after downloading and decompressing all the necessary folders.
All the control methods of different configurations are based on previous works [[1]](#1).

### 5.1 Dependency Packages
Please make sure that the following packages have already been installed before running the source code.
* CoppeliaSim: version 4.6.0 Info: https://www.coppeliarobotics.com/
* imageio: version 2.9.0 Info: https://imageio.readthedocs.io/

### 5.2 Algorithm 1 Find Optimal Reconfiguration

1. Open the Python file '**Algorithm1_Find_Optimal_Reconfiguration.py**' in the folder '**Algorithm**'
2. Before running, please do the following settings:
   * Set the number of quadrotors on line 225 and 226.
   * Set the Fault status of four rotors on line 229 (the default value is rotor_faults = [True, True, True, True]).
   * Set non-symmetric positions on line 231 (We provided examples of 3x2 and 3x3 assemblies for demonstration).

### 5.3 Algorithm 3 Plan Disassembly and Assembly Sequence

1. Open the Python file '**Algorithm3_Plan_Disassembly_Assembly_Sequence.py**' in the folder '**Algorithm**'
2. Before running, please do the following settings:
   * Set the configuration on line 14. 

### 5.4 Simulation
1. Simulation 1: Full disassembly in a 3×2 assembly, Open the file '**3x2_full_disassembly.ttt**' in the folder '**Simulation**'
2. Simulation 2: Partial disassembly in a 3×2 assembly, Open the file '**3x2_partial_disassembly.ttt**' in the folder '**Simulation**'

## 6 Contact Us
If you encounter a bug in your implementation of the code, please do not hesitate to inform me.
* Name: Mr. Rui Huang
* Email: ruihuang@nus.edu.sg

## References
<a id="1">[1]</a> R. HUANG, H. SHENG, C. Qian, R. Ziting, X. Zhen, L. Jiacheng, and L. Tong, “Adaptive configuration control of combined uavs based on leader-wingman mode,” Chinese Journal of Aeronautics, 2024.

<a id="2">[2]</a> N. Gandhi, D. Saldana, V. Kumar, and L. T. X. Phan, “Self-reconfiguration in response to faults in modular aerial systems,” IEEE Robotics and Automation Letters, vol. 5, no. 2, pp. 2522–2529, 2020.
