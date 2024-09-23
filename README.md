# MARS-Reconfig (coming soon)
The **MARS-Reconfig** is a 
[![NetFlix on UWP](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/movie_cover.png?raw=true)](https://youtu.be/SB0hwK33088 "NetFlix on UWP")
|                     A Diagram of a XX             |
:----------------------------------------------------------------------------------------------------------------------------------:
<div align="center">
  <img src="https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/Fig1.png?raw=true" alt="diagram" width="500"/>
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

## 1. Project Overview
The project consists of two folders, which correspond to the **Algorithm** and **Simulation** in the paper that show the following two advantages of our method.
1. We propose an efficient controllability analysis for MARS using a quasi-static model under control constraints.
2. We propose a novel controllability margin (CM)--based method for calculating optimal self-reconfiguration sequences.


## 2. Dependency Packages
Please make sure that the following packages have already been installed before running the source code.
* CoppeliaSim: version 4.6.0 Info: https://www.coppeliarobotics.com/
* imageio: version X.x.x Info: https://scikit-learn.org/stable/whats_new/v1.0.html
* matplotlib: version X.x.x Info: https://scikit-learn.org/stable/whats_new/v1.0.html
* matplotlib: version X.x.x Info: https://scikit-learn.org/stable/whats_new/v1.0.html

## 3. How to Use
First and foremost, the implementation for Auto-Multilift is straightforward to setup. The source code has been comprehensively annotated to facilitate ease of use. To reproduce the simulation results presented in the paper, simply follow the steps outlined below, sequentially, after downloading and decompressing all the necessary folders.
All the control methods of different configurations are based on previous works [[1]](#1).

### A: Algorithm 1 Find Optimal Reconfiguration
  CM calculation in a 3×2 assembly (one failure)    |   CM calculation in a 3×3 assembly (one failure) 
:---------------------------------------------------------------:|:--------------------------------------------------------------:
![cl_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/robot_configuration_3x2.gif) | ![ol_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/robot_configuration_3x3.gif)

1. Open the Python file '**Algorithm1_Find_Optimal_Reconfiguration.py**' in the folder '**Algorithm**'
2. Before running, please do the following settings:
   * Set the number of quadrotors on line 42 (i.e., the fifth number in the 'uav_para' array).
   * Set the load mass value on line 43 (i.e., the first number in the 'load_para' array).
   * Set the MPC horizon on line 52 (the default value is 10).
   * Set the higher-level loss horizon on line 53 (the default value is 20).
4. After completing the above settings, run the file '**main_distributed_autotuning_acados.py**'. In the prompted terminal interface, you will be asked to select the control and sensitivity propagation modes.
   * In our settings, 's' and 'p' denote 'sequential' and 'parallel' computing, respectively.
   * 'c' and 'o' represent 'closed-loop' (our method) and 'open-loop' (the Safe-PDP method [[1]](#1)) training modes.
5. To evaluate the trained model, run the Python file '**main_distributed_autotuning_evaluation_acados.py**'
   * You can skip Step 4 and evaluate the saved models that were previously trained and employed in the paper. To do so, copy the files that end with '.pt' from the folder '**Previously trained models**' (within the folder '**Source code A**') to the folder '**trained data**' (where the retained models via Step 4 will be saved).


### B: Algorithm 3 Plan Disassembly and Assembly Sequence
  Full self-reconfiguration in a 3×2 assembly (one failure)      |   Partial self-reconfiguration in a 3×2 assembly (one failure) 
:---------------------------------------------------------------:|:--------------------------------------------------------------:
![cl_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/full_self_reconfiguration_3x2.gif) | ![ol_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/partial_self_reconfiguration_3x2.gif)

  Full self-reconfiguration in a 3×3 assembly (one failure)    |   Partial self-reconfiguration in a 3×3 assembly (one failure) 
:---------------------------------------------------------------:|:--------------------------------------------------------------:
![cl_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/full_self_reconfiguration_3x3.gif) | ![ol_training](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/partial_self_reconfiguration_3x3.gif)

1. Open the Python file '**XXX.py**' in the folder '**Algorithm**'
2. Before running, please do the following settings:
   * Set the number of quadrotors on line 40 to the same as that used in Section A (i.e., learning of adaptive weightings). 
   * Set the load mass value on line 43 (i.e., the first number in the 'load_para' array).
   * Set the MPC horizon on line 52 (the default value is 10).
   * Set the higher-level loss horizon on line 53 (the default value is 20).
4. After completing the above settings, run the file '**main_distributed_autotuning_acados_tensionref.py**'. In the prompted terminal interface, you will be asked to select the control and sensitivity propagation modes.
   * In our settings, 's' and 'p' denote 'sequential' and 'parallel' computing, respectively.
5. To evaluate the trained model, run the Python file '**main_distributed_acados_tensionref_evaluation.py**'
   * You can skip Step 4 and evaluate the saved models that were previously trained and employed in the paper. To do so, copy the files that end with '.pt' from the folder '**Previously trained models**' (within the folder '**Source code B**') to the folder '**trained data**' (where the retained models via Step 4 will be saved).

### C: Simulation 1: Full disassembly in a 3×2 assembly
1. Open the file '**test.ttt**' in the folder '**Simulation**'

### D: Simulation 2: Partial disassembly in a 3×2 assembly
1. Open the file '**test.ttt**' in the folder '**Simulation**'

### E: Simulation 3: Self-reconfiguration flow after failure of unit No.3 in a 3×2 assembly
1. Open the file '**test.ttt**' in the folder '**Simulation**'

### F: Simulation 4: Full disassembly in a 3×3 assembly
1. Open the file '**test.ttt**' in the folder '**Simulation**'

### G: Simulation 5: Partial disassembly in a 3×3 assembly
1. Open the file '**test.ttt**' in the folder '**Simulation**'

## 4. Contact Us
If you encounter a bug in your implementation of the code, please do not hesitate to inform me.
* Name: Mr. Rui Huang
* Email: ruihuang@nus.edu.sg

## References
<a id="1">[1]</a> HUANG, R., SHENG, H., Qian, C. H. E. N., Ziting, R. A. N., Zhen, X. U. E., Jiacheng, L. I., & Tong, L. I. U. (2024). Adaptive configuration control of combined UAVs based on leader-wingman mode. Chinese Journal of Aeronautics.

