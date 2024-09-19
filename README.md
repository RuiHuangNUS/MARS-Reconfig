# MARS-Reconfig
The **MARS-Reconfig** is a 

|                     A Diagram of a XX             |
:----------------------------------------------------------------------------------------------------------------------------------:
![diagram](https://github.com/RuiHuangNUS/MARS-Reconfig/blob/main/Picture/Fig1.png)
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
The project consists of two folders, which correspond to the ********Algorithm** and **Simulation** in the paper that show the following two advantages of our method.
To address the safety and controllability challenges in the self-reconfiguration process while minimizing the number of reconfiguration steps, we propose a controllability theory-based self-reconfiguration algorithm that incorporates a more practical MARS model with constrained control inputs. Compared to existing state-of-the-art methods [[1]](#1), our approach offers an interpretable and quantifiable measure for determining the robustness of configurations. Specifically, our key contributions include:
1. We propose an efficient controllability analysis for MARS using a quasi-static model under control constraints.
2. We propose a novel controllability margin (CM)--based method for calculating optimal self-reconfiguration sequences.


## 2. Dependency Packages
Please make sure that the following packages have already been installed before running the source code.
* CoppeliaSim: version 4.6.0 Info: https://www.coppeliarobotics.com/
* XXX-learn: version 1.0.2 Info: https://scikit-learn.org/stable/whats_new/v1.0.html

## 3. How to Use
First and foremost, the implementation for Auto-Multilift is straightforward to setup. The source code has been comprehensively annotated to facilitate ease of use. To reproduce the simulation results presented in the paper, simply follow the steps outlined below, sequentially, after downloading and decompressing all the necessary folders.


### A: Algorithm 1 Find Optimal Reconfiguration
 Auto-Multilift       |      Safe-PDP (Open-loop Learning)
:---------------------------------------------------------------:|:--------------------------------------------------------------:
![cl_training](https://github.com/RCL-NUS/Auto-Multilift/assets/70559054/079f47af-ca09-4c64-84f7-152fc96fa71e) | ![ol_training](https://github.com/RCL-NUS/Auto-Multilift/assets/70559054/6762dab9-4859-454d-88d0-d64ca6a2affa)



1. Open the Python file '**main_distributed_autotuning_acados.py**' in the folder '**Algorithm**'
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
|                     Learning Process of the Same Large Multilift System                                                                |
:----------------------------------------------------------------------------------------------------------------------------------------:
![training_tension_ref_cl](https://github.com/RCL-NUS/Auto-Multilift/assets/70559054/e7942afd-684f-4600-acd3-ff3710992ed6)

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
<a id="1">[1]</a> 

