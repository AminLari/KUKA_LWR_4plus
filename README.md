## KUKA LWR4+ Robot Dynamics and Kinematics

## Overview
This project focuses on simulating the kinematic and dynamic model of the KUKA LWR4+, an industrial serial robot, using MATLAB. It includes inverse dynamics, forward and inverse kinematics, and linearization of motion equations.
<p> <img src="https://github.com/user-attachments/assets/973709a6-537d-41a8-b6b0-fd4203f781aa" width="1000"> </p>  

## Features
- **Dynamic Modeling:** Derives and simulates the robot's equations of motion.
- **Kinematics:** Implements forward and inverse kinematics using the DH convention.
- **Matrix Calculations:** Extracts key matrices (M, V, and G) for dynamic analysis.
- **Visualization:** Simulates and visualizes robot motion in MATLAB.

## File Structure
- `KUKA_Linear.slx`: Simulink model for linearizing the manipulator's equations of motion.
- `Robotics_Project.m`: MATLAB script for forward and inverse kinematics and Jacobian matrix computation.
- `Transform.m`: Computes homogeneous transformations based on DH parameters.
- `matrix_calculation.m`: Extracts M, V, and G matrices from torque equations.

---

## Installation

### Prerequisites
- MATLAB version R2022b or later (with Simulink recommended)
- Robotics Toolbox for MATLAB (optional for advanced features)

### Steps
1. **Clone the repository:**
   ```bash
   git clone https://github.com/AminLari/KUKA_LWR_4plus.git
   cd KUKA_LWR_4plus

2. **Run the MATLAB scripts:**
   Use the provided .m files to perform kinematic and dynamic analyses.

3. **Simulate the model:**
   Open KUKA_Linear.slx in MATLAB Simulink and run the simulation for motion analysis.

## Usage
1. **Analyze Kinematics:**
   Run Robotics_Project.m to perform forward and inverse kinematics and compute the Jacobian matrix.

2. **Simulate Dynamics:**
   Use matrix_calculation.m to extract dynamic matrices (M, V, G).
   
3. **Compute Transformations:**
   Use Transform.m to compute and visualize homogeneous transformations.
   
## Results
- Simulate the equations of motion in KUKA_Linear.slx.
<p> <img src="https://github.com/user-attachments/assets/d7ad30f7-1c93-465c-b701-3261c972ad2e" width="1000"> </p>  

- Simulation of trajectory of the robotic arm
<video width="640" height="360" controls>
<source src="https://github.com/user-attachments/assets/cb44c257-c2fb-48c4-86db-4190d95fd581" type="video/mp4">
</video>

## Contact
For questions or suggestions, please contact Amin Lari.

