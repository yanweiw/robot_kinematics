# robot_kinematics

This package generates a class-structure of the kinematic chain of any robot arm given its DH (Denavit-Hartenberg) parameters. With this structure you can compute the following:
- Forward Kinematics; i.e. compute end-effector pose (position + orientation) given joint positions
- Compute Jacobian; i.e. compute the geometric Jacobian of the current joint configuration
- Inverse Kinematics; i.e. compute joint positions for a desired end-effector pose using Pseudo-Inv.+Nullspace or DLS?

The class is implemented in both `c++` and `python`, can be used in real-time. We provide usage examples with: 
 * a 7-DOF Kuka LWR-4+
 * a 6-DOF UR10 robot arm 
 * a 6-DOF Mistibishi robot arm (model ...)

**Note:** The code in this package was forked from [robot-kinematics](https://github.com/epfl-lasa/robot-kinematics) and [Robot-kinematic](https://github.com/epfl-lasa/Robot-kinematic)

**References** Refer to this [IKsurvey](https://www.math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf) for a background on the computations above. DH parameters specific to the UR arms are taken from this [technical report](https://smartech.gatech.edu/bitstream/handle/1853/50782/ur_kin_tech_report_1.pdf). Specific code for the UR arm is adapted from this [repo](https://github.com/mc-capolei/python-Universal-robot-kinematics).

---
## System Requirements
............
............


## Installation, Dependencies and Compilation
............
............



---
## Usage
............
............


---
## Contact
