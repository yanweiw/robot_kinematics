# robot_kinematics

This package generates a class-structure of the kinematic chain of any robot arm given its DH (Denavit-Hartenberg) parameters. With this structure you can compute the following:
- Forward Kinematics; i.e. compute end-effector pose (position + orientation) given joint positions
- Compute Jacobian; i.e. compute the geometric Jacobian of the current joint configuration
- Inverse Kinematics; i.e. compute joint positions for a desired end-effector pose using Pseudo-Inv.+Nullspace or DLS?

The class is implemented in both `c++` and `python`, can be used in real-time. We provide usage examples with: 
 * a 7-DOF Kuka LWR-4+
 * a 6-DOF UR10 robot arm 
 * a 6-DOF Mistubishi robot arm (Cobot model)

**Note:** The code in this package has been adapted from [robot-kinematics](https://github.com/epfl-lasa/robot-kinematics) and [Robot-kinematic](https://github.com/epfl-lasa/Robot-kinematic).

**References** Refer to this [IKsurvey](https://www.math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf) for a background on the computations above. DH parameters specific to the UR arms are taken from this [technical report](https://smartech.gatech.edu/bitstream/handle/1853/50782/ur_kin_tech_report_1.pdf). Specific code for the UR arm is adapted from this [repo](https://github.com/mc-capolei/python-Universal-robot-kinematics).

---
## System Requirements
* This code was written for ROS Indigo in Ubuntu 14.04.
* It may work out-of-the-box on ROS Indigo in Ubuntu 16.04, needs to be tested

## Installation, Dependencies and Compilation
Do the following steps:
* In your catkin src directory clone the repository
```
$ git clone https://github.com/nbfigueroa/robot_kinematics.git
```
* wstool gets all other git repository dependencies, after the following steps you should see extra catkin 
  packages in your src directory.
```
$  wstool init
$  wstool merge robot_kinematics/dependencies.rosinstall 
$  wstool up 
```
* Query and installs all libraries and packages 
```
$ rosdep install --from-paths . --ignore-src --rosdistro indigo 
```
* Finally complie
  ```bash
  $ cd ~/catkin_ws
  $ catkin_make
  $ source devel/setup.bash
  $ catkin_make
  ```
  You might need to source the `./bashrc` file and compile again if the first compliation could not find some of the in-house dependencies. If `roscd` doesn't find the compiled packages run `rospack profile`.

---
## Usage
### robot_kinematics_cpp
............
............
### robot_kinematics_python
............
............

---
**Contact**: [Nadia Figueroa](https://nbfigueroa.github.io/) (nadiafig AT mit dot edu)
