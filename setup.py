## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        "robot_kinematics_python",
        "robot_kinematics_python.generic_kinematics_model",
        "robot_kinematics_python.ur_kinematics_model",
    ],
    package_dir={'': 'robot_kinematics_python'},
)
setup(**d)
