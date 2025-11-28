# PYNOCCHIO

A simple python pip package implementing a wrapper for pinocchio library's robot model object. It implements in a simple manner several useful functions:

- direct kinematics `forward`
- inverse kinematics `ik`
- jacobian calculation `jacobian`
- jacobian time derivative calculation `jacobian_dot`
- jacobian pseudo-inverse `jacobian_pseudo_inv`
- gravity vector `gravity_torque`
- mass matrix calculaiton `mass_matrix`
- coriolis matrix calculaiton `coriolis_matrix`

see the [docs](https://auctus-team.gitlabpages.inria.fr/people/antunskuric/pynocchio)

## Installation
```bash
pip install git+https://gitlab.inria.fr/auctus-team/people/antunskuric/pynocchio.git
```

## Usage

```python
import os
import numpy as np

import pynocchio as pynoc

tip_name = "panda_link8"
pynoc_abs_path = os.path.dirname(os.path.abspath(pynoc.__file__))
urdf_path = "models/urdf/panda.urdf"
# there should be a folder meshes with 2 subfolders visual and collision containing the mesh files
mesh_path = pynoc_abs_path + "/models" 

q0 = np.array([np.pi/2,-0.2,0,-2.1,0,1.9,0])

robot = pynoc.RobotWrapper(tip_name, urdf_path, mesh_path=mesh_path, q=q0)

fv = np.array([0.07, 0.20, 0.04, 0.23, 0.10, -0.01, 0.06])
robot_fric = FrictionRobotWrapper(robot, fv)
```

Parameters:
- tip (str): Name of the robot's end-effector frame.
- urdf_path (str or None): Path to the robot's URDF file (optional, either urdf or xml).
- xml_path (str or None):  Path to the robot's XML file (optional, either urdf or xml).
- mesh_path (str or None): Absolute path to the robot's meshes folder for visualization (optional).
- q (np.ndarray or None): Array containing the robot's initial joint positions (optional).


## Testing the code

```shell
pytest tests/test.py
```

and coverage report

```shell
coverage run -m pytest tests/test.py
coverage report -m
```

and for html version

```shell
coverage html
```

## Examples

### Example 1: Creating a `RobotWrapper` object and calculating the forward kinematics for the end effector.

```python
import numpy as np
from pynocchio import RobotWrapper

# Define the path to the URDF file
robot_path = '/path/to/robot.urdf'

# Create the RobotWrapper object
robot = RobotWrapper(urdf_path=robot_path)

# Define the current robot configuration
q = np.zeros(robot.n)

# Calculate the forward kinematics for the end effector
oMf = robot.forward(q)

# Print the resulting transformation matrix
print(oMf)
```

### Example 2: Creating a `RobotWrapper`  object and calculating the Jacobian matrix for the end effector.

```python
import numpy as np
from pynocchio import RobotWrapper

# Define the path to the URDF file
robot_path = '/path/to/robot.urdf'

# Create the RobotWrapper object
robot = RobotWrapper(urdf_path=robot_path)

# Define the current robot configuration
q = np.zeros(robot.n)

# Calculate the Jacobian matrix for the end effector
J = robot.jacobian(q)

# Print the resulting Jacobian matrix
print(J)
```

In both examples, the `RobotWrapper` class is imported from the `pynocchio` module. The first example calculates the forward kinematics for the end effector of the robot given a certain configuration. The second example calculates the Jacobian matrix for the end effector of the robot given a certain configuration. In both cases, the `RobotWrapper` object is constructed with the path to the URDF file, and then the required function is called with the current robot configuration.



see the [tutorials](https://auctus-team.gitlabpages.inria.fr/people/antunskuric/pynocchio/examples/) for more examples.
