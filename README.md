# pynocchio

A simple python pip package implementing a wrapper for pinocchio library's robot model object. It implements in a simple manner several useful functions:

- direct kinematics `forward`
- inverse kinematics `ik`
- jacobian calculation `jacobain`
- jacobian time derivative calculation `jacobain_dot`
- jacobian pseudo-inverse `jacobian_pseudo_inv`
- gravity vector `gravity_torque`
- mass matrix calculaiton `mass_matrix`
- coriolis matrix calculaiton `coriolis_matrix`

see the [docs](https://auctus-team.gitlabpages.inria.fr/people/antunskuric/pynocchio)

## Install the package

```bash
pip install git+https://gitlab.inria.fr/auctus-team/people/antunskuric/pynocchio.git
```

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

## Example of using the code

### Example 1: Creating a `RobotWrapper` object and calculating the forward kinematics for the end effector.

```python
import numpy as np
from pyocchio import RobotWrapper

# Define the path to the URDF file
robot_path = '/path/to/robot.urdf'

# Create the RobotWrapper object
robot = RobotWrapper(robot_path=robot_path)

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
from pyocchio import RobotWrapper

# Define the path to the URDF file
robot_path = '/path/to/robot.urdf'

# Create the RobotWrapper object
robot = RobotWrapper(robot_path=robot_path)

# Define the current robot configuration
q = np.zeros(robot.n)

# Calculate the Jacobian matrix for the end effector
J = robot.jacobian(q)

# Print the resulting Jacobian matrix
print(J)
```

In both examples, the `RobotWrapper` class is imported from the `pyocchio` module. The first example calculates the forward kinematics for the end effector of the robot given a certain configuration. The second example calculates the Jacobian matrix for the end effector of the robot given a certain configuration. In both cases, the `RobotWrapper` object is constructed with the path to the URDF file, and then the required function is called with the current robot configuration.



see the [tutorials](https://auctus-team.gitlabpages.inria.fr/people/antunskuric/pynocchio/examples/) for more examples.
