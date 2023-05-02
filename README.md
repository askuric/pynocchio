# PYNOCCHIO

A simple python pip package implementing a wrapper for pinocchio library's robot model object. It implements in a simple manner several useful functions:

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
- mesh_path (str or None): Path to the robot's meshes folder for visualization (optional).
- q (np.ndarray[float] or None): Array containing the robot's initial joint positions (optional).

## Methods:

### RobotWrapper

- **forward**(q=None, frame_name=None) -> pin.SE3: *Calculates the forward kinematics of the robot.*
- **dk_position**(q=None, frame_name=None) -> np.ndarray[float]: *Calculates the position of the robot frame in the world frame.*
- **dk_orientation_matrix**(q=None, frame_name=None) -> np.ndarray[np.ndarray[float]]: *Calculates the orientation matrix of the robot frame in the world frame.*
- **jacobian**(q=None, frame_name=None, frame_align=pin.ReferenceFrame.LOCAL_WORLD_ALIGNED) -> np.ndarray[np.ndarray[float]]: *Calculates the Jacobian matrix of the robot.*
- **jacobian_dot**(q=None, dq=None, frame_name=None, frame_align=pin.ReferenceFrame.LOCAL_WORLD_ALIGNED) -> np.ndarray[np.ndarray[float]]: *Calculates the time derivative of the Jacobian matrix.*
- **jacobian_position**(q=None, frame_name=None, frame_align=pin.ReferenceFrame.LOCAL_WORLD_ALIGNED) -> np.ndarray[np.ndarray[float]]: *Calculates the position Jacobian matrix of the robot.*
- **jacobian_pseudo_inv**(q=None, frame_name=None, frame_align=pin.ReferenceFrame.LOCAL_WORLD_ALIGNED) -> np.ndarray[np.ndarray[float]]:* Calculates the pseudo-inverse of the Jacobian matrix.*
- **jacobian_weighted_pseudo_inv**(W: np.ndarray[np.ndarray[float]], q=None, frame_name=None, frame_align=pin.ReferenceFrame.LOCAL_WORLD_ALIGNED) -> np.ndarray[np.ndarray[float]]: *Calculates the weighted pseudo-inverse of the Jacobian matrix.*
- **gravity_torque**(q=None) -> np.ndarray[float]: *Calculates the gravity torque vector.*
- **mass_matrix**(q: np.ndarray[float]) -> np.ndarray[np.ndarray[float]]: *Calculates the mass matrix.*
- **coriolis_matrix**(q=None, dq=None) -> np.ndarray[np.ndarray[float]]: *Calculates the Coriolis matrix.*
- **ik**(oMdes: pin.SE3, q=None, verbose=True) -> np.ndarray[float]: *Performs iterative inverse kinematics to find the robot's configuration corresponding to a desired pose.*
- **direct_dynamic**(tau: np.ndarray[float], q=None, dq=None, f_ext=None) -> np.ndarray[float]: *Calculates the joint accelerations using the direct dynamic model.*

Note: The RobotWrapper class assumes the pinocchio (pin) and meshcat library are installed. The numpy library is also required.

### FrictionRobotWrapper

- **viscous_friction_torque**(self, dq=None) -> np.ndarray[float]: *Calculates the viscous friction torque vector.*
- **coulomb_friction_torque**(self, dq=None) -> np.ndarray[float]: *Calculates the Coulomb friction torque vector.*
- **viscous_friction_offset_torque**(self) -> np.ndarray[float]: *Calculates the Coulomb friction torque vector offset.*

Note: The FrictionRobotWrapper class assumes inherits properties and methods from RobotWrapper. `direct_dynamic` function is modified according to friction model.


<!-- 

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

see the [docs](docs/pynocchio.RobotWrapper.md)

## Install the package

```bash
pip install git+https://gitlab.inria.fr/auctus-team/people/antunskuric/pynocchio.git
```


## Examples 

A simple example of using the wrapper
```python
from pynocchio import RobotWrapper
import numpy as np
panda = RobotWrapper("panda_link8", "panda.urdf")

q0 = np.random.uniform(panda.q_min,panda.q_max)
print("initial q\n", q0)
oMq0 = panda.forward(q0)
print("direct kinamtics for q\n", oMq0)
q_ik = panda.ik(oMq0, verbose=False)
print("ik found q\n", q_ik)
oMq_ik = panda.forward(q_ik)
print("direct kinamtics for ik found q\n", oMq_ik)
```

see the [examples](examples) folder for more examples. -->
