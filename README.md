# pynocchio

A simple python pip package implementing a wrapper for pinocchio library's robot model object. It implements in a simple manner several useful functions:

- direct kinematics `forward`
- inverse kinematics `ik`
- jacobian calculation `jacobain`
- jacobian pseudo-inverse `jacobian_pseudo_inv`
- gravity vector `gravity_torque`
- mass matrix calculaiton `mass_matrix`

see the [docs](docs/pynocchio.RobotWrapper.md)

## Install the package

```bash
pip install git+https://gitlab.inria.fr/auctus-team/people/antunskuric/pynocchio.git
```


## Examples

```python
from pynocchio import RobotWapper.RobotWapper 

panda = RobotWrapper("path/to/urdf")

panda.forward(panda.robot.natural())
```