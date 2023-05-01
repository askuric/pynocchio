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

see the [examples](examples) folder for more examples.
