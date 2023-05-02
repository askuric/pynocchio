import numpy as np
import os

import pynocchio as pynoc

dir_path = os.path.dirname(os.path.abspath(pynoc.__file__))

panda = pynoc.RobotWrapper("panda_link8", dir_path+"/models/urdf/panda.urdf", mesh_path=dir_path+"/models")

q0 = np.random.uniform(panda.q_min,panda.q_max)
dq0 = np.random.uniform(panda.dq_min,panda.dq_max)

panda.update_joint_data(q=q0, dq=dq0)
panda.update_visualisation()

print("initial q\n", q0)
print("initial dq\n", dq0)

M = panda.mass_matrix(q0)
print("mass matrix\n",M)
C = panda.coriolis_matrix(q0,dq0)
print("coriolis matrix\n",C)
g = panda.gravity_torque(q0)
print("gravity vector\n",g)
Jdot = panda.jacobian_dot(q0,dq0)
print("jacobian time derivative\n",Jdot)