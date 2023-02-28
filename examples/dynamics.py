from pynocchio import RobotWrapper
import numpy as np
import pinocchio as pin

panda = RobotWrapper("panda_link8", "panda.urdf")

q0 = np.random.uniform(panda.q_min,panda.q_max)
dq0 = np.random.uniform(panda.dq_min,panda.dq_max)
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