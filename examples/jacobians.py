from pynocchio import RobotWrapper
import numpy as np
import pinocchio as pin

panda = RobotWrapper("panda_link8", "panda.urdf")

q0 = np.random.uniform(panda.q_min,panda.q_max)
print("initial q\n", q0)
J0 = panda.jacobian(q0)
print("jacobian of", panda.tip_link,"\n", J0)
J0_inv = panda.jacobian_pseudo_inv(q0)
print("jacobian paseudo-inverse of", panda.tip_link,"\n", J0_inv)
J0 = panda.jacobian(q0,"panda_link5")
print("jacobian of panda_link5 in LOCAL_WORLD_ALIGNED frame\n", J0)
J0 = panda.jacobian(q0,"panda_link5",pin.WORLD)
print("jacobian of panda_link5 in WORLD frame\n", J0)
J0 = panda.jacobian(q0,"panda_link5",pin.LOCAL)
print("jacobian of panda_link5 in LOCAL frame\n", J0)

