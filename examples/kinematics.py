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

