import numpy as np
import os

import pynocchio as pynoc

dir_path = os.path.dirname(os.path.abspath(pynoc.__file__))

panda = pynoc.RobotWrapper("panda_link8", dir_path+"/models/urdf/panda.urdf")

q0 = np.random.uniform(panda.q_min,panda.q_max)
print("initial q\n", q0)
oMq0 = panda.forward(q0)
print("direct kinamtics for q\n", oMq0)
q_ik = panda.ik(oMq0, verbose=False)
print("ik found q\n", q_ik)
oMq_ik = panda.forward(q_ik)
print("direct kinamtics for ik found q\n", oMq_ik)

