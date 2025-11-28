import numpy as np
import os

import pynocchio as pynoc
import time

from example_robot_data import load

robot = pynoc.RobotWrapper(robot_wrapper=load('panda'))

q0 = np.random.uniform(robot.q_min,robot.q_max)

print("initial q\n", q0)
oMq0 = robot.forward(q0)
print("direct kinamtics for q\n", oMq0)
q_ik = robot.ik(oMq0, q=np.random.uniform(robot.q_min,robot.q_max), verbose=False, qlim=True)
print("ik found q\n", q_ik)
oMq_ik = robot.forward(q_ik)
print("direct kinamtics for ik found q\n", oMq_ik)
# check if solution inside joint limits
print("q_ik inside joint limits: ", np.all(q_ik<robot.q_max) and np.all(q_ik>robot.q_min))
print(q_ik, robot.q_min, robot.q_max)
