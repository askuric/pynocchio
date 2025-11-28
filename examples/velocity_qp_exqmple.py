import numpy as np
import os
import time

import pinocchio as pin
import pynocchio as pynoc

import qpsolvers

# solvers - 'cvxopt', 'osqp', 'quadprog', 'proxqp'
def solve_qp(A,s,x_min=None,x_max=None, G=None, h=None, grad = None, reg_w=1e-7, solver='quadprog'):
    P = A.T@A
    if grad is not None:
        q = -A.T@s + reg_w*grad[:,None]
        P= P+np.eye(len(grad))*reg_w
    else:
        q = -A.T@s
    if x_max is not None:
        G = np.vstack((-np.identity(len(x_max)),np.identity(len(x_min))))
        h = np.hstack((list(-np.array(x_min)),x_max))
    if G is not None:
        G = np.array(G)
        h = np.array(h)
    return qpsolvers.solve_qp(P, q.flatten(), G,h, solver=solver)


dir_path = os.path.dirname(os.path.abspath(pynoc.__file__))
panda = pynoc.RobotWrapper("panda_link8", dir_path+"/models/urdf/panda.urdf")



# PD controller parameters
Kp = 100
Kd = 2*np.sqrt(Kp)
# step time
dt = 0.001
# QP horizon 
t_h = 15*dt

# Generate the trajectory within the control loop
s = time.time()

# regularisation positon
q_mean = (panda.q_min+panda.q_max)/2


i=0
q_c = q0
dq_c, ddq_c, dddq_c = np.zeros(n),np.zeros(n),np.zeros(n)

x_list, dx_list, ddx_list,dddx_list = [], [], [], []
while i < 1000:    
    i = i+1
    
    # calculate the kinematics
    X_cur = panda.forward(q_c)
    J = panda.jacobian(q_c)
    Jdot = panda.jacobian_dot(q_c,dq_c)
    
    # saving variables for the visualisation
    x_list.append(X_cur.translation.copy())
    dx_list.append(J@dq_c)
    ddx_list.append(J@ddq_c + Jdot@dq_c)
    dddx_list.append(J@dddq_c + Jdot@ddq_c)
    dq_c_old = dq_c.copy()
    ddq_c_old = ddq_c.copy()
    
    # calculating the limits
    dq_ub = np.minimum(
            np.minimum(panda.dq_max, (t_h*panda.ddq_max + dq_c).flatten()), 
            ((panda.q_max - q_c)/t_h).flatten())
    dq_lb = np.maximum(
            np.maximum(panda.dq_min, (t_h*panda.ddq_min + dq_c).flatten()), 
            ((panda._min - q_c)/t_h).flatten())
     
    # acceleration pd
    X_log = X_final.actInv(X_cur)
    log_dk = pin.log6(X_log)

    # P controller
    dx_des = Kp*(-(X_final.toActionMatrix()@log_dk)[:,None]) 
    # solve the qp in acceleration
    dq_c = solve_qp(J, dx_des, dq_lb, dq_ub, grad=-(q_mean-q_c), reg_w=50*0.00001, solver='quadprog')    
    
    # simulating the robot
    q_c = np.clip(dq_c*dt + ddq_c*(dt**2)/2 + q_c, q_min,q_max)
    dq_c = dq_c + ddq_c*dt;
    dq_c = np.clip(dq_c, dq_min, dq_max)
    ddq_c = (dq_c - dq_c_old)/dt
    ddq_c = np.clip(ddq_c, ddq_min, ddq_max)
    dddq_c = (ddq_c - ddq_c_old)/dt
    dddq_c = np.clip(dddq_c, dddq_min, dddq_max)
    
    
# time 
t = np.arange(0,len(x_list))*dt
x_fin = X_final.translation


import matplotlib.pyplot as plt
fig, axs = plt.subplots(4, 3)
axs[0,0].set_title("x")
axs[0,0].set_ylabel("position")
axs[0,0].plot(t,np.array(x_list)[:,0],'b')
axs[0,0].plot([0,t[-1]],[x_fin[0],x_fin[0]],'k--')
axs[0,1].set_title("y")
axs[0,1].plot(t,np.array(x_list)[:,1],'b')
axs[0,1].plot([0,t[-1]],[x_fin[1],x_fin[1]],'k--')
axs[0,2].set_title("z")
axs[0,2].plot(t,np.array(x_list)[:,2],'b')
axs[0,2].plot([0,t[-1]],[x_fin[2],x_fin[2]],'k--')
axs[1,0].set_ylabel("velocity")
axs[1,0].plot(np.array(dx_list)[:,0],'b')
axs[1,1].plot(t,np.array(dx_list)[:,1],'b')
axs[1,2].plot(t,np.array(dx_list)[:,2],'b')
axs[2,0].set_ylabel("acceleration")
axs[2,0].plot(t,np.array(ddx_list)[:,0],'b')
axs[2,1].plot(t,np.array(ddx_list)[:,1],'b')
axs[2,2].plot(t,np.array(ddx_list)[:,2],'b')
axs[3,0].set_ylabel("jerk")
axs[3,0].plot(t,np.array(dddx_list)[:,0],'b')
axs[3,1].plot(t,np.array(dddx_list)[:,1],'b')
axs[3,2].plot(t,np.array(dddx_list)[:,2],'b')