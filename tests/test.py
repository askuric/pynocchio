import numpy as np
import time

import pinocchio as pin

from pynocchio import RobotWrapper
from pynocchio import FrictionRobotWrapper

# write a test loading a robot and checking the number of joints
def test_load_robot():
    panda = RobotWrapper('panda_link8',  urdf_path="pynocchio/models/urdf/panda.urdf")
    assert panda.model.nq == 7

# check the meshes file loading
def test_load_visual_with_initial_pose():
    panda_dummy = RobotWrapper('panda_link8',  urdf_path="pynocchio/models/urdf/panda.urdf")
    q0 = pin.randomConfiguration(panda_dummy.model)
    try:
        panda = RobotWrapper('panda_link8',  urdf_path="pynocchio/models/urdf/panda.urdf", mesh_path="pynocchio/models", q=q0)
    except:
        assert False
    time.sleep(0.3)
    assert np.allclose(panda.q, q0)

# check for value exception for no path or xml
def test_load_robot_exception():
    try:
        panda = RobotWrapper('panda_link8')
    except ValueError:
        assert True
        return 
    assert False

# write a test checking the forward kinematics and inverse kinematics
def test_forward_kinematics():
    panda = RobotWrapper('panda_link8',  urdf_path="pynocchio/models/urdf/panda.urdf")
    q0 = pin.randomConfiguration(panda.model)
    oMf = panda.forward(q0)
    # inverse kinematics
    q = panda.ik(oMf, q0)
    assert np.allclose(q, q0)


# check that the robot uses the tip frame as default
def test_default_tip():
    panda = RobotWrapper( urdf_path="pynocchio/models/urdf/panda.urdf")
    assert panda.tip_link == 'panda_FT'


# test dk position and orientation matrix against forward
def test_dk_position():
    panda = RobotWrapper('panda_link8',  urdf_path="pynocchio/models/urdf/panda.urdf")
    q = np.zeros(panda.model.nq)
    oMf = panda.forward(q)
    trans = panda.dk_position(q)
    R = panda.dk_orientation_matrix(q)
    oMf_cmp = pin.SE3(R,trans)
    assert np.allclose(oMf, oMf_cmp, atol=1e-2)

# write a test checking the forward kinematics and inverse kinematics
def test_inverse_kinematics():
    panda = RobotWrapper('panda_link8',  urdf_path="pynocchio/models/urdf/panda.urdf")
    
    oMf = pin.SE3(np.eye(3), np.array([0.3, 0.3, 0.3]))
    q = panda.ik(oMf, verbose=True)
    oMf_ik = panda.forward(q)
    assert np.allclose(oMf, oMf_ik, atol=1e-2)

# check iterations for ik
def test_ik_iterations_limit():
    panda = RobotWrapper('panda_link8',  urdf_path="pynocchio/models/urdf/panda.urdf")
    oMf = pin.SE3(np.eye(3), np.array([1.2, 1.2, 1.2])) #point too far
    q = panda.ik(oMf, verbose=True)
    assert True

# write a test checking the jacobian
def test_jacobian():
    panda = RobotWrapper('panda_link8',  urdf_path="pynocchio/models/urdf/panda.urdf")
    q = np.zeros(panda.model.nq)
    J = panda.jacobian(q)
    J_cmp = np.array([[ 0.,    0.59, -0.,   -0.28 , 0.  ,  0.11,  0.  ],
                    [ 0.09,  0.   , 0.09,  0.  ,  0.09,  0.  ,  0.  ],
                    [ 0.  , -0.09,  0.  ,  0.01,  0.  ,  0.09,  0.  ],
                    [ 0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0. ,   0.  ],
                    [ 0.  ,  1.  ,  0.  , -1.  ,  0.  , -1. ,  -0.  ],
                    [ 1.  ,  0.  ,  1.  ,  0.  ,  1.  ,  0. ,  -1.  ]])
    assert np.allclose(J, J_cmp, atol=1e-2)

# check that jacobian postion is good
def test_jacobian_position():
    panda = RobotWrapper('panda_link8',  urdf_path="pynocchio/models/urdf/panda.urdf")
    q = np.zeros(panda.model.nq)
    J = panda.jacobian(q)
    J_pos = panda.jacobian_position(q)
    assert np.allclose(J[:3,:], J_pos, atol=1e-2)

# check the jacobian pseudo inverse against numpy
def test_jacobian_pseudo_inverse():
    panda = RobotWrapper('panda_link8',  urdf_path="pynocchio/models/urdf/panda.urdf")
    q = np.zeros(panda.model.nq)
    J = panda.jacobian(q)
    J_pinv = panda.jacobian_pseudo_inv(q)
    J_pinv_cmp = np.linalg.pinv(J)
    assert np.allclose(J_pinv, J_pinv_cmp, atol=1e-2)

# check the jacobian weighted pseudo inverse with identity weight against jacobian pseudo inverse
def test_jacobian_weighted_pseudo_inverse():
    panda = RobotWrapper('panda_link8',  urdf_path="pynocchio/models/urdf/panda.urdf")
    q = pin.randomConfiguration(panda.model) # TODO: singular configuration should be avoided.
    J_Wpinv = panda.jacobian_weighted_pseudo_inv(np.eye(panda.model.nq), q)
    J_pinv_cmp = panda.jacobian_pseudo_inv(q)
    assert np.allclose(J_Wpinv, J_pinv_cmp, atol=1e-2)

#test jacobian dot
def test_jacobian_dot():
    panda = RobotWrapper('panda_link8',  urdf_path="pynocchio/models/urdf/panda.urdf")
    q = np.zeros(panda.model.nq)
    qd = np.ones(panda.model.nv)
    J_dot = panda.jacobian_dot(q, qd)
    J_dot_com = np.array([
        [[ 0.,   0. ,   0. ,  -0.08, -0.,   -0.08, -0.26],
        [ 0.,    0.59 ,-0.59, -0.55, -0.32,  0.32,  0.42],
        [ 0.,    0.,    0.  , -0.32,  0. ,  -0.32, -0.  ],
        [ 0.,   -1.,    1.  ,  2. ,  -0. ,   3. ,   1.  ],
        [ 0.,    0.,    0.  ,  0. ,   0. ,   0. ,   0.  ],
        [ 0.,    0.,    0.  ,  0. ,   0. ,   0. ,   0.  ]]
    ])
    assert np.allclose(J_dot, J_dot_com, atol=1e-2)

# write a test checking the mass matrix
def test_mass_matrix():
    panda = RobotWrapper('panda_link8',  urdf_path="pynocchio/models/urdf/panda.urdf")
    q = np.zeros(panda.model.nq)
    M = panda.mass_matrix(q)
    M_comp = np.array(
        [[ 0.235, -0.061,  0.198, 0.024,  0.154,  0.001, -0.049],
         [-0.061,  2.659, -0.060, -1.129, -0.048, -0.032, 0.003],
         [ 0.198, -0.060,  0.198, 0.024,  0.154,  0.001, -0.049],
         [ 0.024, -1.129,  0.024, 0.634,  0.026,  0.040, -0.002],
         [ 0.154, -0.048,  0.154, 0.026,  0.154,  0.001, -0.049],
         [ 0.001, -0.032,  0.001, 0.040,  0.001,  0.081, -0.001],
         [-0.049,  0.003, -0.049, -0.002, -0.049, -0.001, 0.048]]
    )
    assert np.allclose(M, M_comp, atol=1e-2)

# write a test checking the coriolis matrix
def test_coriolis_matrix():
    panda = RobotWrapper('panda_link8',  urdf_path="pynocchio/models/urdf/panda.urdf")
    q = np.zeros(panda.model.nq)
    qd = np.zeros(panda.model.nv)
    C = panda.coriolis_matrix(q, qd)
    assert True

# write a test checking the gravity vector
def test_gravity_vector():
    panda = RobotWrapper('panda_link8',  urdf_path="pynocchio/models/urdf/panda.urdf")
    q = np.zeros(panda.model.nq)
    g = panda.gravity_torque(q)
    g_cmp = np.array([ 0. ,  -3.43, -0. ,  -3.26,  0. ,   1.69,  0.  ])
    assert np.allclose(g, g_cmp, atol=1e-2)

# test direct dynamics method against aba method from pinocchio
def test_direct_dynamics():
    panda = RobotWrapper('panda_link8',  urdf_path="pynocchio/models/urdf/panda.urdf")
    q = np.zeros(panda.model.nq)
    dq = np.zeros(panda.model.nq)
    tau = np.zeros(panda.model.nq)
    ddq = panda.direct_dynamics(tau=tau, q=q, dq=dq)
    ddq_aba = pin.aba(panda.model, panda.data, q, dq, tau)
    assert np.allclose(ddq, ddq_aba, atol=1e-2)

# test direct dynamics method with gravity torque as input
def test_direct_dynamics_gravity_compensation():
    panda = RobotWrapper('panda_link8',  urdf_path="pynocchio/models/urdf/panda.urdf")
    q = np.zeros(panda.model.nq)
    dq = np.zeros(panda.model.nq)
    tau = panda.gravity_torque(q)
    ddq = panda.direct_dynamics(tau=tau, q=q, dq=dq)
    assert np.allclose(ddq, np.zeros(panda.model.nq))

def test_aba_with_external_force():
    panda = RobotWrapper('panda_link8',  urdf_path="pynocchio/models/urdf/panda.urdf")
    q = pin.randomConfiguration(panda.model)
    dq = np.random.uniform(panda.dq_min,panda.dq_max)
    tau = panda.gravity_torque(q)
    fext = np.array([1, 2, 3, 0.4, 0.5, 0.6])
    tau_ext = panda.jacobian(q).T@fext
    #
    ddq = panda.direct_dynamics(tau=tau, q=q, dq=dq, f_ext=fext)
    ddq_aba = panda.direct_dynamics_aba(tau=tau, q=q, dq=dq, tau_ext=tau_ext)
    assert np.allclose(ddq, ddq_aba, atol=1e-2)

# test joint update
def test_joint_data_update():
    panda = RobotWrapper('panda_link8',  urdf_path="pynocchio/models/urdf/panda.urdf")
    q = np.random.uniform(panda.q_min,panda.q_max)
    dq = np.random.uniform(panda.dq_min,panda.dq_max)
    ddq = np.random.uniform(-1*np.array([15,7.5,10,12.5,15,20,20]).T, np.array([15,7.5,10,12.5,15,20,20]).T)
    tau = np.random.uniform(panda.tau_min,panda.tau_max)
    panda.update_joint_data(q=q, dq=dq, ddq=ddq, tau=tau)
    test_data = np.concatenate((q, dq, ddq, tau), axis=None)
    panda_data = np.concatenate((panda.q, panda.dq, panda.ddq, panda.tau), axis=None)
    assert np.allclose(test_data, panda_data, atol=1e-2)

# test joint update with saturation
def test_joint_data_update_saturation():
    panda = RobotWrapper('panda_link8',  urdf_path="pynocchio/models/urdf/panda.urdf")
    # test max
    q = panda.q_max + 1
    dq = panda.dq_max + 1
    tau = panda.tau_max + 1
    panda.update_joint_data(q, dq, tau=tau, apply_saturation=True)
    q_up_sat = np.allclose(panda.q, panda.q_max)
    dq_up_sat = np.allclose(panda.dq, panda.dq_max)
    tau_up_sat = np.allclose(panda.tau, panda.tau_max)
    # test min
    q = panda.q_min - 1
    dq = panda.dq_min - 1
    tau = panda.tau_min - 1
    panda.update_joint_data(q, dq, tau=tau, apply_saturation=True)
    q_down_sat = np.allclose(panda.q, panda.q_min)
    dq_down_sat = np.allclose(panda.dq, panda.dq_min)
    tau_down_sat = np.allclose(panda.tau, panda.tau_min)
    assert np.concatenate((q_up_sat, dq_up_sat, tau_up_sat, q_down_sat, dq_down_sat, tau_down_sat), axis=None).all()

# test the initialisation of the friction model
def test_load_friction_robot_without_friction():
    panda_fric = FrictionRobotWrapper('panda_link8', urdf_path="pynocchio/models/urdf/panda.urdf")
    fv_test = panda_fric.fv == np.zeros((panda_fric.n))
    fc_test = panda_fric.fc == np.zeros((panda_fric.n))
    f0_test = panda_fric.f0 == np.zeros((panda_fric.n))
    assert np.concatenate((fv_test,fc_test,f0_test), axis=None).all()

# test the initialisation of the friction model with friction set
def test_load_friction_robot_with_friction():
    fv = [0.0665, 0.1987, 0.0399, 0.2257, 0.1023, -0.0132, 0.0638]
    fc = [0.2450, 0.1523, 0.1827, 0.3591, 0.2669, 0.1658, 0.2109]
    f0 = [0, 0, 0, 0, 0, 0, 0]
    panda = FrictionRobotWrapper('panda_link8', urdf_path="pynocchio/models/urdf/panda.urdf", fv=fv, fc=fc, f0=f0)
    assert np.allclose(np.concatenate((fv, fc, f0), axis=None), np.concatenate((panda.fv, panda.fc, panda.f0), axis=None), atol=1e-2)

# test setting friction gains from vector of size (1,n) or (n,1)
def test_friction_gains_input():
    fv = np.random.uniform(-1,1,(1,7))
    panda = FrictionRobotWrapper('panda_link8', urdf_path="pynocchio/models/urdf/panda.urdf", fv=fv)
    assert np.allclose(fv.squeeze(), panda.fv, atol=1e-2)

# test direct dynamics method with null friction gains
def test_direct_dynamics_without_friction_against_aba():
    panda = FrictionRobotWrapper('panda_link8', urdf_path="pynocchio/models/urdf/panda.urdf")
    q = pin.randomConfiguration(panda.model)
    dq = np.random.uniform(panda.dq_min,panda.dq_max)
    tau = np.random.uniform(panda.tau_min,panda.tau_max)
    ddq = panda.direct_dynamics(tau=tau, q=q, dq=dq)
    ddq_aba = pin.aba(panda.model, panda.data, q, dq, tau)
    assert np.allclose(ddq, ddq_aba, atol=1e-2)
