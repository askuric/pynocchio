from pynocchio import RobotWrapper
import pinocchio as pin
import numpy as np

#write a test loading a robot and checking the number of joints
def test_load_robot():
    panda = RobotWrapper('panda_link8',  robot_path="examples/panda.urdf")
    assert panda.robot.nq == 7

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
    panda = RobotWrapper('panda_link8',  robot_path="examples/panda.urdf")
    q0 = np.random.rand(panda.robot.nq)
    oMf = panda.forward(q0)
    # inverse kinematics
    q = panda.ik(oMf, q0)
    assert np.allclose(q, q0)


# check that the robot uses the tip frame as default
def test_default_tip():
    panda = RobotWrapper( robot_path="examples/panda.urdf")
    assert panda.tip_link == 'panda_FT'


# test dk position and orientation matrix against forward
def test_dk_position():
    panda = RobotWrapper('panda_link8',  robot_path="examples/panda.urdf")
    q = np.zeros(panda.robot.nq)
    oMf = panda.forward(q)
    trans = panda.dk_position(q)
    R = panda.dk_orientation_matrix(q)
    oMf_cmp = pin.SE3(R,trans)
    assert np.allclose(oMf, oMf_cmp, atol=1e-2)

# write a test checking the forward kinematics and inverse kinematics
def test_inverse_kinematics():
    panda = RobotWrapper('panda_link8',  robot_path="examples/panda.urdf")
    
    oMf = pin.SE3(np.eye(3), np.array([0.2, 0.2, 0.2]))
    q = panda.ik(oMf, verbose=False)
    oMf_ik = panda.forward(q)
    assert np.allclose(oMf, oMf_ik, atol=1e-2)

# check iterations for ik
def test_ik_iterations_limit():
    panda = RobotWrapper('panda_link8',  robot_path="examples/panda.urdf")
    oMf = pin.SE3(np.eye(3), np.array([1.2, 1.2, 1.2])) #point too far
    q = panda.ik(oMf, verbose=True)
    assert True


# write a test checking the jacobian
def test_jacobian():
    panda = RobotWrapper('panda_link8',  robot_path="examples/panda.urdf")
    q = np.zeros(panda.robot.nq)
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
    panda = RobotWrapper('panda_link8',  robot_path="examples/panda.urdf")
    q = np.zeros(panda.robot.nq)
    J = panda.jacobian(q)
    J_pos = panda.jacobian_position(q)
    assert np.allclose(J[:3,:], J_pos, atol=1e-2)

# check the jacobian pseudo inverse against numpy
def test_jacobian_pseudo_inverse():
    panda = RobotWrapper('panda_link8',  robot_path="examples/panda.urdf")
    q = np.zeros(panda.robot.nq)
    J = panda.jacobian(q)
    J_pinv = panda.jacobian_pseudo_inv(q)
    J_pinv_cmp = np.linalg.pinv(J)
    assert np.allclose(J_pinv, J_pinv_cmp, atol=1e-2)

#test jacobian dot
def test_jacobian_dot():
    panda = RobotWrapper('panda_link8',  robot_path="examples/panda.urdf")
    q = np.zeros(panda.robot.nq)
    qd = np.ones(panda.robot.nv)
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
    panda = RobotWrapper('panda_link8',  robot_path="examples/panda.urdf")
    q = np.zeros(panda.robot.nq)
    M = panda.mass_matrix(q)
    M_comp = np.array(
        [[ 0.16, -0.06 , 0.12,  0.02,  0.08,  0. ,  -0.05],
        [-0.06 , 2.66 ,-0.06, -1.13, -0.05 ,-0.03,  0.  ],
        [ 0.12 ,-0.06 ,0.12 , 0.02 , 0.08  ,0.   ,-0.05],
        [ 0.02 ,-1.13 , 0.02,  0.63,  0.03 , 0.04, -0.  ],
        [ 0.08 ,-0.05 , 0.08,  0.03,  0.08 , 0.  , -0.05],
        [ 0.   ,-0.03 , 0.  ,  0.04,  0.   , 0.08, -0.  ],
        [-0.05 , 0.   ,-0.05, -0. ,  -0.05 ,-0.  ,  0.05]]
    )
    assert np.allclose(M, M_comp, atol=1e-2)


# write a test checking the coriolis matrix
def test_coriolis_matrix():
    panda = RobotWrapper('panda_link8',  robot_path="examples/panda.urdf")
    q = np.zeros(panda.robot.nq)
    qd = np.zeros(panda.robot.nv)
    C = panda.coriolis_matrix(q, qd)
    assert True

# write a test checking the gravity vector
def test_gravity_vector():
    panda = RobotWrapper('panda_link8',  robot_path="examples/panda.urdf")
    q = np.zeros(panda.robot.nq)
    g = panda.gravity_torque(q)
    g_cmp = np.array([ 0. ,  -3.43, -0. ,  -3.26,  0. ,   1.69,  0.  ])
    assert np.allclose(g, g_cmp, atol=1e-2)
