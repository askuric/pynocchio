Robot's dynamics matrices computation
=====================================================

The code creates a robot model using the `pynocchio`` library and generates random initial joint positions, velocities, and accelerations. 
It then calculates the mass matrix, Coriolis matrix, gravity vector, and the time derivative of the Jacobian for the given initial joint states of the robot.

.. code-block:: python
    
    import numpy as np
    import pinocchio as pin
    from pynocchio import RobotWrapper

    # Define the path to the URDF file
    robot_path = '/path/to/robot.urdf'
    # Create a wrapper object for the Panda robot
    panda = RobotWrapper("panda_link8", urdf_path=robot_path)

    # Generate random initial joint position and velocity
    q0 = np.random.uniform(panda.q_min, panda.q_max)
    dq0 = np.random.uniform(panda.dq_min, panda.dq_max)

    # Print initial joint position and velocity
    print("Initial joint position: ", q0)
    print("Initial joint velocity: ", dq0)

    # Compute the mass matrix of the robot at the given joint position
    M = panda.mass_matrix(q0)
    print("Mass matrix: \n", M)

    # Compute the Coriolis matrix of the robot at the given joint position and velocity
    C = panda.coriolis_matrix(q0, dq0)
    print("Coriolis matrix: \n", C)

    # Compute the gravity vector of the robot at the given joint position
    g = panda.gravity_torque(q0)
    print("Gravity vector: \n", g)

    # Compute the time derivative of the Jacobian of the robot at the given joint position and velocity
    Jdot = panda.jacobian_dot(q0, dq0)
    print("Jacobian time derivative: \n", Jdot)

    # Generate random torque command
    tau = np.random.uniform(panda.tau_min, panda.tau_max)

    # Compute the direct dynamic of the robot at the given joint configuration for a torque command tau, and no external forces
    ddq = panda.direct_dynamic(tau, q0, dq0)