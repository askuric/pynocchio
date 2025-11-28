Jacobian calculation
=====================================================

This code uses the PyNocchio library to calculate and print the Jacobian matrix of a robot arm given its URDF file path and current joint configuration.

.. code-block:: python
    
    import numpy as np
    from pynocchio import RobotWrapper

    # Define the path to the URDF file
    robot_path = '/path/to/robot.urdf'
    # Create the RobotWrapper object
    robot = RobotWrapper("panda_link8", urdf_path=robot_path)
    # Define the current robot configuration
    q0 = np.random.uniform(panda.q_min,panda.q_max)
    # Calculate the Jacobian matrix for the end effector, for LOCAL_WORLD_ALIGNED frame
    J = robot.jacobian(q0)
    # Print the resulting Jacobian matrix
    print(J)