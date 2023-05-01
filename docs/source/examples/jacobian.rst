Jacobian calculation
=====================================================

This code uses the PyNocchio library to calculate and print the Jacobian matrix of a robot arm given its URDF file path and current joint configuration.

.. code-block:: python
    
    import numpy as np
    from pyocchio import RobotWrapper

    # Define the path to the URDF file
    robot_path = '/path/to/robot.urdf'

    # Create the RobotWrapper object
    robot = RobotWrapper(robot_path=robot_path)

    # Define the current robot configuration
    q = np.zeros(robot.n)

    # Calculate the Jacobian matrix for the end effector
    J = robot.jacobian(q)

    # Print the resulting Jacobian matrix
    print(J)