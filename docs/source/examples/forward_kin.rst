Forward and inverse kinematics calculation
=====================================================

This code loads a robot model from a URDF file and computes the forward kinematics of the end effector for a zero joint configuration, and then prints the resulting transformation matrix.


.. code-block:: python


    import numpy as np
    from pynocchio import RobotWrapper

    # Define the path to the URDF file
    robot_path = '/path/to/robot.urdf'

    # Create the RobotWrapper object
    robot = RobotWrapper(robot_path=robot_path)

    # Define the current robot configuration
    q = np.zeros(robot.n)

    # Calculate the forward kinematics for the end effector
    oMf = robot.forward(q)

    # Print the resulting transformation matrix
    print(oMf) 

    # Calculate the inverse kinematics for the end effector
    q_ik = robot.ik(oMf)

    # Print the resulting joint configuration
    print(q_ik) 