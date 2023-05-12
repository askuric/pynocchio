Forward and inverse kinematics calculation
=====================================================

This code loads a robot model from a URDF file and computes the forward kinematics of the end effector for a random joint configuration.


.. code-block:: python

    import numpy as np
    import os

    import pynocchio as pynoc

    # Define the path to the URDF file
    urdf_path = os.path.dirname(os.path.abspath(pynoc.__file__)) + "/models/urdf/panda.urdf"
    # Create the RobotWrapper object
    panda = pynoc.RobotWrapper("panda_link8", urdf_path)
    # Define the current robot configuration
    q0 = np.random.uniform(panda.q_min,panda.q_max)
    # Calculate the forward kinematics for the end effector
    oMq0 = panda.forward(q0)
    # Calculate the inverse kinematics for the end effector
    q_ik = panda.ik(oMq0, verbose=False)