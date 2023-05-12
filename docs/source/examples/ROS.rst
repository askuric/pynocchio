Example of loading the robot model using ROS
============================================



.. code-block:: python
    
    import rospkg
    import rospy

    import numpy as np
    from pynocchio import RobotWrapper

    # loading the root urdf from robot_description parameter
    robot = RobotWrapper(xml_path=rospy.get_param("robot_description"))

    # Define the current robot configuration
    q0 = np.random.uniform(panda.q_min,panda.q_max)
    # Calculate the Jacobian matrix for the end effector, for LOCAL_WORLD_ALIGNED frame
    J = robot.jacobian(q0)
    # Print the resulting Jacobian matrix
    print(J)




Ros node code could be something like this

.. code-block:: python

    #!/usr/bin/env python
    import rospy
    from sensor_msgs.msg import JointState 

    import rospy

    from pynocchio import RobotWrapper
    # initial joint positions
    q = [0,0,0,0,0,0,0]

    # function receiveing the new joint positions
    def callback(data):
        global q
        q = data.position
        
    def jacobian_calculator():
        global q
        rospy.init_node('calculate_jacobian', anonymous=True)

        # create a subscriber to the joint states 
        rospy.Subscriber('panda/joint_states', JointState, callback, queue_size= 2)

        # loading the root urdf from robot_description parameter
        robot = RobotWrapper(xml_path=rospy.get_param("panda/robot_description"))

        rate = rospy.Rate(100) # 10hz
        while not rospy.is_shutdown():
            
            # calculate jacobian matrix
            print(robot.jacobian(q))

            rate.sleep()

    if __name__ == '__main__':
        jacobian_calculator()