#!/usr/bin/env python
import numpy as np
# URDF parsing an kinematics 
import pinocchio as pin

class RobotWrapper:
    # constructor - reading urdf and constructing the robots kinematic chain
    def __init__(self, tip, robot_path=None, robot_xml=None):
        """
        RobotWrapper constructor

        Args:
            tip: robot end-effector frame name
            robot_path: a part to the robot's urdf file (optional) 
            robot_xml: a string containing robot's urdf/xml (optional)
        """
        if robot_path:
            self.robot = pin.buildModelFromUrdf(robot_path)
        elif robot_xml:
            self.robot = pin.buildModelFromXML(robot_xml)
        else: 
            print("ERROR: no path or robot xml specified!")
            return

        self.tip_link = tip
        # self.base_id = self.robot.getFrameId(base)
        self.tip_id = self.robot.getFrameId(tip)

        # maximal torques
        self.t_max = self.robot.effortLimit.T
        self.t_min = -self.t_max
        # maximal joint velocities
        self.dq_max = self.robot.velocityLimit.T
        self.dq_min = -self.dq_max
        # maximal joint angles
        self.q_max = self.robot.upperPositionLimit.T
        self.q_min = self.robot.lowerPositionLimit.T

        self.data = self.robot.createData()

    # direct kinematics functions 
    def forward(self, q, frame_name=None):
        """
        Forward kinematics calculating function

        Args:
            q: currrent robot configuration
            frame_name: name of the robot frame for which to calculate forward kinematics (optional - default tip frame)
            

        Returns
        --------
            oMf: SE3 transformation matrix of the desired robot frame expressed in the world frame
        """
        frame_id = self.tip_id
        if frame_name is not None:
            frame_id = self.robot.getFrameId(frame_name)
        pin.framesForwardKinematics(self.robot,self.data, np.array(q))
        return self.data.oMf[frame_id]
       
    def dk_position(self, q, frame_name=None): 
        """
        Forward kinematics position calculating function

        Args:
            q: currrent robot configuration
            frame_name: name of the robot frame for which to calculate forward kinematics (optional - default tip frame)
            
        Returns
        --------
            position: cartesian position the desired robot frame expressed in the world frame
        """
        return self.forward(q, frame_name).translation

    def dk_orientation_matrix(self, q, frame_name=None):
        """
        Forward kinematics orientation calculating function

        Args:
            q: currrent robot configuration
            frame_name: name of the robot frame for which to calculate forward kinematics (optional - default tip frame)
  
        Returns
        --------
            oRf: SO3 transformation matrix of the desired robot frame orientation expressed in the world frame
        """
        return self.forward(q,frame_name).rotation

    def jacobian(self, q, frame_name=None, frame_align=pin.LOCAL_WORLD_ALIGNED):
        """
        Jacobian matrix calculating function

        Args:
            q: currrent robot configuration
            frame_name: name of the robot frame for which to calculate the jacobian (optional - default tip frame)
            frame_align: determining which frame to express the jacoiban in. Can be either: LOCAL_WORLD_ALIGNED (default), WORLD or LOCAL           

        Returns
        --------
            J: 6xn jacobian matrix for the desired robot frame 
        """
        frame_id = self.tip_id
        if frame_name is not None:
            frame_id = self.robot.getFrameId(frame_name)
        pin.computeJointJacobians(self.robot,self.data, np.array(q))
        Jac = pin.getFrameJacobian(self.robot, self.data, frame_id, frame_align)
        return np.array(Jac)
    
    def jacobian_dot(self, q, qd, frame_name=None, frame_align=pin.LOCAL_WORLD_ALIGNED):
        """
        Jacobian time derivative matrix calculating function

        Args:
            q: currrent robot configuration
            qd: currrent robot velocity in configuration space
            frame_name: name of the robot frame for which to calculate the jacobian (optional - default tip frame)
            frame_align: determining which frame to express the jacoiban in. Can be either: LOCAL_WORLD_ALIGNED (default), WORLD or LOCAL           

        Returns
        --------
            Jdot: 6xn jacobian time derivative matrix for the desired robot frame 
        """
        frame_id = self.tip_id
        if frame_name is not None:
            frame_id = self.robot.getFrameId(frame_name)
        pin.computeJointJacobiansTimeVariation(self.robot, self.data,np.array(q), np.array(qd))
        Jdot = pin.getFrameJacobianTimeVariation(self.robot, self.data, frame_id, frame_align)
        return np.array(Jdot)

    def jacobian_position(self, q,frame_name=None, frame_align=pin.LOCAL_WORLD_ALIGNED):
        """
        Position jacobian matrix calculating function

        Args:
            q: currrent robot configuration
            frame_name: name of the robot frame for which to calculate the jacobian (optional - default tip frame)
            frame_align: determining which frame to express the jacoiban in. Can be either: LOCAL_WORLD_ALIGNED (default), WORLD or LOCAL           

        Returns
        --------
            J: 3xn jacobian matrix for the desired robot frame 
        """
        return self.jacobian(q, frame_name, frame_align)[:3, :]

    def jacobian_pseudo_inv(self, q, frame_name=None, frame_align=pin.LOCAL_WORLD_ALIGNED):
        """
        Jacobian matrix pseudo inverse calculating function

        Args:
            q: currrent robot configuration
            frame_name: name of the robot frame for which to calculate the jacobian (optional - default tip frame)
            frame_align: determining which frame to express the jacoiban in. Can be either: LOCAL_WORLD_ALIGNED (default), WORLD or LOCAL           

        Returns
        --------
            J_pinv: nx6 jacobian matrix pseudo-inverse for the desired robot frame 
        """
        return np.linalg.pinv(self.jacobian(q,frame_name,frame_align))

    def gravity_torque(self, q):
        """
        Gravity torque vector

        Args:
            q: currrent robot configuration      

        Returns
        --------
            g: n-vector of joint torques due to the gravity
        """
        pin.computeGeneralizedGravity(self.robot,self.data, np.array(q))
        return np.array(self.data.g)

    def mass_matrix(self, q):
        """
        Mass matrix calcualation function

        Args:
            q: currrent robot configuration      

        Returns
        --------
            M: nxn mass matrix 
        """
        pin.crba(self.robot,self.data,np.array(q))
        return np.array(self.data.M)


    def coriolis_matrix(self, q, qd):
        """
        Coriolis matrix calcualation function

        Args:
            q: currrent robot configuration  
            qd: currrent robot velocity in configuration space

        Returns
        --------
            C: nxn Coriolis matrix 
        """
        pin.computeCoriolisMatrix(self.robot,self.data,np.array(q),np.array(qd))
        return np.array(self.data.C)
    
    def ik(self, oMdes, q=None, verbose=True):
        """
        Iterative inverse kinematics based on the example code from
        https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_b-examples_i-inverse-kinematics.html

        Args:
            oMdes:  SE3 matrix expressed in the world frame of the robot's endefector desired pose    
            q: currrent robot configuration (default robot's neutral position)
            verbose: bool variable enabling verbose ouptut (default True)

        Returns
        --------
            q_des: robot configuration corresponding to the desired pose 
        """
        data_ik  = self.robot.createData()

        if q is None:
            q = pin.neutral(self.robot)
        
        # ik parameters
        eps    = 1e-4
        IT_MAX = 1000
        DT     = 1e-1
        damp   = 1e-12

        i=0
        while True:
            pin.framesForwardKinematics(self.robot,data_ik,q)
            dMi = oMdes.actInv(data_ik.oMf[self.tip_id])
            err = pin.log(dMi).vector
            if np.linalg.norm(err) < eps:
                success = True
                break
            if i >= IT_MAX:
                success = False
                break
            J = pin.computeFrameJacobian(self.robot,data_ik,q,self.tip_id)
            v = - J.T.dot(np.linalg.solve(J.dot(J.T) + damp * np.eye(6), err))
            q = pin.integrate(self.robot,q,v*DT)
            if not i % 10 and verbose:
                print('%d: error = %s' % (i, err.T))
            i += 1

        if verbose:
            if success:
                print("Convergence achieved!")
            else :
                print("\nWarning: the iterative algorithm has not reached convergence to the desired precision")
        
        return q
