#!/usr/bin/env python
import numpy as np
# URDF parsing an kinematics 
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import meshcat
import os

class RobotWrapper:
    # constructor - reading urdf/xml and constructing the robots kinematic chain
    def __init__(self, tip:str, urdf_path:(str or None)=None, xml_path:(str or None)=None, mesh_path:(str or None)=None, q:(np.ndarray[float] or None)=None):
        """
        RobotWrapper constructor

        Args:
            tip:        robot end-effector frame name
            urdf_path:  a part to the robot's urdf file (optional) 
            xml_path:   a string containing robot's urdf/xml (optional)
            mesh_path:  a string containing robot's meshes for visualisation (optional)
            q:          an array containing the robot intial joint position (optional)
        """

        if mesh_path:
            if urdf_path:
                self.robot, self.collision_model, self.visual_model = pin.buildModelFromUrdf(urdf_path, mesh_path)
            elif xml_path:
                self.robot, self.collision_model, self.visual_model = pin.buildModelFromXML(xml_path, mesh_path)
            else: 
                print("ERROR: no urdf or xml specified for the robot!")
                return
        else:
            if urdf_path:
                self.robot = pin.buildModelFromUrdf(urdf_path)
            elif xml_path:
                self.robot = pin.buildModelFromXML(xml_path)
            else: 
                print("ERROR: no urdf or xml specified for the robot!")
                return

        self.tip_link = tip
        # self.base_id = self.robot.getFrameId(base)
        self.tip_id = self.robot.getFrameId(tip)

        # maximal torques
        self.tau_max = self.robot.effortLimit.T
        self.tau_min = -self.tau_max
        # maximal joint velocities
        self.dq_max = self.robot.velocityLimit.T
        self.dq_min = -self.dq_max
        # maximal joint angles
        self.q_max = self.robot.upperPositionLimit.T
        self.q_min = self.robot.lowerPositionLimit.T

        self.data = self.robot.createData()

        # robot state
        if q:
            self.q = q
        self.dq = np.zeros((self.robot.nq))
        self.ddq = np.zeros((self.robot.nq))
        self.tau = np.zeros((self.robot.nq))

        if mesh_path:
            self.viz = MeshcatVisualizer(self.model, self.collision_model, self.visual_model)
            self.viz.initViewer(open=True)
            self.viz.loadViewerModel("pinocchio")

    # direct kinematics functions 
    def forward(self, q:np.ndarray[float], frame_name:(str or None)=None) -> pin.SE3:
        """
        Forward kinematics calculating function

        Args:
            q: currrent robot configuration
            frame_name: name of the robot frame for which to calculate forward kinematics (optional - default tip frame)
            

        Returns
        --------
            oMf: SE3 transformation matrix of the desired robot frame expressed in the world frame
        """
        
        if frame_name is not None:
            frame_id = self.robot.getFrameId(frame_name)
        else:
            frame_id = self.tip_id
        pin.framesForwardKinematics(self.robot,self.data, np.array(q))
        return self.data.oMf[frame_id]
       
    def dk_position(self, q:np.ndarray[float], frame_name:(str or None)=None) -> np.ndarray[float]: 
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

    def dk_orientation_matrix(self, q:np.ndarray[float], frame_name:(str or None)=None) -> np.ndarray[np.ndarray[float]]:
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

    def jacobian(self, q:np.ndarray[float], frame_name:(str or None)=None, frame_align:pin.ReferenceFrame=pin.LOCAL_WORLD_ALIGNED) -> np.ndarray[np.ndarray[float]]:
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
        
        if frame_name is not None:
            frame_id = self.robot.getFrameId(frame_name)
        else:
            frame_id = self.tip_id
        pin.computeJointJacobians(self.robot,self.data, np.array(q))
        Jac = pin.getFrameJacobian(self.robot, self.data, frame_id, frame_align)

        return np.array(Jac)
    
    def jacobian_dot(self, q:np.ndarray[float], qd:np.ndarray[float], frame_name:(str or None)=None, frame_align:pin.ReferenceFrame=pin.LOCAL_WORLD_ALIGNED) -> np.ndarray[np.ndarray[float]]:
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
        
        if frame_name is not None:
            frame_id = self.robot.getFrameId(frame_name)
        else:
            frame_id = self.tip_id
        pin.computeJointJacobiansTimeVariation(self.robot, self.data,np.array(q), np.array(qd))
        Jdot = pin.getFrameJacobianTimeVariation(self.robot, self.data, frame_id, frame_align)

        return np.array(Jdot)

    def jacobian_position(self, q:np.ndarray[float], frame_name:(str or None)=None, frame_align:pin.ReferenceFrame=pin.LOCAL_WORLD_ALIGNED) -> np.ndarray[np.ndarray[float]]:
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

    def jacobian_pseudo_inv(self, q:np.ndarray[float], frame_name:(str or None)=None, frame_align:pin.ReferenceFrame=pin.LOCAL_WORLD_ALIGNED) -> np.ndarray[np.ndarray[float]]:
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

    def jacobian_weighted_pseudo_inv(self, q:np.ndarray[float], W:np.ndarray[np.ndarray[float]], frame_name:(str or None)=None, frame_align:pin.ReferenceFrame=pin.LOCAL_WORLD_ALIGNED) -> np.ndarray[np.ndarray[float]]:
        """
        Weighted pseudo inverse
        .. math:: J^{W+} = W^{-1} J^t (JW^{-1}J^t)^{-1}

        Args:
            q:          currrent robot configuration
            W:          weighting matrix (must be invertible)
            frame_name: name of the robot frame for which to calculate the jacobian (optional - default tip frame)
            frame_align: determining which frame to express the jacoiban in. Can be either: LOCAL_WORLD_ALIGNED (default), WORLD or LOCAL    
        
        Returns
        --------
            pwJac:  nx6 jacobian matrix pseudo-inverse for the desired robot frame
        """
        iW = np.linalg.inv(W)
        J = self.jacobian(q, frame_name, frame_align)
        pWJac = iW.dot(J.T).dot( np.linalg.inv( J.dot(iW).dot(J.T) ) )
        
        return pWJac

    def gravity_torque(self, q:np.ndarray[float]) -> np.ndarray[float]:
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

    def mass_matrix(self, q:np.ndarray[float]) -> np.ndarray[np.ndarray[float]]:
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


    def coriolis_matrix(self, q:np.ndarray[float], qd:np.ndarray[float]) -> np.ndarray[np.ndarray[float]]:
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
    
    def ik(self, oMdes:pin.SE3, q:(np.ndarray[float] or None)=None, verbose:bool=True) -> np.ndarray[float]:
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
        
        return np.array(q)
    
    def direct_dynamic(self, q:np.ndarray[float], dq:np.ndarray[float], tau:np.ndarray[float], f_ext:np.ndarray[float]) -> np.ndarray[float]:
        """
        Direct dynamic model

        Arg:
            q:        joint position array
            dq:       joint velocity array
            tau:      torque input array
            f_ext:    external force array (in the global frame) f(rob->env) at the endpoint of the robot
        Returns
        --------
            ddq:      n array of the joint acceleration 
        """
        J = self.jacobian(q)
        A = self.mass_matrix(q)
        C = self.coriolis_matrix(q, dq)
        g = self.gravity_torque(q)
        tau_ext = J.T.dot(f_ext) # TODO: the user should provide tau_ext directly so that f_ext can be forces exerted on any part of the body

        ddq = np.linalg.inv(A).dot(tau - tau_ext - C.dot(dq) - g)
        return np.array(ddq)

    def direct_dynamic_aba(self, q:np.ndarray[float], dq:np.ndarray[float], tau:np.ndarray[float], tau_ext:np.ndarray[float]) -> np.ndarray[float]:
        """
        Direct dynamic model using ABA's method (R. Featherstone 1983), with the implementation described in Analytical Derivatives
        of Rigid Body Dynamics Algorithms, by Justin Carpentier and Nicolas Mansard (http://www.roboticsproceedings.org/rss14/p38.pdf)

        Arg:
            q:        joint position array
            dq:       joint velocity array
            tau:      torque input array
            tau_ext:  external force array (in the local frame) f(rob->env)
        Returns
        --------
            ddq:      n array of the joint acceleration
        """
        ddq = pin.aba(model=self.model, data=self.data, q=q, v=dq, tau=tau, fext=tau_ext)
        return np.array(ddq)
    
    def update_visualisation(self, q:(np.ndarray[float] or None)=None) -> None:
        """
        Update the joint state in the 3D meshcat visualiser

        Arg:
            q:        joint position array
        """
        self.viz.display(q)

    def add_visual_object(self, obj_file_path:str, name_id:str, material=None) -> None:
        """
        Add a 3D object from a dae, obj or stl file. If the name already exists, it updates the object state.

        Arg:
            obj_file_path:      file path of the 3D object (string)
            name_id:            reference identifier for the object (string)
            material:           meshcat.geometry material defining color, texture and opacity of the object
        """
        _, file_extension = os.path.splitext(obj_file_path)
        if file_extension.lower() == ".dae":
            obj = meshcat.geometry.DaeMeshGeometry.from_file(obj_file_path)
        elif file_extension.lower() == ".obj":
            obj = meshcat.geometry.ObjMeshGeometry.from_file(obj_file_path)
        elif file_extension.lower() == ".stl":
            obj = meshcat.geometry.StlMeshGeometry.from_file(obj_file_path)
        else:
            raise ValueError("Unknown mesh file format: {}.".format(file_extension))
        
        if material is None:
            material = meshcat.geometry.MeshPhongMaterial(color=0xff0000, opacity=0.4)

        self.viz.viewer[name_id].set_object(obj, material)