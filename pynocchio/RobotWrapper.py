#!/usr/bin/env python
import numpy as np
import os
# URDF parsing an kinematics 
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import meshcat

class RobotWrapper:
    """
    Class wrapper to use pinocchio robot model and with simplified use for various kinematic and dynamic functions.

    Attributes:
        robot (pin.RobotModel): The pinocchio RobotModel object representing the robot.
        collision_model (pin.CollisionModel): The pinocchio CollisionModel object representing the robot's collision model.
        visual_model (pin.VisualModel): The pinocchio VisualModel object representing the robot's visual model.
        tip_link (str): The name of the robot's end-effector frame.
        tip_id (int): The ID of the robot's end-effector frame.
        tau_max (np.ndarray): An array of the maximum joint torques.
        tau_min (np.ndarray): An array of the minimum joint torques.
        dq_max (np.ndarray): An array of the maximum joint velocities.
        dq_min (np.ndarray): An array of the minimum joint velocities.
        q_max (np.ndarray): An array of the maximum joint angles.
        q_min (np.ndarray): An array of the minimum joint angles.
        data (pin.Data): The pinocchio Data object for storing intermediate computation results.
        q (np.ndarray): An array containing the current joint positions of the robot.
        dq (np.ndarray): An array containing the current joint velocities of the robot.
        ddq (np.ndarray): An array containing the current joint accelerations of the robot.
        tau (np.ndarray): An array containing the current joint torques of the robot.
        viz (MeshcatVisualizer): The MeshcatVisualizer object for visualizing the robot.
    """
    # constructor - reading urdf/xml and constructing the robots kinematic chain
    def __init__(self, tip:str, urdf_path:(str or None)=None, xml_path:(str or None)=None, mesh_path:(str or None)=None, q:(np.ndarray[float] or None)=None):
        """
        RobotWrapper constructor

        Args:
            tip:        robot end-effector frame name
            urdf_path:  Path to the robot's URDF file (optional) 
            xml_path:   Path to the robot's XML file (optional)
            mesh_path:  Path to the robot's meshes folder for visualization (optional)
            q:          An array containing the robot intial joint position (optional)
        """

        if mesh_path:
            if urdf_path:
                self.robot, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(urdf_path, mesh_path)
            elif xml_path:
                self.robot, self.collision_model, self.visual_model = pin.buildModelsFromXML(xml_path, mesh_path)
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
        if q is not None:
            self.q = q # joint position
        self.dq = np.zeros((self.robot.nq)) # joint velocity
        self.ddq = np.zeros((self.robot.nq)) # joint acceleration
        self.tau = np.zeros((self.robot.nq)) # joint torque

        if mesh_path:
            self.viz = MeshcatVisualizer(self.robot, self.collision_model, self.visual_model)
            self.viz.initViewer(open=True)
            self.viz.loadViewerModel("pinocchio")
            if q is not None:
                self.update_visualisation()

    def forward(self, q:(np.ndarray[float] or None)=None, frame_name:(str or None)=None) -> pin.SE3:
        """
        Forward kinematics calculating function

        Args:
            q: currrent robot configuration (optional), if the value is not set, it uses the RobotWrapper state
            frame_name: name of the robot frame for which to calculate forward kinematics (optional - default tip frame)
            

        Returns
        --------
            oMf: SE3 transformation matrix of the desired robot frame expressed in the world frame
        """
        
        if frame_name is not None:
            frame_id = self.robot.getFrameId(frame_name)
        else:
            frame_id = self.tip_id
        if q is None:
            pin.framesForwardKinematics(self.robot,self.data, self.q)
        else:
            pin.framesForwardKinematics(self.robot,self.data, np.array(q))
        return self.data.oMf[frame_id]
       
    def dk_position(self, q:(np.ndarray[float] or None)=None, frame_name:(str or None)=None) -> np.ndarray[float]: 
        """
        Forward kinematics position calculating function

        Args:
            q: currrent robot configuration (optional)
            frame_name: name of the robot frame for which to calculate forward kinematics (optional - default tip frame)
            
        Returns
        --------
            position: cartesian position the desired robot frame expressed in the world frame
        """
        return self.forward(q, frame_name).translation

    def dk_orientation_matrix(self, q:(np.ndarray[float] or None)=None, frame_name:(str or None)=None) -> np.ndarray[np.ndarray[float]]:
        """
        Forward kinematics orientation calculating function

        Args:
            q: currrent robot configuration (optional)
            frame_name: name of the robot frame for which to calculate forward kinematics (optional - default tip frame)
  
        Returns
        --------
            oRf: SO3 transformation matrix of the desired robot frame orientation expressed in the world frame
        """
        return self.forward(q,frame_name).rotation

    def jacobian(self, q:(np.ndarray[float] or None)=None, frame_name:(str or None)=None, frame_align:pin.ReferenceFrame=pin.LOCAL_WORLD_ALIGNED) -> np.ndarray[np.ndarray[float]]:
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
        if q is None:
            q = self.q          

        pin.computeJointJacobians(self.robot,self.data, np.array(q))
        Jac = pin.getFrameJacobian(self.robot, self.data, frame_id, frame_align)

        return np.array(Jac)
    
    def jacobian_dot(self, q:(np.ndarray[float] or None)=None, dq:(np.ndarray[float] or None)=None, frame_name:(str or None)=None, frame_align:pin.ReferenceFrame=pin.LOCAL_WORLD_ALIGNED) -> np.ndarray[np.ndarray[float]]:
        """
        Jacobian time derivative matrix calculating function

        Args:
            q: currrent robot configuration (optional)
            qd: currrent robot velocity in configuration space (optional)
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
        if q is None:
            q = self.q
        if dq is None:
            dq = self.dq

        pin.computeJointJacobiansTimeVariation(self.robot, self.data,np.array(q), np.array(dq))        
        Jdot = pin.getFrameJacobianTimeVariation(self.robot, self.data, frame_id, frame_align)

        return np.array(Jdot)

    def jacobian_position(self, q:(np.ndarray[float] or None)=None, frame_name:(str or None)=None, frame_align:pin.ReferenceFrame=pin.LOCAL_WORLD_ALIGNED) -> np.ndarray[np.ndarray[float]]:
        """
        Position jacobian matrix calculating function

        Args:
            q: currrent robot configuration (optional)
            frame_name: name of the robot frame for which to calculate the jacobian (optional - default tip frame)
            frame_align: determining which frame to express the jacoiban in. Can be either: LOCAL_WORLD_ALIGNED (default), WORLD or LOCAL           

        Returns
        --------
            J: 3xn jacobian matrix for the desired robot frame 
        """
        return self.jacobian(q, frame_name, frame_align)[:3, :]

    def jacobian_pseudo_inv(self, q:(np.ndarray[float] or None)=None, frame_name:(str or None)=None, frame_align:pin.ReferenceFrame=pin.LOCAL_WORLD_ALIGNED) -> np.ndarray[np.ndarray[float]]:
        """
        Jacobian matrix pseudo inverse calculating function

        Args:
            q: currrent robot configuration (optional)
            frame_name: name of the robot frame for which to calculate the jacobian (optional - default tip frame)
            frame_align: determining which frame to express the jacoiban in. Can be either: LOCAL_WORLD_ALIGNED (default), WORLD or LOCAL           

        Returns
        --------
            J_pinv: nx6 jacobian matrix pseudo-inverse for the desired robot frame 
        """
        return np.linalg.pinv(self.jacobian(q,frame_name,frame_align))

    def jacobian_weighted_pseudo_inv(self, W:np.ndarray[np.ndarray[float]], q:(np.ndarray[float] or None)=None, frame_name:(str or None)=None, frame_align:pin.ReferenceFrame=pin.LOCAL_WORLD_ALIGNED) -> np.ndarray[np.ndarray[float]]:
        """
        Weighted pseudo inverse
        .. math:: J^{W+} = W^{-1} J^t (JW^{-1}J^t)^{-1}

        Args:
            W:          weighting matrix (must be invertible)
            q:          currrent robot configuration (optional)
            frame_name: name of the robot frame for which to calculate the jacobian (optional - default tip frame)
            frame_align: determining which frame to express the jacoiban in. Can be either: LOCAL_WORLD_ALIGNED (default), WORLD or LOCAL    
        
        Returns
        --------
            pwJac:  nx6 jacobian matrix pseudo-inverse for the desired robot frame
        """
        if q is None:
            q = self.q
        iW = np.linalg.inv(W)
        J = self.jacobian(q, frame_name, frame_align)
        pWJac = iW.dot(J.T).dot( np.linalg.inv( J.dot(iW).dot(J.T) ) )
        
        return pWJac

    def gravity_torque(self, q:(np.ndarray[float] or None)=None) -> np.ndarray[float]:
        """
        Gravity torque vector

        Args:
            q: currrent robot configuration (optional)

        Returns
        --------
            g: n-vector of joint torques due to the gravity
        """
        if q is None:
            q = self.q
        pin.computeGeneralizedGravity(self.robot,self.data, np.array(q))
        return np.array(self.data.g)

    def mass_matrix(self, q:np.ndarray[float]) -> np.ndarray[np.ndarray[float]]:
        """
        Mass matrix calcualation function

        Args:
            q: currrent robot configuration  (optional) 

        Returns
        --------
            M: nxn mass matrix 
        """
        if q is None:
            q = self.q
        pin.crba(self.robot,self.data,np.array(q))
        return np.array(self.data.M)

    def coriolis_matrix(self, q:(np.ndarray[float] or None)=None, dq:(np.ndarray[float] or None)=None) -> np.ndarray[np.ndarray[float]]:
        """
        Coriolis matrix calcualation function

        Args:
            q: current robot configuration  (optional)
            dq: current robot velocity in configuration space (optional)

        Returns
        --------
            C: nxn Coriolis matrix 
        """
        if q is None:
            q = self.q
        if dq is None:
            dq = self.dq

        pin.computeCoriolisMatrix(self.robot,self.data,np.array(q),np.array(dq))
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
    
    def direct_dynamic(self, tau:np.ndarray[float], q:(np.ndarray[float] or None)=None, dq:(np.ndarray[float] or None)=None, f_ext:(np.ndarray[float] or None)=None) -> np.ndarray[float]:
        """
        Direct dynamic model

        Arg:
            tau:      torque input array 
            q:        joint position array (optional)
            dq:       joint velocity array (optional)
            f_ext:    external force array (in the global frame) f(rob->env) at the endpoint of the robot (default value is a null array)

        Returns
        --------
            ddq:      n array of the joint acceleration 
        """
        if q is None:
            q = self.q
        if dq is None:
            dq = self.dq
        J = self.jacobian(q)
        A = self.mass_matrix(q)
        C = self.coriolis_matrix(q, dq)
        g = self.gravity_torque(q)
        if f_ext is None:
            tau_ext = np.zeros((self.robot.nq))
        else:
            tau_ext = J.T.dot(f_ext) # TODO: the user should provide tau_ext directly so that f_ext can be forces exerted on any part of the body

        ddq = np.linalg.inv(A).dot(tau - tau_ext - C.dot(dq) - g)
        return np.array(ddq)

    def direct_dynamic_aba(self, tau:np.ndarray[float], q:(np.ndarray[float] or None)=None, dq:(np.ndarray[float] or None)=None, tau_ext:(np.ndarray[float] or None)=None) -> np.ndarray[float]:
        """
        Direct dynamic model using ABA's method (R. Featherstone 1983), with the implementation described in Analytical Derivatives
        of Rigid Body Dynamics Algorithms, by Justin Carpentier and Nicolas Mansard (http://www.roboticsproceedings.org/rss14/p38.pdf)

        Arg:
            tau:      torque input array
            q:        joint position array (optional)
            dq:       joint velocity array (optional)
            tau_ext:  external force array (in the local frame) f(rob->env)
        Returns
        --------
            ddq:      n array of the joint acceleration
        """
        if q is None:
            q = self.q
        if dq is None:
            dq = self.dq
        if tau_ext is None:
            tau_ext = np.zeros(self.robot.nq)

        ddq = pin.aba(model=self.model, data=self.data, q=q, v=dq, tau=tau, fext=tau_ext)
        return np.array(ddq)
 
    def update_joint_data(self, q:(np.ndarray or None)=None, dq:(np.ndarray or None)=None, ddq:(np.ndarray or None)=None, tau:(np.ndarray or None)=None, apply_saturation=False):
        """
        Update the joint states of the robot. The joint position, velocity, acceleration and torque are all optional.

        Arg:
            q:                  joint position array (optional)
            dq:                 joint velocity array (optional)
            ddq:                joint acceleration array (optional)
            tau:                joint torque array (optional)
            apply_saturation:   boolean to apply saturation to joint update (default value is False)
        """
        if apply_saturation:
            if q is not None:
                self.apply_joint_position_limits(np.array(q), update_joint_data=True)
            if dq is not None:
                self.apply_joint_velocity_limits(np.array(dq), update_joint_data=True)
            if ddq is not None:
                self.apply_joint_acceleration_limits(np.array(ddq), update_joint_data=True)
            if tau is not None:
                self.apply_joint_effort_limits(np.array(tau), update_joint_data=True)
        else:
            if q is not None:
                self.q = np.array(q)
            if dq is not None:
                self.dq = np.array(dq)
            if ddq is not None:
                self.ddq = np.array(ddq)
            if tau is not None:
                self.t = tau   

    def apply_joint_position_limits(self, q:(np.ndarray[float] or None)=None, update_joint_data:bool=False):
        """
        Clip the joint position q, according to the limits specified in the robot model

        Arg:
            q:                  joint position array (optional)
            update_joint_data:  boolean to update this class self.q with the clipping (default value is False)
        """
        if q is None:
            q = self.q
        q = np.clip(q, self.q_min, self.q_max)
        if update_joint_data:
            self.q = q
        return q

    def apply_joint_velocity_limits(self, dq:(np.ndarray[float] or None)=None, update_joint_data:bool=False):
        """
        Clip the joint velocity dq, according to the limits specified in the robot model

        Arg:
            dq:                  joint velocity array (optional)
            update_joint_data:   boolean to update this class self.dq with the clipping (default value is False)
        """
        if dq is None:
            dq = self.dq
        dq = np.clip(dq, self.dq_min, self.dq_max)
        if update_joint_data:
            self.dq = dq
        return dq

    def apply_joint_acceleration_limits(self, ddq:(np.ndarray[float] or None)=None, update_joint_data:bool=False):
        """
        Clip the joint acceleration ddq, according to the limits specified by the user. If no limits were specified, this function will fail.

        Arg:
            ddq:                 joint acceleration array (optional)
            update_joint_data:   boolean to update this class self.ddq with the clipping (default value is False)
        """
        if ddq is None:
            ddq = self.ddq
        ddq = np.clip(ddq, self.ddq_min, self.ddq_max)
        if update_joint_data:
            self.ddq = ddq
        return ddq

    def apply_joint_jerk_limits(self, dddq:(np.ndarray[float] or None)=None, update_joint_data:bool=False):
        """
        Clip the joint jerk dddq, according to the limits specified by the user. If no limits were specified, this function will fail.

        Arg:
            dddq:                joint jerk array (optional)
            update_joint_data:   boolean to update this class self.dddq with the clipping (default value is False)
        """
        if dddq is None:
            dddq = self.dddq
        dddq = np.clip(dddq, self.dddq_min, self.dddq_max)
        if update_joint_data:
            self.dddq = dddq
        return dddq

    def apply_joint_effort_limits(self, q:(np.ndarray[float] or None)=None, update_joint_data:bool=False):
        """
        Clip the joint torque tau, according to the limits specified in the robot model.

        Arg:
            tau:                 joint torque array (optional)
            update_joint_data:   boolean to update this class self.tau with the clipping (default value is False)
        """
        if tau is None:
            tau = self.tau
        tau = np.clip(tau, self.tau_min, self.tau_max)
        if update_joint_data:
            self.tau = tau
        return tau

    def update_visualisation(self, q:(np.ndarray[float] or None)=None) -> None:
        """
        Update the joint state in the 3D meshcat visualiser

        Arg:
            q:        joint position array

        """
        if q is None:
            q = self.q
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


class FrictionRobotWrapper(RobotWrapper):
    """
    Class wrapper to use pinocchio robot model with friction modelled.

    Attributes:
        fv (np.ndarray): An array containing the viscous friction gain.
        fc (np.ndarray): An array containing the Coulomb friction gain.
        f0 (np.ndarray): An array containing the Coulomb friction offset gain.
    """
    def __init__(self, robot_model, fv:(np.ndarray[float] or None)=None, fc:(np.ndarray[float] or None)=None, f0:(np.ndarray[float] or None)=None):

        super().__init__(robot_model)

        if fv:
            self.set_viscous_friction_gains(fv)
        else:
            self.fv = np.zeros(self.robot.nq)

        if fc:
            self.set_coulomb_friction_gains(fc)
        else:
            self.fc = np.zeros(self.robot.nq)

        if f0:
            self.set_coulomb_friction_offset_gains(f0)
        else:
            self.f0 = np.zeros(self.robot.nq)
    
    def set_viscous_friction_gain(self, fv:(np.ndarray[float] or np.ndarray[np.ndarray[float]])):
        """
        Define the gains of the Coulomb friction torques, such as:
        .. math:: \tau_{fi}(\dot{q}_i) = f_{v,i} \dot{q}_i
        Args:
            fv:       Viscous friction (np.array 1D of size equal to the nb of joints)
        """
        if not isinstance(fv, np.ndarray):
            fv = np.array(fv)

        if fv.ndim == 1 and fv.size == self.model.nq:
            self.fv = fv
        elif fv.ndim == 2 and (fv.shape[0] == 1 or fv.shape[1] == 1):
            self.fv = fv.squeeze()
        else:
            raise ValueError(f"fc must be a 1D array of size {self.model.nq}, according to the number of joint of the robot.")

        self.fv = fv

    def set_coulomb_friction_gain(self, fc:(np.ndarray[float] or np.ndarray[np.ndarray[float]])):
        """
        Define the gains of the Coulomb friction torques, such as:
        .. math:: \tau_{Ci}(\dot{q}_i) = f_{c,i} \sign(\dot{q}_i)
        Args:
            fc:       Coulomb friction (np.array 1D of size equal to the nb of joints)
        """
        if not isinstance(fc, np.ndarray):
            fc = np.array(fc)

        if fc.ndim == 1 and fc.size == self.model.nq:
            self.fc = fc
        elif fc.ndim == 2 and (fc.shape[0] == 1 or fc.shape[1] == 1):
            self.fc = fc.squeeze()
        else:
            raise ValueError(f"fc must be a 1D array of size {self.model.nq}, according to the number of joint of the robot.")

        self.fc = fc

    def set_coulomb_friction_offset_gain(self, fo:(np.ndarray[float] or np.ndarray[np.ndarray[float]])):
        """
        Define the gains of the Coulomb friction offset torques, such as:
        .. math:: \tau_{Ci}(\dot{q}_i) = f_{o,i}
        Args:
            fo:       Coulomb friction offset (np.array 1D of size equal to the nb of joints)
        """       
        if not isinstance(fo, np.ndarray):
            fo = np.array(fo) 

        if fo.ndim == 1 and fo.size == self.model.nq:
            self.fo = fo
        elif fo.ndim == 2 and (fo.shape[0] == 1 or fo.shape[1] == 1):
            self.fo = fo.squeeze()
        else:
            raise ValueError(f"fo must be a 1D array of size {self.model.nq}, according to the number of joint of the robot.")
        
        self.fo = fo

    def viscous_friction_torque(self, dq:(np.array or None)=None) -> np.ndarray[float]:
        """
        Compute the viscous friction torques, such as:
        .. math:: \tau_{fi}(\dot{q}_i) = f_{v,i} \dot{q}_i
        Args:
            dq:       joint velocity array (optional)
        Returns
        --------
            tau_v:     n array of the joint viscous friction torque 
        """
        if dq is None:
            dq = self.dq

        return self.fv.dot(dq)
    
    def colomb_friction_torque(self, dq:(np.array or None)=None) -> np.ndarray[float]:
        """
        Compute the coulomb friction torques, such as:
        .. math:: \tau_{ci}(\dot{q}_i) = f_{c,i}\sign(\dot{q}_i)
        Args:
            dq:       joint velocity array (optional)
        Returns
        --------
            tau_c:     n array of the joint coulomb friction torque 
        """
        if dq is None:
            dq = self.dq

        return self.fc.dot(np.sign(dq))
    
    def colomb_friction_offset_torque(self) -> np.ndarray[float]:
        """
        Compute the coulomb friction torques offset, such as:
        .. math:: \tau_{0i} = f_{0,i}
        
        Returns
        --------
            tau_0:     n array of the joint coulomb friction torque offset
        """

        return self.f0

    def direct_dynamic(self, tau:np.ndarray[float], q:(np.ndarray[float] or None)=None, dq:(np.ndarray[float] or None)=None, f_ext:(np.ndarray[float] or None)=None) -> np.ndarray[float]:
        """
        Direct dynamic model with friction

        Arg:
            tau:      torque input array 
            q:        joint position array (optional)
            dq:       joint velocity array (optional)
            f_ext:    external force array (in the global frame) f(rob->env) at the endpoint of the robot (default value is a null array)
        Returns
        --------
            ddq:      n array of the joint acceleration 
        """
        if q is None:
            q = self.q
        if dq is None:
            dq = self.dq

        J = self.jacobian(q)
        A = self.mass_matrix(q)
        C = self.coriolis_matrix(q, dq)
        g = self.gravity_torque(q)
        tau_f = self.viscous_friction_torque(q) + self.coulomb_friction_torque(q) + self.viscous_friction_offset_torque()

        if f_ext is None:
            tau_ext = np.zeros((self.robot.nq))
        else:
            tau_ext = J.T.dot(f_ext) # TODO: the user should provide tau_ext directly so that f_ext can be forces exerted on any part of the body

        ddq = np.linalg.inv(A).dot(tau - tau_ext - C.dot(dq) - g - tau_f)
        return np.array(ddq)