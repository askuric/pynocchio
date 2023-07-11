#!/usr/bin/env python
import numpy as np
import os
# URDF parsing an kinematics 
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import meshcat

import time

class RobotWrapper:
    """
    Class wrapper to use pinocchio robot model and with simplified use for various kinematic and dynamic functions.

    :ivar pin.RobotModel model: The pinocchio RobotModel object representing the robot.
    :ivar pin.CollisionModel collision_model: The pinocchio CollisionModel object representing the robot's collision model.
    :ivar pin.VisualModel visual_model: The pinocchio VisualModel object representing the robot's visual model.
    :ivar str tip_link: The name of the robot's end-effector frame.
    :ivar int tip_id: The ID of the robot's end-effector frame.
    :ivar np.ndarray tau_max: An array of the maximum joint torques.
    :ivar np.ndarray tau_min: An array of the minimum joint torques.
    :ivar np.ndarray dq_max: An array of the maximum joint velocities.
    :ivar np.ndarray dq_min: An array of the minimum joint velocities.
    :ivar np.ndarray q_max: An array of the maximum joint angles.
    :ivar np.ndarray q_min: An array of the minimum joint angles.
    :ivar pin.Data data: The pinocchio Data object for storing intermediate computation results.
    :ivar np.ndarray q: An array containing the current joint positions of the robot.
    :ivar np.ndarray dq: An array containing the current joint velocities of the robot.
    :ivar np.ndarray ddq: An array containing the current joint accelerations of the robot.
    :ivar np.ndarray tau: An array containing the current joint torques of the robot.
    :ivar MeshcatVisualizer viz: The MeshcatVisualizer object for visualizing the robot.
    """
    def __init__(self, tip:(str or None)=None, urdf_path:(str or None)=None, xml_path:(str or None)=None, mesh_path:(str or None)=None, q:(np.ndarray or None)=None, open_viewer:(bool or None)=False,  robot_wrapper:(pin.RobotWrapper or None)=None, robot_name:(str or None)=None, viewer:(meshcat.Visualizer or None)=None):
        """
        RobotWrapper constructor

        Args:
            tip:        robot end-effector frame name (optional if not specified, the last frame is used)
            urdf_path:  Path to the robot's URDF file (optional, but either urdf_path or xml_path must be specified)
            xml_path:   Path to the robot's XML file (optional, but either urdf_path or xml_path must be specified)
            mesh_path:  Path to the robot's meshes folder for visualization (optional)
            q:          An array containing the robot intial joint position (optional)
            open_viewer: Bool variable specifying if the meshcat viewer will be opened or not, by default False
            robot_wrapper: (pinocchio.RobotWrapper) object
            robot_name: (str) name of the robot (optional, by default 'robot_pinocchio')
            viewer: (meshcat.Visualizer) object (optional, pynocchio will create a new viewer if not specified)


        :raises ValueError: If neither urdf_path nor xml_path is specified.

        """

        self.visual_model = None
        self.collision_model = None

        if robot_name is None:
            robot_name = 'robot_pinocchio'
        

        if robot_wrapper:
            self.model = robot_wrapper.model
            if robot_wrapper.collision_model and robot_wrapper.visual_model:
                self.collision_model = robot_wrapper.collision_model
                self.visual_model =  robot_wrapper.visual_model
        elif mesh_path:
            if urdf_path:
                self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(urdf_path, mesh_path)
            elif xml_path:
                self.model, self.collision_model, self.visual_model = pin.buildModelsFromXML(xml_path, mesh_path)
            else: 
                raise ValueError("ERROR: no urdf or xml specified for the robot!")
        
        else:
            if urdf_path:
                self.model = pin.buildModelFromUrdf(urdf_path)
            elif xml_path:
                self.model = pin.buildModelFromXML(xml_path)
            else: 
                raise ValueError("ERROR: no urdf or xml specified for the robot!")

        if tip is None:
            tip = self.model.frames[-1].name
            print('No tip specified, using the last frame: ', tip)

        self.tip_link = tip
        self.tip_id = self.model.getFrameId(tip)

        # maximal torques
        self.tau_max = self.model.effortLimit.T
        self.tau_min = -self.tau_max
        # maximal joint velocities
        self.dq_max = self.model.velocityLimit.T
        self.dq_min = -self.dq_max
        # maximal joint angles
        self.q_max = self.model.upperPositionLimit.T
        self.q_min = self.model.lowerPositionLimit.T

        # number of joints
        self.n = self.model.nq
        # pinocchio data
        self.data = self.model.createData()

        # robot state
        if q is not None:
            self.q = q # joint position
        self.dq = np.zeros((self.model.nq)) # joint velocity
        self.ddq = np.zeros((self.model.nq)) # joint acceleration
        self.tau = np.zeros((self.model.nq)) # joint torque

        if self.visual_model and self.collision_model:
            self.viz = MeshcatVisualizer(self.model, self.collision_model, self.visual_model)
            self.viz.initViewer(open=open_viewer, viewer=viewer)
            self.viz.loadViewerModel(robot_name)
            if q is not None:
                self.update_visualisation()
            time.sleep(0.2) # small time window for loading the model


    def forward(self, q:(np.ndarray or None)=None, frame_name:(str or None)=None) -> pin.SE3:
        """
        Forward kinematics calculating function

        Args:
            q: current robot configuration (optional), if the value is not set, it uses the RobotWrapper state
            frame_name: name of the robot frame for which to calculate forward kinematics (optional - default tip frame)
            

        Returns
        --------
            oMf: SE3 transformation matrix of the desired robot frame expressed in the world frame
        """
        
        if frame_name is not None:
            frame_id = self.model.getFrameId(frame_name)
        else:
            frame_id = self.tip_id
        if q is None:
            pin.framesForwardKinematics(self.model,self.data, self.q)
        else:
            pin.framesForwardKinematics(self.model,self.data, np.array(q))
        return self.data.oMf[frame_id].copy()
       
    def dk_position(self, q:(np.ndarray or None)=None, frame_name:(str or None)=None) -> np.ndarray: 
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

    def dk_orientation_matrix(self, q:(np.ndarray or None)=None, frame_name:(str or None)=None) -> np.ndarray:
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

    def jacobian(self, q:(np.ndarray or None)=None, frame_name:(str or None)=None, frame_align:pin.ReferenceFrame=pin.LOCAL_WORLD_ALIGNED) -> np.ndarray:
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
            frame_id = self.model.getFrameId(frame_name)
        else:
            frame_id = self.tip_id
        if q is None:
            q = self.q          

        pin.computeJointJacobians(self.model,self.data, np.array(q))
        Jac = pin.getFrameJacobian(self.model, self.data, frame_id, frame_align)

        return np.array(Jac)
    
    def jacobian_dot(self, q:(np.ndarray or None)=None, dq:(np.ndarray or None)=None, frame_name:(str or None)=None, frame_align:pin.ReferenceFrame=pin.LOCAL_WORLD_ALIGNED) -> np.ndarray:
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
            frame_id = self.model.getFrameId(frame_name)
        else:
            frame_id = self.tip_id
        if q is None:
            q = self.q
        if dq is None:
            dq = self.dq

        pin.computeJointJacobiansTimeVariation(self.model, self.data,np.array(q), np.array(dq))        
        Jdot = pin.getFrameJacobianTimeVariation(self.model, self.data, frame_id, frame_align)

        return np.array(Jdot)

    def jacobian_position(self, q:(np.ndarray or None)=None, frame_name:(str or None)=None, frame_align:pin.ReferenceFrame=pin.LOCAL_WORLD_ALIGNED) -> np.ndarray:
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

    def jacobian_pseudo_inv(self, q:(np.ndarray or None)=None, frame_name:(str or None)=None, frame_align:pin.ReferenceFrame=pin.LOCAL_WORLD_ALIGNED) -> np.ndarray:
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

    def jacobian_weighted_pseudo_inv(self, W:np.ndarray, q:(np.ndarray or None)=None, frame_name:(str or None)=None, frame_align:pin.ReferenceFrame=pin.LOCAL_WORLD_ALIGNED) -> np.ndarray:
        """
        Weighted pseudo inverse

        .. math:: J^{W+} = W^{-1} J^t (JW^{-1}J^t)^{-1}

        Args:
            W: weighting matrix (must be invertible)
            q: currrent robot configuration (optional)
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

    def gravity_torque(self, q:(np.ndarray or None)=None) -> np.ndarray:
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
        pin.computeGeneralizedGravity(self.model,self.data, np.array(q))
        return np.array(self.data.g)

    def mass_matrix(self, q:(np.ndarray or None)=None) -> np.ndarray:
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
        pin.crba(self.model,self.data,np.array(q))
        return np.array(self.data.M)

    def coriolis_matrix(self, q:(np.ndarray or None)=None, dq:(np.ndarray or None)=None) -> np.ndarray:
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

        pin.computeCoriolisMatrix(self.model,self.data,np.array(q),np.array(dq))
        return np.array(self.data.C)
    
    def ik(self, oMdes:pin.SE3, q:(np.ndarray or None)=None, qlim:bool=False, verbose:bool=True, iterations:int=1000, precision:float=1e-4, tries:int = 5) -> np.ndarray:
        """
        Iterative inverse kinematics inspired by the pinocchio's example code 
        https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_b-examples_i-inverse-kinematics.html

        The code follows a simple strategy based on the Newton-Euler algorithm to solve the inverse kinematics problem (Jacobian pseudo-inverse method).

        * The algorithm is initialized with the current configuration of the robot (specified in the parameters ``q``) 
        * Iterates until the desired pose is reached (the difference between the current and desired pose is smaller than the specified ``precision``).
        * The algorithm can be instructed to respect the robot's joint limits by setting the ``qlim`` parameter to True (default False).
        * If the algorithm fails to converge, it is reinitialized with a random configuration and tries again (maximum number of ``tries`` can be specified - default 5). 
        * If the algorithm fails to converge after the specified number of tries, it returns the last configuration it reached.
        
        Args:
            oMdes: SE3 matrix expressed in the world frame of the robot's end-effector desired pose    
            q: currrent robot configuration (default robot's neutral position)
            verbose: bool variable enabling verbose output (default True)
            iterations: maximum number of iterations (default 1000)
            precision: desired precision (default 1e-4)
            tries: number of random initializations to try if the first one fails (default 5)
            
        Returns
        --------
            q_des: robot configuration corresponding to the desired pose 
        """
        data_ik  = self.model.createData()

        if q is None:
            q = (self.q_min+self.q_max)/2

        # ik parameters
        eps    = precision
        IT_MAX = iterations
        DT     = 1e-1
        damp   = 1e-12

        for t in range(tries):

            if verbose:
                print("Try: %d" % t)

            if t> 0:
                q = np.random.uniform(self.q_min, self.q_max)
            i=0
            while True:
                pin.framesForwardKinematics(self.model,data_ik,q)
                dMi = oMdes.actInv(data_ik.oMf[self.tip_id])
                err = pin.log(dMi).vector
                if np.linalg.norm(err) < eps:
                    success = True
                    break
                if i >= IT_MAX:
                    success = False
                    break
                J = pin.computeFrameJacobian(self.model,data_ik,q,self.tip_id)
                v = - J.T.dot(np.linalg.solve(J.dot(J.T) + damp * np.eye(6), err))
                q = pin.integrate(self.model,q,v*DT)
                if qlim == True:
                    q = self.apply_joint_position_limits(q)
                if not i % 10 and verbose:
                    print('%d: error = %s' % (i, err.T))
                i += 1

            if success:
                if verbose:
                    print("Convergence achieved!")
                # found 
                break  
            else: 
                if verbose:  # if convergence is not achieved and no initial guess is provided, try again with a random guess
                    print(f"Warning: the iterative algorithm has not reached convergence to the desired precision, for the try number {t}/{tries}")

        if not success:
            print(f"Warning: IK convergence not achieved after {tries} tries. The returned configuration is the last one obtained.")

        return np.array(q)
    
    def direct_dynamics(self, tau:np.ndarray, q:(np.ndarray or None)=None, dq:(np.ndarray or None)=None, f_ext:(np.ndarray or None)=None) -> np.ndarray:
        """
        Direct dynamic model.

        .. math:: \\ddot{q} = A^{-1}(q) \\left( \\tau - J^t(q)f_{ext} - C(q,\\dot{q})\\dot{q} - g(q) \\right)

        Args:
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
            tau_ext = np.zeros((self.model.nq))
        else:
            tau_ext = J.T.dot(f_ext) # TODO: the user should provide tau_ext directly so that f_ext can be forces exerted on any part of the body

        ddq = np.linalg.inv(A).dot(tau - tau_ext - C.dot(dq) - g)
        return np.array(ddq)

    def direct_dynamics_aba(self, tau:np.ndarray, q:(np.ndarray or None)=None, dq:(np.ndarray or None)=None, tau_ext:(np.ndarray or None)=None) -> np.ndarray:
        """
        Direct dynamic model using ABA's method (R. Featherstone 1983), with the implementation described in Analytical Derivatives
        of Rigid Body Dynamics Algorithms, by Justin Carpentier and Nicolas Mansard (http://www.roboticsproceedings.org/rss14/p38.pdf)

        Args:
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
            tau_ext = np.zeros(self.model.nq)

        #ddq = pin.aba(model=self.model, data=self.data, q=q, v=dq, tau=tau, fext=tau_ext)
        ddq = pin.aba(self.model, self.data, q, dq, tau - tau_ext)
        return np.array(ddq)
 
    def update_joint_data(self, q:(np.ndarray or None)=None, dq:(np.ndarray or None)=None, ddq:(np.ndarray or None)=None, tau:(np.ndarray or None)=None, apply_saturation=False):
        """
        Update the joint states of the robot. The joint position, velocity, acceleration and torque are all optional.

        Args:
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
                self.tau = np.array(tau)

    def apply_joint_position_limits(self, q:(np.ndarray or None)=None, update_joint_data:bool=False):
        """
        Clip the joint position q, according to the limits specified in the robot model

        Args:
            q:                  joint position array (optional)
            update_joint_data:  boolean to update this class self.q with the clipping (default value is False)
        """
        if q is None:
            q = self.q
        q = np.clip(q, self.q_min, self.q_max)
        if update_joint_data:
            self.q = q
        return q

    def apply_joint_velocity_limits(self, dq:(np.ndarray or None)=None, update_joint_data:bool=False):
        """
        Clip the joint velocity dq, according to the limits specified in the robot model

        Args:
            dq:                  joint velocity array (optional)
            update_joint_data:   boolean to update this class self.dq with the clipping (default value is False)
        """
        if dq is None:
            dq = self.dq
        dq = np.clip(dq, self.dq_min, self.dq_max)
        if update_joint_data:
            self.dq = dq
        return dq

    def apply_joint_acceleration_limits(self, ddq:(np.ndarray or None)=None, update_joint_data:bool=False):
        """
        Clip the joint acceleration ddq, according to the limits specified by the user. If no limits were specified, this function will fail.

        Args:
            ddq:                 joint acceleration array (optional)
            update_joint_data:   boolean to update this class self.ddq with the clipping (default value is False)
        """
        if ddq is None:
            ddq = self.ddq
        ddq = np.clip(ddq, self.ddq_min, self.ddq_max)
        if update_joint_data:
            self.ddq = ddq
        return ddq

    def apply_joint_jerk_limits(self, dddq:(np.ndarray or None)=None, update_joint_data:bool=False):
        """
        Clip the joint jerk dddq, according to the limits specified by the user. If no limits were specified, this function will fail.

        Args:
            dddq:                joint jerk array (optional)
            update_joint_data:   boolean to update this class self.dddq with the clipping (default value is False)
        """
        if dddq is None:
            dddq = self.dddq
        dddq = np.clip(dddq, self.dddq_min, self.dddq_max)
        if update_joint_data:
            self.dddq = dddq
        return dddq

    def apply_joint_effort_limits(self, tau:(np.ndarray or None)=None, update_joint_data:bool=False):
        """
        Clip the joint torque tau, according to the limits specified in the robot model.

        Args:
            tau:                 joint torque array (optional)
            update_joint_data:   boolean to update this class self.tau with the clipping (default value is False)
        """
        if tau is None:
            tau = self.tau
        tau = np.clip(tau, self.tau_min, self.tau_max)
        if update_joint_data:
            self.tau = tau
        return tau

    def open_viewer(self):
        """
        Open the web visualiser.
        """
        self.viz.viewer.open()

    def update_visualisation(self, q:(np.ndarray or None)=None) -> None:
        """
        Update the joint state in the 3D meshcat visualiser

        Args:
            q:        joint position array

        """
        if q is None:
            q = self.q
        self.viz.display(q)

    def add_visual_object_from_file(self, obj_file_path:str, name_id:str, material=None) -> None:
        """
        Add a 3D object from a dae, obj or stl file. If the name already exists, it updates the object state.

        Args:
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
            material = meshcat.geometry.MeshBasicMaterial(color=0xff0000, opacity=0.4)

        self.viz.viewer[name_id].set_object(obj, material)


    def add_geometry(self, object:meshcat.geometry.Geometry, name_id:str, material=None, transform: pin.SE3 = None) -> None:
        """
        Add a 3D object from a dae, obj or stl file. If the name already exists, it updates the object state.

        Arg:
            obj_file_path:      file path of the 3D object (string)
            name_id:            reference identifier for the object (string)
            material:           meshcat.geometry material defining color, texture and opacity of the object

        """

        if material is None:
            material = meshcat.geometry.MeshBasicMaterial(color=0xff0000, opacity=0.4)

        self.viz.viewer[name_id].set_object(object, material)
        if transform is not None:
            self.viz.viewer[name_id].set_transform(transform.homogeneous)
    