#!/usr/bin/env python
import numpy as np

from pynocchio import RobotWrapper

class FrictionRobotWrapper(RobotWrapper):
    """
    Class wrapper to use pinocchio robot model with friction modelled.

    Attributes:
        fv (np.ndarray): An array containing the viscous friction gain.
        fc (np.ndarray): An array containing the Coulomb friction gain.
        f0 (np.ndarray): An array containing the Coulomb friction offset gain.
    """
    def __init__(self, tip:(str or None)=None, urdf_path:(str or None)=None, xml_path:(str or None)=None, mesh_path:(str or None)=None, q:(np.ndarray[float] or None)=None, fv:(np.ndarray[float] or None)=None, fc:(np.ndarray[float] or None)=None, f0:(np.ndarray[float] or None)=None):

        super().__init__(tip, urdf_path, xml_path, mesh_path, q)

        if fv is not None:
            self.set_viscous_friction_gains(fv)
        else:
            self.fv = np.zeros(self.robot.nq)

        if fc is not None:
            self.set_coulomb_friction_gains(fc)
        else:
            self.fc = np.zeros(self.robot.nq)

        if f0 is not None:
            self.set_coulomb_friction_offset_gains(f0)
        else:
            self.f0 = np.zeros(self.robot.nq)
    
    def set_viscous_friction_gains(self, fv:(np.ndarray[float] or np.ndarray[np.ndarray[float]])):
        """
        Define the gains of the Coulomb friction torques, such as:
        .. math:: \\tau_{fi}(\\dot{q}_i) = f_{v,i} \\dot{q}_i
        Args:
            fv:       Viscous friction (np.array 1D of size equal to the nb of joints)
        """
        if not isinstance(fv, np.ndarray):
            fv = np.array(fv)

        if fv.ndim == 1 and fv.size == self.robot.nq:
            self.fv = fv
        elif fv.ndim == 2 and (fv.shape[0] == 1 or fv.shape[1] == 1):
            self.fv = fv.squeeze()
        else:
            raise ValueError(f"fc must be a 1D array of size {self.robot.nq}, according to the number of joint of the robot.")

        self.fv = fv

    def set_coulomb_friction_gains(self, fc:(np.ndarray[float] or np.ndarray[np.ndarray[float]])):
        """
        Define the gains of the Coulomb friction torques, such as:
        .. math:: \\tau_{Ci}(\\dot{q}_i) = f_{c,i} \\sign(\\dot{q}_i)
        Args:
            fc:       Coulomb friction (np.array 1D of size equal to the nb of joints)
        """
        if not isinstance(fc, np.ndarray):
            fc = np.array(fc)

        if fc.ndim == 1 and fc.size == self.robot.nq:
            self.fc = fc
        elif fc.ndim == 2 and (fc.shape[0] == 1 or fc.shape[1] == 1):
            self.fc = fc.squeeze()
        else:
            raise ValueError(f"fc must be a 1D array of size {self.robot.nq}, according to the number of joint of the robot.")

        self.fc = fc

    def set_coulomb_friction_offset_gains(self, f0:(np.ndarray[float] or np.ndarray[np.ndarray[float]])):
        """
        Define the gains of the Coulomb friction offset torques, such as:
        .. math:: \\tau_{Ci}(\\dot{q}_i) = f_{o,i}
        Args:
            f0:       Coulomb friction offset (np.array 1D of size equal to the nb of joints)
        """       
        if not isinstance(f0, np.ndarray):
            f0 = np.array(f0) 

        if f0.ndim == 1 and f0.size == self.robot.nq:
            self.f0 = f0
        elif f0.ndim == 2 and (f0.shape[0] == 1 or f0.shape[1] == 1):
            self.f0 = f0.squeeze()
        else:
            raise ValueError(f"f0 must be a 1D array of size {self.robot.nq}, according to the number of joint of the robot.")
        
        self.f0 = f0

    def viscous_friction_torque(self, dq:(np.array or None)=None) -> np.ndarray[float]:
        """
        Compute the viscous friction torques, such as:
        .. math:: \\tau_{fi}(\\dot{q}_i) = f_{v,i} \\dot{q}_i
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
        .. math:: \\tau_{ci}(\\dot{q}_i) = f_{c,i}\\sign(\\dot{q}_i)
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
        .. math:: \\tau_{0i} = f_{0,i}
        
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
        tau_f = self.viscous_friction_torque(dq) + self.colomb_friction_torque(q) + self.colomb_friction_offset_torque()

        if f_ext is None:
            tau_ext = np.zeros((self.robot.nq))
        else:
            tau_ext = J.T.dot(f_ext) # TODO: the user should provide tau_ext directly so that f_ext can be forces exerted on any part of the body

        ddq = np.linalg.inv(A).dot(tau - tau_ext - C.dot(dq) - g - tau_f)
        return np.array(ddq)