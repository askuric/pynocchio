import numpy as np
import os

import pynocchio as pynoc
import time
import pinocchio as pin

import pycapacity as pycap
import pycapacity.visual as pycapviz

import meshcat.geometry as g 

dir_path = os.path.dirname(os.path.abspath(pynoc.__file__))
panda = pynoc.RobotWrapper(urdf_path= dir_path+"/models/urdf/panda.urdf", mesh_path=dir_path+"/models", q=np.zeros(7))
# initial joint position
q = (panda.q_min+panda.q_max)/2

# update the robot
panda.update_joint_data(q=q)
panda.update_visualisation()

# initial joint position print
print("initial q\n", q)

# calculate the polytope
opt = {'calculate_faces':True}

# polytope material
mat_poly = g.MeshBasicMaterial(color=0x0022ff, wireframe=True, linewidth=3, opacity=0.2)
# ellipsoid material
mat_ellipse = g.MeshBasicMaterial(color=0xff5500, transparent=True, opacity=0.2)

while True:
    # some sinusoidal motion
    for i in np.sin(np.linspace(-np.pi,np.pi,200)):

        # update the joint position
        q[0] = i
        q[1] = i
        q[2] = i
        # update the robot
        panda.update_joint_data(q=q)
        # update the visualisation
        panda.update_visualisation()

        # calculate the jacobian
        J = panda.jacobian_position(q)

        # calculate the polytope
        vel_poly = pycap.robot.velocity_polytope(J, panda.dq_min, panda.dq_max,options=opt)

        # meshcat triangulated mesh
        poly = g.TriangularMeshGeometry(vertices=vel_poly.vertices.T/5 + panda.forward().translation, faces=vel_poly.face_indices)
        panda.add_geometry(object=poly,
                           name_id='poly', 
                           material=mat_poly)


        # calculate the ellipsoid
        vel_ellipsoid = pycap.robot.velocity_ellipsoid(J, panda.dq_max)
        # meshcat ellipsoid
        ellipsoid = g.Ellipsoid(radii=vel_ellipsoid.radii/5)
        panda.add_geometry(object=ellipsoid, 
                           name_id='ellipse', 
                           material= mat_ellipse,
                           transform=pin.SE3(vel_ellipsoid.rotation, panda.forward().translation))
 
        # wait a bit
        time.sleep(0.01)