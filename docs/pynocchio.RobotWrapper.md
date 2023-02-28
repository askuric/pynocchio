<!-- markdownlint-disable -->

<a href="../pynocchio/RobotWrapper.py#L0"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

# <kbd>module</kbd> `pynocchio.RobotWrapper`






---

<a href="../pynocchio/RobotWrapper.py#L6"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>class</kbd> `RobotWrapper`




<a href="../pynocchio/RobotWrapper.py#L8"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

### <kbd>method</kbd> `__init__`

```python
__init__(tip, robot_path=None, robot_xml=None)
```

RobotWrapper constructor 



**Args:**
 
 - <b>`tip`</b>:  robot end-effector frame name 
 - <b>`robot_path`</b>:  a part to the robot's urdf file (optional)  
 - <b>`robot_xml`</b>:  a string containing robot's urdf/xml (optional) 




---

<a href="../pynocchio/RobotWrapper.py#L75"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

### <kbd>method</kbd> `dk_orientation_matrix`

```python
dk_orientation_matrix(q, frame_name=None)
```

Forward kinematics orientation calculating function 



**Args:**
 
 - <b>`q`</b>:  currrent robot configuration 
 - <b>`frame_name`</b>:  name of the robot frame for which to calculate forward kinematics (optional - default tip frame) 

Returns 
-------- 
 - <b>`oRf`</b>:  SO3 transformation matrix of the desired robot frame orientation expressed in the world frame 

---

<a href="../pynocchio/RobotWrapper.py#L61"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

### <kbd>method</kbd> `dk_position`

```python
dk_position(q, frame_name=None)
```

Forward kinematics position calculating function 



**Args:**
 
 - <b>`q`</b>:  currrent robot configuration 
 - <b>`frame_name`</b>:  name of the robot frame for which to calculate forward kinematics (optional - default tip frame) 

Returns 
-------- 
 - <b>`position`</b>:  cartesian position the desired robot frame expressed in the world frame 

---

<a href="../pynocchio/RobotWrapper.py#L42"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

### <kbd>method</kbd> `forward`

```python
forward(q, frame_name=None)
```

Forward kinematics calculating function 



**Args:**
 
 - <b>`q`</b>:  currrent robot configuration 
 - <b>`frame_name`</b>:  name of the robot frame for which to calculate forward kinematics (optional - default tip frame) 



Returns 
-------- 
 - <b>`oMf`</b>:  SE3 transformation matrix of the desired robot frame expressed in the world frame 

---

<a href="../pynocchio/RobotWrapper.py#L160"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

### <kbd>method</kbd> `gravity_torque`

```python
gravity_torque(q)
```

Gravity torque vector 



**Args:**
 
 - <b>`q`</b>:  currrent robot configuration       

Returns 
-------- 
 - <b>`g`</b>:  n-vector of joint torques due to the gravity 

---

<a href="../pynocchio/RobotWrapper.py#L188"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

### <kbd>method</kbd> `ik`

```python
ik(oMdes, q=None, verbose=True)
```

Iterative inverse kinematics based on the example code from https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_b-examples_i-inverse-kinematics.html 



**Args:**
 
 - <b>`oMdes`</b>:   SE3 matrix expressed in the world frame of the robot's endefector desired pose     
 - <b>`q`</b>:  currrent robot configuration (default robot's neutral position) 
 - <b>`verbose`</b>:  bool variable enabling verbose ouptut (default True) 

Returns 
-------- 
 - <b>`q_des`</b>:  robot configuration corresponding to the desired pose  

---

<a href="../pynocchio/RobotWrapper.py#L89"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

### <kbd>method</kbd> `jacobian`

```python
jacobian(
    q,
    frame_name=None,
    frame_align=pinocchio.pinocchio_pywrap.ReferenceFrame.LOCAL_WORLD_ALIGNED
)
```

Jacobian matrix calculating function 



**Args:**
 
 - <b>`q`</b>:  currrent robot configuration 
 - <b>`frame_name`</b>:  name of the robot frame for which to calculate the jacobian (optional - default tip frame) 
 - <b>`frame_align`</b>:  determining which frame to express the jacoiban in. Can be either: LOCAL_WORLD_ALIGNED (default), WORLD or LOCAL            

Returns 
-------- 
 - <b>`J`</b>:  6xn jacobian matrix for the desired robot frame  

---

<a href="../pynocchio/RobotWrapper.py#L109"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

### <kbd>method</kbd> `jacobian_dot`

```python
jacobian_dot(
    q,
    qd,
    frame_name=None,
    frame_align=pinocchio.pinocchio_pywrap.ReferenceFrame.LOCAL_WORLD_ALIGNED
)
```

Jacobian time derivative matrix calculating function 



**Args:**
 
 - <b>`q`</b>:  currrent robot configuration 
 - <b>`qd`</b>:  currrent robot velocity in configuration space 
 - <b>`frame_name`</b>:  name of the robot frame for which to calculate the jacobian (optional - default tip frame) 
 - <b>`frame_align`</b>:  determining which frame to express the jacoiban in. Can be either: LOCAL_WORLD_ALIGNED (default), WORLD or LOCAL            

Returns 
-------- 
 - <b>`Jdot`</b>:  6xn jacobian time derivative matrix for the desired robot frame  

---

<a href="../pynocchio/RobotWrapper.py#L130"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

### <kbd>method</kbd> `jacobian_position`

```python
jacobian_position(
    q,
    frame_name=None,
    frame_align=pinocchio.pinocchio_pywrap.ReferenceFrame.LOCAL_WORLD_ALIGNED
)
```

Position jacobian matrix calculating function 



**Args:**
 
 - <b>`q`</b>:  currrent robot configuration 
 - <b>`frame_name`</b>:  name of the robot frame for which to calculate the jacobian (optional - default tip frame) 
 - <b>`frame_align`</b>:  determining which frame to express the jacoiban in. Can be either: LOCAL_WORLD_ALIGNED (default), WORLD or LOCAL            

Returns 
-------- 
 - <b>`J`</b>:  3xn jacobian matrix for the desired robot frame  

---

<a href="../pynocchio/RobotWrapper.py#L145"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

### <kbd>method</kbd> `jacobian_pseudo_inv`

```python
jacobian_pseudo_inv(
    q,
    frame_name=None,
    frame_align=pinocchio.pinocchio_pywrap.ReferenceFrame.LOCAL_WORLD_ALIGNED
)
```

Jacobian matrix pseudo inverse calculating function 



**Args:**
 
 - <b>`q`</b>:  currrent robot configuration 
 - <b>`frame_name`</b>:  name of the robot frame for which to calculate the jacobian (optional - default tip frame) 
 - <b>`frame_align`</b>:  determining which frame to express the jacoiban in. Can be either: LOCAL_WORLD_ALIGNED (default), WORLD or LOCAL            

Returns 
-------- 
 - <b>`J_pinv`</b>:  nx6 jacobian matrix pseudo-inverse for the desired robot frame  

---

<a href="../pynocchio/RobotWrapper.py#L174"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

### <kbd>method</kbd> `mass_matrix`

```python
mass_matrix(q)
```

Mass matrix calcualation function 



**Args:**
 
 - <b>`q`</b>:  currrent robot configuration       

Returns 
-------- 
 - <b>`M`</b>:  nxn mass matrix  




---

_This file was automatically generated via [lazydocs](https://github.com/ml-tooling/lazydocs)._
