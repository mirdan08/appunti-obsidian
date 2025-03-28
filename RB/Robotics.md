# Robotics mechanics and kinematics 

A robot is an autonomous system which exists in the physcial world and can sense it's environment and is able to act on it to achieve some goals. 
## definition of robot
Robot can either need or not need interaction with a person:
- A robot is *autonomous* when it can act on the basis of it's own decisions and is not controlled by a human. 
- A robot is *non autonomous* when it is controlled by an operator step by step
- A robot is *semi-autonomous* when control is shared between robot and users, it possible at different levels of autonomy.
A robot can exist in the physical world, which means it is subject to the laws of physics that are different from simulations, in the physical world interactions and physical laws are simulated and somehow approximated.
A robot has sensors which are used to perceive information from the world, i.e. it SENSES it's world. 
A robot can also ACT on it's world and uses *effectors* and *actuators* to achieve what is desired.
and is also intelligent meaning it has some reasoning capability. 
At last robot is intelligent in the sense that it has a reasoning capability to some degree.
## robot components
The components of he robot are then.
- Physical body, which is in the physical world.
- Sensors, so it can perceive its environment.
- Effectors/actuators, so it can take actions.
- Controller so it can be autonomous.

![[Pasted image 20250219094727.png]]
The effectors themselves are of various types like in the image below.

![[Pasted image 20250219094852.png]]
An *effector* is the part of the system which interacts with the environment to achieve a task and is composed also by *actuators* that put energy motion.
## Degree of freedom

The first definition we need is a *degree of freedom*.
Definition: the configuration of a robot is complete specification of the position of every point of the robot. In a mathematical sense it is the minimum number of real valued coordinates/parameters used to represent the configuration of the robot and is called *degree of freedom*.

![[Pasted image 20250219095324.png]]
here we have some examples that combine springs, pendulums and linear motions that can be used to describe completely their state with different relationships with one another.

![[Pasted image 20250219095440.png]]
Other real life examples in this case we define coordinates in the world not motion, as you can see it heavily relies on how you model a problem and how much information you need.

For rigid bodies we have both movement and orientation, see the helicopter below for an example.
![[Pasted image 20250219095606.png]]

In a 3D space we have at most 6 DOFs, 3 are used for translation and movement description and the other for orientation as shown in both the images above and below. However not all objects can move on all these degrees at their will, for example a car can only move on the x and y axis but not z, i.e. it can't move horizontally, and also it cannot rotate on all directions while the helicopter can actually.
![[Pasted image 20250219095716.png]]
Formally we have DOFs controlled by an actuator called *controllable DOFS* ($CDOF$) and the others that aren't. Total number of DOFS is indicated as $TDOF$ and depending on their number we can derive three situations:
- *holonomic* $(CDOF=TDOF)$ : the robot controls all its DOFs.
- *non-holonomic* $(CDOF<TDOF)$ : the robot controls only a part.
- *redundant* $(CDOF>TDOF)$: the robot has more ways to control the DOFs it can control.
## Joints and DOFs
*Joints*, in the mathematical sense, are set of two surfaces that can slide keeping contact one to the another. The link between the two joints is the DOF of the robot and *link 0* is the support base and origin of the reference coordinate frame for robot motion.
![[Pasted image 20250219100701.png]]
Observe that a link can have more degrees of freedom depending on the kind of movement it does as shown in the image above:
- it can revolute, called **Revolute** (R) where it revolves.
- it can slide, called **Prismatic** (P) where it slides.
- it can rotate, called **Helical** (H) where it has an helical movement around an axe.
- it can be **Cylindrical**(C) where it slides and rotate along a certain axe.
- it can be **Universal** (U) , where it moves rotates along two axes
- it can be **Spherical** (S) where the movement can be along a certain sphere with a defined surface.
Most common robot joints are the revolute but can also prismatic with different variations like linear,sliding,telescopic etc.etc.
![[Pasted image 20250219214647.png]]

Now we can see different joint types, in the image below we have two important points:
- $\{0\}$ which is the base frame of the model
- $\{E\}$ which is  the coordinate from of the end-effector where the combination of those joints ends up.

![[Pasted image 20250219102405.png]]
In this case we have $(x_0,y_0)$ that are the base coordinates for the base frame and $x_E,y_E$ that  are the coordinates of the end effector coordinates.
The joint variables for angles or primastic extension are denoted with $q$ and the length for fixed size parts are denoted using $a$.
## robot manipulator
![[Pasted image 20250219215534.png]]
A *robot manipulator* is nothing more then an open kinematic chain.
An open kinematic chain is a sequence of rigid segments or links that are connected using revolute or translational joints actuated by a motor.
An extremity is connected to a support base and the other ones is free and equipped with a tool called end effector.

The human arm is quite difficult to model, it has seven DOFs nad the arm itself has only three joints: shoulder,elbow and wrist with those it controls alls the seven DOF. in particular the shoulder is quite difficult to model as is the hand given the biologic formation.
![[Pasted image 20250219220457.png]]
At its most simple incarnation, as in the image above, a robot manipulator consists of *a robot arm*,*a wrist* and a *grippler* into an arbitrary pose.

Its task to to place an object grabbed by the grippler into an arbitrary position, this kind of robot uses one degree of freedom less so six instead of the seven from the classical arm we have seen before. The arm also gives us the position of the robot end point. The wristr enables the orientation needed for the object to be grabbed by the robot gripper.
We can have various kinds of joints, usually we have three of them and either revolute or translational and can be annotated with a three word label composed by R's and T's like in the image below.

![[Pasted image 20250219102947.png]]
## Joint & cartesian space
The *joint space* is the space in which the $q$ vector of joint variables are defined its dimension is indicated with $N$ that indicates the number of joints in the robot.
The *cartesian space* is where $x=(p,\Phi)^T$ resides and it defines the end-effector position. the dimension of such space is indicated with $M$. And $p$ is the cartesian coordinates while $\Phi$ is the orientation of the end effector.
- $q$ is the vector of the robot positon in the joint space and it contains the joint variables epxressed in degrees.
- $x=(p,\Phi)^T$ is the vector of the robot position in the cartesian space with the cartesian coodrinates $p=(x,y,z)$ and $\Phi=(roll,pitch,yaw)$ is the vector of orientation of the end effector.
The *robot workspace* is the region described by the origin and the end effector is when the robot joints execute all possible motions.

![[Pasted image 20250219230128.png]]
This the PUMA robot manipulator, here we have two main subgroups wihch are the support structure  and the wrist in this case we have 6 joint with a shoulder a trunk which offsets the shoulder and the base frame point and affect the end effector position, and also the wrist position.

The *Robot workspace* indicates the region described by the origin of the end effector when robot joints execute all possible motions
The *Reachable workspace* is the region that can be reached by the end-effector with at least one orientation.
The *Dextrous workspace* is the region that the end-effector can reach with more than one orientation.

Such worksaces are all depending on link lengths and joint ranges of motion, an axemple can be seen in the image below.

![[Pasted image 20250220090307.png]]

For a robot ARM-types ca be of different kinds, below you can see various types

![[Pasted image 20250220090402.png]]

## Representing position and orientation

A point $p\in \mathbb{R}^n$ can be represented as a vector from the reference frame point $\{a\}$ origin to $p$ with the vector $p_a$ . We can have different reference frames in the image below we can see $\{a\}$ and $\{b\}$, for each of them we have unit coordinates $\hat{x},\hat{y}$ .
![[Pasted image 20250219104428.png]]
different reference frames , represented as $\{a\}$ in the image above, give a reference to reconstruct the position $p$, whatever reference we use is ok we just need to have stationary position.

Reference frames can be placed anywhere to have a valid representation of the space in our case we assume to have always a single stationary frame called *fixed frame* denoted with $\{s\}$ also called *space frame*. We can also assume that at least one frame has been attached to a rigid moving body, it is called *body frame* and is denoted with $\{b\}$ and is always assumed to be attached to a rigid body. In the image below we see an example with the fixed frame being static and fixed from which the axes of the space are originating and then we have the second point which can also be represented as the vector $p$ and it is assumed to be attached to an L-shaped body, in this case it also has an angle $\theta$.
Also note that the body frame is attached to some important point of the body but it is not necessary.
![[Pasted image 20250219104618.png]]
such reference frame is stationary,$\{s\}$, and can be used to describe the distance from other reference frames that are used to describe the distance from other non-stationary reference frames, $\{b\}$ , that are attached to a moving rigid body. we don't need stuff like the center of mass but it can be useful.
## Rigid body motions in the plane
The body-frame origin $p$ can be expressed as: 
$$
p=p_x\hat{x}_s + p_y\hat{y}_s
$$
Now we want to descrbe the unit vectors used for the orientation of the body which are $\hat{x}_b,\hat{y}_b$ to do so we can use the following formulas 

$$
\hat{x}_b= \cos \theta \hat{x}_s + \sin \theta \hat{y}_s 
$$

$$
\hat{y}_b= - \cos \alpha \hat{x}_s + \sin \alpha \hat{y}_s 
$$
It becomes clearer when you use a raphical explanation of the formulas, take the following intutions :
![[Pasted image 20250220093426.png]]

The $\cos$ indicates the horizontal component of the unit vectors and $\sin$ indicates the vertical component, their sum allows you to obtain the the respectve unit vectors and you also have their orientation with simple trigonometric formulas where $$\alpha=(\pi -\frac{\pi}{2}-\theta)=\frac{\pi}{2}-\theta$$
so in this case it's a just 90 degrees rotation of $\theta$ and a rotation of $\alpha$ degrees of the unit vectors of the reference frame. From this you can take out $\alpha$ from $\hat{y}_b$, since $\cos(\frac{\pi}{2}-\theta)=sin(\theta)$ and $\sin(\frac{\pi}{2}-\theta)=\cos(\theta)$ you have that
$$
\hat{y}_b=- \sin \theta \hat{x}_s+\cos \theta \hat{y}_s
$$
and in case $\Theta=0$ we have that 

$$
\hat{x}_b=\cos(0)\hat{x}_s+\sin(0)\hat{y}_s=1*\hat{x}_s+0*\hat{y}_s=\hat{x}_s
$$
$$
\hat{y}_b=-\sin(0)\hat{x}_s+\cos(0)\hat{y}_s=0*\hat{x}_s+1*\hat{y}_s=\hat{y}_s
$$
so this is the situation now:
![[Pasted image 20250220094612.png]]
So orientation can be described using only $\theta$  relatively to the fixed frame while the position $p$  expresses the position w.r.t. the reference frame and is just a column vector:
$$
p=\begin{bmatrix}
p_x\\p_y
\end{bmatrix}
$$
we can also use a rotation matrix $P$ to have the same result:
$$
P=[\hat{x}_b\text{ }\hat{y}_b]=\begin{bmatrix}
\cos \theta & - \sin \theta \\
\sin \theta & \cos \theta
\end{bmatrix}
$$
for P we have that: 1. each column must be a unit vetor 2. the two columns must be orthogonal and 3. the remaining DOF is parametrized by $\theta$  as happens with $P$.

So the pair $(P,p)$ provides a description of the orientation and position of $\{b\}$ w.r.t. $\{s\}$ and can be expressed as $(P,p): \{ s \rightarrow b \}$.

Ok now we have a framewor for such cases, say ou have the following situation
![[Pasted image 20250220095535.png]]

For the previous notation $(Q,q): \{ c \rightarrow b \}$ and $(P,p): \{ b \rightarrow s \}$ in the image above.
So here we have a single reference frame and two body frames, we can convert from a body frame to another in this case we want $\{c\}$ relative to $\{ s\}$  and we can have it by converting $Q$ to the reference frame, to do so $R=PQ$ and $r=Pq+p$ and now we have $(R,r):\{c \rightarrow s\}$.
### 3D rigid body motions in the plane
We need to define also orientation and axes direction in 3D, we say that a space is right-handed when we have it organized lke in the image below

![[Pasted image 20250220102605.png]]

While a rotation is said to be positve when it rotates going left like in the image below.
![[Pasted image 20250220102621.png]]


All our reference frames are right-handed and the unit axes $\{\hat{x},\hat{y},\hat{z}\}$ and always satisfy $\hat{x} \times \hat{y} = \hat{z}$ the fixed frame has unit axes $\{\hat{x}_s,\hat{y}_s,\hat{z}_s\}$ while for a body frame we have the unit axes

$$
\begin{bmatrix}
\hat{x}_b \\ \hat{y}_b \\\hat{z}_b
\end{bmatrix}= R
\begin{bmatrix}
\hat{x}_s \\ \hat{y}_s \\\hat{z}_s
\end{bmatrix}
$$
the $p$ vector from the fixed frame has now the formulation:
$$
p=
\begin{bmatrix}
p_1\\p_2\\p_3
\end{bmatrix}
=p_1\hat{x}_s+p_2\hat{y}_s+p_3\hat{z}_s
$$
and $R$ is expressed as:
$$
R=\begin{bmatrix}
r_{11} &r_{12} &r_{13} \\
r_{21} &r_{22} &r_{23} \\
r_{31} &r_{32} &r_{33} \\

\end{bmatrix}=
\begin{bmatrix}
\hat{x}_b&\hat{y}_b&\hat{z}_b
\end{bmatrix}
$$

in 3D a description of the rigid body's position and orientation is described by 12 parameters defined by $p \in \mathbb{R}^3$ and $R \in \mathbb{R}^{3 \times 3}$ where each oftheir values can vary.

![[Pasted image 20250220105655.png]]


A rotation matrix can:
- represent an orientation
- change reference frame in which a vector or a another frame is represented
- rotate a vector or a frame
![[Pasted image 20250225223021.png]]
We assume a fixed frame space $\{s\}$ which is aligned with $\{a\}$ the orientations of the three frames can be written as 
$$
R_a=\begin{bmatrix}
1&0&0\\
0&1&0\\
0&0&1\\
\end{bmatrix},R_b=\begin{bmatrix}
0&-1&0\\
1&0&0\\
0&0&1\\
\end{bmatrix},
R_c=\begin{bmatrix}
0&-1&0\\
0&0&-1\\
1&0&0\\
\end{bmatrix}
$$

in those frames we can write the location of a point $p$ is written as 
$$
p_a=\begin{bmatrix}
1\\1\\0
\end{bmatrix},
p_b=\begin{bmatrix}
1\\-1\\0
\end{bmatrix},
p_c=\begin{bmatrix}
0\\-1\\-1
\end{bmatrix}
$$

Rotations are considered positive when the revolve around the selected axis in a count-clockwise manner when looking the the perspective in the iamge below.

![[Pasted image 20250225223240.png]]

When rotating we can look first at the rotation around a single axis-x in this case we get that x and x' (the result of rotating) are collinear while the other ar obtained by just rotating the reference frame x-y-z for the angle $\alpha$.
![[Pasted image 20250225223856.png]]

A rotational displacement can be described by an homogenous transformation matrix, the first three rows of the matrix corespond to x , y and z of the reference frame while the first three columns refere to the x'-y'-z' axes of the rotated frame, the upper 3x3 matrix is the rotation matrix $H$ and are the cosines o the angles between the axes given by corresponding column and row

$$
Rot(x,\alpha)=\begin{bmatrix}
\cos 0 &\cos 90 &\cos 90 &0\\
\cos 90 &\cos \alpha &\cos (90+\alpha) &0\\
\cos 90 &\cos (90 - \alpha) &\cos \alpha &0\\
 0 & 0 & 0 &1\\
\end{bmatrix}=\begin{bmatrix}
\cos 0 & 0 & 0 &0\\
0 &\cos \alpha & -\sin \alpha &0\\
0 & \sin \alpha &\cos \alpha &0\\
 0 & 0 & 0 &1\\
\end{bmatrix}
$$

This is a useful exercise to better understand rotation matrices, we can do the same thing with y imposing that $y=y'$ and then that
$$
x=x'\cos \beta + z' \sin \beta
$$
$$
z=-x'\sin \beta + z' \cos \beta
$$
This takes the form of the following matrix

$$
Rot(y,\beta)=\begin{bmatrix}
\cos \beta & 0 & \sin \beta & 0 \\
0 & 1 & 0 & 0 \\
-\sin \beta & 0 & \cos \beta & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$
The situation is as described in the image below

![[Pasted image 20250225225740.png]]

We can then use the complete formulation for the three axes and get a sequence of rotations around axes $R$.
![[Pasted image 20250226093057.png]]
And the rotation matrix combining them is defined as

$$R=R_{x,\alpha}R_{y,\phi}R_{z,\theta}$$
We can use a rotation matrix $R$ to represent a body frame $\{b\}$ in the fixed frame $\{s\}$ and a vector $p$ which represent the origin of $\{b\}$ in $\{s\}$, instead o identifying $R$ and $p$ we package them into a matrix 
$$
T=\begin{bmatrix}
R & p \\
0&1\\
\end{bmatrix}=
\begin{bmatrix}
r_{11}&r_{12}&r_{13}&p_1\\
r_{21}&r_{22}&r_{23}&p_2\\
r_{31}&r_{32}&r_{33}&p_3\\
0&0&0&1\\
\end{bmatrix}
$$
And its inverse is also a transformation matrix 
$$
T^{-1}=\begin{bmatrix}
R&p\\
0&1
\end{bmatrix}^{-1}
=\begin{bmatrix}
R^T & -R^Tp\\
0 & 1 \\
\end{bmatrix}
$$

and also the product of a transformation matrix is a transformation matrix.

we can also pack translation into a matrix too

$$
T_{tran}=\begin{bmatrix}
1&0&0&dx\\
0&1&0&dy\\
0&0&1&dz\\
0&0&0&1\\
\end{bmatrix}
$$
It can be used to obtain the coordinates of a point w.rt. different points in space by just multypling it.

$$P_{xyz}=T_{tran}P_{vuw}$$
![[Pasted image 20250226095833.png]]

The image above explains well the concept.

As for transformation composition we can do that through matrix multiplication

$$^AT_C=^AT_B\ ^AT_C$$
![[Pasted image 20250226100347.png]]
## Robot arm kinematics

We can use *kinematics* to make a robot arm move, we have to ways of doing that:
- Direct kinematics: computing the end-effector position in the cartesian space given the robot position in the joint space.
- Inverse kinematics: compute the joint positions to obtain a desired position of the end effector.
![[Pasted image 20250226100821.png]]

In the direct approach for a given robot arm and a vector of joint angles $q$ and link geometric parameters find the position and orientation of end effector or mathematical sense find the vectorial non linear function $K$ s.t.

$$
x=K(q)
$$
In inverse kinematics for the given robot arm with a desired position and orientation of the end effector w.r.t. reference coordinate frame find the corresponding joint variables.
In thi case find a non-linear vectorial function 
$$
q=K^{-1}(x)
$$

Basically you either find the joint variables rom the desired point or find he the result point from the joint variable.

Since ne number of DOFS s higher than the one needed to characterize the joint space, such number of redundancy is obtained as $R=N-M$, thise means multiple solutions at the cost of control and computational complexity.

Inverse kinematics show a series of problems to deal with:
- non linar equations
- not always possible to fin an analytical solutions
- solutions can be multiple,infinite or not possible for give arm kinematica structures, their existence is grante if the desired position and orientation belong the the dextrous workspace of the robot.

Direct kinematics are quite easy to grasp, just use the homogenous matrices you have seen before the multily them to get the result, see the image bew for an example

![[Pasted image 20250226104027.png]]

A homogenous transformation matrix can be either used to describe the pose of a frame w.r.t. a reference frame or the displacement of a frame into a new pose. The difference is given by interpretation, for the first case the upper-left 3x3 matrix is the orientation of the object while the riht hand 3x1 column describes the position, for the second case the matrix corresponds to a rotation and the riht-hand column is the translation.
![[Pasted image 20250226120359.png]]
This is an example to obtain the rotation matrix switch from a reference to another nad we can see that it is justthe combination of various rotations toruh an inhomogenous matrix.

We can start by studying the geometric manipulator model.

We have  matrix $^O H_3 = (^O H_1 D_1)(^1 H_2D_2)(2^ H_3 D_3)$ with matrices $^O H_1,^O H_2,^O H_3,$ that describe the pose of ech joint rame w.r.t. the preceding frame. 
![[Pasted image 20250226121047.png]]
This is the kind of robo we are working with.

We an start by analyzing wht is happening within the parenthesis

$$
^O H_1 D_1 = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & l_1 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
c1 & -s1 & 0 & 0 \\
s1 & c1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}=
\begin{bmatrix}
c1 & -s1 & 0 & 0 \\
s1 & c1 & 0 & 0 \\
0 & 0 & 1 & l_1 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$
$$
\sin(\theta_1)=s1,\cos(\theta_1)=c1
$$
We undestand that the $D_1$ matrix represents rotation around the positive $z_1$ axis and the product is the displacement and pose and displacement of the first joint.

For the second joint w have a rotation around the $z_2$ axis.

$$
^O H_2 D_2 = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & l_2 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
c2 & -s2 & 0 & 0 \\
s2 & c2 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}=
\begin{bmatrix}
c1 & -s1 & 0 & 0 \\
s1 & c1 & 0 & l_2 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$
And it still is a rotation like before just on a different xis, now we see a translaton for a change

$$
^O H_3 D_3 = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & l_3 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & -d_3 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}=
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & l_3 \\
0 & 0 & 1 & -d_3 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

All those combined give the following result

$$
^O H_3 = \begin{bmatrix}
c12 & -s12 & 0 & -l_3s12  -l_2s1\\
s12 & -s12 & 0 & -l_3c12 -l_2c1\\
0 & 0 & 1 & -l_1 - d_3\\
0 & 0 & 0 & 1\\
\end{bmatrix}
$$
with $$
c12= \cos (\theta_1 + \theta_2)=c1c2-s1s2
$$
$$
s12= \sin (\theta_1 + \theta_2)=s1c2+c1s2
$$
## The Universal Robot Description Format

The URDFs an XML file format used by the Robot Operating System to describe kinematics, inertial properties and link of geometric robots in a tree manner.
The URDFdescribes joints and links of a robot:
- Joints: it's used to connect two links a parent link and a child link. A joint can be of different types that are the types we already discussed. 
  Each one has an *origin frame* that defines a position and orientation of the child link frame relative to the parent link frame when the joint variable is zero and we have join which is on the joint's axis.
  Each Joint has a 3-axis vector, a unit vector expressed i the child link frame which is in the direction of positive rotation for a revolute joint or positive translation for a prismatic joint.
- Links: we define mass properties, the elements of a link among the others also include its mass, an origin frame that defies position and orientation of a frame at the link's center of mass relative to its joint frame and an inertia matrix relative to the link center of mass frame.
A URDF file can represent any robot with a tree structure including serial-chain robot arms and hands.

## The Denavit-Hartenberg representation
the D-H representation is an approach to forward kinematics to attach reference frames to each link of the open chain and then derive the forward kinematics frm the knowledge of the relative displacement between adjacent link frames. We use the matrix based method for describing the the relations between adjacent links, the D-H representation consists of homogenous 4x4 transformation matrices which represent the reference frame w.r.t. the previous link. the position of the end effector can be expressed in the base frame coordinates.
![[Pasted image 20250226220955.png]]
This is the model in question.

For link we have 4 geomtric parameters associated to each link:
- 2 describe relative position of adjacent link(the joint parameters)
- 2 describe the link structure
On those geometric parametrs depend the homogenous transformation matrices but only one set is known, we have a joint rotation axis defined by the 2 links connected by the joint. For each axes we have 2 normal lines defined one for each link.

![[Pasted image 20250226221851.png]]

From the kinematics POV a link keeps a fixed cofiguration between 2 joits, he structre of link $i$ is characterized with the length and the angle of the rotation axis of join $i$.
In the picture above:
- $a_i$ = minimum distance along the comon normal line between two joint axes, **LINK LENGHT**
- $\alpha_i$ which is the angle between two joint axes on a plane normal to $a_i$. called **TWIST**
- The position of the $i$-th link w.r.t. the $(i-1)$-th link can be expressedby measuringthe distance an the angle between two adjacent links. 
- The distance $d_i$ = distance between normal lines along the $(i-1)$-th joint axis, called **LINK OFFSET**
- $\theta_i$ = angle between two normal lines on a pan of the normal axis, called **JOINT ANGLE**.

# Robot behaviour

In industrial robotics you have a structured environment that is easy for robots to interact with then we switched to service robots that could operate in an unstructured environment with the development of techniques and theories for perception & action control.

Robot behaviour is divided into two levels:
- high level
- low level
![[Pasted image 20250224213552.png]]
The interaction happens with a UI tht produces inputs to the behaviour control which itself controls controllers that ake robot act, the robot will then inform controllers and behaviour control with extrnal information and results of the actions.

## Robotic paradigms

We need to study the paradigms we can apply to drive robots and wich approaces suit best some situations. Three main paradigms:
- hierarchical
- reactive
- hybrid
All three differ i the way the accepted "primitives" are organized, commonly we accept the following primitives:
- SENSE: take the information from the robot sensors and produce and output to other functions
- PLAN: takes information from the SENSE or from a wold model and produces task for the robot
- ACT: takes the task for PLAN and produces output commands for the robot actuators
The paradigms are implemented using "architectures", and they can be described in 2 ways:
- by the relationships between the 3 commonly accepted primitives of robotics.
- by the way sensory data is processed and distributed through the system.
For each primitive we ca describe the information flow
![[Pasted image 20250224214311.png]]
Robotics architectures give a principled way of organizing a control system, hey impose constraints on the way the control problem can be solved and describe a set of architectural components and how they interacts i.e. they are the building blocks of programming a robot.

An architecture is evaluated by:
- modularity
- portability
- robustness
this is the way primitives are tied together, we sense plan based on the sensed information, such planning produces directives that are acted upon by producing actuator commands.
![[Pasted image 20250224214913.png]]
The loop is closed by the fact that we have that we the sense the world again seeing our changes. Also note that ACT can not only receive directives but also sensed information, this makes it so that we could have an arrow from sense to arrow creating another paradigm called "reactive".

Basically in the way we connect the primitives we have different paradigms, we start with the hierarchical ones
### hierarchical architectures
Hierarchical architectures usually provide cognition between perception and action where we elaborate data between percepting it and acting after reasoning on it

![[Pasted image 20250224215502.png]]
Cognition interprets perception and plans robot tasks:
- SENSE primitives generate a world description
- PLAN uses such description to produce a sequence of tasks
- ACT executes the tasks produced
perception is used to establish an maintain correspondence between the internal world model and external world, a typical world model cotains:
- a representation apriori of the environment of where the robot operates
- a perceived sensory information
- more information needed for task execution
each time world representation is modified when the robot perceives the environment and the action plan is established based on it.

Processessing informations is very important and we have to divide and distribute logically and functionally the tasks.

Horizontal and sequential decompositions of the chain of information processed by the central system
![[Pasted image 20250224220144.png]]
So we have sensors -> perception -> modeling -> planning -> execute motor control then actuators will act on physical world. Information flow is sequential.

Generally PLAN  is structured in 3 levels:
- strategic 
- tactical
- executive
The *strategic* level generates the strategy on the basis of the task to accomplish.
The *tactical* level generates the commands by interpreting instructions from the strategic level.
The *executive* level receives macro-commands generated by the intermediate level ad takes care of real-time control of actuators.
We can see a very easy to understand view of this concepts:
![[Pasted image 20250224220908.png]]
A controller is structured around primitives
![[Pasted image 20250224220953.png]]

This shows better than scheme above and in particular we see that SENSE and ACT are quite simple  the PLAN is more complex and we can see better where its levels come into the scene:
![[Pasted image 20250224221154.png]]
basically we first use sensed information to produce an intermediate representation that can be process to produce actions for the actuators.

We have a lot of drawbacks:
- Time-scale: it takes a lot of time to produce a plan for a real environment
- Space: we don't have the physical memory to compute the plan
- Information: generating a plan for a real environment require updating world model which takes a lot of time
- executing a plan when available is not trivial.
### reactive paradigm

We can derive the reactive paradigm now which can skip planning basically which makes us waste a lot of time and computational power.

So we can do a switch and go from a model-based view to a reactive behaviour based architecture.
![[Pasted image 20250224222210.png]]
We don't use cognition anymore perception and action are fused now.

We ca see that actually complex behaviour can be obtained by simple sensors like heat for example and we can use clever linkings between them. Now our architecture is modified as follows:
![[Pasted image 20250224222548.png]]
World representation is cut off: it is not modeled nor stored in a memory but it is extraced i real time through sensors. And give world model not existing also a priori planning does not exist.

![[Pasted image 20250224223617.png]]
reaction can be treated sing reactive rules through *reactive rules* where we represent each time a rule that given a certain sensor produces an action. the way such actions interacts depends on the action selection mechanism, as shown in the image below:
![[Pasted image 20250224223250.png]]

Basically we either select the action, in this case we have *arbitration* or rules could combine to produce actions, in this case we have a *fusion* selection.


Actually we can use more complex rules to define behaviours based on the sensors
![[Pasted image 20250224223724.png]]
This will produce a set of behaviours that will be concerned with a specific aspect of the overall behaviour, each one is a FSM and work asynchronously and in parallel with the others and uses multiple information flows each one related to a specific robot function.
Logically we see a BEHAVIOUR as a SENSE-ACT reactive couple and we have a set of behaviours.
![[Pasted image 20250224224002.png]]

For behaviour based architectures we have a few assumptions to make:
- We have a *situated agent* which is operating in an ecological niche, it is part o the world and changes it upon receiving new sensory inputs.
- The robo is *behaviour-based* behaviours serve as thebuilding blocks for robotic actions and abstract representational behaviour of the robot, such behaviours are independent computational entities and operate concurrently.
- We have *locality* and behaviour specific sensing using an abstract representational knowledge in perceptual processing is avoided .
- Lastly we have *indipendence* between the behaviours so a shared world model is not possible.
Behaviour based architectures yield high adaptability to environment changes, low computational complexity in each behaiour and the overall computational cost is low, parallelism is possible and heaviours are easily extensible with no world model required. On the other hand the overall beahviour is difficul to predict, managment of concurrency is needed and when increasing the number of behaviours we have the complexity of concurrency management also increasing with consequent difficulty in conflict resolution.
#### subsumption architecture
Behaviours themselves are organized in an architecture based on levels, with *control levels* corresponding to the competence levels of vertical decomposition:
- lower levels concern stuff like basic functions like obstacle avoidance
- higher levels concern more goal-directed actions
Basically higher levels subsume lower ones and level work in an independent and concurrent way.

![[Pasted image 20250224230104.png]]
Here each level is a behaviourm detail we can see below:
![[Pasted image 20250224230519.png]]
Each behaviour has input and output lines, the output lines of a behaviour can be connected to input and output lines of other behaviours, an input signal can be suppressed and replaced with the signal that suppressed it while an output signal can be inhibited.

One way of handling behaviours is using *potential field behaviours*, here vectors represented behaviours and vector summation combines vectors from different behaviours to produce an emergent behaviour.

### hybrid paradigm
Here robot behaviour has plan to some degree that interferees between SENSE and ACT as represented in the imagee below
![[Pasted image 20250225220842.png]]
In the hybrid paradigm directives go to a hybrid SENSE-ACT primitive that produces then actuator commands.

We have a PLAN primitive the has only the strategic and tactical level, the strategic planner makes long-term plans by identifying the sequence of sub-tasks needed to reach the goal and it provides results for the tactical planner which then initializes and monitors the behaviours by coordinating them in time. Thus you now have the situation described in he image below.
![[Pasted image 20250225221610.png]]
This can actually take many forms.
![[Pasted image 20250225221639.png]]

# Sensors

In this section we take care of sensors and perception and sensors in general.

Sensors allow for two kind of perceptions:
- Proprioception: measure variables internal to the system e.g. joint position
- Extroception: measure variables from the working environment e.g. dstance,force etc.etc.

They aim at building an internal model of the model and robot itself, such models influences robot's brain complexity.

## transducers

The first thing we define is a *transducer* which transforms energy of a kind into a energy of another kid and can work as both sensor and actuator, a *sensor* is a device sensitive to physical quantities and transforms them into a measurable and transferable signal by means of a transducer. It acts as both sensor and actuator ad can be of two types:
- Passive transducer: requires external energy for energy conversion.
- Active transducer: convert directly input energy in output without external energy sources.
Based on the kind on input,output and auxiliary energy we can classify transducers:
- Radiant: intensity,frequency,polarization and phase.
- Mechanical: position, velocity, dimension, compliance an force.
- Thermal: temperature, gradient of temperature and heat.
- Electrical: voltage, current, resistivity and capacity.
- Magnetic: field intensity, flow density and permeability.
- chemical: concentrations, crystal structure and aggregation state.
all these kinds types of energy can be either input,output or auxiliary
## Sensor's fundamental properties

A sensor is then defined as a device sensitive to physical quantities that can tranaform it in a transferable signal by means of a transducer, it inherits transducers properties and can be classified as:
- proprio or extero ceptive
- active or passive
- w.r.t. a physical quantity

There are quite a lot of properties useful to describe and understand for a sensor.
- transfer function: it is the relation between quantity of measure and output of the sensor.
- calibration:measuring outputs for known quantities , it can also be a *calibration cycle* where we do a trial to see the whole working range of a sensor usually divided into a part with increasing values and a part with decreasing values.
- hysteresis: a sensor might have for the same input value diverse outputs which depends on the fact that the input value can either increase or decrease while varying. It is measured as the maximum difference between two output curves of the sensor during the calibration cycle, epxressed as % maximum value for he transfer function see the image below.
![[Pasted image 20250305093729.png]]
- accuracy: is the maximum error between actual values and measured values by the sensors
- repeatability: when the same input values applies to a sensors we can have variability in output such variability is called *repeatability*.
Accuracy and repeatability are actually correlated and are used to evaluate a sensors capability.

![[Pasted image 20250305094234.png]]

For  $x_m=\text{average value}$ and $x_v=\text{actual value}$ we can express accuracy as 
$$accuracy=100(x_m-x_v)/x_v$$
so we just measure how much dispersion a sensor can have.

- resolution: is the minimum variation of the input which gives a variation of the output of the sensor.
- sensitiveness: a small variation in input causes small variation in output, this measure is the ratio between input and output variation.
- noise. is the amount of signal in the sensor output which is not given by the input
- stability: how much the sensor can epp its working characteristics for a long time, it either long,medium or short.
There are much more parameters.

This is just an introduction well see in detail

### resolution & sensitivity
With **resolution** we mean the smallest amount of change we can detect in the input that can be detected and accurately indicated by the sensor.
With **sensitivity** we measure the change in output relative to a unit change in the input i.e. the measured quantity.

![[Pasted image 20250325104535.png]]

Here we see batter what happens, the sensitivity is the distance between the two values between the two consecutive steps i.e. the unit change in the input.
### Sensor calibration
Sensors need to be calibrated, the **calibration** is a procedure where we measure sensor's output for known quantities.

We do a **calibration cycle** is the trials that overs the whole working range of a sensor, we have two parts:
1. one with increasing values
2. one with decreasing values
Below we see an example of cycle results for a linear fitting experiment

![[Pasted image 20250325105123.png]]
This is the measurement of a temperature sensor showing value results at the variation of temperature related to the voltage output with a linear fit, in this case relationship is linear and is quite easy to describe and understand.
The curve generated is the *calibration curve*  and it used to measure **linearity**
### Linearity
The linearity is measured as the maximum difference expressed in % of the maximum value between characteristic curve and the reference line.

The linearity is referred to the static calibration plot of the curves obtained when we show output and input amplitude under static conditions, its degree of resemblance to a straight line is the linearity.
![[Pasted image 20250325111707.png]]

the ideal response here is the reference line and he actual response is the response of the parameter. Basically the more we have linearity the easier it is to invert the relationship and obtained back the measured parameter when using the sensor.

### Hysteresis
If a sensor has hysteresis we have that for the same input value output varies, we measure it as the difference between the two output curves of the sensor during the calibration cycle and is expressed as a % of the maximum value for the transfer function

![[Pasted image 20250325112103.png]]
Usually it happens for variations of sensors in he full cycle of measurement.

### repeatability and stability
We use **repeatability** to see how much our sensor is reliable over time, it is defined as deviation between measurements in sequence when the tested object approaches the same value form the same direction each time usually expressed as percentage in the sensor range.

The **stability** is how much a sensor can keep its working characteristics over time , usually classified as wither *short*,*medium* or *long* w.r.t. the time.


## Sensor's role in a robot

The sensor's have two different roles:
- Perception of external state *(extroception)*: measure variable to characterize the working environment
- Perception of the internal state *(proprioception)* : measured variables in the internal system used to control the robot e.g. joint position.
Simple example with two wheeled robot
![[Pasted image 20250305095242.png]]

## position sensors
We can start by examining position sensors
### switches

mechanical switches are the most simple ones with binart data that is either contact or no contact.

![[Pasted image 20250305095443.png]]

applications robotics are for impact sensors, contact sensors and endstop sensors for manipulator joints.

### optical encoders
Measures the angular rotation of a shaft/axle

![[Pasted image 20250305095606.png]]
This is what we are working with, basically in the image the motor revolves around the axles, we have a slotted disk attached to the axle and an emitter shooting a signal to a detector interrupted periodically by the disk,by this mean we can deduce motor rotation velocity.

Motors themselves can have different configurations depending on where the sensor is placed, see the image below to understand better.

![[Pasted image 20250305100024.png]]

We have a motor and a reducer that reduces the number of rotation of an axle from original motor generating movement, a sensor can be placed:
- before motor+reducer and knows the rotation of the original motor.
- after motor+reducer where it knows the final joint rotation.
In the image above you also see some variable to take car of, basically we want to model the relation ships between the join angular position and the actual motor position.
the relationship is expressed as $$\theta=\frac{\theta_m}{k}$$ and we can also consider the sensor error, remember it is expressed as 
$$
\frac{d\theta}{d\theta_m}=\frac{1}{k} \implies d\theta=\frac{1}{k}d\theta_m
$$
The sensor error is reduced by a factor $k$.

The actual rotation is obtained by counting pulses and knowing the number of disks steps and is just the angle betwen different agular positon of out joint, more specifically we can see the situation described before as

![[Pasted image 20250305102613.png]]

Now for 
- $q=\text{joint angular position}$
- $\theta=\text{join position in encoder steps}$
- $k=\text{motor reduction ratio}$
- $R=\text{encoder resolution=\# of steps per turn}$  
$$
q=\frac{\theta \times 360^o}{R \times k}
$$
And the frequency of the pulse train is proportional to angular velocity.
As for the joint angula positon $\theta$ and $R$ determine the percentage of rotation done then $360^o$ converts it to degrees, finally k applies the reduction of rotation by the reducer.

### absolute encoders

We can have a absolute rotation angle and each position is uniquely determined

![[Pasted image 20250305105528.png]]

this is the optical disk used, it has
- k photoswitches
- k code tracks
- and can represent $2^k$ different orientations with an angular resolution of $360^o / 2^k$ 

This is a photo for the actual piece mounted on a motor

![[Pasted image 20250305110315.png]]

In this case the absolute encoder is mounted before motor+reducer.

Positions are encoded through fixed sensors and follow the positive track motion from 0 to 360 degrees as in the image below.

![[Pasted image 20250305111738.png]]

As you can see each position is uniquely determined, we use the gray code to obtain the position as reported in the table below
![[Pasted image 20250305111858.png]]

the special thing is that only two consecutive position have a different character basically.

We can see an example below

![[Pasted image 20250305112303.png]]

### potentiometers

We need to understand first some electronic basics


![[Pasted image 20250305112508.png]]
This is the classical tension applied to a circuit with a resistor with a certain direction given the positive ad negative signs and current by flowing in that direction and a resistor in the circuit.
These components are tied by the following relationship 
$$
v=Ri
$$

A potentiometer is just a variable resistor 
![[Pasted image 20250305112742.png]]
We have a resistance, 3 endpoints and a movable sliders, by moving he slider by hand we actually vary the Variable Voltage Output  in the sensor i.e. the output voltage, the working principle is that the output varies w.r.t. the resistance applied that is more or less present in the circuit.

We divided the full resistance elelment into $L_T$ and $R_T$ ,where $L_T$ is the part left out and $R_T$ the one present. The influenced part is also divided in $L_1$ and $R_1$ .
The relationship betwen them is 
$$
\frac{L_1}{R_1}=\frac{L_T}{R_T}
$$

we have the full tension $v_{ref}=R_Ti$ between endpoint 1 and 3 and we measure the output tension $v_{out}=R_1i$ . Also the relation between the $L$ components is that of a proportion expressed as below.

$$
L_1=L_T\frac{R_1}{R_T}=L_T\frac{v_{out}}{v_{ref}}
$$
### Hall effect sensors
this kind of sensor uses the *hall-effect principle*: where a current $I$ flows it is immersed in a magnetic field of intensity $B$ and voltage $V$ which originate in the direction normal both to current field and to the magnetic field.

![[Pasted image 20250325113300.png]]

such principle is used in much more complicated stuff like gloves and so on
![[Pasted image 20250325114018.png]]

the issue is that we don't get much more data thanthe direction of movement or the activation or not of the sensor or the fact that movement is happening with some degree of intensity.

### Time of flight measurement

We use sensor to gain a sense of ditance from a transducer to an object, this is the situation we are working with
![[Pasted image 20250325114841.png]]

Our transducer emits a pulse , the pulse is the reflected and echoed back at the transducer. by measuring the distance between these  three events we gain information on the distance.
In formulas this is expressed as 
$$d=\frac{t_ev}{2}$$
where 
$$v=\text{avg speed of signals emitted},t_e=\text{time between signal emitted and echo arrived}$$
Such technology can be used for ultrasound sensors .

![[Pasted image 20250325221848.png]]

These are some examples of ultrasound where we can use the logic from before to estimate the distance.

### Laser range sensors

We can user laser waves to measure ranges, in particular we put a pin-hole in short range finding sensors, we use a linear photo-diode as an array detector. The range from a sensor to the object is afunction of th position nd the maximum detected light along the array

![[Pasted image 20250325223523.png]]



![[Pasted image 20250325223726.png]]

### Proximity sensors

We basically detect an object in a spatial neighborhood with no distance measurement:
- passive proximity sensors: detect perturbations of the environment
- active proximity sensors: emit signals and get it back


# Robotic middlewares and ROS

We have a lot of diverse platform components for platforms that compose them depending on the kind of personal device

![[Pasted image 20250305092244.png]]

Basically we have the hardware that allow the SO to run which gives the user the possiblity to have an application and the dev to develop an application. For robot an application is some kind of interaction in the physical world, thw So is ROS and other parts while  HW is obsvious.

A platform is then divided into SW and HW platform.

a SW robot platform includes a lot of tools used to develop application programs such as:
- HW abstraction
- low-level device
- control
- sensing,recognition
- SLAM
- navigation
- manipulaton and package management
- libraries
- debugging
A robot SW platform gives an HW abstraction that combined with SW platforms allows us to develop application programs using software platforms without knowing the HW.

![[Pasted image 20250305092735.png]]
This is the part we are talking about for now.

a robot SW platform gives us 
- reusability
- communication
- availability of support for dev tools
- active community
## Robot Operating System

ROS is a meta OS meaning that it performs scheduling loading monitoring and uses a virtualization layer between applications and distributed computing resources, it acts as a middleware and uses an existing SO, thus meta SO and not SO.

ROS focuses on code reuse maxmization in robotics, we have a set of useful characteristics:
 - distributed process
 - package management
 - public repository + APIs
 - different languages to program
## Yet Another Robot Platform
YARP is a set of libraries, protocols and tools to keep modules and devices to decouple them and like ROS is a meta OS and and has the same objective. 
It is composed by the following components:
- OS: the interface with the OS to support easy data streaming, written to be OS neutral using the opensource ACE (ADAPTIVE Communication Environment) library portable across a very broad range of environments and inehrits portability, almost entirely written in C++.
- sig: signal processing tasks to interface with common used libraries
- dev: interfaces with comon devices used in robotics as framegrabbers digital cameras etc.etc.
in YARP we have a nameserverwhich serves the purpose of maintaining a list of all YARP ports and how to connect to them like a ROS master.

The name is a YARP port usually named "/root" all other programs communicate with the name serer thorugh this port. It is abstracted away using YARP library calls but sometime client don't use yarp libraries thus the need.

Conecting to it is like connecting ot any other port but we need to find the name server and socket port number, an idea is o siply fix the name server on a known machine or you have the server record its contact informaation in config file and other YARP programs will check it to see hwo to reach the name serer if it doesn't work use multicast to discover such server.

A *Port* is an object that can read and write values to peer objects spread throughtout a network of computers and it is possible to create, add and remove connections either from that progra, the command line or another program, ports are speacialized into streaming communciation such as camera images or motor commands and you can swithc networks progtocls for the conections without changing a single line of code. The YARP library supports transmission of a stream of user data across varius protocls or other means like shared memory insulating a user of the library from the detals of the netowrk tecnolgoy.

We use *Connections* to make named entities called *Ports* communicatetogehter they forma YARP networkwhere ports are a nodes and connectons are dgs.

Ports want to move *Content* which is sequences of bytes representing user data, from one thread to another across pocess and machine boundaries, the flow of data can be manipulated an montored externally. A por can send *Content* from a thread to another or across machine boundaries and process boundaries and glow o data can be manipulated and monitored externally. A part can send Content to any other Port and can receive Content from any other number o other ports, if one port is configured to send content the have a Connecton which can be freely added or removed.

The YARP name sere tracks information bout ports, it indexes the information by name like a DNS , to communicate with a port the porperties of that port need to beknown the YARP name server offers a conveniente place to store thee propertiesso that only the name ofthe port is needed to recover them.

A YARP network is made of he following entities:
1. a set of orts
2. a set of connections
3. a set of names
4. a name server
5. a set of registrations
Each port has its unique name, each connecion has a source port and a target port, eachport maintains a list of all connections for which it is the target port. 

In the netowrk we have a single name server and the name server maintains a list of registrations each registration contais ifomration abotu a single port identified by the name.

Communication can occur between:
- two ports
- a port and a name server
- a name server and an external entity

To communicate there must be a Connection betwen them, the connections involvig a certain port can be crete destroyed or queired by commuication between the external entity and that port, usually done by sending "port commands" using the YARP connection protocol.

Ports comunicate with the name server using the "YARP name server protocol"  and such comunication method is used to create.remove,and query registrations.

Here you can see an example of a YARP network

![[Pasted image 20250315113944.png]]

# Navigation

As of now we know that we can assume the robot to be placed in the space, but we need to be sure of how to locate the root in the space.

we want a reference coordinate system, a first idea would be to place it in space as below.
![[Pasted image 20250325234012.png]]
 the second approach is the use the robot itself.
 ![[Pasted image 20250325234041.png]]
 

in the second case we just treat everything relatively to robot and usa polar coordinates to represent ostacles, see the iamge below for an example.

![[Pasted image 20250325234203.png]]

We can also transform using polar coordinates to switch at will from world reference and robot reference frame in the velocity space

![[Pasted image 20250326093536.png]]

The navigation problem can be stated as reaching a final position from a starting position in geometric/sensory terms while avoiding obstacles.

The way we can tackle this is with these instruments:
- Localization: we have a geometric position $(x,y,\theta)$ in an absolute reference system or sensory state in the robot environment.
- Maps/Models: formalization and representation of an environment
- Planning: planning robot movements in the environment

### Maps
A map can be of two kinds:
- Metric/Geometric Maps: they represent the objects in the world using their size and coordinates in the absolute reference frame
- Topological Maps: the represent the objects in the world on the basis of their characteristics and relations among them

The relevant known methods are occupancy grids and geometric descriptions. 
For the occupancy grid the environment is represented on a bidimensional grid usually in a cell fashion where we discretize the world in a simple grid of swaures, see below:

![[Pasted image 20250326094800.png]]

Grids can have a fixed size  like above and below

![[Pasted image 20250326094954.png]]
or variable size to for more accuracy

![[Pasted image 20250326095029.png]]

In the geometric description instead we get an environment represented with geometrical means like segments,obstacles and free space
![[Pasted image 20250326095219.png]]

As for topological maps we have the environment defined in terms of points of interest for the robot and relations among points of interest and the relations between them.
We say a an object to be a *point of interest* when it is relevant to the navigation or the task of the robot.
A point of interest is either define by a position in the robot space or a sensory state.
![[Pasted image 20250326095938.png]]
we can combine a metric map with object recognition and get a meaningful oject labeling for locations on the map.
![[Pasted image 20250326100113.png]]

An easy to understand example for a complex home cis mapping the home envronemnt with points of intereset, first map the in the space
![[Pasted image 20250326100209.png]]
enumerate rooms and for each room number walls clock wise and for each wall number the points clockwise.
![[Pasted image 20250326100232.png]]
finally we get our graph of relationsships.
![[Pasted image 20250326100307.png]]

We can also mix the geometrical and map approaches 

![[Pasted image 20250326100542.png]]

## planning

We now deal with path planning after describin the enviroment around us, the objective is the one described from before, in this case we start a fro a configuration to reach another and avoid obstacles in the mean time.

A simple way of doing so is to consider the robot size, increase the object size virtually and treat the root as a point .
![[Pasted image 20250326100815.png]]

Planning is roughly divide into:
- planning of the path: find the trajectory s.t. they avoid obstacles and reach the goal configuration.
- following of the path: execute the trajectory and avoid unexpected obstacles.

The configuration reachable from the robot are called *robot space* indicated as $C_{space}$ , in this space the robot is a point and obstacles are represented and occupy a ragion called $C_{obstacle}$ while the free region is called $C_{free}$ the path is a trajectory betwen two configs $q_{init}$ and $q_{goal}$ in $C_{space}$ and it belongs to $C_{free}$.

Path planning is done in three ways
### Roadmaps
We connect points in the free space in a network, called *roadmap* made of unidimensional curves in the free space. We just want to connect them along this roadmap.

In the Roadmap we can use two methods:
- the visibility graph
- and the Voronoi diagram

#### Visibility graph
Here we take a visibility graph $G$ whose nodes are the intitial configurations $q_{init}$ and $q_{goal}$ and all vertxes of  the polygons which represent obstales in the map, the edges of $G$ are all the segments that  connect wo nodes in G and don't intersect with obstacle polygons.
![[Pasted image 20250326102121.png]]

A wight can be associated to the edges and corresponding to the distance between the nodes that they connect, a path from start anf stop is found in the graph with alogirthm for minimum paths wihich minimizes the distance.

#### Voronoi diagram

We define free confiurations in teh free space as equidistant from the obstacle region, if the obstacles are polygons the disgram consists ofa finte set of segments and parabolic curves i.e. our roadmap.
![[Pasted image 20250326102427.png]]

We then find a path the goes along those curves to reach our end goal.

### Cell decomposition

It consists of decomposing the free space in regions, named cells, s.t. a path netween to adjacent ones is found and the map represened is called connectivity graph. Two nodes are connecte iff the twocells tha they repsent are connected.
![[Pasted image 20250326102856.png]]

In this case we just use the vertexes to draw vertical lines, each line stops whe nit encounters an obstacle or a wall, when this is one we can treat each one as a nodes in a graph and get our connectivity graph, in the map we can calculate the path and the result of this rocess is a sequence of cells called *canal*, the final path is obtained by taking the mid point of the cell boundaries.
![[Pasted image 20250326103220.png]]#
### potential fields

The robot si a point in space that moves under the influence of an artificial potential produced by the goal configuration and obstacles, the final configuration hsa an attractive potential which pulls the robot towad the goal and obtacles have a repulsive potential which pushes the robot away from them. By summing attractive and repulsive potentials we generate a foce moving the robot toward the goals while avoiding obstacles.

![[Pasted image 20250326103839.png]]

![[Pasted image 20250326103858.png]]
 This is the examples we where toaking about, now we go deeper on potential fields.
We take differentiable potential function $U$ with local minima in $q_{goal}$ , we then pick two function 
$$
U(q)_{att}=\text{attractive potential function},U(q)_{rep}=\text{repulsive potential function}
$$
And our final potential function then becomes
$$
U(q)=U(q)_{att}+U(q)_{rep}
$$
and for point in space the motion of direction is given by
$$
F(q)=-\nabla U(q)=-\nabla U(q)_{att}-\nabla U(q)_{rep}=\begin{bmatrix} \frac{\delta U}{\delta x} \\ \frac{\delta U}{\delta y}  \end{bmatrix}
$$


We need to pick attractive potential, with the requiremnt to have a local minima in $q_{goal}$.
The function we use is the parabolic potential.
$$
U(q)_{att}=\frac{1}{2}k_{att}\rho^2_{goal}(q)
$$
With $\rho_{goal}(q)=||q-q_{goal}||$ i.e. the distance from the goal.

and 
$$F_{att}(q)=-\nabla U(q)_{att}=- k_{att} (q-q_{goal})$$
Now for the repulsive potential we want a protective barrier around the obstacle region to avoid robot contact with the obstacles and not affect it when it is far from obastacles.

$$
U(q)_{rep}=\begin{cases}
\frac{1}{2}k_{rep}( \frac{1}{\rho(q)}-\frac{1}{\rho_0} )^2 \text{ if } p(q)\le \rho_0\\\\
0 \text{ otherwise}
\end{cases}
$$
with $\rho(q)=\text{minimum distance from the object.}$

Now we can place the desired goal and its obstacles with the potential we just discussed, for the current robot confiuration just:
1. sum total force vectors $F(q)$ generate by the potential fields
2. set desired robot velocity $(v,w)$ proportional to $F(q)$.

We can have local minimias occuring when repulsive forces nullifies the attractive force in points different from the goal.

### Path planning

Now we deal with path planning, it differs depending on the representation of space we use.
For topological maps:
- find path in the graph
- translate it into instructions
### Path following
Now we have to stick to our plan, this is the diagram
![[Pasted image 20250326111319.png]]

The result of the planning in a series of points in space $(x_1,y_1,\theta_1),\dots,(x_n,y_n,\theta_n)$ , we find trajecotry and time laws that the robot will follow between each couple of points and te controller makes the robot execute such trajectory.

![[Pasted image 20250326111536.png]]
We use a trajecotory planner to actually follow the the sequence of points, Its is just an interpolation of points 
![[Pasted image 20250326111657.png]]

Now we deal with the hardware architecture of a mobile robot.

![[Pasted image 20250326112332.png]]


The actuatore control has two ways of controlling the actuators:
- position control: setting a position to reach, and it is the rbot finding acclerations and velocities to actally set motors to reach the desired positions
- velocity contrls: it consists in setting a velocity and an acceleration to the wheel motors
![[Pasted image 20250326112914.png]]
This is te Proportional, Integrative and Derivaive control, as we see this is just the 