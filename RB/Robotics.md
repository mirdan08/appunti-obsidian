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
This the PUMA robot manipulator, here we have two main subgroups wihch are the support structure  and the wrist in this case we have 6 joint with a shoulder a trunk which offsets the shoulder and the base frame point.

The *Robot workspace* indicates the region described by the origin of the end effector

In robotics it is essential to be able to reconstruct the end position and angulation of a certain movement on a certain robot, o do so we need a *reference frame* thta allows us to build the kinematic chain needed.
![[Pasted image 20250219104428.png]]
differented reference frames , represented as $\{a\}$ in the image above, give a reference to reconstruct the position $p$, whatever reerence we use is ok we just need to have stationary position.
![[Pasted image 20250219104618.png]]
such reference frame is stationary,$\{s\}$, and can be used to describe the distance from other reference frames that are used to describe the distance from other non-stationary reernce frames, $\{b\}$ , that are attached to a moving rigid body. we don't need stuff like the center of mass.

We can derive the body-frame origin $p$ and we can express it as 
$$
p=p_x\hat{x}_s + p_y\hat{y}_s
$$
