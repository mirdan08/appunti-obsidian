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