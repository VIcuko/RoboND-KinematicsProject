## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./writeup_images/FK_Diagram.jpeg
[image2]: ./writeup_images/IK_Angles.jpg
[image3]: ./writeup_images/Euler1.png
[image4]: ./writeup_images/Euler2.png
[image5]: ./writeup_images/Euler3.gif
[image6]: ./writeup_images/Euler4.gif
[image7]: ./writeup_images/8_cylinders.jpg
[image8]: ./writeup_images/9_cylinders.jpg

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Calculations done on notebook according to the information learned during the course and the analysis of the kuka arm and its urdf file.

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The DH parameters are the following:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | 0
1->2 | - pi/2 | 0.35 | 0 | q2 - pi/2
2->3 | 0 | 1.25 | 0 | 0
3->4 |  -pi/2 | -0.054 | 1.5 | 0
4->5 | pi/2 | 0 | 0 | 0
5->6 | -pi/2 | 0 | 0 | 0
6->EE | 0 | 0 | 0.303 | 0

These DH parameters are obtained from the urdf file following the diagram for the previous image, where: 

· alphas represent the twist angles between the z axis shown in the diagram (not the real world axes as referred in the next values):

+ alpha0 = 0º since the axes are coincident.

+ alpha 1 = -90º being this the turn of Z2 from Z1

Continuing with this logic along the rest of the diagram:

+ alpha 2 = 0º
+ alpha 3 = -90º
+ alpha 4 = 90º
+ alpha 5 = -90º
+ alpha 6 = 0º

· a1, a2 & a3 correspond to the segments drawn in the diagram:
	
+ a1 is the distance along x axis from joint 1 to joint 2 indicated in the urdf file:
```
<joint name="joint_2" type="revolute">
<origin xyz="0.35 0 0.42" rpy="0 0 0"/>
```

**0.35**

+ a2 is the distance from joint 2 to joint 3 along the z axis. Since the urdf file measurements are defined in relation to the previous joint, it is very straightforward:
```
<joint name="joint_3" type="revolute">
<origin xyz="0 0 1.25" rpy="0 0 0"/>`
```

**1.25**

+ a3 is the distance along the z axis 
```
<joint name="joint_4" type="revolute">
<origin xyz="0.96 0 -0.054" rpy="0 0 0"/>`
```

**-0.054**

· d1, d4 & dg correspond to the segments drawn in the diagram:

+ d1 is the distance from joint 0 to joint 2 along the z axis, therefore we have to add the distance along z from joint 0 to 1 and from 1 to 2:
```
<joint name="joint_1" type="revolute">
<origin xyz="0 0 0.33" rpy="0 0 0"/>
```

&

```
<joint name="joint_2" type="revolute">
<origin xyz="0.35 0 0.42" rpy="0 0 0"/>
```

**Resulting in 0.75**

+ d4 is the distance from joint 3 to joint 5 along the x axis:
```
<joint name="joint_4" type="revolute">
<origin xyz="0.96 0 -0.054" rpy="0 0 0"/>
```
&

```
<joint name="joint_5" type="revolute">
<origin xyz="0.54 0 0" rpy="0 0 0"/>
```

**Resulting in 1.5**

+ dg is the distance from joint 5 to the end effector along the x axis:
```
<joint name="joint_6" type="revolute">
<origin xyz="0.193 0 0" rpy="0 0 0"/>
```

&


```<joint name="gripper_joint" type="fixed">
    <parent link="link_6"/>
    <child link="gripper_link"/>
    <origin xyz="0.11 0 0" rpy="0 0 0"/>
```

**Resulting in 0.303**

· theta 2 corresponds to a turn of -90º from x1 axis to x2 axis as shown in the diagram.

Having found these DH parameters, the individual transformation matrices would have the following structure:

 -| - | - | - 
--- | --- | --- | ---
cos(q)           | -sin(q)          | 0| a
sin(q)*cos(alpha)| cos(q)*cos(alpha)| -sin(alpha)| -sin(alpha)*d
sin(q)*sin(alpha)| cos(q)*sin(alpha)|  cos(alpha)|  cos(alpha)*d
0|0              | 0                |1

Then with this structure, I would substitute variables: q, alpha, d & a with the corresponding values for each matrix:

```
T0_1 = DH_T_Matrix(q1, alpha0, d1, a0).subs(s)
T1_2 = DH_T_Matrix(q2, alpha1, d2, a1).subs(s)
T2_3 = DH_T_Matrix(q3, alpha2, d3, a2).subs(s)
T3_4 = DH_T_Matrix(q4, alpha3, d4, a3).subs(s)
T4_5 = DH_T_Matrix(q5, alpha4, d5, a4).subs(s)
T5_6 = DH_T_Matrix(q6, alpha5, d6, a5).subs(s)
T6_G = DH_T_Matrix(q7, alpha6, d7, a6).subs(s)
```

being "s" the DH parameters dictionary

Then, in order to calculate the transform between base_link and gripper_link, I would calculate the product of all the previous matrices:

`T0_G = simplify(T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_G)`

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Theta 1 may be easily calculated as the angle between the plane win which the arm is moving and the x axis on the real world plane. Meaning that the equation for theta 1 would be something like the following:

`theta 1 = atan2(y, x)`

Regarding theta 2 and theta 3, the calculations would be carried out according to the following image:

![alt text][image2]

Where the y axis would correspond to the z axis in the "real world coordinates", being the resulting equations the following:

```
theta2 = pi/2 - angle_a - atan2(Wpos[2] - 0.75, sqrt(Wpos[0]*Wpos[0] + Wpos[1]*Wpos[1]) - 0.35)
theta3 = pi/2 - (angle_b + 0.036)
```

where Wpos is the position of the wrist center. Being 0,1 and 2, its coordinates for x,y,z respectively.

Regarding theta 4, 5 & 6, given they are all centerd on the wrist center in the same point, their values are the corresponding ones to the rotation of each plane, being the resulting equations the following from the extraction of Euler angles from the rotation matrix as indicated in the following calculations obtained from lesson 2.8 "Euler Angles from a Rotation Matrix":

Given the following example rotation matrix:

![alt text][image3]

the goal would be to find the angles alpha, beta, and gamma, given that numerical values for r​ij are known, leading to the following equations:

![alt text][image4]

![alt text][image5]

![alt text][image6]

In the specific case of the kuka arm, the rotation matrix would be:

```
[[-sin(p)*cos(q2 + q3) + sin(q2 + q3)*cos(p)*cos(q1 - y), sin(p)*sin(r)*sin(q2 + q3)*cos(q1 - y) + sin(r)*cos(p)*cos(q2 + q3) + sin(q1 - y)*sin(q2 + q3)*cos(r), sin(p)*sin(q2 + q3)*cos(r)*cos(q1 - y) - sin(r)*sin(q1 - y)*sin(q2 + q3) + cos(p)*cos(r)*cos(q2 + q3)],
[ sin(p)*sin(q2 + q3) + cos(p)*cos(q1 - y)*cos(q2 + q3), sin(p)*sin(r)*cos(q1 - y)*cos(q2 + q3) - sin(r)*sin(q2 + q3)*cos(p) + sin(q1 - y)*cos(r)*cos(q2 + q3), sin(p)*cos(r)*cos(q1 - y)*cos(q2 + q3) - sin(r)*sin(q1 - y)*cos(q2 + q3) - sin(q2 + q3)*cos(p)*cos(r)],
[                                   -sin(q1 - y)*cos(p),                                                       -sin(p)*sin(r)*sin(q1 - y) + cos(r)*cos(q1 - y),                                                       -sin(p)*sin(q1 - y)*cos(r) - sin(r)*cos(q1 - y)]]
```
(Multiplied by the correction matrix).

Solving the angles would result in the following equations:

```
theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```

Where R3_6 is the rotation matrix from joint 3 to joint 6 (previously indicated)

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

In the code, you may first find the calculation for the forward kinematics according to the DH parameters specified earlier in this document. Afterwards, I've carried out the calculations for the Inverse kinematics from the given end effector (wrist center) point from the simulator. With this position, the program calculates the corresponding angles for the robotic arm to reach the exact indicated position.

In order to improve the program runtime, I have directly introduced the resulting matrices for each step, instead of making the program carry out all matrices operations, since this took too much time (up to half a minute). In any case, the operations being carried out may be found commented right before the matrix in the code. Regarding the calculations for theta 2 and 3, they were carried out as specified before.

At first I had some issues with the angle at which the Kuka was leaving the cylinders, since it had a 90º deviation, meaning it always released the cylinders right next to the bin at a 90º movement in theta 5 until I realised that I had left out of the loop the rotation matrix from joint 0 to 3 (R0_3) and therefore was only initialized once per run and affecting the calculation of thetas 4 through 6 giving that 90º deviation.

After having solved this, the robot arm always releases the cylinders in the exact same spot at the middel of the bin. The only issue that could lead to some error is the angle of the grip at the moment of grasping the object, since it sometimes pushes it before grabbing it and then it doesn't pick it up. Also I had to increase the gripping time in order for the arm to grab the object in continuous mode instead of step by step.

After having said all this, the result was very satisfactory, although I have the feeling I might be able to improve processing speed in order for the calculations and movement to be carried out faster. The final results can be seen in the following images with 8 and 9 cylinders in the bin.

![alt text][image7]
![alt text][image8]

