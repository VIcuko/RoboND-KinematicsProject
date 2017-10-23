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
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

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

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | 0
1->2 | - pi/2 | 0.35 | 0 | q2 - pi/2
2->3 | 0 | 1.25 | 0 | 0
3->4 |  -pi/2 | -0.054 | 1.5 | 0
4->5 | pi/2 | 0 | 0 | 0
5->6 | -pi/2 | 0 | 0 | 0
6->EE | 0 | 0 | 0.303 | 0

Having found these DH parameters, the individual transformation matrices would have the following structure:

 |  |  |  
--- | --- | --- | --- | ---
cos(q)           | -sin(q)          | 0| a
sin(q)*cos(alpha)| cos(q)*cos(alpha)| -sin(alpha)| -sin(alpha)*d
sin(q)*sin(alpha)| cos(q)*sin(alpha)|  cos(alpha)|  cos(alpha)*d
0|0              | 0                |1

Then with this structure, I would substitute variables: q, alpha, d & a with the corresponding values for each matrix:

	T0_1 = DH_T_Matrix(q1, alpha0, d1, a0).subs(s)
    T1_2 = DH_T_Matrix(q2, alpha1, d2, a1).subs(s)
    T2_3 = DH_T_Matrix(q3, alpha2, d3, a2).subs(s)
    T3_4 = DH_T_Matrix(q4, alpha3, d4, a3).subs(s)
    T4_5 = DH_T_Matrix(q5, alpha4, d5, a4).subs(s)
    T5_6 = DH_T_Matrix(q6, alpha5, d6, a5).subs(s)
    T6_G = DH_T_Matrix(q7, alpha6, d7, a6).subs(s)

    being "s" the DH parameters dictionary

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


