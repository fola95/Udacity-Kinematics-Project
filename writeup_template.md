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

[image1]: ./misc_images/misc1.png
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

Below is the DH Parameter table and its derivation

[DHImage]: ./udacityDHderivation-FolawiyoCampbell-1.png
![alt text][DHImage]


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.
We need to create individual transforms before performing homogeneous transformation.
Below we have created individual transforms for joint i-1 to i:
```python
	def TF_Matrix(alpha, a, d, q):
		TF = Matrix([[cos(q), -sin(q), 0, a],
		[sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d], 
		[sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha), cos(alpha)*d], 
		[0, 0, 0, 1]])
		return TF
	
	# Create individual transformation matrices
	T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
	T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
	T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
	T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
	T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
	T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
	T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)
```
With this individual transforms we can now find the transformation from the base link to end effector

```
T0_EE = T0_1* T1_2* T2_3* T3_4*T4_5* T5_6*T6_EE
```


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.
Kuka arm falls into the spehrical wrist special case where joint 5 is the common intersection point hence our wrist center (Joint 4, 5 and  joint 6 are revolute joints)  as a result we can decouple this problem into position and orientation kinematics.

The below steps were taken:

Step 1: Create DH Parameter table (see above)

Step 2: Calculate the wrist center relative to the base frame. 

When we examine the parameter diagram we can see it is a simple translation along the z axis and can be give as below.
```python
    #px, py, pz are the enf effector positions
    EE = Matrix([[px],[py], [pz]])
    WC = EE - (0.303) * ROT_EE[:,2]
```

where ROT-EE is the rotation matrix from the x to the y to the z extrinsic

```python

		ROT_EE = ROT_z * ROT_y * ROT_x
		ROT_Error = ROT_z.subs(y, pi)* ROT_y.subs(p, -pi/2)
		ROT_EE = ROT_EE * ROT_Error
		ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y':yaw})
```

![alt text][image2]

Step 3: 

Now that we have the wrist center theta1 can be easily gotten as below:
```python
		theta1 = atan2(WC[1],WC[0])
```
Using the above image we can now readily get theta2 and theta3 with the help of cosine laws

```python

		side_a =1.501
		side_c = 1.25

		side1 = WC[2]-0.75
		side_r = sqrt((WC[0] * WC[0]) + (WC[1] * WC[1])) - 0.35 #a1

		side_b = sqrt(pow(side1,2)+pow(side_r,2))

		angle_a = acos(((side_c*side_c) + (side_b*side_b) - (side_a*side_a))/(2*side_c*side_b))
		angle_b = acos(((side_c*side_c) + (side_a*side_a) - (side_b*side_b))/(2*side_c*side_a))
		angle_c = acos(((side_b*side_b) + (side_a*side_a) - (side_c*side_c))/(2*side_b*side_a))
		
		theta2 = pi/2 - angle_a - atan2(side1,side_r)
		theta3 = pi/2 -(angle_b+0.036)
```
Step 4:

calculate the homogenous transform of joint 1 to 3 this result will then be used to get homogenous transform from joint 3 to 6 whose euler angles can be used to find the remaining theta

```python
		R0_3 = T0_1[0:3, 0:3]* T1_2[0:3, 0:3]* T2_3[0:3, 0:3]
		R0_3 = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})
		R3_6 = R0_3.T * ROT_EE
    
```

Step 5:

We can find theta4, theta5, theat6 using euler angles

```python
    theta4 = atan2(-R3_6[2,2], R3_6[0,2])
    theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
    theta6 = atan2(R3_6[1,1], -R3_6[1,0])
```

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 
This code follows the steps discussed in the kinematic analysis. 
[code can be found here](https://github.com/fola95/Udacity-Kinematics-Project/blob/master/kuka_arm/scripts/IK_server.py)
![alt text][image3]


