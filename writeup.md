## Udacity Robotics Nano Degree Program Project: Kinematics Pick & Place
### KUKA KR210 Inverse Kinematics and ROS
---
**Steps to complete the project:**  
1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code.

[//]: # (Image References)

[image1]: ./misc_images/URDF.png
[image2]: ./misc_images/URDFdefinition.jpeg
[image3]: ./misc_images/DHparameterTable.jpeg
[image4]: ./misc_images/WristCenterCalculation.jpeg
[image5]: ./misc_images/q2q3.jpeg
[image6]: ./misc_images/FK.png
[image7]: ./misc_images/HT.png
[image8]: ./misc_images/GHT.png
[image9]: ./misc_images/GHThandwritten.jpeg
[image10]: ./misc_images/IPK2.png
[image11]: ./misc_images/IPK1.png
[image12]: ./misc_images/q4q5q6.jpeg
[image13]: ./misc_images/q4q5q6IOK.png
[image14]: ./misc_images/1.png
[image15]: ./misc_images/2.png
[image16]: ./misc_images/3.png
[image17]: ./misc_images/4.png
[image18]: ./misc_images/5.png
[image19]: ./misc_images/HTequ.png


## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it.

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Below is a image from forward_kinematics demo. KUKA KR210 is shown in the Rviz 3D view window. Joint_state_publisher in the right lower corner shows all the joints are in the zero center positions. X, Y, and Z axes are represented by red, green, and blue respectively.  

* Rviz URDF Visualization  

    ![alt text][image1]

From kr210.urdf.zacro file, each joint's type, positions, and orientations were obtained and listed in the table shown in the below image. Black dots are the joint origins. The origin of child link is defined in the parent link reference frame and listed in <x,y,z> column.

* URDF File Definition  

    ![alt text][image2]

Denavit-Hartenberg (DH) parameters for KUKA KR210 were defined using the steps below. The DH parameter convention used in this project was introduced in John J Craig's book in 2005.

1. Label Joints from 1 to n  
    - Joints labels were defined in URDF file. KUKA KR210 has 6DOFs, so n = 6
2. Define Joint axes  
    - Joint axes were defined in URDF file.    
3. Label Links from 0 to n  
    - Links were labeled and defined in URDF file. Link 0 is always a fixed base link.
4. Define Z axes directions
    - Z axes directions were obtained by running forward_kinematics demo.Examine which direction the KUKA arm joint rotates when joints_state_publisher is set to nonzero value, and determine positive Z axes directions based on Right-hand rule.  
5. Define X axes that are common normal to both Z(i-1) and Zi  
6. Obtain DH parameters alpha, a, d, and theta
    - alpha: the angle between Z(i-1) and Z about X(i-1) using Right-hand rule.
    -     a: the offset from Z(i-1) to Zi along X(i-1)
    -     d: the offset from X(i-1) to Xi along Zi
    - theta: the angle between X(i-1) to Xi about Zi using Right-hand rule.


* DH Parameters and DH Parameter Table

    ![alt text][image3]

Obtained DH parameters are listed in the table below.

i | alpha(i-1) | a(i-1) | di | thetai
--- | --- | --- | --- | ---
1 | 0     | 0     | .75  |
2 | -pi/2 | .35   | 0    | theta2-pi/2
3 | 0     | 1.25  | 0    |
4 | -pi/2 | -.054 | 1.5  |
5 | pi/2  | 0     | 0    |
6 | -pi/2 | 0     | 0    |
7 | 0     | 0     | .303 | 0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Homogeneous transformation matrix for John J Craig's convention DH parameters is given as:

![alt text][image19]

The above generalized homogeneous trnasformation matrix is defined as a function "homogeneous_transform" in the code, and individual transformation matrix at all joints 1, 2, 3, 4, 5, 6, and 7 (= gripper joint) were calculated.

![alt text][image6]  

The below is the simplified individual transformation matrix with DH parameters.  Note that q2 is defined as theta2-pi/2.

* Individual Transformation Matrices

![alt text][image7]

Given the end effector position <px,py,pz> and orientation <roll,pitch,yaw>, a homogeneous transformation matrix from base_link to end effector link (T0_EE) can be derived as below.

* Generalized Homogeneous Transformation Matrix from base_link to end-effector link

![alt text][image9]

![alt text][image8]

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

NOTE: Kinematic Decoupling
KUKA KR210 has six-DOF with a three-revolute-joint spherical wrist.
These three serial joint axes of spherical wrist intersect at a common point, called wrist center.
This configuration decouples inverse kinematic problem into inverse orientation kinematics and inverse position kinematics.
Given wrist orientation, three-revolute-joint angles can be determined.
Given wrist center position, the other three arm joints can be derived.

**Inverse Position Kinematics Calculation**

The path of the end effector is given, so the end effector position <px,py,pz> and orientation <roll,pitch,yaw>. These positions and orientations are defined in the base_link reference frame (the same as URDF reference frame) which is different from the end effector reference frame when DH parameters were defined. To distinguish them, let rotation matrix from URDF reference frame to the given end effector orientation as R0_EE and let rotation matrix from URDF reference frame to the end effector defined in DH parameter process as R0_GF (GF stands for gripper finger). Then, the rotation matrix, RGF_EE, can be calculated as:

RGF_EE = R0_GF.inv() x R0_EE  

The only difference between link_6 and gripper_finger_link reference frame is the translation in z-axis by d7 and l, where d7 is the distance between link_6 and gripper_link X-axes along the common Z axis and l is the distance between gripper_link and gripper_finger_link X-axes along the common Z axis.

Since the wrist center position <wx,wy,wz> is coincident with link_6 reference frame origin, wrist center can be derived by the equation:

w = p - RGF_EE x [[0],[0],[(d7+l)]]


* Inverse Position Kinematics to Calculate Wrist Center  

![alt text][image4]

![alt text][image10]

Once the position of wrist center is known, the orientations of the first three joints, q1, q2, and q3 can be defined using geometry. The figure below explains how the equations were derived.

* Inverse Position Kinematics to Calculate q1, q2, and q3  

![alt text][image5]

![alt text][image11]

**Inverse Orientation Kinematics Calculation**

The last step is to find orientations of the last three joints, q4, q5, and q6. The rotation matrix from URDF reference frame to the link_6 is equal to RGF_EE, which is already derived in the previous steps. Since q1, q2, and q3 are known, R3_6 can be derived as:

R3_6 = R0_3.inv() x RGF_EE

This is a 3x3 matrix with constant numbers.
Using forward kinematics, R3_6 can be obtained from T3_6 with DH parameters and q4, q5, and q6 as variables.

Equate R3_6 derived from IK and R3_6 derived from FK, three system of equations are obtained and solve for q4, q5, and q6.

The final equations are shown in the figure below.

![alt text][image12]

![alt text][image13]


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.

IK_server.py inverse kinematics calculation steps are explained below.

1. Define DH parameters   
![alt text][image14]  
 [* The variable q2 is defined as (theta2-pi/2).]

2. Obtain end-effector position and orientation  
![alt text][image15]  
[* def tf.transformations.euler_from_quaternion([quaternion, axes=sxyz]) == intrinsic rotation x-y-z]

    Step 3-5 Calculate joint angles q1-q6 using Geometric IK method

3. Calculate Rrpy rotation matric given roll, pitch, and yaw And Obtain wrist center position
![alt text][image16]

4. Calculate q1, q2, and q3
![alt text][image17]

5. Calculate q4, q5, and q6  
![alt text][image18]

**Results**  
KUKA KR210 controlled by IK_server.py successfully completed pick and place cycle 10 out of 10. The video is available in the folder named "video."

In the video, KUKA KR210 Arm follows the planned path and joint orientation and executes pick and place cycle. In the simulation environment, the error in the position and orientation of end effector can be small. In the real physical environment, backlash and link deformation cumulatively cause errors at end effector.

**Further Improvements**  
The inverse kinematics calculation performed in the IK_server.py uses acos() and asin() functions whose outputs are limited within 0 to pi and -pi/2 to pi/2, respectively. However, mechanical operating range for KUKA KR210 arm is more than what is defined by these numerical limitations. These can be improved by adding conditional arrangements to the output.  
