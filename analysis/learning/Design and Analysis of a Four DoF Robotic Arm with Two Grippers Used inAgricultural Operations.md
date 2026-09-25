International Journal of Applied Mathematics, Electronics and Computers 11(2): 079-087, 2023 

e-ISSN: 2147-8228 



## INTERNATIONAL JOURNAL OF APPLIED MATHEMATICS ELECTRONICS AND COMPUTERS 

www.dergipark.org.tr/ijamec 

International Open Access Volume 11 Issue 02 June, 2023 

#### **_Research Article_** 

# **_Design and Analysis of a Four DoF Robotic Arm with Two Grippers Used in Agricultural Operations_** 

## **_Basheer Altawil_**<sup>**_a,_**</sup> **_* , Fatih Cemal Can_**<sup>**_b_**</sup> 



_aİzmir Katip Çelebi University,Robotics Engineering Department,35620 İzmir,Türkiye bİzmir Katip Çelebi University,Mechatronics Engineering Department,35620 İzmir,Türkiye_ 

|ARTICLE INFO|ABSTRACT|
|---|---|
|_Article history:_|Both academic and commercial interest in agricultural robots has increased recently. This is due|
|Received 12 December 2022<br>Accepted 7 April 2023<br>|to the fact that agricultural robots address significant issues such as seasonal labor shortages during<br>harvest and the rising concern over environmentally friendly practices. Because of these, several|
|_Keywords:_<br>ROS<br>IoT<br>Mechanism<br>Robot Manipulators<br>Agricultural Robots<br>Lagrangian mechanics|distinct agricultural robots have already been created for a variety of purposes, with varying<br>degrees of success, including monitoring, spraying, harvesting, transport, etc. As a result,<br>agriculture automation became unfeasible and unprofitable. The purpose of this study is to provide<br>a new methodology for multitasking in performing agriculture operations by designing a 4 Degrees<br>of freedom (4DoF) robotic arm with a new mechanism that has a different configuration with 2<br>grippers. We did kinematics and kinetics calculations using Denavit-Hardenberg (D-H) method<br>with Lagrangian mechanics, and with the help of Christoffel Symbols of the First Kind, also We<br>used Robot Operating System (ROS) to provide potential solutions using it. It is easy to be paired<br>with other open-source technologies, such as android or IoT technologies. The robot arm can work<br>synchronously with other hardware, sensors, cameras, and agricultural machines concerning<br>farming operations. In conclusion, we believe that this new configuration will open a door for<br>agricultural tasks to be easily automated and achieved using robotic technologies.|



This is an open access article under the CC BY-SA 4.0 license. (https://creativecommons.org/licenses/by-sa/4.0/) 

### **1. Introduction** 

Agriculture-related tasks are now very expensive, timeconsuming, and challenging to complete. Robot farmers are one of the cutting-edge technologies that will transform the agriculture industry and deal with its problems. Therefore, combining agricultural and technological instruments is now required to enhance the effectiveness and output of any agriculture system, making it easier for farmers to produce a large quantity of food that can satisfy market demands. As a result, by 2050, it is anticipated that the combined cost of agriculture and technology will be almost $240 billion[1]. 

Agri-robotics can adapt to changes in farm scales, crop varieties, soil moisture content, growth patterns, greenhouse chambers (GHC), glasshouses, vertical farms, and hydroponic closed and open systems. Moreover, robotic technologies are taking a huge part in these 

operations, especially, robotic arms that will be implemented on mobile robots, and it is becoming more difficult to manage all of these operations, and it's becoming more complex to cover all of the details with limited resources of robotics software and hardware[2]. As a result, using the Robotic Operating System (ROS) is preferred, as it can simplify all complex operations in one package. In recent years, research on agricultural robot arms has advanced quickly. Motion planning and control are vital to study areas for the collaborative operations of dual-arm robots [3]. 

The quick verification of novel prototypes, algorithms, and/or applications of motion in robotic sectors, as well as the optimization of system performance, have all been made possible by simulation approaches in the development of robotics. Without relying on a real complete hardware system, a robotic simulator can model the motion of robots and all objects in a virtual work 

> * Corresponding author. E-mail address: _basheeraltawil@gmail.com_ DOI: 10.18100/ijamec.1217072 

Basheer et al., International Journal of Applied Mathematics Electronics and Computers 11(02): 079-087, 2023 

envelope, saving time and money. This has lately led to an increase in interest in robotic simulation as it is a necessary case to design any robotic system, especially for agricultural operations purposes, see Figure 1 [4][5]. 

In our study, we used RVIZ, GAZEBO, and MOVEIT to mimic the motion of the robot arm concerning real environment task-performing. Path planning, real-time control, and all kinematic, dynamic solutions were all imitated using ROS. For path planning, the ROS framework was employed to avoid abrupt changes during fertilizing  and harvesting operations. Using the SOLIDWORKS CAD application, a 4-DoF manipulator with 2 distinct end effectors was developed to harvest and manipulate the objects. Then, we installed the Unified Robotics Description Format (URDF) as a CAD plugin using SOLIDWORKS. This plugin assists in converting CAD-designed robots into a format that ROS can read and show on RVIZ and Gazebo in integration with MOVEIT. 



**Figure 1.** Gazebo and RVIZ simulator 

With our 4 degrees of freedom (4-DoF), see Figure 2, Figure 3, which can be easily integrated with a mobile robotic platform, it will be easy and simple to be able to implement and achieve the tasks. This paper will cover the design of the robot arm parts and carry on every analysis required for the motion and control of a robotic arm. 



**Figure 2.** 4DoF Manipulator CAD Design 

As a result, kinematic analysis, and kinetic analysis are completed to prepare the robot arm for ROS platform control and simulation. Numerous CAD programs can be used to create the articulated robot arms so we used both 

SOLIDWORKS and Fusion360, 3D computer-aided design programs to prepare the parts that will be used later. 



**Figure 3.** 4-DoF Manipulator Assembly 

Analysis for robotics, particularly robotic manipulators, is extremely tough, and difficult. Therefore, many wellknown scientists are striving to minimize the problems and make them more adaptable for those who are interested in robotics science. We used the four Denavit–Hardenberg parameters (D-H parameters) in Figure 4, which are used for attaching reference frames to the links of a spatial kinematic chain, or robot manipulator. 



**Figure 4.** D-H Link Parameters 

This convention was established in 1955 by Jacques Denavit and Richard Hardenberg to standardize the coordinate frames for spatial links [6]. 



**Figure 5.** Robotic Arm Kinematic Labelling 

The relationship between the joints that work as connecting parts and the links that make up the kinematic chain is determined by the kinematic modeling and 

**- 80 -** 

Basheer et al., International Journal of Applied Mathematics Electronics and Computers 11(02): 079-087, 2023 

analysis of a robot manipulator [7] which basically depend on Denavit–Hardenberg parameters that we used in this paper. 

After deciding the D-H parameters, arm kinematic labeling, see Figure 5 is presented in this paper which will help us to obtain homogenous transformation matrices to carry out forward kinematic (FW), inverse kinematic (IK), and Jacobian calculations that will be used to select the joint actuators and control the position and linear velocity of both first and second end effectors of the robot manipulator. 

### **2. Kinematics and Kinetic Analysis** 

As previously noted, Denavit-Hardenberg was the method we employed for our analysis. This methodology comes in a variety of forms, which the scientist briefly discussed. The strategy we employed follows the DenavitHardenberg classical convention. As shown in Fig. 4, and Fig. 5, we follow this mythology to prepare DH parameters, and kinematic labeling respectively to obtain the DH table shown in Table 1 which will be necessary for the coming analysis. 

**Table 1.** Denavit-Hardenberg classical Convention Table 

|Link, i|ai|𝑎i|𝑑i|𝛩i|Joint rotation<br>limits (Degree)|
|---|---|---|---|---|---|
|1|a1|0|0|𝛩1|0-170|
|2|a2|−<sup>π</sup><br>2|0|𝛩2|0-170|
|3|a3|−<sup>π</sup><br>2|0|𝛩3|0-170|
|4|0|−<sup>π</sup><br>2|𝑑4|0|Fixed|
|4e1|0|0|𝑑4e1|𝛩4|0-170|
|4e2|a4e2|0|0|𝛩4|0-170|



The set of parameters that are mentioned in table I and Figure 4, and Figure 5, twist angle 𝒂i, offset distance ai, translation distance 𝒅i, and joint angle 𝜣i. For articulated mechanisms, some of these parameters are constant, and some of them are variable. 4e1, and 4e2 in our study, they are representing the end effector 1, and end effector 2 respectively. 

#### **_2.1. Homogeneous Transformation Matrices and Kinematic Analysis._** 

To reach the formulation presented in (2), we must first construct homogeneous transformation matrices (HTM) by vectorially multiplying the four matrices by going on every singular axis respectively as shown in (1) that we got from each parameter in Table 1. The matrices for each unique frame have been created depending on rotation and translation principles [8]. To determine the last point in which we are interested, we can multiply them individually. 







The parameters expressed in (2) are 𝐶(𝛩𝑖) , 𝑆(𝛩𝑖) , 𝑆(αi) , 𝐶(αi) which represents the abbreviation of cos(𝛩𝑖),sin(𝛩𝑖), sin(αi), and  cos(αi) respectively and they will be used with all formulations in all kinematic and dynamic analyses. From equations 1 and 2 we can relate all joints to the reference frame like in the following expressions. In (3), we obtain the transformation matrix that relates joint 1 to the base frame. 



In (4), we obtain the transformation matrix that relates joint 2 to the base frame. Moreover, the expression 𝛩12 is representing the algebraic summation of 𝛩1, 𝛩2. 



In (5), we obtain the transformation matrix that relates joint 3 to the base frame. We generate new parameters which are 𝛽, 𝛾, 𝛿, 𝜀, P3X, P3Y, P3Z, P4X, P4Y, and P4Z that take expressions into them to minimize the size of the matrix as it is shown in equations group 1. 



In (6), we obtain the transformation matrix that relates 

**- 81 -** 

Basheer et al., International Journal of Applied Mathematics Electronics and Computers 11(02): 079-087, 2023 

joint4 to the base frame and we substitute the new parameters generated in group 1 like in (5). 



In (7), we obtain the transformation matrix that relates end effector 1 to the base frame and we substitute the new parameters that we obtained in equation group 2, like in (5), and (6). Moreover, the expression 𝛩34 is representing the algebraic substruction of 𝛩3, 𝛩4. 

∀= 𝐶(𝛩12)𝐶(𝛩34) 



_Pe1x=_ 𝑎1𝐶(𝛩1) + a2𝐶(𝛩12) + 𝑎3𝛽−d4𝛾+ d4𝑒1𝑆(𝛩12) 

first gripper, and second gripper respectively. Therefore, X1, Y1, Z1, and X2, Y2, Z2 are representing the position of gripper one and gripper two on space concerning the base frame. 

**_2.1.1. Forward kinematics analysis for the first gripper_** 



**_2.1.2. Forward kinematics analysis for the second gripper_** 





_Pe1z=_ −𝑎3𝑆(𝛩3) −d4𝐶(𝛩3) 

_Pe2x=_ 𝑎1𝐶(𝛩1) + 𝑎2𝐶(𝛩12) + 𝑎3𝛽+ 𝑎4𝑒2𝛽− 𝑑4𝛾 _Pe2y=_ 𝑎1𝑆(𝛩1) + 𝑎2𝑆(𝛩12) + 𝑎3𝛿+ 𝑎4𝑒2𝛿− 𝑑4𝜀 



In this formulation, we have new extra parameters which are like it shown in equation group 2. 



In (8), we obtain the transformation matrix that relates end effector 2 to the base frame and we substitute the new parameters that we prepared in equations group 2. 



A manipulator's forward kinematics determines the end effector's position and orientation in cartesian space using the input joint angles. The aforementioned matrix expresses the end-effector position, as can be seen by glancing at it. As shown in Matrix 7 and 8, we expressed the first-end effector and the second-end effector onto the base frame respectively[8]. Therefore, we can get the rotation and translation of them so that the forward kinematics analysis will be like the following. Equations group3, and group4, show the forward kinematics for the 

#### **_2.2. Serial Manipulator Kinetic Analysis_** 

Robotics kinetic calculations assess and explain a robot's motion and external forces using mathematical models and equations. This may involve computations in the fields of dynamics and kinematics, which both deal with the forces acting on the robot and how they affect its mobility. The robot's movement can be controlled, its behavior predicted, and its stability and safety can be guaranteed using these calculations. Inverse kinematics, forward kinematics, and motion planning are a few methods frequently utilized in dynamic computations for robotics [9]. In robotics, dynamic computations can be carried out in a variety of ways, including Lagrangian mechanics, Newton-Euler methods, Kane's method, Featherstone's algorithm, and Hybrid Dynamics. In this article calculations to analyze the 4-DoF serial robot manipulator, we used Lagrangian mechanics in combination with Christoffel Symbols of the First Kind to simplify the dynamical calculations. 

Depending on the Lagrangian principle that aims to derive the governing general formulation of the dynamical equation of motion will be like it shown in (9). 



The first term is called the inertia forces, the second term is called the Coriolis and centrifugal force, and the third term is representing the gravitational effects forces Where  𝜏 , 𝜃̈  ,𝐽<sup>𝑇</sup> 𝐹𝑒 , 𝑓𝑟   are representing joint torque, joint acceleration, Jacobian transpose, the force exerted on the joint, and friction force respectively. (10) is show the first term of (9) which is representing the inertia matrix. 

**- 82 -** 

Basheer et al., International Journal of Applied Mathematics Electronics and Computers 11(02): 079-087, 2023 



By implementing the calculations shown in (10), we could end up with the 4X4 matrix for the first term of the general formulation of the dynamic equation, see (11). 



**Table 2.** Christoffel Symbols 𝑐𝑖𝑗𝑘 values Table 

|||Column-1|
|---|---|---|
|ROW1|C1R1|C111+C121+C131+C141|
|ROW2|C1R2|C112+C122+C132+C142|
|ROW3|C1R3|C113+C123+C133+C143|
|ROW4|C1R4|C114+C124+C134+C144|
|||Column-2|
|ROW1|C2R1|C211+C221+C231+C241|
|ROW2|C2R2|C212+C222+C232+C242|
|ROW3|C2R3|C213+C223+C233+C243|
|ROW4|C2R4|C214+C224+C234+C244|
|||Column-3|
|ROW1|C3R1|C311+C321+C331+C341|
|ROW2|C3R2|C312+C322+C332+C342|
|ROW3|C3R3|C313+C323+C333+C343|
|ROW4|C3R4|C314+C324+C234+C344|
|||Column-4|
|ROW1|C4R1|C411+C421+C431+C441|
|ROW2|C4R2|C412+C422+C432+C442|
|ROW3|C4R3|C413+C423+C433+C443|
|ROW4|C4R4|C414+C424+C434+C444|







<!-- Start of picture text -->
Figure 6.  First Joint Torque<br>Figure 7.  Second Joint Torque<br><!-- End of picture text -->





<!-- Start of picture text -->
Figure 8.  Third Joint Torque<br><!-- End of picture text -->





<!-- Start of picture text -->
Figure9.  Fourth Joint Torque<br><!-- End of picture text -->

In addition, in the second term of (9) which is representing Coriolis, Centrifugal forces will be calculated the **_Christoffel Symbols of the First Kind_ , see** (12), making the calculations simple and more accurate when we will use them in torque computation. Coriolis force. 



This kind of force is generated in a reference frame that is rotating relative to an inertial frame. Moreover, a "fictitious" force known as the centrifugal force affects things traveling in a circular motion in a frame of reference that is not inertial. 

𝑐𝑖𝑗𝑘′ 𝑠 calculations totally depend on 𝑖 , 𝑗 , 𝑘 factors. Therefore, the N elements of it calculated using the equation ( 𝒄𝒊𝒋𝒌)𝑵 **=** 𝟒<sup>𝟑</sup> **=64** elements, because, we have three factors which are 𝑖 , 𝑗 , 𝑘 , and we have 4 cases of each of them in total regarding robot arm joints. After using (12), we could generate **64** values for 𝒄𝒊𝒋𝒌 , then organize them as shown in Table 2. After implementing the principles shown in Table 2, we could obtain the final 4X4 Coriolis, Centrifugal forces Matrix, see (13) that will be used in torque computation. 

**- 83 -** 

Basheer et al., International Journal of Applied Mathematics Electronics and Computers 11(02): 079-087, 2023 



Also, we re-formalize the general mechanical equation to be suitable for final torque computation, see (14). This equation is the governing equation for torque computation, we sum two 4X4  matrices which are inertia Mmatrix, and Vmatrix which represent the inertia matrix, and Coriolis, Centrifugal forces Matrix respectively. The final result of torque will be the robotic arm links length, and masses in terms of 𝜃 , 𝜃̇ , 𝑎𝑛𝑑  𝜃̈ , and the joint torques will be drawn regarding these variances concerning time-variant. 



𝑘= 1 ,2, … , 𝑛  and 𝑛 is the number of robot arm joints. On the right-hand side, is the final torque of the joints. Finally, we used **Mathematica** platform to compute these calculations and draw the final torques of joint1, joint2, joint3, and joint4 as shown in Figure 6, Figure 7, Figure 8, and Figure 9, respectively. All codes can be found in our Githb repository link under the name of **<u>4-DoF Robotic Arm Analysis and Control.</u>** 

### **3. Manipulator Design and Implementation** 

This robot is an articulated manipulator with four degrees of freedom, as we mentioned in earlier sections, and revolute joints at every joint. Because the mechanism was made for testing purposes, the mechanical design was chosen based on the requirements that the mechanism may be used on an experiment table. Additionally, it was found that the restricted rotation angle for all joints depends on a range from 0 to 170[24]. Figure 3 shows the assembled side of the robot manipulator after 3D printing, whereas Figure 2 shows the CAD drawing of the developed mechanism designed using SOLIDWORKS. 

Manipulator parts are made to be appropriate for prototyping and printing after being converted from CAD to extension STL[10]. The components are made to look like genuine robot arms to users so that we may address aesthetics, industrial design, and formality, just like an established robotics company. Thanks to the SW2URDF plugin for making it simple to convert the design to ROS and begin controlling it. 

As shown in Figurec2 from the CAD design illustrates how there are two end effectors, one of which is a peristaltic pump that was purchased on the market and will be in charge of fertilizing the plant. The other device is a gripper with a micro servo motor that will be in charge of harvesting and gripping. The LX-16A servo motor from the HI-WONDER firm is used to power the robot arm's joints. The market has made the controllers available that use the bus servo controlling protocol. We created our connections to retain the servo motor and the links of the 

manipulator due to the diverse configurations of the robot's mechanism. 

We realized that the actuators attached to the first and third joints, respectively, are bearing large torque because they were chosen based on the joint torque that we determined during the dynamic study of the robot arm. The first joint is responsible for hauling all of the robot's components, and the third joint rotates around the X-axis while defying gravity to account for gravity's influence on the situation. However, as the first and second joints move perpendicular to gravity as part of the SCARA mechanism, we did not include the gravity factor[7]. 

### **4. Controlling and Software** 

The robot arm will move following the requirements of the plant and the plant's readiness based on the feedback we will gather from the system we are using. Because of this, the entire governing algorithm will operate as indicated in Figure 10. The inputs for the system's master can come from a variety of sources. The expert who visits the farm daily, weekly, or even monthly can make use of these resources. Additionally, it can be obtained through the sensors that were mounted on the system during installation. 



<!-- Start of picture text -->
i<br>a<br>a<br><!-- End of picture text -->

**Figure 10.** The Entire System Controlling Algorithm 

These sensors may be vision sensors, such as cameras, or they may be digital, such as nutrient solution sensors or soil analyzer sensors. After receiving the inputs, we may examine them in the database we set up on the backend of our software frame and provide the instructions for the robot to follow to complete the required work [11]. 

#### **_4.1. Preparing ROS Environment and its operating system._** 

A robotic operating system (ROS) is a versatile and open-source software framework and we have been using it in this study. It is A collection of software libraries and development tools that are used to create robotic devices and programs. We used Melodic Moriena on Ubuntu 18.04 to create ROS packages for our robot arm. It only works with Ubuntu 18.04 Bionic Beaver when using ROS Melodic which we used in this project, and it is operating with the Linux operating system [12]. 

The robot arm kinematic Model has to be equivalent to 

**- 84 -** 

Basheer et al., International Journal of Applied Mathematics Electronics and Computers 11(02): 079-087, 2023 

the real robot. Therefore, the robot model in ROS contains crucial packages and some important nodes that aid in creating the 3D robot models in the virtual frame. These packages employ the Unified Robot Description Format (URDF). So, the manipulator model is described in the URDF, which is an XML specification. To perform robot work, we have to prepare the inertia matrix, collision detection matrix, robot joints, and arms visualization, the transmission of joints actuators, reduction rate, actuators that will be used, gazebo plugin, sensor plugin, and required controllers that we may use during task implementation. 



<!-- Start of picture text -->
a et pacman ss 7 ream 7 ewcnd 9 rane 5 — =<br>——<br><!-- End of picture text -->

**Figure 11.** RVIZ with its joint publisher GUI 

Following the preparation of the packages we previously discussed, as shown in Figure 11, we could launch the files using a Linux terminal and visualize our robotic arm to interact with it using RVIZ, GAZEBO, and MOVEIT interfaces. There are two packages to deal with to interact with the robot arm which are joint_state_publisher and robot_state_publisher. These packages are responsible for transforming the internal case of the joint of the robot arm so that we can move, stop, and visualize the status of every singular joint by helping these two packages. 



<!-- Start of picture text -->
one<br><!-- End of picture text -->

**Figure 12.** Path Planning with MOVEIT 

In addition, we use **_rosserial_** protocol to enable the serial connection of the servo system of our robot arm for LX16A servo motor and the extra hardware elements such as Arduino microcontroller to the ROS system. 



<!-- Start of picture text -->
RoboticOperatingSystem (ROS) — Control Hardware Structure<br>= = a | E<br>100 > aa<br><!-- End of picture text -->

**Figure 13.** Bus Servo connecting Arduino with LX-16 

Through the use of serial libraries, the type of activity (Subscriber/ Publisher or Service) and a message format are directly established on the device. Additionally, a Python node on the host computer relays messages and makes ROS aware of any nodes generated on the device. Devices can have any functionality as long as they communicate utilizing the serial connection and the **_rosserial_** format[13], see Figure.13. 

#### **_4.3. Controlling the inputs coming from the agricultural system._** 

#### **_4.2. Preparing the controlling packages and their algorithms._** 

As we have already indicated, there are already created packages available for editing, modifying, and improving. MOVEIT, which enables us to operate robotic arms and create the desired trajectories, is one of the most intriguing packages. The procedure that MOVEIT work depends on robot kinematics and dynamics KDL, and inverse kinematics of the robot arm so that by bringing the end effector of the robot arm to the desired position in different places, we can implement and repeat that action as much as we can, see Figure.12. 

As we mentioned before, we will get some data from some sensors, experts, and/or cameras. These data will help us to decide where the robot arm should go during its work. Therefore, after collecting all these data, the robot should know which plant needs to be getting some operations such as harvesting and/or pesticide spraying as we mentioned before. 

**- 85 -** 

Basheer et al., International Journal of Applied Mathematics Electronics and Computers 11(02): 079-087, 2023 



<!-- Start of picture text -->
:| [eon]<br>gy — —momaisn | ° Gee<br>=<br>Ocecte =. |i ener an<br>sotooe : 7<br><!-- End of picture text -->

**Figure 14.** The internal control algorithm of the robot arm 

However, firstly, we count the places where the robot should go and prioritize those places one by one depending on the readiness of the plant, the quantity of the plant that needs to be harvested, and the need for the plant that will be sprayed. After considering all the previous parameters, we can decide the exact location of every singular plant, and the MOVEIT group will give the command to the robot controller that will help the robot arm hardware and physical interface to move to the target to implement its task, see Figure 14. 

### **5. Results and Discussion** 

In this study, we created an articulated robot arm with new configurations that will be better suited for use in agriculture and the execution of its tasks. We perform the mathematical calculations required for the modeling and control of the robot arm. Additionally, we created, produced, and modified the robot's components so that they could be used with ROS APPs.Additionally, we developed a system control method that will aid researchers and developers who wish to carry out further research in this area. By integrating an arm into a mobile robot, the robot will be able to gather information from the agricultural field and determine where it should move. We could not build the complete agricultural system because of the enormous expense needed, so we built just the robot arm using 3D printing and simulated it using ROS interfaces. 

We used mathematical modeling that primarily relies on Denavit-Hardenberg principles, which enabled us to create a new labeling table from which we were able to derive the homogenous transformation matrices for the model we created. The serial robotics arms supported by this new model configuration will have fresh images that will aid researchers and developers in finding solutions to global problems. 

For mechanical calculations, as can be seen from joint torque graphs Figure 6 through Figure 9, our mechanism's joints' torque varies from joint to joint, necessitating the employment of various motors with various torques. However, we employed the **_LX-16A_** servo motor in our investigation, which has a 15 kg.cm maximum torque. This caused joint 1 and joint 3 of the manipulators to have various deficiencies and vibrations during experimental work, which can help us choose new actuators. However, 

because of the restricted funds available for the project, it was challenging to choose new, more expensive actuators when ROS would have been sufficient to complete and run the simulation. 

### **6. Conclusion** 

Over the past ten years, there has been a substantial increase in research efforts aimed at creating agricultural robots that can efficiently complete laborious field jobs. A commercial level of robotics has not been attained for agricultural applications and researchers, developers, and robotics companies are intensively trying to commercialize robotics systems to become more helpful to users. Making an agriculture machine that can be integrated with ROS to be ready to be included in the agricultural system is difficult today, so the tasks where we must succeed as developers include developing its description model, ROS sensor drivers, and ROS controller interfaces with the required controllers. 

That’s why we design a new simple model with a new controlling algorithm that can enable developers to create and put into practice useful agriculture automation, which is another step toward increasing agriculture productivity and efficiency and enhancing food security for lighting future generations' dreams. 

#### **Acknowledgment** 

This work is supported by the Department of Robotics Engineering, Graduate School of Natural and Applied Sciences, İzmir Katip Çelebi University. 

### **Author's Note** 

Previous version of this paper was presented at 10th International Conference on Advanced Technologies (ICAT'22), 25-27 November 2022, Van, Turkey. 

### **References** 

[1] Mitra, Manu. Robotic farmers in agriculture. Advances in Robotics & Mechanical Engineering.2019; pp. 91-93,doi: 10.32474/ARME.2019.01.000125. 

[2] Concepcion II, Ronnie S.,and et al. ‘Denavit-Hartenberg-based Analytic Kinematics and Modeling of 6R Degrees of Freedom Robotic Arm for Smart Farming. Journal of Computational Innovations and Engineering Applications . 5.2 .2021 ; pp.1-7. 

[3] R Shamshiri, Redmond, Cornelia Weltzien, Ibrahim A. Hameed, Ian J Yule, Tony E Grift, Siva K. Balasundram, Lenka Pitonakova, Desa Ahmad, and Girish Chowdhary. Research and development in agricultural robotics: A perspective of digital farming.2018. doi: 10.25165/j.ijabe.20181104.4278. 

[4] Deng, H., Xiong, J., and Xia, Z. Mobile manipulation task simulation using ROS with MoveIt.IEEE International Conference on Real-time Computing and Robotics _(RCAR)_ IEEE.2017, ;pp. 612-616. doi: 10.1109/rcar.2017.8311930. 

[5] Cruz Ulloa, C., Krus, A., Barrientos, A., Del Cerro, J., and  Valero, C. Robotic fertilisation using localisation systems based on point clouds in strip-cropping fields. Agronomy. 11.1.2020 ; pp. 11. doi:10.3390/agronomy11010011. 

[6] Jha, Aparna, Manoj Soni, and Mohd Suhaib. _Simulation and kinematic analysis of  KUKA KR5 Arc robot_ . Vol. 1149. No. 1 . IOP Conference Series: Materials Science and Engineering.. IOP Publishing ; 2021. 

**- 86 -** 

Basheer et al., International Journal of Applied Mathematics Electronics and Computers 11(02): 079-087, 2023 

[7] A. Candemir, F. C. Can, and M. Engineering, “Pick & Place Task Implementation of a Scara Manipulator via Robot Operating System and Machine Vision,” vol. 7, no. 6, pp. 420–430, 2022. 

[8] M. Brandstötter, A. Angerer, and M. Hofbaur, “An Analytical Solution of the Inverse Kinematics Problem of Industrial Serial Manipulators with an Ortho-parallel Basis and a Spherical Wrist,” 2014. [Online]. Available: <u>https://www.researchgate.net/publication/264212870.</u> [9] Tsai, Lung-Wen _. Robot analysis : the mechanics of serial and parallel manipulators ,_ John Wiley and Sons _;_ 1999 _._ [10] Poon, R. J. M. _Design and Control of a Mounted Robotic Arm Tool Changer and Measurement Tools for Agriculture_ (Doctoral thessis), Massachusetts Institute of Technology). 2021; pp. 122. https://hdl.handle.net/1721.1/139075. 

[11]  Ulloa, C. C., Krus, A., Barrientos, A., Del Cerro, J., and Valero, C. Trend Technologies for Robotic Fertilization Process in Row Crops. _Frontiers in Robotics and AI_ . 2022; pp. _9_ . <u>https://doi.org/10.3389/frobt.2022.808484.</u> [12] J. M. O’Kane, A gentle introduction to ROS. Jason M. O’Kane, 2014. 

[13] Zubrycki, Igor, and Grzegorz Granosik. Introducing modern robotics with ros and Arduino, including case studies. Journal of Automation Mobile Robotics and Intelligent Systems 8.1 . 2014 ; pp. 69-75. doi:10.14313/jamris_1-2014/9. 

**- 87 -** 

