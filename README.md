#ME495 Mini-Project: Sensable Phantom Omni and KDL

##Installing Phantom Omni Software and Drivers

To install the Phantom Omni Software on you computer follow these detailed [instructions](http://robotics.mech.northwestern.edu/~jarvis/omni-install.md).

Exceptions that might occur:

1.After creating the udev rules for the Phantom Omni you will need to open the *50-phantom-firewire.rules* file with *sudo* permissions to be able to edit and save it. The same applies for */etc/ld.so.conf.d* which is created at a later stage. In example:

```python
sudo gedit /lib/udev/rules.d/50-phantom-firewire.rules
```

2.After installtion, running 

```python
sudo dpkg -i openhaptics-ae_3.0-2_amd64.deb
```

from the *OpenHaptics_Linux_v3_0/OpenHaptics-AE 3.0/64-bit* folder, might not show the files *libPHANToMIO.so* or *libPHANToMIO.so.4.3* as described in the instructions. In that case no duplicates were created, therefore one can skip the step of removing them from */usr/lib64*. Your Omni Drivers will still work fine.

##Installing required ROS packages to interact with the Omni

You will need the following packages:

1. [phantom_omni](https://github.com/danepowell/phantom_omni): Provides a node i.e. with rviz visualisation, joint state publishing, force manipulation and reporting button events.


2. [omni_description](https://github.com/danepowell/omni_description):Contains the urdf model for the Sensable Phantom Omni

You can install these packages by cloning them into a correctly set-up [catkin workspace][1] from their respective github repositories. In the *catkin_ws/src* directory run:
[1]:http://wiki.ros.org/catkin/Tutorials/create_a_workspace

```python
git clone https://github.com/danepowell/phantom_omni
git clone https://github.com/danepowell/omni_description
```
Build your catkin workspace:
```python
cd ~/catkin_ws
catkin_make
```

##Exploring phantom_omni and omni_description

From a terminal launch the phantom_omni package:

```python
roslaunch phantom_omni omni.launch
```
Rviz should startup and a visualization of the omni will appear. Play around with the Omni!

In a new terminal explore the ROS topics:

```python
rostopic list
```
In this project we will primarily be using the */omni1_joint_states* and */omni1_force_feedback*.

Get more information about a topic using*rostopic info /TOPICNAME* i.e:

```python
rostopic info /omni1_joint_states
```
The node */omni1* is publishing to */omni1_joint_states* and *omni1_robot_state_publisher* is subscribing to it. The message data type is *sensor_msgs/JointState*.

To further see the description of the message type *sensor_msgs/JointState* run:

```python
rosmsg show sensor_msgs/JointState
```

In this project we will be using the *position* component of *JointState* which can store the values for all the joint angles of the Omni.  

If intersted in the data that is being published to a certain topic from the terminal, make use of: *rostopic echo /TOPICNAME* i.e:

```python
rostopic echo /omni1_joint_states
```

Runnning the command *view_frames* listens to the /tf frames that are being broadcast and creates a *frames.pdf* file, which contains a tree of how the frames are conected.



##Creating additional packages

This section covers creating a new package. The **nodes** developed in this is package are **capable of**: 

1.**Using tf to lookup the transform** from the frame located at the root of the Omni’s URDF to the frame at the end of the URDF

2.Using KDL to compute the **forward**, as well as **inverse** kinematics of the Omni

3.Using *transformations.py* to get the transforms from KDL and tf expressed as elements of SE(3)

4.Restricting the Phantom Omni to an axis floating in space

**<span style="color:red">NOTE: If you do not want to manually recreate the demo package of this project, then clone [me495_phantom_omni](https://github.com/sherifm/me495_phantom_omni) and build your catkin workspace analogous to the previous packages!!!</span>** (Skip to 'Exploring the ME495 Mini-Project)

###Create a package with the required dependencies

Following this [tutorial](http://wiki.ros.org/ROS/Tutorials/CreatingPackage), create a package in the catkin source directory:

```python
cd ~/catkin_ws/src
catkin_create_pkg me495_phantom_omni omni_description phantom omni roscpp rospy tf
```

For our purposes you will depend on the following packages that need to be installed:

```python
sudo apt-get install ros-indigo-robot-model
sudo apt-get install ros-indigo-urdfdom
sudo apt-get install ros-indigo-urdfdom-py

``` 

The above packages are debian packages that could be installed via *apt-get*. However, the [hrl-kdl](https://github.com/gt-ros-pkg/hrl-kdl) ROS package will also be depended on:

```python
cd ~/catkin_ws/src
git clone https://github.com/gt-ros-pkg/hrl-kdl
```
###Writing the package node

Create a python script for your nodes:

```python
gedit ~/catkin_ws/src/me495_phantom_omni/omni_mini_proj.py
gedit ~/catkin_ws/src/me495_phantom_omni/force_controller.py
```

Save them and make them executable

```python
sudo chmod +x ~/catkin_ws/src/me495_phantom_omni/omni_mini_proj.py
sudo chmod +x ~/catkin_ws/src/me495_phantom_omni/force_controller.py
```

Paste the code from this [repository](https://github.com/sherifm/me495_phantom_omni/tree/master/src) into the respective python scripts. 

Create a launchfile named *omni.launch* in *me495_phantom_omni/src* (same directory as the nodes) and paste [this](https://github.com/sherifm/me495_phantom_omni/blob/master/src/me495_omni.launch) code into it.

Build your workspace with  
```python
catkin_make
```
You have now emulated cloning the repository and may move onto exploring the demo.

The package that has been cloned from github or recreated manually provides an elaborately documented code that implement the goals of this mini-project.

##Exploring the ME495 Mini-Project / me495_phantom_omni package

This project uses elementary P-conrollers to calculate constraint forces. The control optimization is not the focus of this project. 

**<span style="color:red">Caution: The Phantom's arm may go into resonance! GRIP THE STYLUS FIRMLY BEFORE STARTING THE LAUNCH FILE. </span>**

Launch the me495 mini-project demo by starting the launchfile. Grip the stylus firmly as it will jump to its constrained axis. The forces are set to be manageable. If it goes in resonance, simply grab the arm manually and stabilize it. 

```python
roslaunch me495_phantom_omni me495_omni.launch
```
As the nodes launch, rviz should begin to simulate the robot arm (this is a function of the phantom_omni package). The me495_phantom_omni package is publishing the desired constraint forces. It's also printing interesting feedback data in two separate terminals that should look like this:

[INSERT IMAGE]

The first matrix - an element of the SE(3) space - was calculated using the [tf funcitons](http://wiki.ros.org/tf) package. It calculates the transformation from the fixed frame located at the base of the urdf model /base to the body frame located near the tip of the stylus. 

(Tip: use *view_frames* again to view the frames! Upon launching the project, the launchfile is publishing an additional fixed frame floating in mid-air, which is utilized for simplified constraint geometry calculations):

```python
<node pkg="tf" type="static_transform_publisher" name="floating_frame_broadcaster"
	args="0.28 0 0.1 0 0 0  /base /floating 100" />
```

The second matrix being displayed in the large terminal is also an element of SE(3) that shows the same transformation as the previous one. It is obtained by parsing the [urdf](http://wiki.ros.org/urdf) date to a [kdl](http://wiki.ros.org/kdl/Tutorials) chain that captures the Omni's kinematics, subscribing to the Omni's joint states and finally computing the forward kinematics.

Using *tf.transformtaions.is_same_transform (matrix1,matrix2)* the two matrices are checked for equality.

To test the inverse kinematics, the joint angles *q_sensors* were offset offset by a constant to emulate an initial guess. Using the position of the stylus, the joint angles were obtained with the KDL inverse kinematics tools. 

The bottom-most vector shows the difference *delta_q* between the sensor joint angles *q_sensor* and the ones obtained through inverse kinematics. 

In a separate smaller window the constraint forces acting on the stylus are being displayed

 




