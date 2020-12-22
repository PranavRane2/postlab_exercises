# Post Lab Exercises

## Exercises

### Exercise 1

**AIM**: Publish and Subscribe your Name using the Publisher() and Subscriber() class.

**Method**:

1. Create catkin workspace: `mkdir -p catkin_workspace/src`
1. Create catkin package: `catkin_create_pkg pub_sub rospy roscpp std_msgs`
1. Create scripts: `cd src/pub_sub/src && mkdir scripts && cd scripts && code Talker.py && code Listener.py`
1. Add the scripts in this directory
1. Catkin Make: `catkin_make`

>**Code**
>Talker.py
>
```python=3
#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():       
        hello_str = "Pranav Rane"
        rospy.loginfo(hello_str)
        pub.publish(hello_str)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

>Listener.py

```python=3
#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + '\t I heard %s', data.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

```

**OUTPUT**

![image](https://i.imgur.com/QYZL8ox.png)

---

### Exercise 2

**AIM**: Draw a 'D' using Turtlesim:

**Method**:

1. Create catkin package: `catkin_create_pkg move_turtle_d rospy roscpp std_msgs`
2. Create script: `cd src/move_turtle_d/src && mkdir scripts && cd scripts && code turtlesim_d.py`
3. Add the script in this directory
4. Catkin Make: `catkin_make`
5. Open Turtle sim with: `rosrun turtlesim turtlesim_node`
6. Run the script with: `rosrun move_turtle_d DrawD.py`

>**Code**
>DrawD.py
>

```python=3
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty

X = 0.0
Y = 0.0
yaw = 0.0


def pose_callback(pose):
    global X, Y, yaw
    rospy.loginfo("X=%f, Y=%f\n", pose.x, pose.y)
    X = pose.x
    Y = pose.y
    yaw = pose.theta


def move(speed, distance, is_forward):
    velocity_message = Twist()

    global X, Y
    X0 = X
    Y0 = Y

    if is_forward:
        velocity_message.linear.x = abs(speed)
    else:
        velocity_message.linear.x = -abs(speed)

    distance_moved = 0.0
    loop_rate = rospy.Rate(10)
    cmd_vel_topic = '/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    while True:
        rospy.loginfo("Turtlesim moves forward")
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()

        # rospy.loginfo("%f %f %f %f", X,Y,X0,Y0)

        distance_moved = abs(0.5 * math.sqrt(((X - X0) ** 2) + ((X - Y0) ** 2)))
        print(distance_moved)

        if not (distance_moved < distance):
            rospy.loginfo("reached")
            rospy.logwarn("Stopping the Robot")
            break

    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)


def rotate(angular_speed_degree, relative_angle_degree, clockwise):
    global yaw

    velocity_message = Twist()
    velocity_message.linear.x = 0
    velocity_message.angular.z = 0

    theta0 = yaw
    angular_speed = math.radians(abs(angular_speed_degree))

    if clockwise:
        velocity_message.angular.z = -abs(angular_speed)

    else:
        velocity_message.angular.z = abs(angular_speed)

    angle_moved = 0.0
    loop_rate = rospy.Rate(10)
    and_vel_topic = '/turtle1/cmd_vel'

    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    t0 = rospy.Time.now().to_sec()

    while (True):

        rospy.loginfo("Turtlesim rotates")
        velocity_publisher.publish(velocity_message)

        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1 - t0) * angular_speed_degree
        loop_rate.sleep()

        if current_angle_degree > relative_angle_degree:
            rospy.loginfo("reached")
            break

    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)


def go_to_goal(x_goal, y_goal):
    global X
    global Y, yaw

    velocity_message = Twist()
    cmd_vel_topic = '/turtle1/cmd_vel'

    while True:
        K_linear = 0.5

        distance = abs(math.sqrt(((x_goal - X) ** 2) + ((y_goal - Y) ** 2)))
        linear_speed = distance * K_linear
        K_angular = 4.0

        desired_angle_goal = math.atan2(y_goal - Y, x_goal - X)
        angular_speed = (desired_angle_goal - yaw) * K_angular

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)

        print('x=', X, 'y=', Y)

        if distance < 0.01:
            break

def setDesiredOrientation(desired_angle_radians):
    relative_angle_radians = desired_angle_radians - yaw

    if relative_angle_radians > 0:
        clockwise = 1
    else:
        clockwise = 0

    print(relative_angle_radians)
    print(desired_angle_radians)
    rotate(30, math.degrees(abs(relative_angle_radians)), clockwise)

def D(speed):
    global X, Y, yaw
    velocity_message = Twist()
    velocity_message.linear.x = speed
    velocity_message.angular.z = -2.5

    # Semicircle
    while True:
        velocity_publisher.publish(velocity_message)

        if 5.44 < X < 5.54:
            break

    velocity_message = Twist()
    velocity_publisher.publish(velocity_message)

    # Orient to Spawn
    setDesiredOrientation(-1.507)

    # Reach Spawn
    curr_dist = Y
    go_to_goal(5.5,5.5)

if __name__ == '__main__':
    try:
        rospy.init_node('turtlesim_motion_pose', anonymous=True)
        cmd_vel_topic = '/turtle1/cmd_vel'

        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        position_topic = '/turtle1/pose'

        rospy.Subscriber(position_topic, Pose, pose_callback)

        D(6)

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated")
```

**OUTPUT**

> Sucessfully Drew a D with the pyhton script.

![image](https://i.imgur.com/YJeQz2S.png)

---

### Exercise 3

**AIM**: Move the turtlebot in a Hexagon path

**Method**:

1. Create catkin package: `catkin_create_pkg move_turtle_hex rospy roscpp std_msgs`
2. Create script: `cd src/move_turtle_hex/src && mkdir scripts && cd scripts && code turtlesim_hex.py`
3. Add the script in this directory
4. Catkin Make: `catkin_make`
5. Open Turtle sim with: `rosrun turtlesim turtlesim_node`
6. Run the script with: `rosrun move_turtle_d Hexagon.py`

>**Code**
>Hexagon.py
>
```python=3
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty

X = 0.0
Y = 0.0
yaw = 0.0


def pose_callback(pose):
    global X, Y, yaw
    rospy.loginfo("X=%f, Y=%f\n", pose.x, pose.y)
    X = pose.x
    Y = pose.y
    yaw = pose.theta

def move(speed, distance, is_forward):
    velocity_message = Twist()

    global X, Y
    X0 = X
    Y0 = Y

    if is_forward:
        velocity_message.linear.x = abs(speed)
    else:
        velocity_message.linear.x = -abs(speed)

    distance_moved = 0.0
    loop_rate = rospy.Rate(10)
    cmd_vel_topic = '/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    while True:
        rospy.loginfo("Turtlesim moves forward")
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()

        # rospy.loginfo("%f %f %f %f", X,Y,X0,Y0)

        distance_moved = math.sqrt(((X - X0) ** 2) + ((Y - Y0) ** 2))
        print(distance_moved,X,Y,X0,Y0)

        if not (distance_moved < distance):
            rospy.loginfo("reached")
            rospy.logwarn("Stopping the Robot")
            break

    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)


def rotate(angular_speed_degree, relative_angle_degree, clockwise):
    global yaw

    velocity_message = Twist()
    velocity_message.linear.x = 0
    velocity_message.angular.z = 0

    theta0 = yaw
    angular_speed = math.radians(abs(angular_speed_degree))

    if clockwise:
        velocity_message.angular.z = -abs(angular_speed)
    else:
        velocity_message.angular.z = abs(angular_speed)

    angle_moved = 0.0
    loop_rate = rospy.Rate(10)
    and_vel_topic = '/turtle1/cmd_vel'

    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    t0 = rospy.Time.now().to_sec()

    while (True):

        rospy.loginfo("Turtlesim rotates")
        velocity_publisher.publish(velocity_message)

        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1 - t0) * angular_speed_degree
        loop_rate.sleep()

        if current_angle_degree > relative_angle_degree:
            rospy.loginfo("reached")
            break

    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)


def go_to_goal(x_goal, y_goal):
    global X
    global Y, yaw

    velocity_message = Twist()
    cmd_vel_topic = '/turtle1/cmd_vel'

    while True:
        K_linear = 0.5

        distance = abs(math.sqrt(((x_goal - X) ** 2) + ((y_goal - Y) ** 2)))
        linear_speed = distance * K_linear
        K_angular = 4.0

        desired_angle_goal = math.atan2(y_goal - Y, x_goal - X)
        angular_speed = (desired_angle_goal - yaw) * K_angular

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)

        print('x=', X, 'y=', Y)
        if distance < 0.01:
            break

def hexagon(side_length):

    for _ in range(6):
        move(1.0,side_length,True)
        rotate(10,60,False)

if __name__ == '__main__':

    try:
        rospy.init_node('turtlesim_motion_pose', anonymous=True)
        cmd_vel_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        position_topic = '/turtle1/pose'
        rospy.Subscriber(position_topic, Pose, pose_callback)
        time.sleep(1)
        hexagon(2.0)
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated")

```

**OUTPUT**

> Successfully launched a turtlebot and moved it in the hexagon shape

![image](https://i.imgur.com/MTGewJe.png)

---

### Exercise 4

**AIM**: Create a two wheeled robot urdf with a caster wheel.

**Method**:

1. Create catkin package: `catkin_create_pkg three_wheel rospy roscpp std_msgs`
2. Create urdf files: `cd src/three_wheel/src && mkdir urdf && mkdir launch && cd urdf && code wheelie.urdf && cd ../launch && code gazebo.launch`
3. Add the launch file and urdf definition in this directory
4. Launch using roslaunch three_wheel gazebo.launch

>**Code**
>gazebo.launch
>

```xml=1.0
<?xml version="1.0" encoding="UTF-8"?>
<launch>
  <!-- We resume the logic in gazebo_ros package empty_world.launch, -->

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <!-- <arg name="world_name" value="$(find autorickshaw)/worlds/ddrobot.world"/>    -->
    <arg name="paused" default="false"/>
    <arg name="use_sim_time" default="true"/>
    <arg name="gui" default="true"/>
    <arg name="headless" default="false"/>
    <arg name="debug" default="false"/>
  </include>

  <!-- Spawn dd_robot into Gazebo -->
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" output="screen"
     args="-file $(find three_wheel)/urdf/wheelie.urdf -urdf -model wheelie" />

</launch>

```

>wheelie.urdf code

```xml=1.0
<?xml version='1.0'?>
<robot name="wheelie">

  <gazebo>
    <static>False</static>
  </gazebo>


  <link name="dummy"/>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <material name="blue">
        <color rgba="0 0.5 1 1"/>
      </material>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
          <box size=".75 0.5 0.20"/>
      </geometry>
    </visual>
    <!-- Base collision, mass and inertia -->
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
          <box size="0.75 0.5 0.20"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5"/>
      <inertia ixx="0.13" ixy="0.0" ixz="0.0" iyy="0.21" iyz="0.0" izz="0.13"/>
    </inertial>

    <!-- Caster -->
    <visual name="caster">
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
      <origin xyz="0.2 0 -0.125" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.05" />
      </geometry>
    </visual>
    <!-- Caster collision, mass and inertia -->
    <collision>
      <origin xyz="0.2 0 -0.125" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.05" />
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>

  </link>

  <gazebo reference="base_link">
    <material>Gazebo/BlueGlow</material>
    <pose>0 0 3 0 0 0</pose>
  </gazebo>

  <joint name="dummy_joint" type="fixed">
    <parent link="dummy"/>
    <child link="base_link"/>
  </joint>

  <!-- Right Wheel -->
  <link name="right_wheel">
    <visual>
      <material name="black">
        <color rgba="0.05 0.05 0.05 1"/>
      </material>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.1" radius="0.2" />
      </geometry>
    </visual>
    <!-- Right Wheel collision, mass and inertia -->
    <collision>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.1" radius="0.2" />
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>

  </link>

  <gazebo reference="right_wheel">
    <material>Gazebo/BlackTransparent</material>
  </gazebo>

  <!-- Right Wheel joint -->
  <joint name="joint_right_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="-0.2 -0.30 0.025" rpy="0 0 0" /> 
    <axis xyz="0 1 0" />
  </joint>

  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <material name="black">
        <color rgba="0.05 0.05 0.05 1"/>
      </material>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.1" radius="0.2" />
      </geometry>
    </visual>
    <!-- Left Wheel collision, mass and inertia -->
    <collision>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.1" radius="0.2" />
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>

  </link>

  <gazebo reference="left_wheel">
    <material>Gazebo/BlackTransparent</material>
  </gazebo>

  <!-- Left Wheel joint -->
  <joint name="joint_left_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="-0.2 0.30 0.025" rpy="0 0 0" /> 
    <axis xyz="0 1 0"/>
  </joint>

</robot>
```

**OUTPUT**

> Sucessfully created a 2 wheeled bot with a Caster Wheel
>  
![image](https://i.imgur.com/r4VvYTQ.png)

---

### Exercise 4

**AIM**: To Create a manipulator using URDF
**METHOD**:

1. Create catkin package: `catkin_create_pkg three_link rospy roscpp std_msgs`
2. Create urdf/launch files: `cd src/three_link/src && mkdir urdf && mkdir launch && cd urdf && code link.urdf && cd ../launch/ && code gazebo.launch`
3. Add the files from this directory
4. Catkin Make:`catkin_make`
5. Launch the bot: `roslaunch three_link gazebo.launch`
6. Note: From ROS Textbook

>**Code**
> rrbot.xacro

```xml=1.0
<?xml version="1.0"?>
<!-- Revolute-Revolute Manipulator -->
<robot name="rrbot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Constants for robot dimensions -->
  <xacro:property name="PI" value="3.14"/>
  <xacro:property name="width" value="0.1" />   <!-- Beams are square in length and width -->
  <xacro:property name="height1" value="2" />   <!-- Link 1 -->
  <xacro:property name="height2" value="1" />   <!-- Link 2 -->
  <xacro:property name="height3" value="1" />   <!-- Link 3 -->
  <xacro:property name="axle_offset" value="0.05" /> <!-- Space between joint and end of beam -->
  <xacro:property name="damp" value="0.7" />    <!-- damping coefficient -->

  <!-- Default Inertial -->
  <xacro:macro name="default_inertial" params="z_value i_value mass">
    <inertial>
      <origin xyz="0 0 ${z_value}" rpy="0 0 0"/>
      <mass value="${mass}" />
      <inertia ixx="${i_value}" ixy="0.0" ixz="0.0"
               iyy="${i_value}" iyz="0.0"
               izz="${i_value}" />
      </inertial>
  </xacro:macro>

  <!-- Import Rviz colors -->
  <xacro:include filename="$(find manipulator)/urdf/materials.xacro" />

  <!-- Import gripper URDF -->
  <xacro:include filename="$(find manipulator)/urdf/gripper.xacro" />  

  <!-- Import Gazebo elements, including Gazebo colors -->
  <xacro:include filename="$(find manipulator)/urdf/rrbot.gazebo" />

  <!-- Used for fixing rrbot frame to Gazebo world frame -->
  <link name="world"/>

  <joint name="fixed" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
  </joint>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 ${height1/2}" rpy="0 0 0"/>
      <geometry>
 <box size="${width} ${width} ${height1}"/>
      </geometry>
      <material name="red"/>
    </visual>

    <collision>
      <origin xyz="0 0 ${height1/2}" rpy="0 0 0"/>
      <geometry>
 <box size="${width} ${width} ${height1}"/>
      </geometry>
    </collision>

    <xacro:default_inertial z_value="${height1/2}" i_value="1.0" mass="1"/>
  </link>

  <!-- Joint between Base Link and Middle Link -->
  <joint name="joint_base_mid" type="revolute">
    <parent link="base_link"/>
    <child link="mid_link"/>
    <origin xyz="0 ${width} ${height1 - axle_offset}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="${damp}"/>
    <limit effort="100.0" velocity="0.5" lower="-3.14" upper="3.14" />
  </joint>

  <!-- Middle Link -->
  <link name="mid_link">
    <visual>
      <origin xyz="0 0 ${height2/2 - axle_offset}" rpy="0 0 0"/>
      <geometry>
 <box size="${width} ${width} ${height2}"/>
      </geometry>
      <material name="green"/>
    </visual>

    <collision>
      <origin xyz="0 0 ${height2/2 - axle_offset}" rpy="0 0 0"/>
      <geometry>
 <box size="${width} ${width} ${height2}"/>
      </geometry>
    </collision>

    <xacro:default_inertial z_value="${height2/2 - axle_offset}" i_value="1.0" mass="1"/>
  </link>

  <!-- Joint between Middle Link and Top Link -->
  <joint name="joint_mid_top" type="revolute"> 
    <parent link="mid_link"/>
    <child link="top_link"/>
    <origin xyz="0 ${width} ${height2 - axle_offset*2}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/> 
    <dynamics damping="${damp}"/> 
    <limit effort="100.0" velocity="0.5" lower="-3.14" upper="3.14" />
  </joint>

  <!-- Top Link -->
  <link name="top_link">
    <visual>
      <origin xyz="0 0 ${height3/2 - axle_offset}" rpy="0 0 0"/>
      <geometry>
 <box size="${width} ${width} ${height3}"/>
      </geometry>
      <material name="blue"/>
    </visual>

    <collision>
      <origin xyz="0 0 ${height3/2 - axle_offset}" rpy="0 0 0"/>
      <geometry>
 <box size="${width} ${width} ${height3}"/>
      </geometry>
    </collision>

    <xacro:default_inertial z_value="${height3/2 - axle_offset}" i_value="1.0" mass="1"/>
  </link>

  <transmission name="transmission1">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint_base_mid">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor1">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <transmission name="transmission2">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint_mid_top">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor2">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

</robot>
```

>rrbot_gazebo.launch

```xml=1.0
<?xml version="1.0" encoding="UTF-8"?>
<launch>
  <!-- We resume the logic in gazebo_ros package empty_world.launch, -->
  <!-- changing only the name of the world to be launched -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find manipulator)/worlds/rrbot.world"/>
   
    <arg name="paused" default="false"/>
    <arg name="use_sim_time" default="true"/>
    <arg name="gui" default="true"/>
    <arg name="headless" default="false"/>
    <arg name="debug" default="false"/>

  </include>

  <!-- Load the URDF into the ROS Parameter Server -->
  <param name="robot_description"
  command="$(find xacro)/xacro '$(find manipulator)/urdf/rrbot.xacro'" />

  <!-- Spawn rrbot into Gazebo -->
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
     args="-param robot_description -urdf -model rrbot" />

</launch>
```

**OUTPUT**
> Sucessfully implemented the 3-link Manipulator using ROS Robotics by Examples

![image](https://i.imgur.com/R6R76JJ.png)
![image](https://i.imgur.com/FXJ2rdf.png)
![image](https://i.imgur.com/r3BJKOJ.png)

---

### Exercise 5

**AIM**: Display the location of the ball from the base position using OpenCV + ROS (using RGB image)

**Method**:

1. Save your ros distribution in a variable export `ROS_DIST=<noetic|melodic|kinetic>`
2. Install USB Camera support in ROS `sudo apt-get install ros-$ROS_DIST-usb-cam`
3. Install OpenCV `sudo apt-get install opencv-python`
4. Verify Installations are in $PATH
5. Start: `roscore`
6. Launch USB Camera: `rosrun usb_cam usb_cam_node`
7. Run the opencv script: `rosrun imagedetect image_detect.py`

>**Code**
> ros_to_opencv2.py:
>

```python
#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError


bridge = CvBridge()
frame = None

def blob_detect(image, hsv_min, hsv_max, blur=0, blob_params=None, imshow=False):
    if blur > 0:
        image = cv2.blur(image, (blur, blur))

        if imshow:
            cv2.imshow("Blur", image)
            cv2.waitKey(0)

    # BGR to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # - Apply HSV threshold
    mask = cv2.inRange(hsv, hsv_min, hsv_max)

    # - Show HSV Mask
    if imshow:
        cv2.imshow("HSV Mask", mask)

    # - dilate makes the in range areas larger
    mask = cv2.dilate(mask, None, iterations=2)

    if imshow:
        cv2.imshow("Dilate Mask", mask)
        cv2.waitKey(0)

    mask = cv2.erode(mask, None, iterations=2)

    # - Show dilate/erode mask
    if imshow:
        cv2.imshow("Erode Mask", mask)
        cv2.waitKey(0)

    if blob_params is None:
        # Set up the SimpleBlobdetector with default parameters.
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 0;
        params.maxThreshold = 50;

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 5000
        params.maxArea = 500000

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.5

        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.5

    else:
        params = blob_params

        # - Apply blob detection
    detector = cv2.SimpleBlobDetector_create(params)

    # Reverse the mask: blobs are black on white
    reversemask = 255 - mask

    if imshow:
        cv2.imshow("Reverse Mask", reversemask)
        cv2.waitKey(0)

    keypoints = detector.detect(reversemask)

    return keypoints, reversemask


def draw_keypoints(image, keypoints, line_color=(255, 0, 255), imshow=True):
    im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([image]), line_color,
                                          cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    if imshow:
        cv2.imshow("Keypoints", im_with_keypoints)

    return (im_with_keypoints)

def draw_window(image, window_adim, color=(255, 0, 0), line=5, imshow=False):
    rows = image.shape[0]
    cols = image.shape[1]

    x_min_px = int(cols * window_adim[0])
    y_min_px = int(rows * window_adim[1])
    x_max_px = int(cols * window_adim[2])
    y_max_px = int(rows * window_adim[3])

    # -- Draw a rectangle from top left to bottom right corner
    image = cv2.rectangle(image, (x_min_px, y_min_px), (x_max_px, y_max_px), color, line)

    if imshow:
        cv2.imshow("Keypoints", image)

    return (image)

def apply_search_window(image, window_adim=[0.0, 0.0, 1.0, 1.0]):
    rows = image.shape[0]
    cols = image.shape[1]
    x_min_px = int(cols * window_adim[0])
    y_min_px = int(rows * window_adim[1])
    x_max_px = int(cols * window_adim[2])
    y_max_px = int(rows * window_adim[3])

    # --- Initialize the mask as a black image
    mask = np.zeros(image.shape, np.uint8)

    # --- Copy the pixels from the original image corresponding to the window
    mask[y_min_px:y_max_px, x_min_px:x_max_px] = image[y_min_px:y_max_px, x_min_px:x_max_px]

    # --- return the mask
    return (mask)

def get_blob_relative_position(image, keyPoint):
    rows = float(image.shape[0])
    cols = float(image.shape[1])
    # print(rows, cols)
    center_x = 0.5 * cols
    center_y = 0.5 * rows
    # print(center_x)
    x = (keyPoint.pt[0] - center_x) / (center_x)
    y = (keyPoint.pt[1] - center_y) / (center_y)
    return (x, y)


def image_callback(ros_image):
    global frame
    try:
        frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        cv2.imshow("image", frame)
    except CvBridgeError as e:
        print(e)

    lower_color = (100, 100, 100)
    higher_color = (200, 200, 200)

    string1 = "first frame"
    string2 = ""
    string3 = ""
    window = [0, 0, 1, 1]

    keypoints, _ = blob_detect(frame, lower_color, higher_color, blur=3, blob_params=None, imshow=False)
    final_image = draw_keypoints(frame, keypoints, imshow=True)
    cv2.putText(final_image, string1, (25, 25), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 1)
    cv2.putText(final_image, string2, (25, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 1)
    cv2.putText(final_image, string3, (25, 75), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 1)
    cv2.imshow("final image", final_image)
    for i, keyPoint in enumerate(keypoints):
        x = keyPoint.pt[0]
        y = keyPoint.pt[1]
        s = keyPoint.size
        print("kp %d: s = %3d   x = %3d  y= %3d" % (i, s, x, y))

        x, y = get_blob_relative_position(frame, keyPoint)
        print(" x = %3d  y= %3d" % (x, y))
        string1 = 'x=' + str(x)
        string2 = 'y=' + str(y)
        string3 = 's=' + str(s)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('p'):
        return

def listener():
    rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)

    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        rospy.init_node('ar_tracking_node', anonymous=True, disable_signals=True)
        rate = rospy.Rate(100)
        listener()

    except rospy.ROSInterruptException:
        pass

```

**OUTPUT**
>Sucessfully detected circular object and found coordinates w.r.t. base.

![image](https://i.imgur.com/i2MI1lG.png)

---
