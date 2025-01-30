# InterIIT-Aerial-Robotics

**Instructions for running the codes:**
1) Navigate to the workspace folder in terminal
2) paste the command in terminal
```bash
colcon build --symlink-install
source install/setup.bash
```
3) To run the motor failure script
```bash
ros2 run px4_ros_com failure_injection.py --ros-args -p motor_number:=1
```

4) To start the controller scripts of pid and log the important data
```bash
ros2 run px4_ros_com stable_landing_pid.py >> logger.txt
```
5) We need to start frames controller script to convert the frames of the drone according to the equations written.
```bash
ros2 run px4_ros_com frames_controller.py 
```
6)  To start the controller scripts for ftc
```bash
ros2 run px4_ros_com stable_landing_ftc.py
```

**Seperate script for Motor Failure detection in python**
```bash
ros2 run px4_ros_com failure_detection.py
```
**Seperate script for  Motor Failure detection in C++**
```bash
ros2 run px4_ros_com failure_detection_cpp
```
**For Motor Failure Injection**
```bash
ros2 run px4_ros_com failure_injection.py --ros-args -p motor_number:=1
```
change the motor number from 1 to 4

**Individual Motor Control**
```bash
ros2 run px4_ros_com actuator_control.py
```



## Note:
### First Complete the build of PX4 using make px4-sitl and make px4_sitl gazebo-classic
**i) To add gazebo motor Plugin do the below changes:**
1) navigate to
  ```bash
  cd drone_setup/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
  ```
2) Open CMakeLists.txt
3) In line 40 change
   replace **option(BUILD_ROS2_PLUGINS "Enable building ROS2 dependent plugins" OFF)** to
   ```bash
   option(BUILD_ROS2_PLUGINS "Enable building ROS2 dependent plugins" ON)
   ```
4) replace the whole   if (BUILD_ROS2_PLUGINS) to endif line code to the below code . (It approximately starts from line 422)

  ```bash
  if (BUILD_ROS2_PLUGINS)
  add_library(gazebo_motor_failure_plugin SHARED src/gazebo_motor_failure_plugin.cpp)
  target_link_libraries(gazebo_motor_failure_plugin ${GAZEBO_libraries} ${rclcpp_LIBRARIES})
  list(APPEND plugins gazebo_motor_failure_plugin)
  message(STATUS "adding gazebo_motor_failure_plugin to build")

  include_directories(
    include
    ${geometry_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
  )

  target_link_libraries(gazebo_motor_failure_plugin
    ${ament_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
  )
endif()
```
5) Navigate to src of sitl_gazebo-classic where you edited the CmakeList
6) Open the code of gazebo_motor_failure_plugin.cpp and replace the whole code with below code.
  ```bash
/*
 * Copyright 2017 Nuno Marques, PX4 Pro Dev Team, Lisbon
 * Copyright 2017 Siddharth Patel, NTU Singapore
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gazebo_motor_failure_plugin.h>

namespace gazebo {

GazeboMotorFailure::GazeboMotorFailure() :
    ModelPlugin(),
    ROS_motor_num_sub_topic_(kDefaultROSMotorNumSubTopic),
    motor_failure_num_pub_topic_(kDefaultMotorFailureNumPubTopic)
{ }

GazeboMotorFailure::~GazeboMotorFailure() {
  this->updateConnection_.reset();
}

void GazeboMotorFailure::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

  this->namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    this->namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  motor_failure_pub_ = node_handle_->Advertise<msgs::Int>(motor_failure_num_pub_topic_, 1);

  if (_sdf->HasElement("ROSMotorNumSubTopic")) {
    this->ROS_motor_num_sub_topic_ = _sdf->GetElement("ROSMotorNumSubTopic")->Get<std::string>();
  }

  if (_sdf->HasElement("MotorFailureNumPubTopic")) {
    this->motor_failure_num_pub_topic_ = _sdf->GetElement("MotorFailureNumPubTopic")->Get<std::string>();
  }

  // ROS2 Topic subscriber
  // Initialize ROS2, if it has not already been initialized.
  if (!rclcpp::ok()) {
    int argc = 0;
    char **argv = NULL;
    rclcpp::init(argc, argv);
  }

  // Create our ROS2 node. This acts in a similar manner to the Gazebo node
  this->ros_node_ = rclcpp::Node::make_shared("motor_failure");

  // Create a named topic, and subscribe to it.
  subscription = this->ros_node_->create_subscription<std_msgs::msg::Int32>(
		  this->ROS_motor_num_sub_topic_, 10, 
		  std::bind(&GazeboMotorFailure::motorFailNumCallBack, this, std::placeholders::_1));
  std::cout << "[gazebo_motor_failure_plugin]: Subscribe to ROS topic: "<< ROS_motor_num_sub_topic_ << std::endl;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboMotorFailure::OnUpdate, this, std::placeholders::_1));
}

void GazeboMotorFailure::OnUpdate(const common::UpdateInfo &info) {
    this->motor_failure_msg_.set_data(motor_Failure_Number_);
    this->motor_failure_pub_->Publish(motor_failure_msg_);
    rclcpp::spin_some(this->ros_node_);
}

void GazeboMotorFailure::motorFailNumCallBack(const std_msgs::msg::Int32::SharedPtr msg) { 
  this->motor_Failure_Number_ = msg->data;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMotorFailure);
}
```
7) Now navigate to 
  ```bash
  cd drone_setup/PX4-Autopilot/
  make px4_sitl
  make px4_sitl gazebo-classic
  ```
8) Check in folder of PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic libgazebo_motor_failure_plugin.so plugin created.
9) **Go to iris.sdf file of the PX4-AutoPilot folder and paste the following below the plugins for rotor**
```bash
<plugin name="motor_failure" filename="libgazebo_motor_failure_plugin.so">
  <robotNamespace/>
  <ROSMotorNumSubTopic>/motor_failure/motor_number</ROSMotorNumSubTopic>
  <MotorFailureNumPubTopic>/gazebo/motor_failure_num</MotorFailureNumPubTopic>
</plugin>
```

**ii) To remove poll timeout error which could be potentially caused by actuator_control.py code
Replace from <enable_lockstep>1</enable_lockstep> to**
```bash
<enable_lockstep>0</enable_lockstep>
```
**again in same sdf file**

# InterIIT-Aerial-Robotics

**For Motor Failure detection in python**
```bash
ros2 run px4_ros_com failure_detection.py
```
**For Motor Failure detection in C++**
```bash
ros2 run px4_ros_com failure_detection_cpp
```
**For Motor Failure Injection**
```bash
ros2 run px4_ros_com failure_injection.py --ros-args -p motor_number:=1
```
change the motor number from 1 to 4

**Individual Motor Control**
```bash
ros2 run px4_ros_com actuator_control.py
```

**Stable Landing Motor Control**
```bash
ros2 run px4_ros_com stable_landing.py 
```


## Note:
### First Complete the build of PX4 using make px4-sitl and make px4_sitl gazebo-classic
**i) To add gazebo motor Plugin do the below changes:**
1) navigate to
  ```bash
  cd drone_setup/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
  ```
2) Open CMakeLists.txt
3) In line 40 change
   replace **option(BUILD_ROS2_PLUGINS "Enable building ROS2 dependent plugins" OFF)** to
   ```bash
   option(BUILD_ROS2_PLUGINS "Enable building ROS2 dependent plugins" ON)
   ```
4) replace the whole   if (BUILD_ROS2_PLUGINS) to endif line code to the below code . (It approximately starts from line 422)

  ```bash
  if (BUILD_ROS2_PLUGINS)
  add_library(gazebo_motor_failure_plugin SHARED src/gazebo_motor_failure_plugin.cpp)
  target_link_libraries(gazebo_motor_failure_plugin ${GAZEBO_libraries} ${rclcpp_LIBRARIES})
  list(APPEND plugins gazebo_motor_failure_plugin)
  message(STATUS "adding gazebo_motor_failure_plugin to build")

  include_directories(
    include
    ${geometry_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
  )

  target_link_libraries(gazebo_motor_failure_plugin
    ${ament_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
  )
endif()
```
5) Navigate to src of sitl_gazebo-classic where you edited the CmakeList
6) Open the code of gazebo_motor_failure_plugin.cpp and replace the whole code with below code.
  ```bash
/*
 * Copyright 2017 Nuno Marques, PX4 Pro Dev Team, Lisbon
 * Copyright 2017 Siddharth Patel, NTU Singapore
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gazebo_motor_failure_plugin.h>

namespace gazebo {

GazeboMotorFailure::GazeboMotorFailure() :
    ModelPlugin(),
    ROS_motor_num_sub_topic_(kDefaultROSMotorNumSubTopic),
    motor_failure_num_pub_topic_(kDefaultMotorFailureNumPubTopic)
{ }

GazeboMotorFailure::~GazeboMotorFailure() {
  this->updateConnection_.reset();
}

void GazeboMotorFailure::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

  this->namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    this->namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  motor_failure_pub_ = node_handle_->Advertise<msgs::Int>(motor_failure_num_pub_topic_, 1);

  if (_sdf->HasElement("ROSMotorNumSubTopic")) {
    this->ROS_motor_num_sub_topic_ = _sdf->GetElement("ROSMotorNumSubTopic")->Get<std::string>();
  }

  if (_sdf->HasElement("MotorFailureNumPubTopic")) {
    this->motor_failure_num_pub_topic_ = _sdf->GetElement("MotorFailureNumPubTopic")->Get<std::string>();
  }

  // ROS2 Topic subscriber
  // Initialize ROS2, if it has not already been initialized.
  if (!rclcpp::ok()) {
    int argc = 0;
    char **argv = NULL;
    rclcpp::init(argc, argv);
  }

  // Create our ROS2 node. This acts in a similar manner to the Gazebo node
  this->ros_node_ = rclcpp::Node::make_shared("motor_failure");

  // Create a named topic, and subscribe to it.
  subscription = this->ros_node_->create_subscription<std_msgs::msg::Int32>(
		  this->ROS_motor_num_sub_topic_, 10, 
		  std::bind(&GazeboMotorFailure::motorFailNumCallBack, this, std::placeholders::_1));
  std::cout << "[gazebo_motor_failure_plugin]: Subscribe to ROS topic: "<< ROS_motor_num_sub_topic_ << std::endl;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboMotorFailure::OnUpdate, this, std::placeholders::_1));
}

void GazeboMotorFailure::OnUpdate(const common::UpdateInfo &info) {
    this->motor_failure_msg_.set_data(motor_Failure_Number_);
    this->motor_failure_pub_->Publish(motor_failure_msg_);
    rclcpp::spin_some(this->ros_node_);
}

void GazeboMotorFailure::motorFailNumCallBack(const std_msgs::msg::Int32::SharedPtr msg) { 
  this->motor_Failure_Number_ = msg->data;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMotorFailure);
}
```
7) Now navigate to 
  ```bash
  cd drone_setup/PX4-Autopilot/
  make px4_sitl
  make px4_sitl gazebo-classic
  ```
8) Check in folder of PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic libgazebo_motor_failure_plugin.so plugin created.
9) **Go to iris.sdf file of the PX4-AutoPilot folder and paste the following below the plugins for rotor**
```bash
<plugin name="motor_failure" filename="libgazebo_motor_failure_plugin.so">
  <robotNamespace/>
  <ROSMotorNumSubTopic>/motor_failure/motor_number</ROSMotorNumSubTopic>
  <MotorFailureNumPubTopic>/gazebo/motor_failure_num</MotorFailureNumPubTopic>
</plugin>
```

**ii) To remove poll timeout error which could be potentially caused by actuator_control.py code
Replace from <enable_lockstep>1</enable_lockstep> to**
```bash
<enable_lockstep>0</enable_lockstep>
```
**again in same sdf file**





    



