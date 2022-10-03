#include "alphanumeric_viewer.hpp"

AlphanumericViewer::AlphanumericViewer() : as2::Node("alphanumeric_viewer") {

}

void AlphanumericViewer::run(){
  return;
}
void AlphanumericViewer::setupNode(){
  interface_printout_stream << std::fixed << std::setprecision(2) << std::setfill('0');

  self_localization_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      this->generate_local_name(as2_names::topics::self_localization::pose),
      as2_names::topics::sensor_measurements::qos,
      std::bind(&AlphanumericViewer::poseCallback, this, std::placeholders::_1));

  self_localization_speed_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      this->generate_local_name(as2_names::topics::self_localization::twist),
      as2_names::topics::self_localization::qos,
      std::bind(&AlphanumericViewer::twistCallback, this, std::placeholders::_1));

  battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      this->generate_local_name(as2_names::topics::sensor_measurements::battery),
      as2_names::topics::sensor_measurements::qos,
      std::bind(&AlphanumericViewer::batteryCallback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      this->generate_local_name(as2_names::topics::sensor_measurements::imu),
      as2_names::topics::sensor_measurements::qos,
      std::bind(&AlphanumericViewer::imuCallback, this, std::placeholders::_1));

  status_sub_ = this->create_subscription<as2_msgs::msg::PlatformInfo>(
      this->generate_local_name(as2_names::topics::platform::info),
      as2_names::topics::platform::qos,
      std::bind(&AlphanumericViewer::platformCallback, this, std::placeholders::_1));

  actuator_command_pose_sub_ = this->create_subscription<as2_msgs::msg::PlatformInfo>(
      this->generate_local_name(as2_names::topics::actuator_command::pose),
      as2_names::topics::actuator_command::qos,
      std::bind(&AlphanumericViewer::actuatorPoseCallback, this, std::placeholders::_1));

  actuator_command_thrust_sub_ = this->create_subscription<as2_msgs::msg::Thrust>(
      this->generate_local_name(as2_names::topics::actuator_command::thrust),
      as2_names::topics::actuator_command::qos,
      std::bind(&AlphanumericViewer::actuatorThrustCallback, this, std::placeholders::_1));

  actuator_command_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      this->generate_local_name(as2_names::topics::actuator_command::twist),
      as2_names::topics::actuator_command::qos,
      std::bind(&AlphanumericViewer::actuatorSpeedCallback, this, std::placeholders::_1));

  controller_info_sub_ = this->create_subscription<as2_msgs::msg::ControllerInfo>(
      this->generate_local_name(as2_names::topics::controller::info),
      as2_names::topics::controller::qos_info,
      std::bind(&AlphanumericViewer::controllerCallback, this, std::placeholders::_1));

  position_reference_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      this->generate_local_name(as2_names::topics::motion_reference::pose),
      as2_names::topics::motion_reference::qos,
      std::bind(&AlphanumericViewer::poseReferenceCallback, this, std::placeholders::_1));

  speed_reference_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      this->generate_local_name(as2_names::topics::motion_reference::twist),
      as2_names::topics::motion_reference::qos,
      std::bind(&AlphanumericViewer::speedReferenceCallback, this, std::placeholders::_1));

  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      this->generate_global_name(as2_names::topics::sensor_measurements::gps),
      as2_names::topics::sensor_measurements::qos,
      std::bind(&AlphanumericViewer::gpsCallback, this, std::placeholders::_1));

    initscr();
    start_color();
    use_default_colors();  
    curs_set(0);
    noecho();
    nodelay(stdscr, TRUE);
    erase();
    refresh();
    init_pair(1, COLOR_GREEN, -1);
    init_pair(2, COLOR_RED, -1);
    init_pair(3, COLOR_YELLOW, -1);
    init_pair(4, COLOR_CYAN, -1);

    printSensorMenu();

    char command = 0;

    // 0 Sensor
    // 1 Navigation
    int window = 0;
}

  void AlphanumericViewer::poseCallback (const geometry_msgs::msg::PoseStamped::SharedPtr _msg){
    self_localization_pose_ = *_msg;
    current_pose_aux = true;
  }
  void AlphanumericViewer::twistCallback (const geometry_msgs::msg::TwistStamped::SharedPtr _msg){
    self_localization_twist_ = *_msg;
    current_speed_aux = true;
  }
  void AlphanumericViewer::batteryCallback (const sensor_msgs::msg::BatteryState::SharedPtr _msg){
    battery_status_ = *_msg;
    battery_aux = true;
  }
  void AlphanumericViewer::imuCallback (const sensor_msgs::msg::Imu::SharedPtr _msg){
    imu_ = *_msg;
    imu_aux = true;
  }
  void AlphanumericViewer::platformCallback (const as2_msgs::msg::PlatformInfo::SharedPtr _msg){
    platform_info_ = *_msg;
    platform_info_aux = true;
  }
  void AlphanumericViewer::actuatorPoseCallback (const geometry_msgs::msg::PoseStamped::SharedPtr _msg){
    actuator_pose_ = *_msg;
    actuator_command_pose_aux = true;
  }
  void AlphanumericViewer::actuatorThrustCallback (const as2_msgs::msg::Thrust::SharedPtr _msg){
    actuator_thrust_ = *_msg;
    actuator_command_thrust_aux = true;
  }
  void AlphanumericViewer::actuatorSpeedCallback (const geometry_msgs::msg::TwistStamped::SharedPtr _msg){
    actuator_twist_ = *_msg;
    actuator_command_twist_aux = true;
  }
  void AlphanumericViewer::controllerCallback (const as2_msgs::msg::ControllerInfo::SharedPtr _msg){
    controller_info_ = *_msg;
    controller_info_aux = true;
  }
  void AlphanumericViewer::poseReferenceCallback (const geometry_msgs::msg::PoseStamped::SharedPtr _msg){
    reference_pose_ = *_msg;
    current_pose_reference_aux = true;
  }
  void AlphanumericViewer::speedReferenceCallback (const geometry_msgs::msg::TwistStamped::SharedPtr _msg){
    reference_twist_ = *_msg;
    current_speed_reference_aux = true;
  }
  void AlphanumericViewer::trajectoryReferenceCallback (const as2_msgs::msg::TrajectoryWaypoints::SharedPtr _msg){
    reference_traj_ = *_msg;
    current_trajectory_reference_aux = true;
  }
  void AlphanumericViewer::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr _msg){
    gps_ = *_msg;
    gps_aux = true;
  }

  void AlphanumericViewer::printNavigationMenu(){
      move(0,0);
      printw("                - ALPHANUMERIC VIEWER OF AERIAL ROBOTICS DATA -");
      move(1,0);
      printw("                        Key: S (sensors), N (navigation)");
      move(2,0);
      printw("                                          ^             ");
      //Measurements
      move(4,0);
      printw(" MEASUREMENTS");
      move(5,0);
      printw(" Pose(z):");
      move(6,0);
      printw(" Speed(xyz):");
      move(7,0);
      printw(" Pose(ypr):");
      move(8,0);
      printw(" Speed(ypr):");
      move(9,0);
      printw(" Accel.(xyz):");

      //Localization
      move(4,42);
      printw("LOCALIZATION");
      move(5,42);
      printw("Pose(xyz):");
      move(6,42);
      printw("Speed(xyz):");
      move(7,42);
      printw("Pose(ypr):");
      move(8,42);
      printw("Speed(ypr):");
      move(9,42);
      printw("Status:");

      //References
      move(11,42);
      printw("REFERENCES");
      move(12,42);
      printw("Pose(xyz):");
      move(13,42);
      printw("Speed(xyz):");
      move(14,42);
      printw("Pose(yaw):");
      move(15,42);
      printw("Speed(yaw):");
      move(16,42);
      printw("Control mode:");

      //Actuator commands
      move(11,0);
      printw(" ACTUATOR COMMANDS");
      move(12,0);
      printw(" Pose(pitch,roll):");
      move(13,0);
      printw(" Speed(z):");
      move(14,0);
      printw(" Thrust:");
      move(15,0);
      printw(" Speed(yaw):");
  }

  void AlphanumericViewer::printSensorMenu(){
      move(0,0);
      printw("                - ALPHANUMERIC VIEWER OF AERIAL ROBOTICS DATA -");
      move(1,0);
      printw("                        Key: S (sensors), N (navigation)");
      move(2,0);
      printw("                             ^                          ");
      //Left column  
      move(3,0);
      printw(" Drone id:");
      move(5,0);
      printw(" Battery charge:");
      move(7,0);
      printw(" Speed (x,y,z):");
      move(9,0);
      printw(" Pose IMU (yaw,pitch,roll):");
      move(11,0);
      printw(" Speed IMU (yaw,pitch,roll):");
      move(13,0);
      printw(" Acceleration IMU (x,y,z):");

      //Right column
      move(3,50);
      printw("Altitude (-z):");
      move(5,50);
      printw("Altitude (sea level):");
      move(7,50);
      printw("Temperature:");
  }

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn AlphanumericViewer::on_configure(
    const rclcpp_lifecycle::State& _state) {
  // Set subscriptions, publishers, services, actions, etc. here.
  setupNode();
  return CallbackReturn::SUCCESS;
};

CallbackReturn AlphanumericViewer::on_deactivate(
    const rclcpp_lifecycle::State& _state) {
  // Clean up subscriptions, publishers, services, actions, etc. here.
  return CallbackReturn::SUCCESS;
};

CallbackReturn AlphanumericViewer::on_shutdown(
    const rclcpp_lifecycle::State& _state) {
  // Clean other resources here.

  return CallbackReturn::SUCCESS;
};
