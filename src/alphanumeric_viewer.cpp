#include "alphanumeric_viewer.hpp"

AlphanumericViewer::AlphanumericViewer() : as2::Node("alphanumeric_viewer") {

}

void AlphanumericViewer::run(){

        command = getch();
        switch (command){
            case 'S':
            case 's':  // Sensor
                erase();
                refresh();
                printSensorMenu();
                window = 0;
            break;
            case 'N':
            case 'n':  // Navigation
                erase();
                refresh();
                printNavigationMenu();
                window = 1;
            break;
        }

        //Print values
        switch (window){
            case 0:
                printSensorValues();
            break;
            case 1:
                printNavigationValues();
            break;
        }

        //Refresh
        refresh();
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

  /*trajectory_reference_sub_ = this->create_subscription<as2_msgs::msg::TrajectoryWaypoints::SharedPtr>(
      this->generate_local_name(as2_names::topics::motion_reference::trajectory),
      as2_names::topics::motion_reference::traj_gen_qos,
      std::bind(&AlphanumericViewer::trajectoryReferenceCallback, this, std::placeholders::_1));*/

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
  /*void AlphanumericViewer::trajectoryReferenceCallback (const as2_msgs::msg::TrajectoryWaypoints::SharedPtr _msg){
    reference_traj_ = *_msg;
    current_trajectory_reference_aux = true;
  }*/
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

void AlphanumericViewer::printStream(float var,bool aux) {
    if(aux){
        interface_printout_stream.clear();
        interface_printout_stream.str(std::string());
        if (var > -0.01){
            interface_printout_stream << std::setw(5) << std::internal << fabs(var);
            attron(COLOR_PAIR(1));printw(" %s",interface_printout_stream.str().c_str());attroff(COLOR_PAIR(1));
        }else{
            interface_printout_stream << std::setw(6) << std::internal << var;
            attron(COLOR_PAIR(2));printw("%s",interface_printout_stream.str().c_str());attroff(COLOR_PAIR(2));
        }
    }else{
        printw("--.--");
    }
} 

//Print float using stringstream with 3 units
void AlphanumericViewer::printStream3(float var,bool aux) {
    if(aux){
        interface_printout_stream.clear();
        interface_printout_stream.str(std::string());
        if (var > -0.01){
            interface_printout_stream << std::setw(6) << std::internal << fabs(var);
            attron(COLOR_PAIR(1));printw(" %s",interface_printout_stream.str().c_str());attroff(COLOR_PAIR(1));
        }else{
            interface_printout_stream << std::setw(7) << std::internal << var;
            attron(COLOR_PAIR(2));printw("%s",interface_printout_stream.str().c_str());attroff(COLOR_PAIR(2));
        }
    }else{
        printw("---.--");
    }
} 

//Print double using stringstream
void AlphanumericViewer::printStream(double var,bool aux) {
    if(aux){
        interface_printout_stream.clear();
        interface_printout_stream.str(std::string());
        if (var > -0.01){
            interface_printout_stream << std::setw(5) << std::internal << fabs(var);
            attron(COLOR_PAIR(1));printw(" %s",interface_printout_stream.str().c_str());attroff(COLOR_PAIR(1));
        }else{
            interface_printout_stream << std::setw(6) << std::internal << var;
            attron(COLOR_PAIR(2));printw("%s",interface_printout_stream.str().c_str());attroff(COLOR_PAIR(2));
        }
    }else{
        printw("--.--");
    }
}

void AlphanumericViewer::printSensorValues(){
    //DroneID
    move(4,4);
    attron(COLOR_PAIR(4));printw("%s",this->get_namespace());attroff(COLOR_PAIR(4));
    //Battery
    move(6,4);
    printBattery();
    //Speed
    move(8,4);
    printStream(self_localization_twist_.twist.linear.x,current_speed_aux);printw(",");
    move(8,11);
    printStream(self_localization_twist_.twist.linear.y,current_speed_aux);printw(",");
    move(8,18);
    printStream(self_localization_twist_.twist.linear.z,current_speed_aux);printw(" m/s   ");

    //Pose IMU
    tf2::Matrix3x3 imu_m(tf2::Quaternion (imu_.orientation.x,imu_.orientation.y,imu_.orientation.z,imu_.orientation.w));
    double r = 0; double p = 0; double yaw = 0;
    imu_m.getRPY(r, p, yaw);
    if (std::isnan(r)) r = 0.0; 
    if (std::isnan(p)) p = 0.0; 
    if (std::isnan(yaw)) yaw = 0.0; 

    move(10,4);
    printStream(yaw,imu_aux);printw(",");
    move(10,11);
    printStream(p,imu_aux);printw(",");
    move(10,18);
    printStream(r,imu_aux);printw(" rad   ");

    //Speed IMU
    move(12,4);
    printStream(imu_.angular_velocity.z,imu_aux);printw(",");
    move(12,11);
    printStream(imu_.angular_velocity.y,imu_aux);printw(",");
    move(12,18);
    printStream(imu_.angular_velocity.x,imu_aux);printw(" rad/s  ");

    //Acceleration IMU
    move(14,4);
    printStream(imu_.linear_acceleration.x,imu_aux);printw(",");
    move(14,11);
    printStream(imu_.linear_acceleration.y,imu_aux);printw(",");
    move(14,18);
    printStream(imu_.linear_acceleration.z,imu_aux);printw(" m/s2   ");

    //Altitude
    move(4,52);
    printStream(self_localization_pose_.pose.position.z,current_pose_aux);printw(" m");
    //Altitude sea level
    /*move(6,52);
    printStream(altitude_sea_level_msg.point.z,altitude_sea_level_aux);printw(" m");
    //Temperature
    move(8,52);
    printStream(temperature_msg.temperature,temperature_aux);printw(" Degrees celsius"); */
}

void AlphanumericViewer::printNavigationValues(){
    //Measurements
    move(5,14);
    printStream(self_localization_pose_.pose.position.z,current_pose_aux);printw(" m");
    //Speed
    move(6,14);
    printStream(self_localization_twist_.twist.linear.x,current_speed_aux);printw(",");
    move(6,21);
    printStream(self_localization_twist_.twist.linear.y,current_speed_aux);printw(",");
    move(6,28);
    printStream(self_localization_twist_.twist.linear.z,current_speed_aux);printw(" m/s   ");

    //Pose IMU
    tf2::Matrix3x3 imu_m(tf2::Quaternion (imu_.orientation.x,imu_.orientation.y,imu_.orientation.z,imu_.orientation.w));
    double r = 0; double p = 0; double yaw = 0;
    imu_m.getRPY(r, p, yaw);
    if (std::isnan(r)) r = 0.0; 
    if (std::isnan(p)) p = 0.0; 
    if (std::isnan(yaw)) yaw = 0.0; 

    move(7,14);
    printStream(yaw,imu_aux);printw(",");
    move(7,21);
    printStream(p,imu_aux);printw(",");
    move(7,28);
    printStream(r,imu_aux);printw(" rad   ");

    //Speed IMU
    move(8,14);
    printStream(imu_.angular_velocity.z,imu_aux);printw(",");
    move(8,21);
    printStream(imu_.angular_velocity.y,imu_aux);printw(",");
    move(8,28);
    printStream(imu_.angular_velocity.x,imu_aux);printw(" rad/s  ");

    //Acceleration IMU
    move(9,14);
    printStream(imu_.linear_acceleration.x,imu_aux);printw(",");
    move(9,21);
    printStream(imu_.linear_acceleration.y,imu_aux);printw(",");
    move(9,28);
    printStream(imu_.linear_acceleration.z,imu_aux);printw(" m/s2   ");

    //Localization
    //Pose
    move(5,53);
    printStream3(self_localization_pose_.pose.position.x,current_pose_aux);printw(",");
    move(5,61);
    printStream3(self_localization_pose_.pose.position.y,current_pose_aux);printw(",");
    move(5,69);
    printStream3(self_localization_pose_.pose.position.z,current_pose_aux);printw(" m "); 
    //Speed
    move(6,54);
    printStream(self_localization_twist_.twist.linear.x,current_speed_aux);printw(",");
    move(6,61);
    printStream(self_localization_twist_.twist.linear.y,current_speed_aux);printw(",");
    move(6,68);
    printStream(self_localization_twist_.twist.linear.z,current_speed_aux);printw(" m/s "); 
    //Pose(ypr)
    tf2::Matrix3x3 pose_m(tf2::Quaternion (self_localization_pose_.pose.orientation.x,self_localization_pose_.pose.orientation.y,self_localization_pose_.pose.orientation.z,self_localization_pose_.pose.orientation.w));
    pose_m.getRPY(r, p, yaw);
    if (std::isnan(yaw)) yaw = 0.0; if (std::isnan(r)) r = 0.0; if (std::isnan(p)) p = 0.0;
    move(7,54);
    printStream(yaw,current_pose_aux);printw(",");
    move(7,61);
    printStream(p,current_pose_aux);printw(",");
    move(7,68);
    printStream(r,current_pose_aux);printw(" rad ");     
    //Speed(ypr)
    move(8,54);
    printStream(self_localization_twist_.twist.angular.z,current_speed_aux);printw(",");
    move(8,61);
    printStream(self_localization_twist_.twist.angular.y,current_speed_aux);printw(",");
    move(8,68);
    printStream(self_localization_twist_.twist.angular.x,current_speed_aux);printw(" rad/s ");
    //State
    move(9,54);
    printQuadrotorState();

    //Actuator commands
    if(thrust_aux){
        move(12,19);
        printStream(actuator_thrust_.thrust,thrust_aux);printw(" N ,");
        move(12,26);
        printStream(actuator_thrust_.thrust_normalized,thrust_aux);printw(" normalized  ");
        //Speed(z)
        move(13,19);
        printStream(actuator_twist_.twist.linear.z,actuator_command_twist_aux);printw(" m/s  ");
        //Thrust
        /*move(14,19);
        printStream(actuator_twist_.twist.angular.z,thrust_aux);printw(" N  ");*/
        //Speed(yaw)
        move(15,19);
        printStream(actuator_twist_.twist.angular.z,actuator_command_twist_aux);printw(" rad/s  ");
    }else{
        //Pitch roll
        tf2::Matrix3x3 actuator_m(tf2::Quaternion (actuator_pose_.pose.orientation.x,actuator_pose_.pose.orientation.y,actuator_pose_.pose.orientation.z,actuator_pose_.pose.orientation.w));
        r = 0; p = 0; yaw = 0;
        actuator_m.getRPY(r, p, yaw);
        if (std::isnan(r)) r = 0.0; 
        if (std::isnan(p)) p = 0.0; 
        move(12,19);
        printStream(p,current_pose_aux);printw(",");
        move(12,26);
        printStream(r,current_pose_aux);printw(" rad  ");
        //Speed(z)
        move(13,19);
        printStream(actuator_twist_.twist.linear.z,current_speed_aux);printw(" m/s  ");
        //Thrust
        /*move(14,19);
        printStream(thrust_msg.thrust.z,thrust_aux);printw(" N  ");*/
        //Speed(yaw)
        move(15,19);
        printStream(actuator_twist_.twist.angular.z,current_speed_aux);printw(" rad/s  ");
    }

    //References
    //Pose
    move(12,53);
    printStream3(reference_pose_.pose.position.x,current_pose_reference_aux);printw(",");
    move(12,61);
    printStream3(reference_pose_.pose.position.y,current_pose_reference_aux);printw(",");
    move(12,69);
    printStream3(reference_pose_.pose.position.z,current_pose_reference_aux);printw(" m "); 
    //Speed
    move(13,54);
    printStream(reference_twist_.twist.linear.x,current_speed_reference_aux);printw(",");
    move(13,61);
    printStream(reference_twist_.twist.linear.y,current_speed_reference_aux);printw(",");
    move(13,68);
    printStream(reference_twist_.twist.linear.z,current_speed_reference_aux);printw(" m/s ");
    //Pose (yaw)
    tf2::Matrix3x3 pose_ref_m(tf2::Quaternion (reference_pose_.pose.orientation.x,reference_pose_.pose.orientation.y,reference_pose_.pose.orientation.z,reference_pose_.pose.orientation.w));
    r = 0; p = 0; yaw = 0;
    pose_ref_m.getRPY(r, p, yaw);
    if (std::isnan(yaw)) yaw = 0.0; 
    move(14,54);
    printStream(yaw,current_pose_reference_aux);printw(" rad");
    //Speed (yaw)
    move(15,54);
    printStream(reference_twist_.twist.angular.z,current_speed_reference_aux);printw(" rad/s  ");  
    //Control mode
    move(16,56);
    printControlMode();  
}

void AlphanumericViewer::printBattery(){
    if(battery_aux){
        interface_printout_stream << std::fixed << std::setprecision(0) << std::setfill(' ');
        interface_printout_stream.clear();
        interface_printout_stream.str(std::string());
        clrtoeol(); refresh();
        float percentage = battery_status_.percentage * 100;
        interface_printout_stream << std::setw(2) << std::internal << percentage;
        if(battery_status_.percentage == 1) {
            attron(COLOR_PAIR(1));printw(" %s",interface_printout_stream.str().c_str());attroff(COLOR_PAIR(1));
        }
        if(battery_status_.percentage > 0.5 && battery_status_.percentage < 1) {
            attron(COLOR_PAIR(1));printw(" %s",interface_printout_stream.str().c_str());attroff(COLOR_PAIR(1));
        }
        if(battery_status_.percentage <= 0.5 && battery_status_.percentage > 0.2) {
            attron(COLOR_PAIR(3));printw(" %s",interface_printout_stream.str().c_str());attroff(COLOR_PAIR(3));
        }
        if(battery_status_.percentage <= 0.2) {
            attron(COLOR_PAIR(2));printw(" %s",interface_printout_stream.str().c_str());attroff(COLOR_PAIR(2));  
        }
    }else{ //Battery has not been received
        printw("---");        
    }
    printw(" %%");
    interface_printout_stream << std::fixed << std::setprecision(2) << std::setfill('0');
}

void AlphanumericViewer::printQuadrotorState(){
    switch (platform_info_.status.state) {
        case as2_msgs::msg::PlatformStatus::LANDED:
            printw("LANDED    ");
            break;
        case as2_msgs::msg::PlatformStatus::FLYING:
            printw("FLYING    ");
            break;
        case as2_msgs::msg::PlatformStatus::EMERGENCY:
            printw("EMERGENCY ");
            break;
        case as2_msgs::msg::PlatformStatus::DISARMED:
            printw("DISARMED  ");
            break;
        case as2_msgs::msg::PlatformStatus::TAKING_OFF:
            printw("TAKING OFF");
            break;
        case as2_msgs::msg::PlatformStatus::LANDING:
            printw("LANDING   ");
            break;
    }
}

void AlphanumericViewer::printControlMode(){
    switch (controller_info_.current_control_mode.control_mode) {
    case as2_msgs::msg::ControlMode::UNSET:
        printw("UNSET        ");
        break;
    case as2_msgs::msg::ControlMode::HOVER:
        printw("HOVER        ");
        break;
    case as2_msgs::msg::ControlMode::POSITION:
        printw("POSITION     ");
        break;
    case as2_msgs::msg::ControlMode::SPEED:
        printw("SPEED        ");
        break; 
    case as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE:
        printw("SPEEDINAPLANE");
        break;
    case as2_msgs::msg::ControlMode::ATTITUDE:
        printw("ATTITUDE     ");
        break;                   
    case as2_msgs::msg::ControlMode::ACRO:
        printw("ACRO         ");
        break;  
    case as2_msgs::msg::ControlMode::TRAJECTORY:
        printw("TRAJECTORY   ");
        break;  
    case as2_msgs::msg::ControlMode::ACEL:
        printw("ACEL         ");
        break;  
    default:
        printw("UNKNOWN      ");
        break;
    }
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
  endwin();
  return CallbackReturn::SUCCESS;
};
