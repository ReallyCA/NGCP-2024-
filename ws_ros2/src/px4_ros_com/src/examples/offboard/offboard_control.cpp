// useful references: https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard/offboard_control.cpp, https://github.com/PX4/PX4-Autopilot/issues/22288, https://discuss.px4.io/t/offboard-mode-trajectory-setpoint/32850
// cmd: ros2 run px4_ros_com offboard_control

#include <cmath>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/trajectory_waypoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardPlane : public rclcpp::Node {
public:
  OffboardPlane(std::vector<std::vector<float>> local_waypoints) : Node("offboard_plane"), local_waypoints_(local_waypoints) {
    
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    qos_profile.keep_last(1);

    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    trajectory_waypoint_publisher_ = this->create_publisher<TrajectoryWaypoint>("/fmu/in/trajectory_waypoint", 10);

    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    vehicle_local_position_setpoint_publisher_ = this->create_publisher<VehicleLocalPositionSetpoint>("/fmu/in/vehicle_local_position_setpoint", 10);
    vehicle_status_subcriber = this->create_subscription<VehicleStatus>("/fmu/out/vehicle_status", qos_profile, std::bind(&OffboardPlane::vehicle_status_callback, this, std::placeholders::_1));
    vehicle_odometry_subscriber = this->create_subscription<VehicleOdometry>("/fmu/out/vehicle_odometry", qos_profile, std::bind(&OffboardPlane::vehicle_odometry_callback, this, std::placeholders::_1));
    

    std::thread t1(&OffboardPlane::run, this);
    t1.detach();
  }

  void arm();
  void disarm();
  void takeoff();
  void land();
  void run();
  void orbit();

private:
 
  std::vector<std::vector<float>> local_waypoints_;
  size_t waypoint_index_ = 0;
  px4_msgs::msg::VehicleStatus vehicle_status_;
  px4_msgs::msg::VehicleOdometry vehicle_odometry_;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectoryWaypoint>::SharedPtr trajectory_waypoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleLocalPositionSetpoint>::SharedPtr vehicle_local_position_setpoint_publisher_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subcriber;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_subscriber;

  void publish_offboard_control_mode();
  void publish_trajectory_setpoint(float x, float y, float z);
  void publish_trajectory_waypoint(float x, float y, float z);
  void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0);
  void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void publish_vehicle_loiter_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0, float param5 = 0.0, float param6 = 0.0, float param7 = 0.0);
};

void OffboardPlane::run() {

  float prev_d = 9999;
  bool mission_completed = false;

  // int i = 0;
  // auto takeoff_time = this->get_clock()->now();
  // bool takeoff_completed = false;

  // this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_HOME, 0.0, 0.0);
  // this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN, 0.0, 0.0);

  while (rclcpp::ok()) {
    //# Loiter around this MISSION for X turns |Turns| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|

    // if(i==10) {
    //   this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    //   arm();
    //   takeoff();
    //   takeoff_time = this->get_clock()->now();
    // }
    // if (i<11) {
    //   i++;
    // }

    // if(i>10) {
    //   if(this->get_clock()->now()-takeoff_time > 10s && !takeoff_completed) {
    //     takeoff_completed = true;
    //     this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    //   }
    // }
    
    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

    float pos_n = vehicle_odometry_.position[0];
    float pos_e = vehicle_odometry_.position[1];
    float pos_d = vehicle_odometry_.position[2];

    float waypoint_x = local_waypoints_[waypoint_index_][0];
    float waypoint_y = local_waypoints_[waypoint_index_][1];
    float waypoint_z = local_waypoints_[waypoint_index_][2];

    float dx = local_waypoints_[waypoint_index_][0] - pos_n;
    float dy = local_waypoints_[waypoint_index_][1] - pos_e;
    float dz = local_waypoints_[waypoint_index_][2] - pos_d;
    float distance_to_waypoint = sqrt(dx * dx + dy * dy + dz * dz);

    if (distance_to_waypoint < 10.0) {

      if (waypoint_index_ + 1 == 2) {
        RCLCPP_INFO(this->get_logger(), "All waypoints have been traversed!");
        mission_completed = true;
      } else {
        waypoint_index_++;
      }
    }
    if(!mission_completed) {
      RCLCPP_INFO(this->get_logger(), "Distance to waypoint: %f, setpoint: %zu", distance_to_waypoint, waypoint_index_);
      RCLCPP_INFO(this->get_logger(), "Position: N: %f, E:, %f, D: %f", pos_n, pos_e, pos_d);
      if(prev_d < distance_to_waypoint) {
        RCLCPP_INFO(this->get_logger(), "wrong wayy!");
      }
    }

    prev_d = distance_to_waypoint;
    
    publish_offboard_control_mode();
    publish_trajectory_setpoint(waypoint_x, waypoint_y, waypoint_z);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 2Hz
  }
}

void OffboardPlane::orbit() {
  //publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_ORBIT, 5, 1);
  this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LOITER_TURNS, 5, 0.0, 10);
}

void OffboardPlane::land() {
  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0, 0.0);
  RCLCPP_INFO(this->get_logger(), "Land command send");
}

void OffboardPlane::arm() {
  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
  RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void OffboardPlane::disarm() {
  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
  RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void OffboardPlane::takeoff() {
  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 0.0, 0.0);
  RCLCPP_INFO(this->get_logger(), "Takeoff command send");
}

void OffboardPlane::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
  vehicle_status_ = *msg;
}

void OffboardPlane::vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
  vehicle_odometry_ = *msg;
}


void OffboardPlane::publish_offboard_control_mode() {
  px4_msgs::msg::OffboardControlMode msg{};
  msg.position = true;
  msg.velocity = true;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.thrust_and_torque = false;
  msg.direct_actuator = false;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  offboard_control_mode_publisher_->publish(msg);
}

void OffboardPlane::publish_trajectory_setpoint(float x, float y, float z) {
  px4_msgs::msg::TrajectorySetpoint msg{};
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  msg.position = {x,y,z};
  msg.velocity = {5,5,5};

  // float delta_x = local_waypoints_[waypoint_index_][0] - vehicle_odometry_.position[0];
  // float delta_y = local_waypoints_[waypoint_index_][1] - vehicle_odometry_.position[1];
 
  // float yaw = atan2(delta_y, delta_x);
  // if(yaw == 0) {
  //   msg.yawspeed = 0;
  // } else {
  //   msg.yawspeed = 0.5;
  // }

  // msg.yaw = yaw;

  RCLCPP_INFO(this->get_logger(), "Publishing Trajectory Setpoint: x: %.2f (%.2f), y: %.2f (%.2f), z: %.2f (%.2f), yaw: %.2f, yaw_spd: %.2f", 
              msg.position[0], msg.velocity[0], msg.position[1], msg.velocity[1], msg.position[2], msg.velocity[2], msg.yaw, msg.yawspeed);
  // pub
  trajectory_setpoint_publisher_->publish(msg);
}

void OffboardPlane::publish_trajectory_waypoint(float x, float y, float z) {
  px4_msgs::msg::TrajectoryWaypoint msg{};
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  msg.position = {x,y,z};
  msg.velocity = {5,5,5};

  RCLCPP_INFO(this->get_logger(), "Publishing Trajectory Setpoint: x: %.2f (%.2f), y: %.2f (%.2f), z: %.2f (%.2f)", 
              msg.position[0], msg.velocity[0], msg.position[1], msg.velocity[1], msg.position[2], msg.velocity[2]);
  // pub
  trajectory_waypoint_publisher_->publish(msg);
}


void OffboardPlane::publish_vehicle_command(uint16_t command, float param1, float param2, float param3) {
  px4_msgs::msg::VehicleCommand msg{};
  msg.param1 = param1;
  msg.param2 = param2;
  msg.param3 = param3;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  vehicle_command_publisher_->publish(msg);
}

// void OffboardPlane::publish_vehicle_loiter_command(uint16_t command, float param1, float param2, float param3, float param4, float param5, float param6, float param7) {
//   px4_msgs::msg::VehicleCommand msg{};
//   msg.param1 = param1;
//   msg.param2 = param2;
//   msg.param3 = param3;
//   msg.param4 = param4;
//   msg.param5 = param5;
//   msg.param6 = param6;
//   msg.param7 = param7;
//   msg.command = command;
//   msg.target_system = 1;
//   msg.target_component = 1;
//   msg.source_system = 1;
//   msg.source_component = 1;
//   msg.from_external = true;
//   msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
//   vehicle_command_publisher_->publish(msg);
// }

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::vector<std::vector<float>> local_waypoints = {{100.0, 50, -30.0}, {100, 100, -30}};


  auto node = std::make_shared<OffboardPlane>(local_waypoints);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}