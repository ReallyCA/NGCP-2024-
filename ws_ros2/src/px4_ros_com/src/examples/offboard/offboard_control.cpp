// useful references: https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard/offboard_control.cpp, https://github.com/PX4/PX4-Autopilot/issues/22288, https://discuss.px4.io/t/offboard-mode-trajectory-setpoint/32850
// cmd: ros2 run px4_ros_com offboard_control
// https://discuss.px4.io/t/position-trajectory-to-mc-position-control/7376/5

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

private:
 
  std::vector<std::vector<float>> local_waypoints_;
  size_t waypoint_index_ = 0;
  px4_msgs::msg::VehicleStatus vehicle_status_;
  px4_msgs::msg::VehicleOdometry vehicle_odometry_;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleLocalPositionSetpoint>::SharedPtr vehicle_local_position_setpoint_publisher_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subcriber;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_subscriber;

  void publish_offboard_control_mode();
  void publish_trajectory_setpoint(float x, float y, float z);
  void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0);
  void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  std::vector<std::vector<float>> get_loiter_waypoints(float x, float y, float z, float r, int n);
};

void OffboardPlane::run() {

  float prev_d = 9999;
  bool mission_completed = false;
  bool loiter_mode = false;
  bool loiter_calculated = false;
  size_t loiter_index = 0;
  float x, y, z = 0;
  std::vector<std::vector<float>> loiter_waypoints;

  while (rclcpp::ok()) {
    
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_HOME, 1.0);
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

    float pos_n = vehicle_odometry_.position[0];
    float pos_e = vehicle_odometry_.position[1];
    float pos_d = vehicle_odometry_.position[2];

    x = local_waypoints_[waypoint_index_][0];
    y = local_waypoints_[waypoint_index_][1];
    z = local_waypoints_[waypoint_index_][2];

    if(loiter_mode) {
      x = loiter_waypoints[loiter_index][0];
      y = loiter_waypoints[loiter_index][1];
      z = loiter_waypoints[loiter_index][2];
    }

    float dx = x - pos_n;
    float dy = y - pos_e;
    float dz = z - pos_d;
    float distance = sqrt(dx * dx + dy * dy + dz * dz);

    RCLCPP_INFO(this->get_logger(), "Distance to waypoint: %f", distance);

    if(distance < 5.0) {
      if(!loiter_calculated) {
        loiter_waypoints = get_loiter_waypoints(x,y,z, 50, 6);
        loiter_calculated = true;
      }

      if(loiter_mode) {
        if(loiter_index + 1 == loiter_waypoints.size()) {
          loiter_mode = false;
          loiter_calculated = false;
        } else {
          loiter_index++;
        }

      } else {
        if(waypoint_index_ + 1 == local_waypoints_.size()) {
          mission_completed = true;
          RCLCPP_INFO(this->get_logger(), "MISSION COMPLETED!!");
        } else {
          waypoint_index_++;
          loiter_mode = true;
        }
      }
    }

    
    publish_offboard_control_mode();
    publish_trajectory_setpoint(x, y, z);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 2Hz
  }
}

// RCLCPP_INFO(this->get_logger(), "Distance to waypoint: %f, setpoint: %zu", distance, waypoint_index_);
// if(prev_d < distance) {
//   RCLCPP_INFO(this->get_logger(), "wrong wayy!");
// }
// prev_d = distance;

std::vector<std::vector<float>> OffboardPlane::get_loiter_waypoints(float x, float y, float z, float r, int n) {
  std::vector<std::vector<float>> loiter_waypoints;
  const double PI = 3.141592653589793;

  for(int i = 0; i < n; i++) {
    double theta = 2 * PI * i / n;
    float x_n = x + r * cos(theta);
    float y_n = y + r * sin(theta);
    loiter_waypoints.push_back({x_n, y_n, z});
  }

  return loiter_waypoints;
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
  msg.velocity = {NAN, NAN, NAN};

  RCLCPP_INFO(this->get_logger(), "Publishing Trajectory Setpoint: x: %.2f, y: %.2f, z: %.2f", 
              msg.position[0], msg.position[1], msg.position[2]);
  // pub
  trajectory_setpoint_publisher_->publish(msg);
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

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::vector<std::vector<float>> local_waypoints = {{100.0, 50, -30.0}, {-100, -50, -30}};

  auto node = std::make_shared<OffboardPlane>(local_waypoints);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}