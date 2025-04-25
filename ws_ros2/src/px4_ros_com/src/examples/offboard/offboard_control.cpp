// useful references: https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard/offboard_control.cpp, https://github.com/PX4/PX4-Autopilot/issues/22288, https://discuss.px4.io/t/offboard-mode-trajectory-setpoint/32850
// cmd: ros2 run px4_ros_com offboard_control
// https://discuss.px4.io/t/position-trajectory-to-mc-position-control/7376/5

#include <cmath>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
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

// MISSION PARAMETERS
const int OFFBOARD_LOITER_TIME = 10; // adjust loiter time in seconds
const std::vector<std::vector<float>> MISSION_SETPOINTS = {{100.0, 50, -30.0}, {-100, -50, -30}};


// DON'T TOUCH
class OffboardPlane : public rclcpp::Node {
public:
  OffboardPlane(const std::vector<std::vector<float>>& local_setpoints) : Node("offboard_plane"), local_setpoints_(local_setpoints) {
    
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    qos_profile.keep_last(1);

    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    vehicle_status_subcriber = this->create_subscription<VehicleStatus>("/fmu/out/vehicle_status", qos_profile, std::bind(&OffboardPlane::vehicle_status_callback, this, std::placeholders::_1));
    vehicle_odometry_subscriber = this->create_subscription<VehicleOdometry>("/fmu/out/vehicle_odometry", qos_profile, std::bind(&OffboardPlane::vehicle_odometry_callback, this, std::placeholders::_1));
    
    std::thread t1(&OffboardPlane::run, this);
    t1.detach();
  }

  void run();

private:
  std::vector<std::vector<float>> local_setpoints_;
  size_t waypoint_index_ = 0;
  px4_msgs::msg::VehicleStatus vehicle_status_;
  px4_msgs::msg::VehicleOdometry vehicle_odometry_;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subcriber;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_subscriber;

  void publish_offboard_control_mode();
  void publish_trajectory_setpoint(float x, float y, float z);
  void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0);
  void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
};


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<OffboardPlane>(MISSION_SETPOINTS);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


// MEMBER FUNCTIONS DEFINITION
void OffboardPlane::run() {

  using clock = std::chrono::steady_clock;
  clock::time_point loiter_start_time;
  bool mission_completed = false;
  bool loiter_mode = false;
  const std::chrono::seconds loiter_duration(OFFBOARD_LOITER_TIME);

  while (rclcpp::ok() && !mission_completed) {
    
    float pos_n = vehicle_odometry_.position[0];
    float pos_e = vehicle_odometry_.position[1];
    float pos_d = vehicle_odometry_.position[2];

    float x = local_setpoints_[waypoint_index_][0];
    float y = local_setpoints_[waypoint_index_][1];
    float z = local_setpoints_[waypoint_index_][2];

    float dx = x - pos_n;
    float dy = y - pos_e;
    float dz = z - pos_d;
    float distance = sqrt(dx * dx + dy * dy + dz * dz);

    if(!loiter_mode) {
      RCLCPP_INFO(this->get_logger(), "Distance to waypoint: %f", distance);
      if (distance < 10.0) {
        loiter_mode = true;
        loiter_start_time = clock::now();
      }
    }

    if(loiter_mode) {
      auto now = clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - loiter_start_time);
      auto remaining = loiter_duration - elapsed;

      RCLCPP_INFO(this->get_logger(), "Loiter mode, time remaining: %ld s", remaining.count());

      if(now - loiter_start_time > loiter_duration) {
        loiter_mode = false;
        if(waypoint_index_ + 1 == local_setpoints_.size()) {
          publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 3); // SWITCH TO NATIVE HOLD/ LOITER MODE
          RCLCPP_INFO(this->get_logger(), "Mission completed, switching to Hold mode...");
          mission_completed = true;
          return;
        }
        waypoint_index_++;
      }
    }

    publish_offboard_control_mode();
    if(vehicle_status_.nav_state == vehicle_status_.NAVIGATION_STATE_OFFBOARD) {
      publish_trajectory_setpoint(x, y, z);
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 2Hz
  }
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



// NOT USED
// void OffboardPlane::land() {
//   publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0, 0.0);
//   RCLCPP_INFO(this->get_logger(), "Land command send");
// }

// void OffboardPlane::arm() {
//   publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
//   RCLCPP_INFO(this->get_logger(), "Arm command send");
// }

// void OffboardPlane::disarm() {
//   publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
//   RCLCPP_INFO(this->get_logger(), "Disarm command send");
// }

// void OffboardPlane::takeoff() {
//   publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 0.0, 0.0);
//   RCLCPP_INFO(this->get_logger(), "Takeoff command send");
// }
