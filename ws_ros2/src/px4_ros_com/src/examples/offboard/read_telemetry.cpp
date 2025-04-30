// Open in another terminal 
// cmd: ros2 run px4_ros_com read_telemetry

#include <cmath>
#include <px4_msgs/msg/vehicle_odometry.hpp>
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

class TelemetryReader : public rclcpp::Node {
public:
  TelemetryReader() : Node("telemetry_reader") {
    
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    qos_profile.keep_last(1);

    vehicle_odometry_subscriber = this->create_subscription<VehicleOdometry>("/fmu/out/vehicle_odometry", qos_profile, std::bind(&TelemetryReader::vehicle_odometry_callback, this, std::placeholders::_1));
    
    std::thread t1(&TelemetryReader::run, this);
    t1.detach();
  }

  void run();

private:
  px4_msgs::msg::VehicleOdometry vehicle_odometry_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_subscriber;
  void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
};


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TelemetryReader>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


void TelemetryReader::run() {

  while (rclcpp::ok()) {
    
    float pos_n = vehicle_odometry_.position[0];
    float pos_e = vehicle_odometry_.position[1];
    float pos_d = vehicle_odometry_.position[2];
    
    RCLCPP_INFO(this->get_logger(), "Odometry data: N: %f, E: %f, D: %f", pos_n, pos_e, pos_d);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 2Hz
  }
}

void TelemetryReader::vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
  vehicle_odometry_ = *msg;
}