#include "math.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>
#include <iostream>
#include <vector>

using namespace std::chrono_literals;

class JointStateSubscriber : public rclcpp::Node {
public:
  JointStateSubscriber() : Node("encoder_ticks") {
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&JointStateSubscriber::jointStatesCallback, this,
                  std::placeholders::_1));
    last_time_ = std::chrono::steady_clock::now();

    // Initialize encoder tick counters to 0
    accumulated_ticks_ = {0, 0, 0, 0};
  }

private:
  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    double wheel_radius = 0.05;
    // encoder ticks per wheel revolution
    int ticks_per_rotation = 360;

    if (msg->velocity.size() == 4) {
      std::vector<double> wheel_velocities = {
          msg->velocity[0], // front_right_wheel_velocity
          msg->velocity[1], // rear_left_wheel_velocity
          msg->velocity[2], // rear_right_wheel_velocity
          msg->velocity[3]  // front_left_wheel_velocity
      };

      auto current_time = std::chrono::steady_clock::now();
      auto elapsed_time =
          std::chrono::duration_cast<std::chrono::duration<double>>(
              current_time - last_time_);
      last_time_ = current_time;

      std::vector<int64_t> encoder_ticks;
      for (size_t i = 0; i < wheel_velocities.size(); ++i) {
        double ticks = (wheel_velocities[i] * elapsed_time.count() /
                        (2 * M_PI * wheel_radius)) *
                       ticks_per_rotation;
        accumulated_ticks_[i] +=
            static_cast<int64_t>(ticks); // Accumulate encoder ticks
        encoder_ticks.push_back(accumulated_ticks_[i]);
      }

      // Use the encoder_ticks as needed
      for (size_t i = 0; i < encoder_ticks.size(); ++i) {
        std::string wheel_name = msg->name[i];
        std::cout << wheel_name << ": " << encoder_ticks[i] << " ticks"
                  << std::endl;
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  std::chrono::time_point<std::chrono::steady_clock> last_time_;
  std::vector<int64_t> accumulated_ticks_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointStateSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
