// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointStatePublisher : public rclcpp::Node
{
public:
  JointStatePublisher()
  : Node("joint_state_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Publish joint states immediately
    auto message = sensor_msgs::msg::JointState();
    message.header.stamp = this->now();  // Timestamp the message

    // Define joint names for left and right arms
    message.name = {
      "joint1_left", "joint2_left", "joint3_left", "joint4_left",
      "joint1_right", "joint2_right", "joint3_right", "joint4_right"
    };

    // Manually set joint positions
    message.position = {0.0, -100, -9, -46, 0.0, -100, -9, -46}; // Example positions
    message.velocity = {};  // Optional: leave empty if not used
    message.effort = {};    // Optional: leave empty if not used

    RCLCPP_INFO(this->get_logger(), "Publishing joint states:");
    for (size_t i = 0; i < message.name.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "  %s: position=%f", message.name[i].c_str(), message.position[i]);
    }

    publisher_->publish(message);
    rclcpp::shutdown(); // Shut down after publishing once
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStatePublisher>());
  return 0;
}
