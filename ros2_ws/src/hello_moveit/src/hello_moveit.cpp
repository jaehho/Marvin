#include <memory>
#include <vector>
#include <string>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joint_state.hpp>

class RobotMover : public rclcpp::Node
{
public:
  RobotMover()
  : Node("robot_mover"), joint_state_received_(false)
  {
    auto topic_callback =
      [this](sensor_msgs::msg::JointState::SharedPtr msg) -> void {
        std::lock_guard<std::mutex> lock(mutex_);
        if (msg->position.empty()) {
          RCLCPP_WARN(this->get_logger(), "Received an empty joint state message. Ignoring...");
          return;
        }
        joint_positions_ = msg->position;
        joint_state_received_ = true;
      };

    subscription_ =
      this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, topic_callback);
  }

  void move_robot()
  {
    using moveit::planning_interface::MoveGroupInterface;

    // Create MoveGroupInterface
    auto move_group_interface = std::make_shared<MoveGroupInterface>(this->shared_from_this(), "marvin");

    // Wait for joint states
    RCLCPP_INFO(this->get_logger(), "Waiting for joint states...");
    while (rclcpp::ok()) {
      rclcpp::spin_some(this->get_node_base_interface());
      if (joint_state_received_) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Prevent busy waiting
    }

    if (!joint_state_received_) {
      RCLCPP_ERROR(this->get_logger(), "No joint state received. Aborting motion.");
      return;
    }

    // Use received joint positions
    {
      std::lock_guard<std::mutex> lock(mutex_);
      move_group_interface->setJointValueTarget(joint_positions_);
    }

    // Plan and execute motion
    auto const [success, plan] = [&]() {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface->plan(msg));
      return std::make_pair(ok, msg);
    }();

    if (success) {
      move_group_interface->execute(plan);
      RCLCPP_INFO(this->get_logger(), "Motion executed successfully.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed!");
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  std::vector<double> joint_positions_;
  bool joint_state_received_;
  std::mutex mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotMover>();
  node->move_robot();
  rclcpp::shutdown();
  return 0;
}
