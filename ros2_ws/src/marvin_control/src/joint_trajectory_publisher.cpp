#include <memory>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>


using namespace std::chrono_literals;

class JointTrajectoryPublisher : public rclcpp::Node
{
public:
  JointTrajectoryPublisher()
  : Node("pose_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/target_joint_trajectory", 10);

    auto timer_callback =
      [this]() -> void {
        // Define alternating joint positions
        std::vector<double> joint_positions1 = {0.0, -100, -9, -46, 0.0, -100, -9, -46};
        std::vector<double> joint_positions2 = {0.0, 90, -6, -81, 0.0, 90, -6, -81};

        std::vector<double> current_positions = (count_ % 2 == 0) ? joint_positions1 : joint_positions2;

        // Create JointTrajectory message
        auto message = trajectory_msgs::msg::JointTrajectory();
        message.header.stamp = this->now();
        message.joint_names = {
          "joint1_left", "joint2_left", "joint3_left", "joint4_left",
          "joint1_right", "joint2_right", "joint3_right", "joint4_right"
        };
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = current_positions;
        point.time_from_start.sec = 1;  // Set time for trajectory execution

        message.points.push_back(point);

        RCLCPP_INFO(this->get_logger(), "Publishing joint positions");
        this->publisher_->publish(message);

        count_++;
      };

    // Publish every 7 seconds
    timer_ = this->create_wall_timer(7s, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointTrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}
