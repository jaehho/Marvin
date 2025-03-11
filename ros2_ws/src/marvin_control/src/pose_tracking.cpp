/*#include <rclcpp/rclcpp.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <deque>

class RealTimeMarvinMimic : public rclcpp::Node
{
public:
    RealTimeMarvinMimic() : Node("real_time_marvin_mimic")
    {
        // Publisher for MoveIt Servo JointJog commands
        joint_jog_publisher_ = this->create_publisher<control_msgs::msg::JointJog>(
            "/servo_node/delta_joint_cmds", rclcpp::SystemDefaultsQoS());

        // Subscriber to receive real-time joint trajectory updates
        joint_trajectory_subscriber_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/target_joint_trajectory", 10, std::bind(&RealTimeMarvinMimic::jointCallback, this, std::placeholders::_1));

        // Timer to publish smoothed commands at 20Hz (every 0.05s)
        update_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&RealTimeMarvinMimic::publishSmoothedJointJog, this));

        RCLCPP_INFO(this->get_logger(), "Real-time Marvin Joint Mimic Node Started");
    }

private:
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_jog_publisher_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_subscriber_;
    rclcpp::TimerBase::SharedPtr update_timer_;

    // Joint names from Marvin URDF
    std::vector<std::string> joint_names_ = {
        "joint1_left", "joint2_left", "joint3_left", "joint4_left",
        "joint1_right", "joint2_right", "joint3_right", "joint4_right"
    };

    // FIFO Queue for smoothing joint trajectory
    std::deque<std::vector<double>> joint_buffer_;
    const int buffer_size_ = 5; // Sliding window size

    // Callback: Stores received joint angles in FIFO queue
    void jointCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        // Ensure we have valid joint trajectory
        if (msg->points.size() < joint_names_.size())
            return;

        std::vector<double> joint_positions(msg->points.begin()->positions.begin(), msg->points.begin()->positions.begin() + joint_names_.size());

        RCLCPP_INFO(this->get_logger(), "Received Joint Trajectory with %lu points", msg->points.size());

        if (msg->points.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty trajectory! Ignoring.");
            return;
        }

        std::vector<double> joint_positions = msg->points[0].positions;

        if (joint_positions.size() != joint_names_.size())
        {
            RCLCPP_WARN(this->get_logger(), "Joint positions size mismatch! Expected %lu, got %lu", 
                    joint_names_.size(), joint_positions.size());
            return;
        }

        // Maintain FIFO buffer (sliding window)
        if (joint_buffer_.size() >= buffer_size_)
            joint_buffer_.pop_front();
        joint_buffer_.push_back(joint_positions);

    }

    // Compute smoothed joint values
    std::vector<double> computeSmoothedJoints()
    {
        if (joint_buffer_.empty())
            return std::vector<double>(joint_names_.size(), 0.0);

        std::vector<double> avg_joint_positions(joint_names_.size(), 0.0);
        
        // Compute mean joint position over the buffer
        for (const auto &joints : joint_buffer_)
            for (size_t i = 0; i < joint_names_.size(); ++i)
                avg_joint_positions[i] += joints[i];

        for (size_t i = 0; i < joint_names_.size(); ++i)
            avg_joint_positions[i] /= joint_buffer_.size();

        return avg_joint_positions;
    }

    // Publishes smoothed JointJog commands
    void publishSmoothedJointJog()
    {
        if (joint_buffer_.empty()) 
        {
            RCLCPP_WARN(this->get_logger(), "Joint buffer is empty! Cannot publish.");
            return;
        }

        std::vector<double> smoothed_joints = computeSmoothedJoints();
        control_msgs::msg::JointJog joint_jog_msg;

        joint_jog_msg.header.stamp = this->now();
        joint_jog_msg.header.frame_id = "world";  
        joint_jog_msg.joint_names = joint_names_;
        joint_jog_msg.velocities.resize(joint_names_.size(), 0.0); // Must use velocities!
        joint_jog_msg.duration = 0.05;  // Control loop frequency

        if (joint_buffer_.size() > 1)
        {
            auto prev_joints = joint_buffer_[joint_buffer_.size() - 2];  
            for (size_t i = 0; i < joint_names_.size(); ++i)
            {
                joint_jog_msg.velocities[i] = (smoothed_joints[i] - prev_joints[i]) / 0.05;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Publishing JointJog velocity: First Joint Velocity: %f", 
                    joint_jog_msg.velocities[0]);

        joint_jog_publisher_->publish(joint_jog_msg);
    }
};

// Main function
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RealTimeMarvinMimic>());
    rclcpp::shutdown();
    return 0;
}
*/ 

/*#include <rclcpp/rclcpp.hpp>
#include <control_msgs/msg/joint_jog.hpp>

class TestVelocityPublisher : public rclcpp::Node {
public:
    TestVelocityPublisher() : Node("test_velocity_publisher") {
        joint_jog_publisher_ = this->create_publisher<control_msgs::msg::JointJog>(
            "/servo_node/delta_joint_cmds", 10);

        // Publish at 10Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&TestVelocityPublisher::publish_velocity_command, this));

        RCLCPP_INFO(this->get_logger(), "Velocity command publisher started!");
    }

private:
    void publish_velocity_command() {
        auto msg = control_msgs::msg::JointJog();
        msg.header.stamp = this->now();
        msg.header.frame_id = "world";

        // Define joint names
        msg.joint_names = {
            "joint1_left", "joint2_left", "joint3_left", "joint4_left",
            "joint1_right", "joint2_right", "joint3_right", "joint4_right"
        };

        // Define a small velocity to test
        msg.velocities = {0.1, -0.1, 0.2, -0.2, 0.1, -0.1, 0.2, -0.2};
        msg.duration = 0.1;  // Duration in seconds

        RCLCPP_INFO(this->get_logger(), "Publishing JointJog velocity: First Joint Velocity: %f", msg.velocities[0]);
        
        joint_jog_publisher_->publish(msg);
    }

    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_jog_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestVelocityPublisher>());
    rclcpp::shutdown();
    return 0;
}
*/
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <moveit_servo/utils/common.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("servo_tutorial");

  // Create a publisher to send joint jog commands
  auto joint_jog_publisher = node->create_publisher<control_msgs::msg::JointJog>("/servo_node/delta_joint_cmds", rclcpp::SystemDefaultsQoS());

  // Create a client to switch input type
  auto switch_input_client = node->create_client<moveit_msgs::srv::ServoCommandType>("/servo_node/switch_command_type");
  
  //idk what this does but it is in the Moveit tutorial and removing it causes timeout error below w command switch
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  std::thread executor_thread([&executor]() { executor->spin(); });
  
  // Create the ROS message.
  auto request = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
  request->command_type = moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG;
  auto response_future = switch_input_client->async_send_request(request);
  if (response_future.wait_for(std::chrono::duration<double>(3.0)) == std::future_status::timeout)
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Timed out waiting for MoveIt servo command switching request.");
  }
  const auto response = response_future.get();
  if (response.get()->success)
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Switched to command input type: Joint Jog");
  }
  else
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Could not switch MoveIt servo command input type.");
  }


  // Define joint jog command
  control_msgs::msg::JointJog joint_jog_msg;
  joint_jog_msg.header.stamp = node->now();
  joint_jog_msg.header.frame_id = "torso";  // Update with your robot's base frame
  joint_jog_msg.joint_names = {"joint1_left", "joint2_left", "joint3_left", "joint4_left",
            "joint1_right", "joint2_right", "joint3_right", "joint4_right"};   // Modify with your robot's joint names
  joint_jog_msg.velocities = {0.1, -0.1, 0.2, -0.2, 0.1, -0.1, 0.2, -0.2};               // Joint velocities in rad/s
  joint_jog_msg.duration = 0.1;  // Use a double instead of rclcpp::Duration

  // Publish joint jog commands in a loop
  rclcpp::WallRate rate(10);  // 10 Hz
  for (int i = 0; i < 50; ++i) // Send 50 messages (~5 seconds of motion)
  {
    joint_jog_msg.header.stamp = node->now();
    joint_jog_publisher->publish(joint_jog_msg);
    rate.sleep();
  }

  executor->cancel();
  if (executor_thread.joinable())
  {
    executor_thread.join();
  }

  rclcpp::shutdown();
}

