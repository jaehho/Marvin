#include <rclcpp/rclcpp.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <deque>

class RealTimeMarvinMimic : public rclcpp::Node
{
public:
    RealTimeMarvinMimic() : Node("real_time_marvin_mimic")
    {
        // Publisher for MoveIt Servo JointJog commands
        joint_jog_publisher_ = this->create_publisher<control_msgs::msg::JointJog>(
            "/servo_node/delta_joint_cmds", rclcpp::SystemDefaultsQoS());

        // Subscriber to receive real-time joint state updates
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/target_joint_states", 10, std::bind(&RealTimeMarvinMimic::jointCallback, this, std::placeholders::_1));

        // Timer to publish smoothed commands at 20Hz (every 0.05s)
        update_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&RealTimeMarvinMimic::publishSmoothedJointJog, this));

        RCLCPP_INFO(this->get_logger(), "Real-time Marvin Joint Mimic Node Started");
    }

private:
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_jog_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::TimerBase::SharedPtr update_timer_;

    // Joint names from Marvin URDF
    std::vector<std::string> joint_names_ = {
        "joint1_left", "joint2_left", "joint3_left", "joint4_left",
        "joint1_right", "joint2_right", "joint3_right", "joint4_right",
        "gripper_left", "gripper_right"
    };

    // FIFO Queue for smoothing joint states
    std::deque<std::vector<double>> joint_buffer_;
    const int buffer_size_ = 5; // Sliding window size

    // Callback: Stores received joint angles in FIFO queue
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Ensure we have valid joint states
        if (msg->position.size() < joint_names_.size())
            return;

        std::vector<double> joint_positions(msg->position.begin(), msg->position.begin() + joint_names_.size());

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
        if (joint_buffer_.empty()) return;

        std::vector<double> smoothed_joints = computeSmoothedJoints();
        control_msgs::msg::JointJog joint_jog_msg;

        joint_jog_msg.header.stamp = this->now();
        joint_jog_msg.header.frame_id = "world";  // Adjust to Marvin’s base frame
        joint_jog_msg.joint_names = joint_names_;
        joint_jog_msg.velocities.resize(joint_names_.size(), 0.0);
        joint_jog_msg.duration = 0.05;  // 50ms control cycle

        // Compute joint velocities from position difference (Δθ / Δt)
        if (joint_buffer_.size() > 1)
        {
            auto prev_joints = joint_buffer_[joint_buffer_.size() - 2];  // Previous smoothed state
            for (size_t i = 0; i < joint_names_.size(); ++i)
                joint_jog_msg.velocities[i] = (smoothed_joints[i] - prev_joints[i]) / 0.05;
        }

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
