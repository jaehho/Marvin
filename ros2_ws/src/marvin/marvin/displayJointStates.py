import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.joint_positions = [[] for _ in range(12)]  # Adjust this based on the expected number of joints
        self.time_stamps = []

    def listener_callback(self, msg):
        current_time = time.time()  # Use system time for simplicity
        if not self.time_stamps:
            self.start_time = current_time
        self.time_stamps.append(current_time - self.start_time)
        # Keep only the last 10 seconds of data
        while self.time_stamps and (self.time_stamps[-1] - self.time_stamps[0] > 10):
            self.time_stamps.pop(0)
            for positions in self.joint_positions:
                if positions:
                    positions.pop(0)
        for idx, position in enumerate(msg.position):
            if idx < len(self.joint_positions):
                self.joint_positions[idx].append(position)

def plot_joint_positions(node):
    fig, ax = plt.subplots()
    ax.set_title('Joint Positions Over Time (Last 10 seconds)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position')

    def update(frame):
        rclpy.spin_once(node, timeout_sec=0)  # Non-blocking spin to check for new messages
        if not node.time_stamps:
            return
        current_time = node.time_stamps[-1] if node.time_stamps else 0
        ax.set_xlim(max(0, current_time - 10), current_time)
        ax.set_ylim(-2, 2)  # Adjust based on your joint position data range
        ax.clear()
        for idx, positions in enumerate(node.joint_positions):
            if positions:
                ax.plot(node.time_stamps, positions, label=f'Joint {idx}')
        ax.legend(loc='upper right')

    ani = FuncAnimation(fig, update, interval=100)  # Update the plot every 100 milliseconds
    plt.show()

def main(args=None):
    rclpy.init(args=args)
    joint_state_subscriber = JointStateSubscriber()

    try:
        plot_joint_positions(joint_state_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        joint_state_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()