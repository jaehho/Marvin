import rclpy
from rclpy.node import Node
from custom_interfaces.msg import PoseLandmark
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  # Importing for 3D plotting capabilities

class PoseLandmarkVisualizer(Node):
    def __init__(self):
        super().__init__('pose_landmark_visualizer')
        self.subscription = self.create_subscription(PoseLandmark, 'pose_landmarks', self.pose_landmark_callback, 10)
        self.pose_landmarks = []  # This will store the latest landmarks

    def pose_landmark_callback(self, msg):
        # Reset the landmarks list with the latest data from the message
        self.pose_landmarks = [{'x': point.x, 'y': point.y, 'z': point.z} for point in msg.point]

def plot_pose_landmarks(node):
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.view_init(elev=1, azim=-179) 

    def update(frame):
        rclpy.spin_once(node, timeout_sec=0)  # Non-blocking spin to check for new messages
        if not node.pose_landmarks:
            return
        ax.clear()
        ax.set_xlim3d(-0.5, 0.5)
        ax.set_ylim3d(-0.5, 0.5)
        ax.set_zlim3d(-0.5, 0.5)

        ys = [lm['x'] for lm in node.pose_landmarks]
        zs = [-lm['y'] for lm in node.pose_landmarks]
        xs = [lm['z'] for lm in node.pose_landmarks]
        ax.scatter(xs, ys, zs, c='blue', marker='o')
        
        index_mapping = {
            11: 0, 12: 1, 13: 2, 14: 3,
            15: 4, 16: 5, 23: 6, 24: 7,
        }

        # Original connections list
        original_connections = [(11, 12), (11, 13), (11, 23), (12, 14), (12, 24), (13, 15), (14, 16), (23, 24)]

        # Apply the mapping to the connections
        connections = [(index_mapping[start_idx], index_mapping[end_idx]) for start_idx, end_idx in original_connections]
        for start_idx, end_idx in connections:
            ax.plot([xs[start_idx], xs[end_idx]], [ys[start_idx], ys[end_idx]], 
                            [zs[start_idx], zs[end_idx]], 'ro-')
            
    ani = FuncAnimation(fig, update, interval=100)  # Update the plot every 100 milliseconds
    plt.show()

def main(args=None):
    rclpy.init(args=args)
    pose_landmark_visualizer = PoseLandmarkVisualizer()

    try:
        plot_pose_landmarks(pose_landmark_visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        pose_landmark_visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
