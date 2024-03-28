import rclpy
import subprocess
from rclpy.node import Node
from custom_interfaces.msg import PoseLandmark
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class PoseLandmarkVisualizer(Node):
    def __init__(self):
        super().__init__('pose_landmark_visualizer')
        self.subscription = self.create_subscription(PoseLandmark, 'pose_landmarks', self.pose_landmark_callback, 1)
        self.pose_landmarks = []  # This will store the latest landmarks

    def pose_landmark_callback(self, msg):
        # Reset the landmarks list with the latest data from the message
        self.pose_landmarks = [{'x': point.x, 'y': point.y, 'z': point.z} for point in msg.point]

def plot_pose_landmarks(node):
    fig = plt.figure()

    # Create subplots
    ax_front = fig.add_subplot(221)  # Front view
    ax_top = fig.add_subplot(222)  # Top view
    ax_side = fig.add_subplot(223)  # Side view
    ax_3d = fig.add_subplot(224, projection='3d')  # 3D view

    def get_primary_monitor_dimensions():
        try:
            # Execute xrandr to get display information
            result = subprocess.run(['xrandr'], stdout=subprocess.PIPE)
            output = result.stdout.decode('utf-8')
            
            # Search for the primary monitor line and extract its resolution
            for line in output.splitlines():
                if "primary" in line:
                    dimensions = line.split()[3].split('+')[0]
                    width, height = map(int, dimensions.split('x'))
                    return width, height
            # Default fallback in case no primary is found
            return 1920, 1080
        except Exception as e:
            print(f"Failed to get monitor dimensions: {e}")
            return 1920, 1080
        
    screen_width, screen_height = get_primary_monitor_dimensions()
    window_width = screen_width // 2
    window_height = screen_height
    window_position_x = 0
    window_position_y = 0
    geometry_str = f"{window_width}x{window_height}+{window_position_x}+{window_position_y}"

    try:
        plt.get_current_fig_manager().window.geometry(geometry_str)
    except Exception as e:
        print(f"Error adjusting window position and size: {e}")

    # Set labels for each subplot
    ax_front.set_title('Front View')
    ax_top.set_title('Top View')
    ax_side.set_title('Side View')
    ax_3d.set_title('3D View')

    # Setting labels for 2D plots
    ax_front.set_xlabel('Y')
    ax_front.set_ylabel('Z')
    ax_top.set_xlabel('Y')
    ax_top.set_ylabel('X')
    ax_side.set_xlabel('X')
    ax_side.set_ylabel('Z')

    # Setting labels for 3D plot
    ax_3d.set_xlabel('X')
    ax_3d.set_ylabel('Y')
    ax_3d.set_zlabel('Z')

    # Set limits for all axes
    default_limits = (-0.5, 0.5)
    z_limits = (-0.5, 1)
    ax_front.set_xlim(default_limits)
    ax_front.set_ylim(z_limits)
    ax_top.set_xlim(default_limits)
    ax_top.set_ylim(default_limits)
    ax_side.set_xlim(default_limits)
    ax_side.set_ylim(z_limits)
    ax_3d.set_xlim3d(default_limits)
    ax_3d.set_ylim3d(default_limits)
    ax_3d.set_zlim3d(z_limits)
    ax_3d.view_init(elev=1, azim=-179) 

    def update(frame):
        rclpy.spin_once(node, timeout_sec=0)  # Check for new messages
        if not node.pose_landmarks:
            return

        # Clear previous points
        ax_front.cla()
        ax_top.cla()
        ax_side.cla()
        ax_3d.cla()
        
        # Set labels for each subplot
        ax_front.set_title('Front View')
        ax_top.set_title('Top View')
        ax_side.set_title('Side View')
        ax_3d.set_title('3D View')

        # Setting labels for 2D plots
        ax_front.set_xlabel('Y')
        ax_front.set_ylabel('Z')
        ax_top.set_xlabel('Y')
        ax_top.set_ylabel('X')
        ax_side.set_xlabel('X')
        ax_side.set_ylabel('Z')

        # Setting labels for 3D plot
        ax_3d.set_xlabel('X')
        ax_3d.set_ylabel('Y')
        ax_3d.set_zlabel('Z')

        # Set limits for all axes
        ax_front.set_xlim(default_limits)
        ax_front.set_ylim(z_limits)
        ax_top.set_xlim(default_limits)
        ax_top.set_ylim(default_limits)
        ax_side.set_xlim(default_limits)
        ax_side.set_ylim(z_limits)
        ax_3d.set_xlim3d(default_limits)
        ax_3d.set_ylim3d(default_limits)
        ax_3d.set_zlim3d(z_limits)

        # Extract coordinates
        ys = [lm['x'] for lm in node.pose_landmarks]
        zs = [-lm['y'] for lm in node.pose_landmarks]
        xs = [lm['z'] for lm in node.pose_landmarks]

        # Plot points in each view
        flip_xs = [-x for x in xs]
        flip_ys = [-y for y in ys]
        ax_front.scatter(flip_ys, zs, c='blue')
        ax_top.scatter(flip_ys, flip_xs, c='red')
        ax_side.scatter(xs, zs, c='green')
        ax_3d.scatter(xs, ys, zs, c='black')

        index_mapping = {
            11: 0, 12: 1, 13: 2, 14: 3,
            15: 4, 16: 5, 23: 6, 24: 7,
        }

        # Original connections list
        original_connections = [(11, 12), (11, 13), (11, 23), (12, 14), (12, 24), (13, 15), (14, 16), (23, 24)]

        # Apply the mapping to the connections
        connections = [(index_mapping[start_idx], index_mapping[end_idx]) for start_idx, end_idx in original_connections]
        for start_idx, end_idx in connections:
            # Front View (use X and Z)
            ax_front.plot([-ys[start_idx], -ys[end_idx]], [zs[start_idx], zs[end_idx]], 'bo-')
            
            # Top View (use X and Y)
            ax_top.plot([-ys[start_idx], -ys[end_idx]], [-xs[start_idx], -xs[end_idx]], 'ro-')
            
            # Side View (use Z and Y)
            ax_side.plot([xs[start_idx], xs[end_idx]], [zs[start_idx], zs[end_idx]], 'go-')
            
            # 3D View
            ax_3d.plot([xs[start_idx], xs[end_idx]], [ys[start_idx], ys[end_idx]], [zs[start_idx], zs[end_idx]], 'ko-')

    ani = FuncAnimation(fig, update, interval=10)  # Update every 10 ms
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
