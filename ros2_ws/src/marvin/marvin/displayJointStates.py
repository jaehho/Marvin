from curses import window
import rclpy
import  subprocess
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

class JointStateSubscriber(Node):
    def __init__(self, selected_joints=None, display_names=None):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription( JointState, 'joint_states', self.listener_callback, 10)
        self.joint_positions = []
        self.time_stamps = []
        self.joint_names = []
        self.selected_joints = selected_joints or []
        self.display_names = display_names or {}
        self.sides = ['left', 'right']

        self.initialize_joint_names()

    def initialize_joint_names(self):
        robot_joint_names = []
        for side in self.sides:
            robot_joint_names.extend([
                f'{side}_joint1', f'{side}_joint2', f'{side}_joint3',
                f'{side}_joint4', f'{side}_gripper', f'{side}_gripper_sub'
            ])
        if self.selected_joints:
            self.joint_names = [name for name in robot_joint_names if name in self.selected_joints]
        else:
            self.joint_names = robot_joint_names
        self.joint_positions = [[] for _ in self.joint_names]

    def listener_callback(self, msg):
        current_time = time.time()
        if not self.time_stamps:
            self.start_time = current_time
        self.time_stamps.append(current_time - self.start_time)
        
        while self.time_stamps and (self.time_stamps[-1] - self.time_stamps[0] > 10):
            self.time_stamps.pop(0)
            for positions in self.joint_positions:
                if positions:
                    positions.pop(0)
        
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                self.joint_positions[idx].append(position)

def plot_joint_positions(node):
    fig, ax = plt.subplots()


    def get_monitors_dimensions():
        try:
            # Execute xrandr to get display information
            result = subprocess.run(['xrandr'], stdout=subprocess.PIPE, check=True)
            output = result.stdout.decode('utf-8')

            # Initialize variables
            primary_dimensions = None
            secondary_dimensions = None

            # Search through each line of the xrandr output
            for line in output.splitlines():
                # Look for lines indicating a connected monitor and its resolution
                if " connected" in line:
                    dimensions_part = line.split()  # Split each part of the line
                    for part in dimensions_part:
                        # Check if part contains resolution information (e.g., 1920x1080)
                        if 'x' in part and '+' in part:
                            dimensions = part.split('+')[0]  # Extract resolution
                            width, height = map(int, dimensions.split('x'))
                            # Determine if the monitor is primary or secondary based on order of detection
                            if "primary" in line:
                                primary_dimensions = (width, height)
                            elif not secondary_dimensions:  # Assign to secondary if not already assigned
                                secondary_dimensions = (width, height)
                            break  # Break the loop once dimensions are found

            # Set default dimensions if not detected
            if not primary_dimensions:
                primary_dimensions = (1920, 1080)
            if not secondary_dimensions:
                secondary_dimensions = "No secondary monitor detected"

            return primary_dimensions, secondary_dimensions
        except subprocess.CalledProcessError as e:
            print(f"Failed to get monitor dimensions: {e}")
            return (1920, 1080), "No secondary monitor detected"
    
    primary, secondary = get_monitors_dimensions()
    screen_width, screen_height = primary
    window_width = screen_width // 2
    window_height = screen_height // 2
    window_position_x = window_width
    window_position_y = 0
    geometry_str = f"{window_width}x{window_height}+{window_position_x}+{window_position_y}"

    try:
        plt.get_current_fig_manager().window.geometry(geometry_str)
    except Exception as e:
        print(f"Error adjusting window position and size: {e}")

    ax.set_title('Joint Angles Over Time (Last 10 seconds)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angle (rad)')

    def update(frame):
        ax.clear()
        rclpy.spin_once(node, timeout_sec=0)
        if not node.time_stamps:
            return
        current_time = node.time_stamps[-1] if node.time_stamps else 0
        ax.set_xlim(max(0, current_time - 10), current_time)
        
        for idx, positions in enumerate(node.joint_positions):
            if positions:
                original_name = node.joint_names[idx]
                # Use the display name if available, otherwise use the original name
                display_name = node.display_names.get(original_name, original_name)
                color = 'red' if 'right' in original_name else 'blue'
                ax.plot(node.time_stamps, positions, label=display_name, color=color)
        ax.legend(loc='upper left')

    ani = FuncAnimation(fig, update, interval=10)
    plt.show()

def main(args=None):
    rclpy.init(args=args)
    selected_joints = ['left_joint1', 'right_joint1', 'left_joint2', 'right_joint2', 'left_joint3', 'right_joint3', 
                    #    'left_joint4', 'right_joint4', 'left_gripper', 'right_gripper', 'left_gripper_sub', 'right_gripper_sub'
                       ]
    
    display_names = {
        'left_joint1': 'Left Shoulder Flexion',
        'right_joint1': 'Right Shoulder Flexion',
        'left_joint2': 'Left Shoulder Abduction',
        'right_joint2': 'Right Shoulder Abduction',
        'left_joint3': 'Left Elbow Flexion',
        'right_joint3': 'Right Elbow Flexion',
        # 'left_joint4': 'Left Wrist Rotation',
        # 'right_joint4': 'Right Wrist Rotation',
        # 'left_gripper': 'Left Gripper',
        # 'right_gripper': 'Right Gripper',
        # 'left_gripper_sub': 'Left Gripper Sub',
        # 'right_gripper_sub': 'Right Gripper Sub'
    }

    joint_state_subscriber = JointStateSubscriber(selected_joints=selected_joints, display_names=display_names)

    try:
        plot_joint_positions(joint_state_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        joint_state_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
