import rclpy
from rclpy.node import Node
from custom_interfaces.msg import PoseLandmark
import numpy as np
from std_msgs.msg import Float64  # Import Float64 message type
from .vector import vector_from_points, calculate_angle

class ShoulderAngleSubscriber(Node):

    def __init__(self):
        super().__init__('shoulder_angle_subscriber')
        self.subscription = self.create_subscription(
            PoseLandmark,
            'pose_landmarks',
            self.listener_callback,
            10)
        # Change publisher to publish Float64 messages on the shoulder_joint topic
        self.shoulder_angle_publisher = self.create_publisher(Float64, 'shoulder_joint', 10)

    def listener_callback(self, msg):
        labels = msg.label
        points = msg.point

        # Find indices for shoulders, elbows, and hips
        left_shoulder_idx = labels.index('left_shoulder')
        right_shoulder_idx = labels.index('right_shoulder')
        left_elbow_idx = labels.index('left_elbow')
        right_elbow_idx = labels.index('right_elbow')
        left_hip_idx = labels.index('left_hip')
        right_hip_idx = labels.index('right_hip')

        # Calculate vectors
        left_upper_arm = vector_from_points(points[left_shoulder_idx], points[left_elbow_idx])
        right_upper_arm = vector_from_points(points[right_shoulder_idx], points[right_elbow_idx])
        left_shoulder_to_hip = vector_from_points(points[left_shoulder_idx], points[left_hip_idx])
        right_shoulder_to_hip = vector_from_points(points[right_shoulder_idx], points[right_hip_idx])

        # Calculate angles using the shoulder-to-hip vector as reference
        left_shoulder_angle = -(calculate_angle(left_upper_arm, left_shoulder_to_hip) - 90)
        right_shoulder_angle = calculate_angle(right_upper_arm, right_shoulder_to_hip)

        # Convert angle to radians and publish
        left_shoulder_angle_msg = Float64()
        left_shoulder_angle_msg.data = np.deg2rad(left_shoulder_angle)
        self.shoulder_angle_publisher.publish(left_shoulder_angle_msg)

def main(args=None):
    print('running')
    rclpy.init(args=args)
    shoulder_angle_subscriber = ShoulderAngleSubscriber()
    rclpy.spin(shoulder_angle_subscriber)
    shoulder_angle_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()