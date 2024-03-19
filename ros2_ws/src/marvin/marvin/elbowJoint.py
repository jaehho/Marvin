import rclpy
from rclpy.node import Node
from custom_interfaces.msg import PoseLandmark
import numpy as np
from std_msgs.msg import Float64  # Import Float64 message type
from .vector import vector_from_points, calculate_angle

class ElbowAngleSubscriber(Node):

    def __init__(self):
        super().__init__('elbow_angle_subscriber')
        self.subscription = self.create_subscription(
            PoseLandmark,
            'pose_landmarks',
            self.listener_callback,
            10)
        # Change publisher to publish Float64 messages on the shoulder_joint topic
        self.elbow_angle_publisher = self.create_publisher(Float64, 'elbow_joint', 10)

    def listener_callback(self, msg):
        labels = msg.label
        points = msg.point

        # Use hardcoded indices for shoulders, elbows, and wrists
        left_shoulder_idx = labels.index('left_shoulder')
        right_shoulder_idx = labels.index('right_shoulder')
        left_elbow_idx = labels.index('left_elbow')
        right_elbow_idx = labels.index('right_elbow')
        left_wrist_idx = labels.index('left_wrist')
        right_wrist_idx = labels.index('right_wrist')

        # Calculate vectors for arms
        left_forearm = vector_from_points(points[left_wrist_idx], points[left_elbow_idx])
        right_forearm = vector_from_points(points[right_wrist_idx], points[right_elbow_idx])
        left_upper_arm = vector_from_points(points[left_shoulder_idx], points[left_elbow_idx])
        right_upper_arm = vector_from_points(points[right_shoulder_idx], points[right_elbow_idx])

        # Calculate angles
        left_elbow_angle = calculate_angle(left_upper_arm, left_forearm)
        right_elbow_angle = calculate_angle(right_upper_arm, right_forearm)

        # Convert angle to radians and publish
        left_elbow_angle_msg = Float64()
        left_elbow_angle_msg.data = left_elbow_angle
        self.elbow_angle_publisher.publish(left_elbow_angle_msg)

def main(args=None):
    rclpy.init(args=args)
    elbow_angle_subscriber = ElbowAngleSubscriber()
    rclpy.spin(elbow_angle_subscriber)
    elbow_angle_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()