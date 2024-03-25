import numpy as np
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import PoseLandmark
from std_msgs.msg import Float64
from .vector import vector_from_points, calculate_angle

class ElbowAngleSubscriber(Node):
    def __init__(self):
        super().__init__('elbow_flexion_subscriber')
        self.side = self.declare_parameter('side', 'left').get_parameter_value().string_value

        self.subscription = self.create_subscription(PoseLandmark, 'pose_landmarks', self.listener_callback, 10)
        self.publisher = self.create_publisher(Float64, f'{self.side}_elbow_flexion', 10)

    def listener_callback(self, msg):
        labels, points = msg.label, msg.point

        # Dynamically select indices based on the chosen arm side
        shoulder_idx = labels.index(f'{self.side}_shoulder')
        elbow_idx = labels.index(f'{self.side}_elbow')
        wrist_idx = labels.index(f'{self.side}_wrist')

        # Calculate vectors
        forearm = vector_from_points(points[wrist_idx], points[elbow_idx])
        upper_arm = vector_from_points(points[shoulder_idx], points[elbow_idx])

        # Calculate angle
        elbow_flexion = calculate_angle(upper_arm, forearm)

        # Publish angle
        elbow_flexion_msg = Float64()
        elbow_flexion_msg.data = elbow_flexion
        self.publisher.publish(elbow_flexion_msg)

def main(args=None):
    rclpy.init(args=args)
    elbow_flexion_subscriber = ElbowAngleSubscriber()
    rclpy.spin(elbow_flexion_subscriber)
    elbow_flexion_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
