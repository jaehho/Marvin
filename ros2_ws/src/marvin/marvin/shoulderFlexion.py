import numpy as np
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import PoseLandmark
from std_msgs.msg import Float64
from .vector import vector_from_points, calculate_angle, project_vector_onto_plane

class ShoulderFlexionNode(Node):
    def __init__(self):
        super().__init__('shoulder_flexion')
        self.side = self.declare_parameter('side', 'left').get_parameter_value().string_value
        self.opposite_side = 'left' if self.side == 'right' else 'right'

        # Pre-calculate label strings to avoid doing it in every callback
        self.labels = {
            f'{self.side}_shoulder': None,
            f'{self.opposite_side}_shoulder': None,
            f'{self.side}_elbow': None,
            f'{self.opposite_side}_elbow': None,
            f'{self.side}_hip': None,
            f'{self.opposite_side}_hip': None
        }

        self.subscription = self.create_subscription(PoseLandmark, 'pose_landmarks', self.listener_callback, 10)
        self.publisher = self.create_publisher(Float64, f'{self.side}_shoulder_flexion', 10)

    def listener_callback(self, msg):
        labels, points = msg.label, msg.point
        
        # Update indexes if they have not been set
        if not self.labels[f'{self.side}_shoulder']:
            self.update_indexes(labels)

        # Calculate vectors and angles based on pre-determined labels
        upper_arm = vector_from_points(points[self.labels[f'{self.side}_shoulder']], points[self.labels[f'{self.side}_elbow']])
        shoulder_to_hip = vector_from_points(points[self.labels[f'{self.side}_shoulder']], points[self.labels[f'{self.side}_hip']])
        shoulder_to_shoulder = vector_from_points(points[self.labels[f'{self.side}_shoulder']], points[self.labels[f'{self.opposite_side}_shoulder']])

        # Project vectors onto the shoulder_to_shoulder plane
        projected_upper_arm = project_vector_onto_plane(upper_arm, shoulder_to_shoulder)
        projected_shoulder_to_hip = project_vector_onto_plane(shoulder_to_hip, shoulder_to_shoulder)

        flexion = calculate_angle(projected_upper_arm, projected_shoulder_to_hip)

        self.publish_flexion(flexion)

    def update_indexes(self, labels):
        for label in self.labels.keys():
            self.labels[label] = labels.index(label)

    def publish_flexion(self, angle):
        flexion_msg = Float64()
        flexion_msg.data = angle
        self.publisher.publish(flexion_msg)

def main():
    rclpy.init()
    node = ShoulderFlexionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
