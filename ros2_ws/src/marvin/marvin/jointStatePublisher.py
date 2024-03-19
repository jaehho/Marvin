import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_publisher')
        self.left_shoulder_flexion_subscription = self.create_subscription(
            Float64,
            'left_shoulder_flexion',
            self.handle_left_shoulder_flexion,
            10)
        # self.right_shoulder_flexion_subscription = self.create_subscription(
        #     Float64,
        #     'right_shoulder_flexion',
        #     self.handle_right_shoulder_flexion,
        #     10)
        self.left_shoulder_adduction_subscription = self.create_subscription(
            Float64,
            'left_shoulder_adduction',
            self.handle_left_shoulder_adduction,
            10)
        # self.right_shoulder_adduction_subscription = self.create_subscription(
        #     Float64,
        #     'right_shoulder_adduction',
        #     self.handle_right_shoulder_adduction,
        #     10)
        self.elbow_subscription = self.create_subscription(
            Float64,
            'elbow_joint',
            self.handle_elbow_joint,
            10)
        
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # Use a dictionary to keep track of the latest joint angles
        self.joint_angles = {'left_shoulder_flexion': 0.0, 
                             'left_shoulder_adduction': 0.0, 
                             'elbow': 0.0}

    def handle_left_shoulder_flexion(self, msg):
        self.joint_angles['left_shoulder_flexion'] = msg.data
        self.publish_joint_state()

    def handle_left_shoulder_adduction(self, msg):
        self.joint_angles['left_shoulder_adduction'] = msg.data
        self.publish_joint_state()

    def handle_elbow_joint(self, msg):
        self.joint_angles['elbow'] = msg.data
        self.publish_joint_state()

    def publish_joint_state(self):
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['joint1','joint2','joint3','joint4','gripper','gripper_sub']
        joint_state_msg.position = [self.joint_angles['left_shoulder_flexion'], 
                                    -self.joint_angles['left_shoulder_adduction'] + np.pi/2, 
                                    self.joint_angles['elbow'] + np.pi/2, 
                                    0.0,0.0,0.0]
        joint_state_msg.velocity = []  # Leave empty if not used
        joint_state_msg.effort = []  # Leave empty if not used

        # Publish the JointState message
        self.joint_state_publisher.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    rclpy.spin(joint_state_publisher)
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()