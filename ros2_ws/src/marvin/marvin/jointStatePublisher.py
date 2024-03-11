import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_publisher')
        # Initialize the subscription to the shoulder and elbow joint topics
        self.shoulder_subscription = self.create_subscription(
            Float64,
            'shoulder_joint',
            self.handle_shoulder_joint,
            10)
        self.elbow_subscription = self.create_subscription(
            Float64,
            'elbow_joint',
            self.handle_elbow_joint,
            10)
        
        # Initialize the publisher for the JointState message
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # Use a dictionary to keep track of the latest joint angles
        self.joint_angles = {'shoulder': 0.0, 
                             'elbow': 0.0}

    def handle_shoulder_joint(self, msg):
        # Update the shoulder joint angle
        self.joint_angles['shoulder'] = msg.data
        # Publish the combined joint state
        self.publish_joint_state()

    def handle_elbow_joint(self, msg):
        # Update the elbow joint angle
        self.joint_angles['elbow'] = msg.data
        # Publish the combined joint state
        self.publish_joint_state()

    def publish_joint_state(self):
        # Prepare a JointState message
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['joint1','joint2','joint3','joint4','gripper','gripper_sub']  # Example names
        joint_state_msg.position = [0.0,self.joint_angles['shoulder'], self.joint_angles['elbow'],0.0,0.0,0.0]  # Assuming you calculate these
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