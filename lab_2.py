import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
import numpy as np

class ForwardKinematics(Node):

    def __init__(self):
        super().__init__('forward_kinematics')
        self.joint_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.joint_subscription  # prevent unused variable warning

        self.position_publisher = self.create_publisher(
            Float64MultiArray,
            'leg_front_r_end_effector_position',
            10)

        self.marker_publisher = self.create_publisher(
            Marker,
            'marker',
            10)

        timer_period = 0.1  # publish FK information and marker at 10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.joint_positions = None

    def listener_callback(self, msg):
        # Extract the positions of the joints related to leg_front_r
        joints_of_interest = ['leg_front_r_1', 'leg_front_r_2', 'leg_front_r_3']
        self.joint_positions = [msg.position[msg.name.index(joint)] for joint in joints_of_interest]

    def forward_kinematics(self, theta1, theta2, theta3):

        def rotation_x(angle):
            # rotation about the x-axis implemented for you
            return np.array([
                [1, 0, 0, 0],
                [0, np.cos(angle), -np.sin(angle), 0],
                [0, np.sin(angle), np.cos(angle), 0],
                [0, 0, 0, 1]
            ])

        def rotation_y(angle):
            return np.array([
                [np.cos(angle), 0, np.sin(angle), 0],
                [0, 1, 0, 0],
                [-np.sin(angle), 0, np.cos(angle), 0],
                [0, 0, 0, 1]
            ])
        
        def rotation_z(angle):
            return np.array([
                [np.cos(angle), -np.sin(angle), 0, 0],
                [np.sin(angle), np.cos(angle), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])

        def translation(x, y, z):
            return np.array([
                [1, 0, 0, x],
                [0, 1, 0, y],
                [0, 0, 1, z],
                [0, 0, 0, 1]
            ])

        # T_0_1 (base_link to leg_front_r_1)
        T_0_1 = translation(0.07500, -0.0445, 0) @ rotation_x(1.57080) @ rotation_z(theta1)

        # T_1_2 (leg_front_r_1 to leg_front_r_2)
        ## TODO: Implement the transformation matrix from leg_front_r_1 to leg_front_r_2
        T_1_2 = translation(0, 0, 0.039) @ rotation_y(-1.57080) @ rotation_z(theta2)

        # T_2_3 (leg_front_r_2 to leg_front_r_3)
        ## TODO: Implement the transformation matrix from leg_front_r_2 to leg_front_r_3
        T_2_3 = translation(0, -0.0494, 0.0685) @ rotation_y(1.57080) @ rotation_z(theta3)

        # T_3_ee (leg_front_r_3 to end-effector)
        T_3_ee = translation(0.06231, -0.06216, 0.018)

        # TODO: Compute the final transformation. T_0_ee is a concatenation of the previous transformation matrices
        T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee

        # TODO: Extract the end-effector position. The end effector position is a 3x3 matrix (not in homogenous coordinates)
        end_effector_position = T_0_ee[:3,3]

        return end_effector_position


    def timer_callback(self):
        if self.joint_positions is not None:
            # Joint angles
            theta1 = self.joint_positions[0]
            theta2 = self.joint_positions[1]
            theta3 = self.joint_positions[2]

            end_effector_position = self.forward_kinematics(theta1, theta2, theta3)

            marker = Marker()
            marker.header.frame_id = '/base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = marker.SPHERE
            marker.id = 0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.pose.position.x = end_effector_position[0]
            marker.pose.position.y = end_effector_position[1]
            marker.pose.position.z = end_effector_position[2]
            self.marker_publisher.publish(marker)

            position = Float64MultiArray()
            position.data = end_effector_position
            self.position_publisher.publish(position)
            # self.get_logger().info(f'theta1 = {theta1:.1f}, theta2 = {theta2:.1f}, theta3 = {theta3:.1f}')
            self.get_logger().info(f'End-Effector Position: x={end_effector_position[0]:.2f}, y={end_effector_position[1]:.2f}, z={end_effector_position[2]:.2f}')

def main(args=None):
    rclpy.init(args=args)

    forward_kinematics = ForwardKinematics()

    rclpy.spin(forward_kinematics)

    forward_kinematics.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()