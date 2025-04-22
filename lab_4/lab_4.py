import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
np.set_printoptions(precision=3, suppress=True)

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

class InverseKinematics(Node):

    def __init__(self):
        super().__init__('inverse_kinematics')
        self.joint_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.joint_subscription  # prevent unused variable warning

        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_command_controller/commands',
            10
        )

        self.joint_positions = None
        self.joint_velocities = None
        self.target_joint_positions = None
        self.counter = 0

        # Trotting gate positions, already implemented
        # touch_down_position = np.array([0.05, 0.0, -0.14])
        # stand_position_1 = np.array([0.025, 0.0, -0.14])
        # stand_position_2 = np.array([0.0, 0.0, -0.14])
        # stand_position_3 = np.array([-0.025, 0.0, -0.14])
        # liftoff_position = np.array([-0.05, 0.0, -0.14])
        # mid_swing_position = np.array([0.0, 0.0, -0.05])

        # Fast
        # touch_down_position = np.array([0.1, 0.0, -0.14])
        # stand_position_1 = np.array([0.05, 0.0, -0.14])
        # stand_position_2 = np.array([0.0, 0.0, -0.14])
        # stand_position_3 = np.array([-0.05, 0.0, -0.14])
        # liftoff_position = np.array([-0.1, 0.0, -0.14])
        # mid_swing_position = np.array([0.0, 0.0, -0.05])

        # Slow
        touch_down_position = np.array([0.005, 0.0, -0.14])
        stand_position_1 = np.array([0.002, 0.0, -0.14])
        stand_position_2 = np.array([0.0, 0.0, -0.14])
        stand_position_3 = np.array([-0.002, 0.0, -0.14])
        liftoff_position = np.array([-0.005, 0.0, -0.14])
        mid_swing_position = np.array([0.0, 0.0, -0.05])

        # Diamond
        # touch_down_position = np.array([0.05, 0.0, -0.14])
        # stand_position_1 = np.array([0.025, 0.0, -0.14])
        # stand_position_2 = np.array([0.0, 0.0, -0.14])
        # stand_position_3 = np.array([-0.025, 0.0, -0.14])
        # liftoff_position = np.array([-0.05, 0.0, -0.14])
        # mid_swing_position = np.array([0.0, 0.0, -0.05])
        
        ## trotting
        # TODO: Implement each leg’s trajectory in the trotting gait.
        rf_ee_offset = np.array([0.06, -0.09, 0])
        rf_ee_triangle_positions = np.array([
        stand_position_1, stand_position_2, stand_position_3, liftoff_position,mid_swing_position,touch_down_position
        ]) + rf_ee_offset
        
        lf_ee_offset = np.array([0.06, 0.09, 0])
        lf_ee_triangle_positions = np.array([
            liftoff_position,mid_swing_position,touch_down_position, stand_position_1,stand_position_2,stand_position_3
        ]) + lf_ee_offset
        
        rb_ee_offset = np.array([-0.11, -0.09, 0])
        rb_ee_triangle_positions = np.array([
            liftoff_position,mid_swing_position,touch_down_position, stand_position_1,stand_position_2,stand_position_3

        ]) + rb_ee_offset
        
        lb_ee_offset = np.array([-0.11, 0.09, 0])
        lb_ee_triangle_positions = np.array([
            stand_position_1, stand_position_2, stand_position_3, liftoff_position,mid_swing_position,touch_down_position
            
        ]) + lb_ee_offset


        self.ee_triangle_positions = [rf_ee_triangle_positions, lf_ee_triangle_positions, rb_ee_triangle_positions, lb_ee_triangle_positions]
        self.fk_functions = [self.fr_leg_fk, self.fl_leg_fk, self.br_leg_fk, self.bl_leg_fk]

        self.target_joint_positions_cache, self.target_ee_cache = self.cache_target_joint_positions()
        print(f'shape of target_joint_positions_cache: {self.target_joint_positions_cache.shape}')
        print(f'shape of target_ee_cache: {self.target_ee_cache.shape}')


        self.pd_timer_period = 1.0 / 200  # 200 Hz
        self.ik_timer_period = 1.0 / 100   # 10 Hz
        self.pd_timer = self.create_timer(self.pd_timer_period, self.pd_timer_callback)
        self.ik_timer = self.create_timer(self.ik_timer_period, self.ik_timer_callback)


    def fr_leg_fk(self, theta):
        # Already implemented in Lab 2
        T_RF_0_1 = translation(0.07500, -0.08350, 0) @ rotation_x(1.57080) @ rotation_z(theta[0])
        T_RF_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
        T_RF_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta[2])
        T_RF_3_ee = translation(0.06231, -0.06216, 0.01800)
        T_RF_0_ee = T_RF_0_1 @ T_RF_1_2 @ T_RF_2_3 @ T_RF_3_ee
        return T_RF_0_ee[:3, 3]

    def fl_leg_fk(self, theta):
        ################################################################################################
        # TODO: implement forward kinematics here
        ################################################################################################
        T_LF_0_1 = translation(0.07500, 0.08350, 0) @ rotation_x(1.57080) @ rotation_z(-theta[0])
        T_LF_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
        T_LF_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(-theta[2]) # could be wrong
        T_LF_3_ee = translation(0.06231, -0.06216, -0.01800)
        T_LF_0_ee = T_LF_0_1 @ T_LF_1_2 @ T_LF_2_3 @ T_LF_3_ee
        return T_LF_0_ee[:3, 3]

    def br_leg_fk(self, theta):
        ################################################################################################
        # TODO: implement forward kinematics here
        ################################################################################################
        T_RB_0_1 = translation(-0.07500, -0.07250, 0) @ rotation_x(1.57080) @ rotation_z(theta[0])
        T_RB_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
        T_RB_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta[2])
        T_RB_3_ee = translation(0.06231, -0.06216, 0.01800)
        T_RB_0_ee = T_RB_0_1 @ T_RB_1_2 @ T_RB_2_3 @ T_RB_3_ee
        return T_RB_0_ee[:3, 3]

    def bl_leg_fk(self, theta):
        ################################################################################################
        # TODO: implement forward kinematics here
        ################################################################################################
        T_LB_0_1 = translation(-0.07500, 0.072500, 0) @ rotation_x(1.57080) @ rotation_z(-theta[0])
        T_LB_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
        T_LB_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(-theta[2]) # could be wrong
        T_LB_3_ee = translation(0.06231, -0.06216, -0.01800)
        T_LB_0_ee = T_LB_0_1 @ T_LB_1_2 @ T_LB_2_3 @ T_LB_3_ee
        return T_LB_0_ee[:3, 3]

    def forward_kinematics(self, theta):
        return np.concatenate([self.fk_functions[i](theta[3*i: 3*i+3]) for i in range(4)])

    def listener_callback(self, msg):
        joints_of_interest = [
            'leg_front_r_1', 'leg_front_r_2', 'leg_front_r_3', 
            'leg_front_l_1', 'leg_front_l_2', 'leg_front_l_3', 
            'leg_back_r_1', 'leg_back_r_2', 'leg_back_r_3', 
            'leg_back_l_1', 'leg_back_l_2', 'leg_back_l_3'
        ]
        self.joint_positions = np.array([msg.position[msg.name.index(joint)] for joint in joints_of_interest])
        self.joint_velocities = np.array([msg.velocity[msg.name.index(joint)] for joint in joints_of_interest])

    def inverse_kinematics_single_leg(self, target_ee, leg_index, initial_guess=[0, 0, 0]):
        leg_forward_kinematics = self.fk_functions[leg_index]

        def cost_function(theta):
            cur_ee = leg_forward_kinematics(theta)
            ################################################################################################
            # TODO: [already done] paste lab 3 inverse kinematics here
            ################################################################################################
            # Use the forward_kinematics method to get the current end-effector position.

            # Calculate the L1 distance between the current and target end-effector positions.
            L1_error = [np.abs(cur_ee[0] - target_ee[0]), np.abs(cur_ee[1] - target_ee[1]), np.abs(cur_ee[2] - target_ee[2])]

            # Return the sum of squared L1 distances as the cost (AKA the squared L2 norm of the error vector).
            L2_cost = L1_error[0] ** 2 + L1_error[1] ** 2 + L1_error[2] ** 2

            return L2_cost, np.array(L1_error)
            
        def gradient(theta, epsilon=1e-3):
            grad = np.zeros(3)
            ################################################################################################
            # TODO: [already done] paste lab 3 inverse kinematics here
            ################################################################################################
            fq = cost_function(theta)[0]

            d_1 = ( cost_function(theta + np.array([epsilon, 0, 0]))[0] - fq ) / epsilon

            d_2 = ( cost_function(theta + np.array([0, epsilon, 0]))[0] - fq ) / epsilon

            d_3 = ( cost_function(theta + np.array([0, 0, epsilon]))[0] - fq ) / epsilon

            return np.array([d_1, d_2, d_3])

        theta = np.array(initial_guess)
        learning_rate = 10 # TODO:[already done] paste lab 3 inverse kinematics here
        max_iterations = 50 # TODO: [already done] paste lab 3 inverse kinematics here
        tolerance = 0.001 # TODO: [already done] paste lab 3 inverse kinematics here

        cost_l = []
        for _ in range(max_iterations):
            ################################################################################################
            # TODO: [already done] paste lab 3 inverse kinematics here
            ################################################################################################
            grad = gradient(theta)
            if np.mean(cost_function(theta)[1]) < tolerance:
                print("hi")
                break
            theta = theta - learning_rate * grad 

        return theta

    def interpolate_triangle(self, t, leg_index):
        ################################################################################################
        # TODO: implement interpolation for all 4 legs here
        ################################################################################################
        # print(len(self.ee_triangle_positions[leg_index]))
        state = (t*6) % 6
        i = int(state)  
        alpha = state - i 

        p0 = self.ee_triangle_positions[leg_index][i]
        p1 = self.ee_triangle_positions[leg_index][(i+1) % 6]

        # if leg_index == 0: #front right
        #     p0 = self.rf_ee_triangle_positions[i]
        #     p1 = self.rf_ee_triangle_positions[(i + 1) % 6]
        # elif leg_index == 1: #front left
        #     p0 = self.lf_ee_triangle_positions[i]
        #     p1 = self.lf_ee_triangle_positions[(i + 1) % 6]
        # elif leg_index == 2: #back right
        #     p0 = self.rb_ee_triangle_positions[i]
        #     p1 = self.rb_ee_triangle_positions[(i + 1) % 6]
        # elif leg_index == 3: #back left
        #     p0 = self.lb_ee_triangle_positions[i]
        #     p1 = self.lb_ee_triangle_positions[(i + 1) % 6]
        return (1 - alpha) * p0 + alpha * p1


    def cache_target_joint_positions(self):
        # Calculate and store the target joint positions for a cycle and all 4 legs
        target_joint_positions_cache = []
        target_ee_cache = []
        for leg_index in range(4):
            target_joint_positions_cache.append([])
            target_ee_cache.append([])
            target_joint_positions = [0] * 3
            for t in np.arange(0, 1, 0.02):
                print(t)
                target_ee = self.interpolate_triangle(t, leg_index)
                target_joint_positions = self.inverse_kinematics_single_leg(target_ee, leg_index, initial_guess=target_joint_positions)

                target_joint_positions_cache[leg_index].append(target_joint_positions)
                target_ee_cache[leg_index].append(target_ee)

        # (4, 50, 3) -> (50, 12)
        target_joint_positions_cache = np.concatenate(target_joint_positions_cache, axis=1)
        target_ee_cache = np.concatenate(target_ee_cache, axis=1)
        
        return target_joint_positions_cache, target_ee_cache

    def get_target_joint_positions(self):
        target_joint_positions = self.target_joint_positions_cache[self.counter]
        target_ee = self.target_ee_cache[self.counter]
        self.counter += 1
        if self.counter >= self.target_joint_positions_cache.shape[0]:
            self.counter = 0
        return target_ee, target_joint_positions

    def ik_timer_callback(self):
        if self.joint_positions is not None:
            target_ee, self.target_joint_positions = self.get_target_joint_positions()
            current_ee = self.forward_kinematics(self.joint_positions)

            self.get_logger().info(
                f'Target EE: {target_ee}, \
                Current EE: {current_ee}, \
                Target Angles: {self.target_joint_positions}, \
                Target Angles to EE: {self.forward_kinematics(self.target_joint_positions)}, \
                Current Angles: {self.joint_positions}')

    def pd_timer_callback(self):
        if self.target_joint_positions is not None:
            command_msg = Float64MultiArray()
            command_msg.data = self.target_joint_positions.tolist()
            self.command_publisher.publish(command_msg)

def main():
    rclpy.init()
    inverse_kinematics = InverseKinematics()
    
    try:
        rclpy.spin(inverse_kinematics)
    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        # Send zero torques
        zero_torques = Float64MultiArray()
        zero_torques.data = [0.0] * 12
        inverse_kinematics.command_publisher.publish(zero_torques)
        
        inverse_kinematics.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
