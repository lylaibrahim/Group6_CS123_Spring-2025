from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
import numpy as np
import time


IMAGE_WIDTH = 1400


# TODO: Add your new constants here


TIMEOUT = 1.0 #TODO threshold in timer_callback
SEARCH_YAW_VEL = 0.5 #TODO searching constant
TRACK_FORWARD_VEL = 1.0 #TODO tracking constant
KP = 3.0 #TODO proportional gain for tracking






class State(Enum):
   SEARCH = 0
   TRACK = 1


class StateMachineNode(Node):
   def __init__(self):
       super().__init__('state_machine_node')


       self.detection_subscription = self.create_subscription(
           Detection2DArray,
           '/detections',
           self.detection_callback,
           10
       )


       self.command_publisher = self.create_publisher(
           Twist,
           'cmd_vel',
           10
       )


       self.timer = self.create_timer(0.1, self.timer_callback)
       self.state = State.TRACK


       # TODO: Add your new member variables here
       self.kp = KP
       self.last_detection_time = self.get_clock().now().nanoseconds
       self.lastX = 0


       #pass # TODO


   def detection_callback(self, msg):
       """
       Determine which of the HAILO detections is the most central detected objec
       """
       detectionValue = []
       #pass TODO: Part 1
       # breakpoint()
       if len(msg.detections) == 0:
           return
       for detection in msg.detections:
           val = detection.bbox.center.position.x
           val -= 662
           val /= 319
           detectionValue.append(val)
      
       # print(detectionValue)
       self.lastX = min(detectionValue, key=lambda x:abs(x-self.lastX)) if len(detectionValue)!=0 else 0
       self.last_detection_time = self.get_clock().now().nanoseconds


       print(self.lastX)
       # return central


       #breakpoint()


   def timer_callback(self):
       """
       Implement a Stimer callback that sets the moves through the state machine based on if the time since the last detection is above a threshold TIMEOUT
       """
      
       now = self.get_clock().now().nanoseconds
       elapsed_time = float((now - self.last_detection_time)) / 1E9


       if elapsed_time > TIMEOUT: # TODO: Part 3.2
           self.state = State.SEARCH
           print("searching")
       else:
           self.state = State.TRACK
           print("tracking")


       yaw_command = 0.0
       forward_vel_command = 0.0


       if self.state == State.SEARCH:
           pass # TODO: Part 3.1
           yaw_command = SEARCH_YAW_VEL
           forward_vel_command = 0.0
       elif self.state == State.TRACK:
            # TODO: Part 2 / 3.4
           yaw_command = - self.kp * self.lastX
           forward_vel_command = TRACK_FORWARD_VEL


       cmd = Twist()
       cmd.angular.z = yaw_command
       cmd.linear.x = forward_vel_command
       self.command_publisher.publish(cmd)


def main():
   rclpy.init()
   state_machine_node = StateMachineNode()


   try:
       rclpy.spin(state_machine_node)
   except KeyboardInterrupt:
       print("Program terminated by user")
   finally:
       zero_cmd = Twist()
       state_machine_node.command_publisher.publish(zero_cmd)


       state_machine_node.destroy_node()
       rclpy.shutdown()


if __name__ == '__main__':
   main()



