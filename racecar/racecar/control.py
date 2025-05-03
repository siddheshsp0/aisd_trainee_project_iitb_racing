#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

class Commander(Node):

    def __init__(self):
        super().__init__('user_controller')
        self.create_subscription(Twist, '/cmd_vel', self.callback, 10)
        self.vel = 0.0
        self.steer = 0.0
        self.vel_cmd = [0.0]*4
        self.pos_cmd = [0.1]*2

        self.pub_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        self.vel_cmd[0] = self.vel
        self.vel_cmd[1] = self.vel
        self.vel_cmd[2] = self.vel
        self.vel_cmd[3] = self.vel

        self.pos_cmd[0] = self.steer
        self.pos_cmd[1] = self.steer
        pos_array = Float64MultiArray(data=self.pos_cmd) 
        vel_array = Float64MultiArray(data=self.vel_cmd) 
        self.pub_pos.publish(pos_array)
        self.pub_vel.publish(vel_array)
        self.pos_cmd = [0.1]*2
        self.vel_cmd = [0.0]*4
    
    def callback(self, msg):
        self.steer = msg.angular.z*.3
        self.vel = msg.linear.x
        self.get_logger().info(f'vel: {self.vel} steer:{self.steer}')

def main():
    rclpy.init(args=None)
    commander = Commander()

    try:
        rclpy.spin(commander)
    except KeyboardInterrupt:
        pass

    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()