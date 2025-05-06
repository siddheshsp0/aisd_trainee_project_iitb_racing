# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist,Pose
# from nav_msgs.msg import Odometry as Odom
# import numpy as np
# import math
# from math import sin, cos
# # from tf_transformations import euler_from_quaternion
# from scipy.spatial.transform import Rotation as R

# from visualization_msgs.msg import Marker



# class VehicleModel(Node):
#     def __init__(self):
#         self.mass = 4.0 # kg
#         self.Iz = 0.0586 # kg m2
#         self.wheel_base = 0.35 # Metres
#         self.com = self.wheel_base/2 # From the front
#         self.lf = self.com
#         self.lr = self.wheel_base - self.com

#         self.C_stiff_f = 4.2 # N/rad, Cornering stiffness of front tire
#         self.C_stiff_r = 4.2 # N/rad, Cornering stiffness of rear tire

#         self.d_state_dt = np.array([[0.0,], 
#                                     [0.0,],
#                                     [0.0,]])
#         self.state = np.array([[0.0,],
#                                [0.0,],
#                                [0.0,]])
#         self.glob_state = np.array([[0.0, 0.0], [5.0, 5.0 ], [0.0, 0.0], [0.0, 0.0]]) # (x y) (dx/dt dy/dt) (yaw) (w omega)
#         self.Vlon = 0.0

#         self.sec_history = 0.0
#         self.nanosec_history = 0.0


#         super().__init__('vehicle_model')
        # self.bicycle_publisher = self.create_publisher(Pose, '/pose/bicycle', 10)
        # self.point_mass_publisher = self.create_publisher(Pose, '/pose/point_mass', 10)
        # self.odometry_sub = self.create_subscription(Odom, '/odom', self.odo_callback, 5)
        # self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 5)

        # self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        # # self.timer = self.create_timer(0.5, self.publish_arrow)  # Publish every 0.5s


#     def cmd_callback(self, msg):
#         self.Vlon = msg.linear.x
#         self.state[0][0] = msg.linear.y
#         self.state[2][0] = msg.angular.z

#     def odo_callback(self, msg):
#         dt = msg.header.stamp.sec + (msg.header.stamp.nanosec)/(1e9) - self.sec_history - (self.nanosec_history)/(1e9)
#         self.sec_history=msg.header.stamp.sec
#         self.nanosec_history=msg.header.stamp.nanosec



#         r = R.from_quat([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
#         roll, pitch, yaw = r.as_euler('xyz', degrees=True)  # Convert to roll, pitch, yaw
#         # roll, pitch, yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.w])

#         self.glob_state[0][0] = msg.pose.pose.position.x
#         self.glob_state[0][1] = msg.pose.pose.position.y
#         self.glob_state[2][0] = yaw

#         self.state[0][0] = msg.twist.twist.linear.y # VLat
#         self.Vlon = msg.twist.twist.linear.x # VLon
#         # self.state[2][0] = msg.twist.twist.angular.z
#         self.state[2][0] = 0.5
#         delta = math.atan(self.state[2][0] * self.wheel_base / self.Vlon) if(self.Vlon!=0.0) else 0.0 # delta

#         self.calc_update_state(delta, dt)

#         bicycle_publish_msg = Pose()
#         bicycle_publish_msg.position.x = self.glob_state[0][0]
#         bicycle_publish_msg.position.y = self.glob_state[0][1]
#         bicycle_publish_msg.position.z = 0.0
#         bicycle_publish_msg.orientation.x = 0.0
#         bicycle_publish_msg.orientation.y = 0.0
#         bicycle_publish_msg.orientation.z = self.glob_state[2][0]

#         self.bicycle_publisher.publish(bicycle_publish_msg)
#         self.get_logger().info(f'Published x: {self.glob_state[0][0]}, y:{self.glob_state[0][1]}, yaw: {self.glob_state[2][0]}')

#         # Publishing marker



#         marker = Marker()
#         marker.header.frame_id = "arrow_marker"
#         marker.header.stamp = self.get_clock().now().to_msg()

#         marker.ns = "arrow_namespace"
#         marker.id = 0
#         marker.type = Marker.ARROW
#         marker.action = Marker.ADD

#         # Position (start of the arrow)
#         marker.pose.position.x = self.glob_state[0][0]
#         marker.pose.position.y = self.glob_state[0][1]
#         marker.pose.position.z = 0.0

#         # Orientation (quaternion: no rotation)
#         marker.pose.orientation.x = msg.pose.pose.orientation.x
#         marker.pose.orientation.y = msg.pose.pose.orientation.y
#         marker.pose.orientation.z = msg.pose.pose.orientation.z
#         marker.pose.orientation.w = msg.pose.pose.orientation.w

#         # Scale (length & thickness of arrow)
#         marker.scale.x = 1.0  # Arrow length
#         marker.scale.y = 0.1  # Arrow width
#         marker.scale.z = 0.1  # Arrow height

#         # Color (RGBA)
#         marker.color.r = 1.0  # Red
#         marker.color.g = 0.0  # Green
#         marker.color.b = 0.0  # Blue
#         marker.color.a = 1.0  # Alpha (1 = fully visible)

#         # Publish the marker
#         self.publisher.publish(marker)
#         self.get_logger().info("Arrow marker published!")







#     def calc_update_state(self, delta, dt):
#         self.d_state_dt = np.dot(np.array(
#             [
#                 [-(self.C_stiff_r + self.C_stiff_f)/(self.mass * self.Vlon), 0, (self.C_stiff_r * self.lr - self.C_stiff_f * self.lf)/(self.mass * self.Vlon)],
#                 [0, 0, 1],
#                 [(self.lr * self.C_stiff_r - self.lf * self.C_stiff_f)/(self.Iz * self.Vlon), 0, -(self.lf * self.lf * self.C_stiff_f + self.lr * self.lr * self.C_stiff_r)/(self.Iz * self.Vlon)],
#             ]
#         ), self.state) + delta * np.array([[self.C_stiff_f/self.mass],
#                                    [0.0],
#                                    [self.C_stiff_f*self.lf/self.Iz],])

#         self.state += self.d_state_dt*dt
#         self.glob_state[1][0] = self.Vlon * cos(self.state[1][0]) - self.state[0][0] * sin(self.state[1][0])
#         self.glob_state[1][1] = self.Vlon * sin(self.state[1][0]) + self.state[0][0] * cos(self.state[1][0])

#         self.glob_state[0][0] += self.glob_state[1][0]*dt
#         self.glob_state[0][1] += self.glob_state[1][1]*dt
#         # self.glob_state[2][0] += self.state[2][0]*dt




# def main(args=None):
#     rclpy.init(args=args)
#     model = VehicleModel()
#     rclpy.spin(model)
#     model.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist,Pose
from nav_msgs.msg import Odometry as Odom
from visualization_msgs.msg import Marker
from math import sin, cos, atan




class State():
    def __init__(self, x, y, yaw, Vlon, state):
        self.x=x
        self.y=y
        self.yaw=yaw
        self.Vlon=Vlon
        self.state=state

    


class VehicleModel(Node):
    def __init__(self):
        self.dt = 0.01
        self.mass = 4.0 # kg
        self.Iz = 0.0586 # kg m2
        self.wheel_base = 0.35 # Metres
        self.com = self.wheel_base/2 # From the front
        self.lf = self.com
        self.lr = self.wheel_base - self.com
        self.C_stiff_f = 4.2 # N/rad, Cornering stiffness of front tire
        self.C_stiff_r = 4.2 # N/rad, Cornering stiffness of rear tire
        self.input = Twist()
        self.input.linear.x=0.0
        self.input.linear.y=0.0
        self.input.linear.z=0.0
        self.input.angular.x=0.0
        self.input.angular.y=0.0
        self.input.angular.z=0.0
        self.state_history = State(0,0,0,0,np.array([[0.0],[0.0],[0.0]]),)

        self.Vlon = 0.0
        self.w = 0.0

        super().__init__('vehicle_model')
        self.bicycle_publisher = self.create_publisher(Pose, '/pose/bicycle', 10)
        self.point_mass_publisher = self.create_publisher(Pose, '/pose/point_mass', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 5)

        self.vehicle_timer = self.create_timer(self.dt, self.callback)
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)



        # Marker things
        self.marker = Marker()
        self.marker.header.frame_id = "world"
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.ns = "arrow_namespace"
        self.marker.id = 0
        self.marker.type = Marker.ARROW
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.5  # Arrow length
        self.marker.scale.y = 0.1  # Arrow width
        self.marker.scale.z = 0.1  # Arrow height
        # Color (RGBA)
        self.marker.color.r = 1.0  
        self.marker.color.g = 0.0   
        self.marker.color.b = 0.0   
        self.marker.color.a = 1.0
        self.marker.pose.position.x =0.0
        self.marker.pose.position.y =0.0
        self.marker.pose.position.z =0.0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0

        self.publisher.publish(self.marker)
        # self.get_logger().info(f'Published x: {self.glob_state[0][0]}, y:{self.glob_state[0][1]}, yaw: {self.glob_state[2][0]}')


    def cmd_callback(self, msg):
        self.input = msg
        self.Vlon = msg.linear.x
        self.w = msg.angular.z


    def callback(self):
        w = self.w
        Vlon = self.Vlon
        if(Vlon!=0):
            delta = atan(w * self.wheel_base / Vlon)
        else:
            delta =0.0
        self.state_history.state[0][0] = self.input.linear.y
        new_state = self.calc_next_state(delta, Vlon, w)

        # Position (start of the arrow)
        self.marker.pose.position.x = float(new_state.x)
        self.marker.pose.position.y = float(new_state.y)
        self.marker.pose.position.z = 0.0

        # Orientation (writing yaw=k in terms of quaternions is z=sin(k/2) and w =cos(k/2))
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = sin(new_state.yaw/2)
        self.marker.pose.orientation.w = cos(new_state.yaw/2)

        message = Pose()
        message.position.x = float(new_state.x)
        message.position.y = float(new_state.y)
        message.position.z = 0.0

        message.orientation.x = 0.0
        message.orientation.y = 0.0
        message.orientation.z = sin(new_state.yaw/2)
        message.orientation.w = cos(new_state.yaw/2)


        # msg_dummy = Pose()
        # msg_dummy.orientation.x = 0.0
        # msg_dummy.orientation.y = 0.0
        # msg_dummy.orientation.z = 0.0
        # msg_dummy.orientation.w = 0.0
        # msg_dummy.position.x = 10.0
        # msg_dummy.position.y = 10.0
        # msg_dummy.position.z = 10.0
        # self.bicycle_publisher.publish(msg_dummy)
        self.bicycle_publisher.publish(message)
        self.publisher.publish(self.marker)
        # self.get_logger().info(f"New state: x={message.position.x}, y={message.position.y} \n Vlon={Vlon}, w={w}")

        self.state_history = new_state






    def calc_next_state(self,delta, Vlon, w):
        if(Vlon!=0):
            state_calc_matrix = np.array([
                [-(self.C_stiff_r + self.C_stiff_f)/(self.mass * Vlon), 0, (self.C_stiff_r * self.lr - self.C_stiff_f * self.lf)/(self.mass * Vlon)],
                [0, 0, 1],
                [(self.lr * self.C_stiff_r - self.lf * self.C_stiff_f)/(self.Iz * Vlon), 0, -(self.lf * self.lf * self.C_stiff_f + self.lr * self.lr * self.C_stiff_r)/(self.Iz * Vlon)],
            ])
            d_state_dt = np.dot(state_calc_matrix, self.state_history.state) + np.array([[self.C_stiff_f/self.mass], [0.0], [self.C_stiff_f*self.lf/self.Iz]]) * delta
            next_state_matrix = self.state_history.state + self.dt * d_state_dt
            next_state_matrix[2] = w
            Vx = Vlon * cos(next_state_matrix[1][0]) - next_state_matrix[0][0] * sin(next_state_matrix[1][0])
            Vy = Vlon * sin(next_state_matrix[1][0]) + next_state_matrix[0][0] * cos(next_state_matrix[1][0])
            self.get_logger().info(f"Vx: {Vx}, Vy: {Vy}, phi:{next_state_matrix[1][0]}, Vlon: {Vlon}, Vlat: {next_state_matrix[0][0]}")
            # next_state = State(
            #     self.state_history.x + Vx*self.dt,
            #     self.state_history.y + Vy*self.dt,
            #     self.state_history.yaw + next_state_matrix[2][0] * self.dt,
            #     Vlon,
            #     next_state_matrix,
            # )
            next_state = State(
                self.state_history.x + Vx * self.dt,
                self.state_history.y + Vy * self.dt,
                self.state_history.yaw + next_state_matrix[2][0] * self.dt,
                Vlon,
                next_state_matrix,
            )


            # Remove this line ###############
            # next_state.state[2] = 0.0
            # Remove this!!
            return next_state
        else:
            return self.state_history






def main(args=None):
    rclpy.init(args=args)
    model = VehicleModel()
    rclpy.spin(model)
    model.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
