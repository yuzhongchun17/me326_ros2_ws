#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np

# Transformation 
import tf_transformations

# plot
import json

# import the message type to use
from geometry_msgs.msg import Point, Twist, PoseStamped
from nav_msgs.msg import Odometry
# from visualization_msgs.msg import Marker

class CircularTrajectoryController(Node): 
    def __init__(self):
        super().__init__('circular_trajectory_controller')
        #this is the distance of the point P (x,y) that will be controlled for position. The locobot base_link frame points forward in the positive x direction, the point P will be on the positive x-axis in the body-fixed frame of the robot mobile base
        self.L = 0.1

        # Declare proportional gain parameters with default values
        self.declare_parameter('Kp_linear', 1.0)
        self.declare_parameter('Kp_angular', 1.0)
        # Get parameters
        self.Kp_linear = self.get_parameter('Kp_linear').get_parameter_value().double_value
        self.Kp_angular = self.get_parameter('Kp_angular').get_parameter_value().double_value

        # Trajectory parameters
        self.radius = 0.5  # Circle radius in meters
        self.angular_velocity = 2 * np.pi / 20  # 2*pi radians in 20 seconds for a full circle
        self.duration = 20.0  # seconds

        # Current pose of the robot (robot state)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Initialization time
        self.t_init = self.get_clock().now()

        # Subscribe to odometry
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Publish to velocity control
        self.mobile_base_vel_publisher = self.create_publisher(Twist, '/locobot/diffdrive_controller/cmd_vel_unstamped', 10)

        # Publish stamped twist for rosbag (save for plotting)
        self.position_msg_publisher = self.create_publisher(PoseStamped, '/locobot/position_stamped', 10)

        # Lists to store data for plotting
        self.time_stamps = []
        self.desired_x_arr = []
        self.actual_x_arr = []
        self.desired_y_arr = []
        self.actual_y_arr = []
        self.results_file = f'results_kp_{self.Kp_linear}.json'

    
    def calculate_desired_trajectory(self, elapsed_time):
        """
        Computes the desired position on a circular trajectory based on the elapsed time.

        The trajectory is defined by a circle with a specified radius, and the function calculates
        the position (desired_x, desired_y) on this circle corresponding to the elapsed time, completing
        a full loop in 10 seconds.

        Args:
            elapsed_time (float): The time elapsed since the start of the trajectory in seconds.

        Returns:
            tuple: The desired x and y coordinates on the circular trajectory.
        """
        desired_x = self.radius * np.cos(elapsed_time * 2 * np.pi / 10)
        desired_y = self.radius * np.sin(elapsed_time * 2 * np.pi / 10)
        return desired_x, desired_y

    
    def odom_callback(self, msg):
        """
        Callback function for processing odometry data. It updates the robot's current position (x, y) and
        orientation (yaw) based on the received odometry message.

        The odometry message's orientation, given in quaternion form, is converted to yaw in Euler angles.

        Args:
            msg (Odometry): The received odometry message, containing the robot's position and orientation.

        Updates:
            self.current_x (float): Current x-coordinate of the robot.
            self.current_y (float): Current y-coordinate of the robot.
            self.current_yaw (float): Current yaw of the robot in radians, extracted from the quaternion.
        """
        # Update robot's current pose
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation # quaternion

        # Euler from quaternion to get current yaw angle (in radians)
        _,_,self.current_yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Get odom reading then calculate and publish control
        self.control_callback()


    def control_callback(self):
        """
        Periodically called to update the robot's velocity based on its position relative to a desired circular trajectory.
        Calculates the current position error, computes proportional control inputs for linear and angular velocities,
        and publishes these velocities to move the robot along the trajectory. Stops the robot if the elapsed time exceeds
        the trajectory duration.
        """
        t_cur = self.get_clock().now()
        t_elapsed = (t_cur - self.t_init).nanoseconds / 1e9

        # Step 1: Calculate the point P location
        point_P = Point()
        point_P.x = self.current_x + self.L * np.cos(self.current_yaw)
        point_P.y = self.current_y + self.L * np.sin(self.current_yaw)
        point_P.z = 0.1  # placeholder, not used in 2D control
    
        # Step 2: Calculate the error for circular trajectory
        if t_elapsed <= self.duration:
            desired_x, desired_y = self.calculate_desired_trajectory(t_elapsed)
            error_vect = np.array([desired_x - point_P.x, 
                                   desired_y - point_P.y])
            
            # Step 3: Calculate control input
            Kp_linear = 1.0  # Proportional gain for linear control
            Kp_angular = 1.0  # Proportional gain for angular control
            distance_error = np.linalg.norm(error_vect) # normalize distance error
            angle_to_target = np.arctan2(error_vect[1], error_vect[0])
            angle_error = angle_to_target - self.current_yaw
            angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi # normalize angle err to -pi to pi
            
            # Control inputs
            v = Kp_linear * distance_error # linear velocity
            u = Kp_angular * angle_error # angular velocity
            
            # Step 4: Publish the control message
            control_msg = Twist()
            control_msg.linear.x = v
            control_msg.angular.z = u
            self.mobile_base_vel_publisher.publish(control_msg)

            # Extra: Store control messages with timestamp for plotting later
            pos_msg_stamped = PoseStamped()
            pos_msg_stamped.header.stamp = t_cur.to_msg()  # Assign current time
            pos_msg_stamped.pose.position.x = self.current_x
            pos_msg_stamped.pose.position.y = self.current_y
            self.position_msg_publisher.publish(pos_msg_stamped)

            # Append the current time and x-coordinates to the lists
            self.time_stamps.append(t_elapsed)
            self.desired_x_arr.append(desired_x) # x
            self.actual_x_arr.append(self.current_x)
            self.desired_y_arr.append(desired_y) # y
            self.actual_y_arr.append(self.current_y)

        else:
            self.stop_robot()

    def stop_robot(self):
        # Stop the robot after completing the trajectory
        print("Trajectory finished!")
        control_msg = Twist()
        control_msg.linear.x = 0.0
        control_msg.angular.z = 0.0
        self.mobile_base_vel_publisher.publish(control_msg)

        # Save results to a json file
        # self.save_results()

        # Plot for quick check
        # self.plot_trajectory_data()

    # def save_results(self):
    #     # Save the timestamps and actual x-coordinates to a file
    #     results = {
    #         'time_stamps': self.time_stamps,
    #         'desired_x': self.desired_x_arr,
    #         'actual_x': self.actual_x_arr,
    #         'desired_y': self.desired_y_arr,
    #         'actual_y': self.actual_y_arr

    #     }
    #     with open(self.results_file, 'w') as f:
    #         json.dump(results, f)


    # for quick check after each round        
    # def plot_trajectory_data(self):
    #     import matplotlib.pyplot as plt

    #     plt.figure(figsize=(10, 5))
    #     plt.plot(self.time_stamps, self.desired_xs, label='Desired x', marker='o')
    #     plt.plot(self.time_stamps, self.actual_xs, label='Actual x', marker='x')
    #     plt.title('Desired vs Actual x-coordinate Over Time')
    #     plt.xlabel('Time (seconds)')
    #     plt.ylabel('x-coordinate (meters)')
    #     plt.legend()
    #     plt.grid(True)
    #     plt.show()

def main(args=None):
    rclpy.init(args=args)
    circular_trajectory_controller = CircularTrajectoryController()
    rclpy.spin(circular_trajectory_controller)
    circular_trajectory_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()