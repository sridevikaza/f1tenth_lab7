#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry

import os
import atexit

from scipy.interpolate import splprep, splev
from transforms3d.euler import quat2euler


path_to_directory = '/sim_ws/src/f1tenth_lab7/waypoints'
os.makedirs(path_to_directory, exist_ok=True)

file_path = os.path.join(path_to_directory, 'mpc_waypoints.csv')
file = open(file_path, 'w')

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoints_logger')
        self.subscription = self.create_subscription(
            Odometry,
            'ego_racecar/odom',
            self.save_waypoint,
            10)
        self.subscription

    def save_waypoint(self, data):
        # extract yaw
        roll, pitch, yaw = quat2euler([
            data.pose.pose.orientation.w,
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
        ])

        # write x, y, yaw, and velocity to the file

        file.write('%f, %f, %f, %f\n' % (data.pose.pose.position.x, 
                                         data.pose.pose.position.y, 
                                         yaw, 
                                         data.twist.twist.linear.x))

def shutdown():
    file.close()

def main(args=None):
    rclpy.init(args=args)
    waypoint_logger = WaypointLogger()
    atexit.register(shutdown)
    print(f'saving waypoints to {file_path}')
    rclpy.spin(waypoint_logger)
    waypoint_logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()