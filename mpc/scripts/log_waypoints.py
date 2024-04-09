#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import atexit
from nav_msgs.msg import Odometry
import os
from time import gmtime, strftime
from numpy import linalg as LA
from geometry_msgs.msg import Quaternion
import math


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
        q = data.pose.pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

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
