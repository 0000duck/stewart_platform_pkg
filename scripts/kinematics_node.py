#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from stewart_platform_interfaces.msg import TransformsPosVel, LegsLenVel
from stewart_platform_pkg.inverse_kinematics import InverseKinematics

import numpy as np


class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        self.ik = InverseKinematics()

        self.dof_ref_sub = self.create_subscription(
            TransformsPosVel,
            'dof_ref',
            self.__dof_ref_callback,
            1)
        self.dof_ref_sub
        
        self.leg_ref_pub = self.create_publisher(
            LegsLenVel,
            'leg_ref',
            1)
        
        self.get_logger().info('Controller node running!')
        self.get_logger().info('Subscribed to topic dof_ref and publishes legs - length and velocity, to leg_ref')


    def __dof_ref_callback(self, msg):

        pose = np.array([msg.pose.x, msg.pose.y, msg.pose.z, msg.pose.roll, msg.pose.pitch, msg.pose.yaw])
        twist = np.array([msg.twist.x, msg.twist.y, msg.twist.z, msg.twist.roll, msg.twist.pitch, msg.twist.yaw])

        stroke = self.ik.calc_output(pose, twist)

        pub_msg = LegsLenVel()

        pub_msg.leg[0].length   = stroke[0][0]
        pub_msg.leg[1].length   = stroke[0][1]
        pub_msg.leg[2].length   = stroke[0][2]
        pub_msg.leg[3].length   = stroke[0][3]
        pub_msg.leg[4].length   = stroke[0][4]
        pub_msg.leg[5].length   = stroke[0][5]

        pub_msg.leg[0].velocity = stroke[1][0]
        pub_msg.leg[1].velocity = stroke[1][1]
        pub_msg.leg[2].velocity = stroke[1][2]
        pub_msg.leg[3].velocity = stroke[1][3]
        pub_msg.leg[4].velocity = stroke[1][4]
        pub_msg.leg[5].velocity = stroke[1][5]

        self.leg_ref_pub.publish(pub_msg)



def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()
