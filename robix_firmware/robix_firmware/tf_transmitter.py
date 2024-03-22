#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class AnotherNode(Node):
    def __init__(self):
        super().__init__("another_node")
        self.subscription_odom = self.create_subscription(Odometry,"odom",self.odom_callback,10)

    def odom_callback(self, msg):
        # Extraer los valores de posición x e y del mensaje Odometry
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        br_ = TransformBroadcaster(self)
        transform_stamped_ = TransformStamped()
        transform_stamped_.header.frame_id = "odom"
        transform_stamped_.child_frame_id = "base_footprint"
        transform_stamped_.transform.translation.x = x
        transform_stamped_.transform.translation.y = y
        transform_stamped_.header.stamp = self.get_clock().now().to_msg()

        # Publish the TF
        br_.sendTransform(transform_stamped_)



def main():
    rclpy.init()
    another_node = AnotherNode()
    rclpy.spin(another_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
