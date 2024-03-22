#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.constants import S_TO_NS
from rclpy.time import Time
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdometryPublisher(Node):

    def __init__(self):
        super().__init__("odometry_publisher")

        #----Parámetros de las ruedas----
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)
        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        #---Parámetros de la odometría---
        self.linear = 0
        self.angular = 0
        self.x = 0
        self.y = 0
        self.theta = 0
        self.last_x = 0
        self.last_y = 0
        self.last_theta = 0

        #----Suscriptores de encoders---
        self.vel_enc_right = None
        self.vel_enc_left = None

        #----Publicadores y suscriptores
        self.odom_pub_ = self.create_publisher(Odometry, "odom", 10)
        self.right_wheel_sub = self.create_subscription(Float64, "right_wheel_value", self.right_wheel, 10)
        self.left_wheel_sub = self.create_subscription(Float64, "left_wheel_value", self.left_wheel, 10)
  
        self.prev_time_ = self.get_clock().now()

        # Inicia un temporizador para publicar la odometría cada 0.1 segundos
        #self.timer_period = 0.1  # 0.1 segundos
        #self.timer = self.create_timer(self.timer_period, self.publish_odom)


    def right_wheel(self, msg):
        enc_right = msg.data
        self.vel_enc_right=enc_right

    def left_wheel(self, msg):
        enc_left = msg.data
        self.vel_enc_left=enc_left     

        self.get_logger().info("RIGHT %f" % self.vel_enc_right)
        self.get_logger().info("LEFT %f" % self.vel_enc_left)

        #---Tomamos los valores desde Arduino---
        self.vel_enc_der = self.vel_enc_right
        self.vel_enc_izq = self.vel_enc_left
        #----Calculamos la velocidad Lineal y Angular
        self.linear = (self.vel_enc_der + self.vel_enc_izq) * (self.wheel_radius_ * np.pi) / (2 * 60)
        self.angular = (self.vel_enc_der - self.vel_enc_izq) * (self.wheel_radius_ * np.pi) / (self.wheel_separation_ * 60)
        
        #----Calcular el tiempo transcurrido desde la última actualización
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time_).nanoseconds / S_TO_NS

        #----Calculamos la posición x, y y el ángulo theta
        #self.x = self.last_x + self.linear * np.cos(self.theta) * dt
        #self.y = self.last_y + self.linear * np.sin(self.theta) * dt

        self.x = self.last_x + (self.linear * np.cos(self.theta) * dt)+ (self.angular * np.sin(self.theta) * dt)
        self.y = self.last_y + (self.linear * np.sin(self.theta) * dt)+ (self.angular * np.cos(self.theta) * dt)
        self.theta = self.last_theta + self.angular * dt
        #----Actualizamos los valores de x, y y theta
        self.last_x = self.x
        self.last_y = self.y
        self.last_theta = self.theta


        #----Generamos los valores para la publicación de la Odometría
        odom_msg= Odometry()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        #Setear la posicion
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        #Setear la velocidad
        odom_msg.twist.twist.linear.x = self.linear 
        odom_msg.twist.twist.angular.z = self.angular
        #Realizar la publicación de odometria
        self.odom_pub_.publish(odom_msg)
        # Actualizar el tiempo previo
        self.prev_time_ = current_time


    
def main():
    rclpy.init()
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)
    odometry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
