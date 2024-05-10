#!/usr/bin/env python3
import rclpy
import serial
import numpy as np
import math
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped #Publicar mensaje de cmd_vel
from tf2_ros import TransformBroadcaster
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry #Publicar mensaje de odometria
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from tf_transformations import quaternion_from_euler

class VelocidadRuedas(Node):
        def __init__(self):
            super().__init__("velocidad_ruedas")

            self.declare_parameter("wheel_radius", 0.033)
            self.declare_parameter("wheel_separation", 0.17)

            self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
            self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

            self.ser = serial.Serial('/dev/ttyUSB1', 115200)  # Cambia 'COM17' según tu puerto
            self.ser.flush()

            # Variables para almacenar los valores de la rueda derecha y la rueda izquierda
            self.right_wheel_value = 0
            self.left_wheel_value = 0
            self.rotational_left= 0
            self.rotational_right= 0
            self.x_ = 0
            self.y_ = 0
            self.theta_ = 0


            # Suscriptor al joint y publicador de odom

            #self.joint_sub_ = self.create_subscription(JointState,"joint_states", self.jointCallback, 10)
            self.odom_pub_ = self.create_publisher(Odometry, "odom", 10)


            while True:
              # Leer una línea de datos desde el puerto serial
                encoder_read = self.ser.readline().decode().strip()  # Decodificar bytes y eliminar espacios en blanco

                # Separar los datos por la coma
                parts = encoder_read.split(',')

                # Iterar sobre las partes de los datos recibidos
                for part in parts:
                    # Verificar si la parte comienza con 'r' o 'l' para determinar la rueda correspondiente
                    if part.startswith('r'):
                        # Si la letra es 'p', multiplicar por 1, si es 'n', multiplicar por -1
                        multiplier = 1 if part[1] == 'p' else -1
                        self.right_wheel_value = float(part[2:]) * multiplier  # Eliminar los primeros dos caracteres ('r' y la letra 'p' o 'n') y convertir el valor a float
                    elif part.startswith('l'):
                        # Si la letra es 'p', multiplicar por 1, si es 'n', multiplicar por -1
                        multiplier= 1 if part[1] == 'p' else -1
                        self.left_wheel_value = float(part[2:]) * multiplier # Eliminar los primeros dos caracteres ('l' y la letra 'p' o 'n') y convertir el valor a float

                    # Implements the inverse differential kinematic model
                    # Given the position of the wheels, calculates their velocities
                    # then calculates the velocity of the robot wrt the robot frame
                    # and then converts it in the global frame and publishes the TF


                    # Calculate the rotational speed of each wheel
                    self.rotational_left= self.left_wheel_value
                    self.rotational_right= self.right_wheel_value

                    # Calculate the linear and angular velocity
                    linear = (self.wheel_radius_ * self.rotational_right + self.wheel_radius_ * self.rotational_left) / 2
                    angular = (self.wheel_radius_ * self.rotational_right - self.wheel_radius_ * self.rotational_left) / self.wheel_separation_

                    # Calculate the position increment
                    d_s = (self.wheel_radius_ * self.rotational_right + self.wheel_radius_ * self.rotational_left) / 2
                    d_theta = (self.wheel_radius_ * self.rotational_right- self.wheel_radius_ * self.rotational_left) / self.wheel_separation_
                    self.theta_ += d_theta
                    self.x_ += d_s * math.cos(self.theta_)
                    self.y_ += d_s * math.sin(self.theta_)


                    #self.get_logger().info("x %f" % self.x_)
                    #self.get_logger().info("y %f" % self.y_)
                    #self.get_logger().info("theta %f" % self.theta_)
                   #self.get_logger().info("linear %f" % linear)
                    #self.get_logger().info("angular %f" % angular)
                    self.get_logger().info("x %f, y %f, theta %f, linear %f, angular %f" % (self.x_, self.y_, self.theta_, linear, angular))

                    odom_msg = Odometry()
                    odom_msg.header.frame_id = "odom"
                    odom_msg.child_frame_id = "base_footprint"
                    odom_msg.header.stamp = self.get_clock().now().to_msg()
                    q = quaternion_from_euler(0, 0, self.theta_)
                    # Llenar el mensaje de odometría

                    odom_msg.pose.pose.position.x = self.x_
                    odom_msg.pose.pose.position.y = self.y_
                    odom_msg.pose.pose.orientation.x = q[0]
                    odom_msg.pose.pose.orientation.y = q[1]
                    odom_msg.pose.pose.orientation.z = q[2]
                    odom_msg.pose.pose.orientation.w = q[3]
                    odom_msg.pose.pose.position.z = 0.0
                    odom_msg.twist.twist.linear.x = linear
                    odom_msg.twist.twist.angular.z = angular

                    # Publicar el mensaje de odometría
                    self.odom_pub_.publish(odom_msg)

                    br_ = TransformBroadcaster(self)
                    transform_stamped_ = TransformStamped()
                    transform_stamped_.header.frame_id = "odom"
                    transform_stamped_.child_frame_id = "base_footprint"
                    transform_stamped_.transform.translation.x = self.x_
                    transform_stamped_.transform.translation.y = self.y_
                    transform_stamped_.transform.rotation.x = q[0]
                    transform_stamped_.transform.rotation.y = q[1]
                    transform_stamped_.transform.rotation.z = q[2]
                    transform_stamped_.transform.rotation.w = q[3]
                    transform_stamped_.header.stamp = self.get_clock().now().to_msg()

                    # Publish the TF
                    br_.sendTransform(transform_stamped_)




def main():
    rclpy.init()
    velocidas_ruedas = VelocidadRuedas()
    rclpy.spin(velocidas_ruedas)
    velocidas_ruedas.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
