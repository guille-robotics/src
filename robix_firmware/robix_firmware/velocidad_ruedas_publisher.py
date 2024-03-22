#!/usr/bin/env python3
import rclpy
import serial
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped #Publicar mensaje de cmd_vel
from nav_msgs.msg import Odometry #Publicar mensaje de odometria
from std_msgs.msg import Float64

class VelocidadRuedas(Node):
        def __init__(self):
            super().__init__("velocidad_ruedas")

            # Inicializar los publicadores para los tópicos /right_wheel_value y /left_wheel_value
            self.right_wheel_pub = self.create_publisher(Float64,"right_wheel_value",10)
            self.left_wheel_pub = self.create_publisher(Float64,"left_wheel_value",10)

            #Parametros arduino
            self.declare_parameter("port", "/dev/ttyACM0")
            self.declare_parameter("baudrate", 115200)
            self.port_ = self.get_parameter("port").value
            self.baudrate_ = self.get_parameter("baudrate").value
            self.arduino_ = serial.Serial(port=self.port_, baudrate=self.baudrate_, timeout=0.1)

            # Variables para almacenar los valores de la rueda derecha y la rueda izquierda
            self.right_wheel_value = None
            self.left_wheel_value = None
        
        # Iniciar el bucle principal del nodo
            #if rclpy.ok() and self.arduino_.is_open:
            while True :
                
                # Leer una línea de datos desde el puerto serial
                encoder_read = self.arduino_.readline().decode().strip()  # Decodificar bytes y eliminar espacios en blanco
                
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
            
                # Publicar los valores de la rueda derecha y la rueda izquierda, si están disponibles
                if self.right_wheel_value is not None:
                    msg_right=Float64()
                    msg_right.data=self.right_wheel_value
                    self.right_wheel_pub.publish(msg_right)
                if self.left_wheel_value is not None:
                    msg_left=Float64()
                    msg_left.data=self.left_wheel_value
                    self.left_wheel_pub.publish(msg_left)

                #self.get_logger().info("RIGHT %f" % self.right_wheel_value)
                #self.get_logger().info("LEFT %f" % self.left_wheel_value)

                #if self.right_wheel_value is not None:
                #    self.get_logger().info("RIGHT %f" % self.right_wheel_value)
                #else:
                #    self.get_logger().info("RIGHT: No value available")

                #if self.left_wheel_value is not None:
                #    self.get_logger().info("LEFT %f" % self.left_wheel_value)
                #else:
                #    self.get_logger().info("LEFT: No value available")


def main():
    rclpy.init()
    velocidas_ruedas = VelocidadRuedas()
    rclpy.spin(velocidas_ruedas)
    velocidas_ruedas.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
