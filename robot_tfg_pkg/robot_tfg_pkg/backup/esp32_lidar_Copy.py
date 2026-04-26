import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import math

class Esp32SensoresNode(Node):
    def __init__(self):
        super().__init__('nodo_sensores_esp32')
        
        self.pub_lidar = self.create_publisher(Range, '/sensor_distancia', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
            self.ser.reset_input_buffer()
            self.get_logger().info('Conectado al ESP32 (TF-Luna + MPU-9150)')
        except Exception as e:
            self.get_logger().error(f'Error conectando: {e}')
            self.ser = None

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.dt = 0.01 # 100 Hz

        self.timer = self.create_timer(self.dt, self.leer_serial)

    # --- NEW PURE MATHEMATICAL FUNCTION ---
    def euler_a_cuaternion(self, roll, pitch, yaw):
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw

    def leer_serial(self):
        if self.ser and self.ser.in_waiting > 0:
            try:
                linea = self.ser.readline().decode('utf-8').strip()
                
                # --- PROCESS LIDAR ---
                if linea.startswith('L:'):
                    valor_str = linea[2:]
                    if valor_str.isdigit():
                        distancia_m = int(valor_str) / 100.0
                        msg = Range()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = "tf_luna_link"
                        msg.radiation_type = Range.INFRARED
                        msg.range = distancia_m
                        self.pub_lidar.publish(msg)
                
                # --- PROCESS IMU + CALCULATE ORIENTATION ---
                elif linea.startswith('I:'):
                    valores_str = linea[2:].split(',')
                    if len(valores_str) == 6:
                        ax, ay, az = map(float, valores_str[0:3])
                        gx, gy, gz = map(float, valores_str[3:6])

                        # Calculate Roll and Pitch using the accelerometer
                        self.roll = math.atan2(ay, az)
                        self.pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
                        
                        # Calculate Roll and Pitch using the accelerometer
                        self.yaw += gz * self.dt

                        # Convert using our mathematical function (Goodbye library issues!)
                        qx, qy, qz, qw = self.euler_a_cuaternion(self.roll, self.pitch, self.yaw)

                        # Create IMU message
                        msg = Imu()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = "imu_link"
                        
                        msg.orientation.x = qx
                        msg.orientation.y = qy
                        msg.orientation.z = qz
                        msg.orientation.w = qw
                        
                        msg.linear_acceleration.x = ax
                        msg.linear_acceleration.y = ay
                        msg.linear_acceleration.z = az
                        msg.angular_velocity.x = gx
                        msg.angular_velocity.y = gy
                        msg.angular_velocity.z = gz
                        msg.orientation_covariance[0] = -1.0 
                        
                        self.pub_imu.publish(msg)

                        # Publish Transform for RViz
                        t = TransformStamped()
                        t.header.stamp = msg.header.stamp
                        t.header.frame_id = 'map'
                        t.child_frame_id = 'imu_link'
                        
                        t.transform.translation.x = 0.0
                        t.transform.translation.y = 0.0
                        t.transform.translation.z = 0.0
                        
                        t.transform.rotation.x = qx
                        t.transform.rotation.y = qy
                        t.transform.rotation.z = qz
                        t.transform.rotation.w = qw
                        
                        self.tf_broadcaster.sendTransform(t)

            except Exception as e:
                pass 

def main(args=None):
    rclpy.init(args=args)
    nodo = Esp32SensoresNode()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
